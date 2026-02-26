#!/usr/bin/env python3
"""Joint auto-search for signs, offsets, and command compensation."""

import argparse
import json
import math
import os
import random
import re
import time
from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple

import rospy
from controller_manager_msgs.srv import (
    LoadController,
    LoadControllerRequest,
    SwitchController,
    SwitchControllerRequest,
    UnloadController,
    UnloadControllerRequest,
)
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from geometry_msgs.msg import Twist


@dataclass
class KeyCase:
    key: str
    vx: float
    vy: float
    wz: float


FULL_CASES: List[KeyCase] = [
    KeyCase("i", 0.5, 0.0, 0.0),
    KeyCase("u", 0.5, 0.0, 1.0),
    KeyCase("o", 0.5, 0.0, -1.0),
    KeyCase("j", 0.0, 0.0, 1.0),
    KeyCase("l", 0.0, 0.0, -1.0),
    KeyCase(",", -0.5, 0.0, 0.0),
    KeyCase("m", -0.5, 0.0, -1.0),
    KeyCase(".", -0.5, 0.0, 1.0),
    KeyCase("I", 0.5, 0.0, 0.0),
    KeyCase("U", 0.5, 0.5, 0.0),
    KeyCase("O", 0.5, -0.5, 0.0),
    KeyCase("J", 0.0, 0.5, 0.0),
    KeyCase("L", 0.0, -0.5, 0.0),
    KeyCase("<", -0.5, 0.0, 0.0),
    KeyCase("M", -0.5, 0.5, 0.0),
    KeyCase(">", -0.5, -0.5, 0.0),
    KeyCase("t", 0.0, 0.0, 0.0),
    KeyCase("b", 0.0, 0.0, 0.0),
    KeyCase("k", 0.0, 0.0, 0.0),
]


def _yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _pose_delta_body(x0: float, y0: float, yaw0: float, x1: float, y1: float):
    dx_world = x1 - x0
    dy_world = y1 - y0
    cos_yaw = math.cos(yaw0)
    sin_yaw = math.sin(yaw0)
    dx_body = cos_yaw * dx_world + sin_yaw * dy_world
    dy_body = -sin_yaw * dx_world + cos_yaw * dy_world
    return dx_body, dy_body


def _axis_ok(value: float, target: float, pos_thr: float, zero_thr: float) -> bool:
    if target > 1e-9:
        return value > pos_thr
    if target < -1e-9:
        return value < -pos_thr
    return abs(value) < zero_thr


def _candidate_key(candidate: Dict) -> Tuple:
    return (
        tuple(candidate["rolling"]),
        tuple(candidate["vx"]),
        tuple(candidate["vy"]),
        tuple(candidate["wz"]),
        tuple(round(v, 6) for v in candidate["offsets"]),
        tuple(round(v, 6) for v in candidate["matrix"]),
    )


def _pick_cases(cases_text: str) -> List[KeyCase]:
    if not cases_text:
        return list(FULL_CASES)
    keys = {token.strip() for token in cases_text.split(",") if token.strip()}
    return [case for case in FULL_CASES if case.key in keys]


class MotionProbe:
    def __init__(self, model_name: str, cmd_topic: str, state_topic: str):
        self._model_name = model_name
        self._state = None
        self._model_index = -1
        self._publisher = rospy.Publisher(cmd_topic, Twist, queue_size=10)
        self._subscriber = rospy.Subscriber(
            state_topic, ModelStates, self._state_callback, queue_size=1
        )

    def _state_callback(self, message: ModelStates):
        self._state = message

    def wait_ready(self, timeout_sec: float):
        start = time.time()
        while not rospy.is_shutdown():
            if self._state is not None and self._model_name in self._state.name:
                self._model_index = self._state.name.index(self._model_name)
                return
            if time.time() - start > timeout_sec:
                raise RuntimeError("Timeout waiting for gazebo model state.")
            rospy.sleep(0.05)
        raise RuntimeError("ROS shutdown before probe became ready.")

    def pose(self):
        if self._state is None or self._model_index < 0:
            raise RuntimeError("Probe not ready.")
        pose = self._state.pose[self._model_index]
        return pose.position.x, pose.position.y, _yaw_from_quaternion(pose.orientation)

    def publish_zero(self, duration_sec: float, hz: float):
        zero = Twist()
        rate = rospy.Rate(hz)
        end_time = time.time() + duration_sec
        while time.time() < end_time and not rospy.is_shutdown():
            self._publisher.publish(zero)
            rate.sleep()

    def run_case(
        self,
        case: KeyCase,
        settle_sec: float,
        segment_sec: float,
        release_sec: float,
        hz: float,
    ):
        if settle_sec > 0.0:
            self.publish_zero(settle_sec, hz)

        x0, y0, yaw0 = self.pose()
        command = Twist()
        command.linear.x = case.vx
        command.linear.y = case.vy
        command.angular.z = case.wz

        rate = rospy.Rate(hz)
        start = time.time()
        while time.time() - start < segment_sec and not rospy.is_shutdown():
            self._publisher.publish(command)
            rate.sleep()

        x1, y1, yaw1 = self.pose()
        dx, dy = _pose_delta_body(x0, y0, yaw0, x1, y1)
        dyaw = _normalize_angle(yaw1 - yaw0)

        if release_sec > 0.0:
            self.publish_zero(release_sec, hz)
            x2, y2, yaw2 = self.pose()
            rdx, rdy = _pose_delta_body(x1, y1, yaw1, x2, y2)
            rdyaw = _normalize_angle(yaw2 - yaw1)
        else:
            rdx, rdy, rdyaw = 0.0, 0.0, 0.0

        return {
            "key": case.key,
            "delta_body": {"dx": dx, "dy": dy, "dyaw": dyaw},
            "release": {"dx": rdx, "dy": rdy, "dyaw": rdyaw},
        }


class ControllerReloader:
    def __init__(self, controller_name: str):
        self._controller_name = controller_name
        rospy.wait_for_service("/controller_manager/load_controller", timeout=20.0)
        rospy.wait_for_service("/controller_manager/unload_controller", timeout=20.0)
        rospy.wait_for_service("/controller_manager/switch_controller", timeout=20.0)
        rospy.wait_for_service("/gazebo/set_model_state", timeout=20.0)
        self._load_srv = rospy.ServiceProxy(
            "/controller_manager/load_controller", LoadController
        )
        self._unload_srv = rospy.ServiceProxy(
            "/controller_manager/unload_controller", UnloadController
        )
        self._switch_srv = rospy.ServiceProxy(
            "/controller_manager/switch_controller", SwitchController
        )
        self._set_model_state_srv = rospy.ServiceProxy(
            "/gazebo/set_model_state", SetModelState
        )

    def set_candidate(self, candidate: Dict):
        prefix = "/sentry_chassis_controller"
        rospy.set_param(prefix + "/wheel_rolling_signs", list(candidate["rolling"]))
        rospy.set_param(prefix + "/wheel_direction_signs/vx", list(candidate["vx"]))
        rospy.set_param(prefix + "/wheel_direction_signs/vy", list(candidate["vy"]))
        rospy.set_param(prefix + "/wheel_direction_signs/wz", list(candidate["wz"]))
        rospy.set_param(prefix + "/steer_zero_offsets", list(candidate["offsets"]))
        rospy.set_param(prefix + "/command_compensation_matrix", list(candidate["matrix"]))

    def _switch(self, start: List[str], stop: List[str]):
        request = SwitchControllerRequest()
        request.start_controllers = list(start)
        request.stop_controllers = list(stop)
        request.strictness = 2
        if hasattr(request, "start_asap"):
            request.start_asap = False
        if hasattr(request, "timeout"):
            request.timeout = 0.0
        try:
            self._switch_srv(request)
        except rospy.ServiceException:
            pass

    def reload(self):
        self._switch(start=[], stop=[self._controller_name])
        try:
            self._unload_srv(UnloadControllerRequest(name=self._controller_name))
        except rospy.ServiceException:
            pass

        response = self._load_srv(LoadControllerRequest(name=self._controller_name))
        if not response.ok:
            return False
        self._switch(start=[self._controller_name], stop=[])
        return True

    def reset_model(self, model_name: str):
        request = SetModelStateRequest()
        state = ModelState()
        state.model_name = model_name
        state.pose.orientation.w = 1.0
        state.reference_frame = "world"
        request.model_state = state
        try:
            self._set_model_state_srv(request)
        except rospy.ServiceException:
            pass


def _evaluate_candidate(
    probe: MotionProbe,
    cases: List[KeyCase],
    settle_sec: float,
    segment_sec: float,
    release_sec: float,
    hz: float,
    linear_pos_thr: float,
    linear_zero_thr: float,
    yaw_pos_thr: float,
    yaw_zero_thr: float,
    release_pos_thr: float,
    release_yaw_thr: float,
):
    results = []
    score = 0
    for case in cases:
        result = probe.run_case(case, settle_sec, segment_sec, release_sec, hz)
        delta = result["delta_body"]
        release = result["release"]
        expectation_ok = {
            "vx": _axis_ok(delta["dx"], case.vx, linear_pos_thr, linear_zero_thr),
            "vy": _axis_ok(delta["dy"], case.vy, linear_pos_thr, linear_zero_thr),
            "wz": _axis_ok(delta["dyaw"], case.wz, yaw_pos_thr, yaw_zero_thr),
        }
        release_ok = (
            abs(release["dx"]) < release_pos_thr
            and abs(release["dy"]) < release_pos_thr
            and abs(release["dyaw"]) < release_yaw_thr
        )
        passed = all(expectation_ok.values()) and release_ok
        result["expectation_ok"] = expectation_ok
        result["release_ok"] = release_ok
        result["pass"] = passed
        score += sum(1 for ok in expectation_ok.values() if ok)
        score += 1 if release_ok else 0
        results.append(result)

    pass_count = sum(1 for r in results if r["pass"])
    return {
        "score": score,
        "pass_count": pass_count,
        "fail_count": len(cases) - pass_count,
        "failed_keys": [r["key"] for r in results if not r["pass"]],
        "results": results,
    }


def _better(lhs: Dict, rhs: Dict) -> bool:
    return (lhs["pass_count"], lhs["score"]) > (rhs["pass_count"], rhs["score"])


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _mutate_candidate(parent: Dict, rng: random.Random, sign_mutation_prob: float) -> Dict:
    child = {
        "rolling": list(parent["rolling"]),
        "vx": list(parent["vx"]),
        "vy": list(parent["vy"]),
        "wz": list(parent["wz"]),
        "offsets": list(parent["offsets"]),
        "matrix": list(parent["matrix"]),
    }

    mutation_count = 1 if rng.random() < 0.7 else 2
    for _ in range(mutation_count):
        pick = rng.random()
        if pick < sign_mutation_prob:
            bits = []
            for key in ("rolling", "vx", "vy", "wz"):
                for idx in range(4):
                    bits.append((key, idx))
            key, idx = bits[rng.randrange(len(bits))]
            child[key][idx] *= -1
        elif pick < sign_mutation_prob + 0.60:
            idx = rng.randrange(4)
            step = rng.choice([0.4, 0.2, 0.1, 0.05, 0.02])
            if rng.random() < 0.5:
                step = -step
            child["offsets"][idx] = _normalize_angle(child["offsets"][idx] + step)
        elif pick < sign_mutation_prob + 0.85:
            delta = rng.choice([0.04, 0.02, 0.01, 0.005])
            if rng.random() < 0.5:
                delta = -delta
            child["matrix"][6] = _clamp(child["matrix"][6] + delta, -0.30, 0.30)
        else:
            delta = rng.choice([0.10, 0.05, 0.02])
            if rng.random() < 0.5:
                delta = -delta
            child["matrix"][8] = _clamp(child["matrix"][8] + delta, 0.60, 1.40)

    return child


def _replace_line(content: str, pattern: str, replacement: str) -> str:
    updated, count = re.subn(pattern, replacement, content, count=1, flags=re.MULTILINE)
    if count != 1:
        raise RuntimeError("Failed to update config with pattern: " + pattern)
    return updated


def _write_candidate_to_config(config_path: str, candidate: Dict):
    with open(config_path, "r", encoding="utf-8") as handle:
        content = handle.read()

    content = _replace_line(
        content,
        r"(^\s*wheel_rolling_signs:\s*)\[[^\]]+\]",
        "\\1[" + ", ".join(str(v) for v in candidate["rolling"]) + "]",
    )
    content = _replace_line(
        content,
        r"(^\s*vx:\s*)\[[^\]]+\]",
        "\\1[" + ", ".join(str(v) for v in candidate["vx"]) + "]",
    )
    content = _replace_line(
        content,
        r"(^\s*vy:\s*)\[[^\]]+\]",
        "\\1[" + ", ".join(str(v) for v in candidate["vy"]) + "]",
    )
    content = _replace_line(
        content,
        r"(^\s*wz:\s*)\[[^\]]+\]",
        "\\1[" + ", ".join(str(v) for v in candidate["wz"]) + "]",
    )

    offsets_block = "\n".join(
        ["  steer_zero_offsets:", *[f"  - {value:.9f}" for value in candidate["offsets"]]]
    )
    content = _replace_line(
        content,
        r"^\s*steer_zero_offsets:\n(?:\s*-\s*[-+0-9eE\.]+\n){4}",
        offsets_block + "\n",
    )

    matrix_text = ", ".join(f"{value:.6f}" for value in candidate["matrix"])
    content = _replace_line(
        content,
        r"(^\s*command_compensation_matrix:\s*)\[[^\]]+\]",
        "\\1[" + matrix_text + "]",
    )

    with open(config_path, "w", encoding="utf-8") as handle:
        handle.write(content)


def _load_candidate_from_params() -> Dict:
    prefix = "/sentry_chassis_controller"
    matrix = list(rospy.get_param(prefix + "/command_compensation_matrix"))
    if len(matrix) != 9:
        raise RuntimeError("command_compensation_matrix must contain 9 values")

    offsets = list(rospy.get_param(prefix + "/steer_zero_offsets"))
    if len(offsets) != 4:
        raise RuntimeError("steer_zero_offsets must contain 4 values")

    def _load_sign(path: str) -> List[int]:
        values = list(rospy.get_param(path))
        if len(values) != 4:
            raise RuntimeError(path + " must contain 4 values")
        for value in values:
            if value not in (-1, 1):
                raise RuntimeError(path + " values must be -1 or 1")
        return [int(v) for v in values]

    return {
        "rolling": _load_sign(prefix + "/wheel_rolling_signs"),
        "vx": _load_sign(prefix + "/wheel_direction_signs/vx"),
        "vy": _load_sign(prefix + "/wheel_direction_signs/vy"),
        "wz": _load_sign(prefix + "/wheel_direction_signs/wz"),
        "offsets": [float(v) for v in offsets],
        "matrix": [float(v) for v in matrix],
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="sentry")
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--state-topic", default="/gazebo/model_states")
    parser.add_argument("--controller-name", default="sentry_chassis_controller")
    parser.add_argument("--ready-timeout-sec", type=float, default=30.0)
    parser.add_argument("--hz", type=float, default=30.0)
    parser.add_argument("--random-seed", type=int, default=20260225)
    parser.add_argument("--iterations", type=int, default=80)
    parser.add_argument("--top-k", type=int, default=8)
    parser.add_argument(
        "--reset-model",
        action="store_true",
        help="Reset Gazebo model pose after each controller reload.",
    )
    parser.add_argument(
        "--quick-keys",
        default="i,u,o,j,l,,,m,.,J,L,<,>,U,O,M",
        help="Comma-separated key set for quick search objective.",
    )
    parser.add_argument(
        "--sign-mutation-prob",
        type=float,
        default=0.0,
        help="Probability of sign-bit mutation in each mutation step.",
    )

    parser.add_argument("--quick-settle-sec", type=float, default=0.12)
    parser.add_argument("--quick-segment-sec", type=float, default=0.28)
    parser.add_argument("--quick-release-sec", type=float, default=0.35)
    parser.add_argument("--full-settle-sec", type=float, default=0.8)
    parser.add_argument("--full-segment-sec", type=float, default=1.2)
    parser.add_argument("--full-release-sec", type=float, default=1.5)

    parser.add_argument("--linear-pos-thr", type=float, default=0.002)
    parser.add_argument("--linear-zero-thr", type=float, default=0.03)
    parser.add_argument("--yaw-pos-thr", type=float, default=0.01)
    parser.add_argument("--yaw-zero-thr", type=float, default=0.18)
    parser.add_argument("--release-pos-thr", type=float, default=0.012)
    parser.add_argument("--release-yaw-thr", type=float, default=0.02)

    parser.add_argument("--write-config", action="store_true")
    parser.add_argument("--config-path", default="")
    args = parser.parse_args()
    args.sign_mutation_prob = _clamp(args.sign_mutation_prob, 0.0, 0.95)

    rng = random.Random(args.random_seed)
    quick_cases = _pick_cases(args.quick_keys)
    if not quick_cases:
        raise RuntimeError("quick key set is empty")

    rospy.init_node("auto_search_teleop_joint", anonymous=True)
    probe = MotionProbe(args.model, args.cmd_topic, args.state_topic)
    probe.wait_ready(args.ready_timeout_sec)
    reloader = ControllerReloader(args.controller_name)

    baseline = _load_candidate_from_params()
    reloader.set_candidate(baseline)
    if not reloader.reload():
        raise RuntimeError("Failed to reload baseline candidate.")
    if args.reset_model:
        reloader.reset_model(args.model)

    baseline_quick = _evaluate_candidate(
        probe,
        quick_cases,
        args.quick_settle_sec,
        args.quick_segment_sec,
        args.quick_release_sec,
        args.hz,
        args.linear_pos_thr,
        args.linear_zero_thr,
        args.yaw_pos_thr,
        args.yaw_zero_thr,
        args.release_pos_thr,
        args.release_yaw_thr,
    )
    baseline_full = _evaluate_candidate(
        probe,
        FULL_CASES,
        args.full_settle_sec,
        args.full_segment_sec,
        args.full_release_sec,
        args.hz,
        args.linear_pos_thr,
        args.linear_zero_thr,
        args.yaw_pos_thr,
        args.yaw_zero_thr,
        args.release_pos_thr,
        args.release_yaw_thr,
    )
    print(
        f"[baseline] quick={baseline_quick['pass_count']}/{len(quick_cases)} "
        f"full={baseline_full['pass_count']}/19 failed={','.join(baseline_full['failed_keys'])}",
        flush=True,
    )

    best_candidate = baseline
    best_quick = baseline_quick
    pool: List[Tuple[Dict, Dict]] = [(baseline, baseline_quick)]
    seen = {_candidate_key(baseline)}
    trials = []

    for i in range(args.iterations):
        if rng.random() < 0.75:
            parent = best_candidate
        else:
            parent = pool[rng.randrange(len(pool))][0]

        candidate = _mutate_candidate(parent, rng, args.sign_mutation_prob)
        key = _candidate_key(candidate)
        if key in seen:
            continue
        seen.add(key)

        reloader.set_candidate(candidate)
        if not reloader.reload():
            continue
        if args.reset_model:
            reloader.reset_model(args.model)

        quick_eval = _evaluate_candidate(
            probe,
            quick_cases,
            args.quick_settle_sec,
            args.quick_segment_sec,
            args.quick_release_sec,
            args.hz,
            args.linear_pos_thr,
            args.linear_zero_thr,
            args.yaw_pos_thr,
            args.yaw_zero_thr,
            args.release_pos_thr,
            args.release_yaw_thr,
        )
        trials.append(
            {
                "iter": i + 1,
                "quick": quick_eval,
                "candidate": candidate,
            }
        )
        print(
            f"[iter {i + 1:03d}] quick={quick_eval['pass_count']}/{len(quick_cases)} "
            f"failed={','.join(quick_eval['failed_keys'])}",
            flush=True,
        )

        if _better(quick_eval, best_quick):
            best_candidate = candidate
            best_quick = quick_eval
            print(
                f"[iter {i + 1:03d}] new_best quick={best_quick['pass_count']}/{len(quick_cases)}",
                flush=True,
            )

        pool.append((candidate, quick_eval))
        pool.sort(key=lambda item: (item[1]["pass_count"], item[1]["score"]), reverse=True)
        if len(pool) > args.top_k:
            pool = pool[: args.top_k]

    full_rank = []
    best_full = None
    best_full_candidate = None
    evaluated_full = set()
    for candidate, quick_eval in pool + [(best_candidate, best_quick), (baseline, baseline_quick)]:
        key = _candidate_key(candidate)
        if key in evaluated_full:
            continue
        evaluated_full.add(key)
        reloader.set_candidate(candidate)
        if not reloader.reload():
            continue
        if args.reset_model:
            reloader.reset_model(args.model)
        full_eval = _evaluate_candidate(
            probe,
            FULL_CASES,
            args.full_settle_sec,
            args.full_segment_sec,
            args.full_release_sec,
            args.hz,
            args.linear_pos_thr,
            args.linear_zero_thr,
            args.yaw_pos_thr,
            args.yaw_zero_thr,
            args.release_pos_thr,
            args.release_yaw_thr,
        )
        full_rank.append({"candidate": candidate, "quick": quick_eval, "full": full_eval})
        if best_full is None or _better(full_eval, best_full):
            best_full = full_eval
            best_full_candidate = candidate
        print(
            f"[full] pass={full_eval['pass_count']}/19 "
            f"failed={','.join(full_eval['failed_keys'])}",
            flush=True,
        )

    if best_full_candidate is None:
        raise RuntimeError("No full candidate evaluated.")

    reloader.set_candidate(best_full_candidate)
    reloader.reload()
    if args.reset_model:
        reloader.reset_model(args.model)

    config_path = args.config_path
    if args.write_config:
        if not config_path:
            config_path = os.path.abspath(
                os.path.join(
                    os.path.dirname(__file__),
                    "..",
                    "config",
                    "chassis_controller.yaml",
                )
            )
        _write_candidate_to_config(config_path, best_full_candidate)

    output = {
        "meta": {
            "iterations": args.iterations,
            "random_seed": args.random_seed,
            "top_k": args.top_k,
        },
        "baseline": {
            "candidate": baseline,
            "quick": baseline_quick,
            "full": baseline_full,
        },
        "best": {
            "candidate": best_full_candidate,
            "quick": best_quick,
            "full": best_full,
        },
        "full_rank": full_rank,
        "trials": trials,
        "write_config": bool(args.write_config),
        "config_path": config_path,
    }
    print(json.dumps(output, indent=2))

    return 0 if best_full["pass_count"] == len(FULL_CASES) else 2


if __name__ == "__main__":
    raise SystemExit(main())
