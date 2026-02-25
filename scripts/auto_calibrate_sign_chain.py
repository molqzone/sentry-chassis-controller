#!/usr/bin/env python3
"""Local-search calibration for rolling + direction sign chain."""

import argparse
import copy
import json
import os
import random
import re
import time
from dataclasses import dataclass
from typing import Dict, List, Sequence

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

DEFAULT_QUICK_KEYS = ["i", "u", "o", "j", "l", ",", ".", "J", "L", "<", ">"]


def _yaw_from_quaternion(q) -> float:
    import math

    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _normalize_angle(angle: float) -> float:
    import math

    return math.atan2(math.sin(angle), math.cos(angle))


def _pose_delta_body(x0: float, y0: float, yaw0: float, x1: float, y1: float):
    import math

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


def _parse_sign_vector(raw: str) -> List[int]:
    values = [int(token.strip()) for token in raw.split(",") if token.strip()]
    if len(values) != 4:
        raise RuntimeError(f"Sign vector must contain 4 integers: {raw}")
    for value in values:
        if value not in (-1, 1):
            raise RuntimeError(f"Sign value must be -1 or 1: {raw}")
    return values


def _candidate_to_bits(candidate: Dict[str, List[int]]) -> List[int]:
    return (
        list(candidate["rolling"])
        + list(candidate["vx"])
        + list(candidate["vy"])
        + list(candidate["wz"])
    )


def _bits_to_candidate(bits: Sequence[int]) -> Dict[str, List[int]]:
    if len(bits) != 16:
        raise RuntimeError("sign-chain bit vector must contain 16 values")
    return {
        "rolling": list(bits[0:4]),
        "vx": list(bits[4:8]),
        "vy": list(bits[8:12]),
        "wz": list(bits[12:16]),
    }


def _flip_bit(candidate: Dict[str, List[int]], bit_index: int) -> Dict[str, List[int]]:
    bits = _candidate_to_bits(candidate)
    bits[bit_index] *= -1
    return _bits_to_candidate(bits)


def _random_candidate(rng: random.Random) -> Dict[str, List[int]]:
    bits = [rng.choice([-1, 1]) for _ in range(16)]
    return _bits_to_candidate(bits)


def _cases_by_keys(keys: Sequence[str]) -> List[KeyCase]:
    key_set = set(keys)
    return [case for case in FULL_CASES if case.key in key_set]


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

    def set_sign_params(self, candidate: Dict[str, List[int]]):
        rospy.set_param(
            "/sentry_chassis_controller/wheel_rolling_signs", list(candidate["rolling"])
        )
        rospy.set_param(
            "/sentry_chassis_controller/wheel_direction_signs/vx", list(candidate["vx"])
        )
        rospy.set_param(
            "/sentry_chassis_controller/wheel_direction_signs/vy", list(candidate["vy"])
        )
        rospy.set_param(
            "/sentry_chassis_controller/wheel_direction_signs/wz", list(candidate["wz"])
        )

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
        score += sum(1 for ok in expectation_ok.values() if ok)
        score += 1 if release_ok else 0
        result["expectation_ok"] = expectation_ok
        result["release_ok"] = release_ok
        result["pass"] = passed
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


def _write_signs_to_config(config_path: str, candidate: Dict[str, List[int]]):
    with open(config_path, "r", encoding="utf-8") as handle:
        content = handle.read()

    replacements = [
        (
            r"(^\s*wheel_rolling_signs:\s*)\[[^\]]+\]",
            "\\1[" + ", ".join(str(v) for v in candidate["rolling"]) + "]",
        ),
        (
            r"(^\s*vx:\s*)\[[^\]]+\]",
            "\\1[" + ", ".join(str(v) for v in candidate["vx"]) + "]",
        ),
        (
            r"(^\s*vy:\s*)\[[^\]]+\]",
            "\\1[" + ", ".join(str(v) for v in candidate["vy"]) + "]",
        ),
        (
            r"(^\s*wz:\s*)\[[^\]]+\]",
            "\\1[" + ", ".join(str(v) for v in candidate["wz"]) + "]",
        ),
    ]

    for pattern, replacement in replacements:
        content, count = re.subn(pattern, replacement, content, count=1, flags=re.MULTILINE)
        if count != 1:
            raise RuntimeError(f"Failed to update config by pattern: {pattern}")

    with open(config_path, "w", encoding="utf-8") as handle:
        handle.write(content)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="sentry")
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--state-topic", default="/gazebo/model_states")
    parser.add_argument("--controller-name", default="sentry_chassis_controller")
    parser.add_argument("--ready-timeout-sec", type=float, default=30.0)
    parser.add_argument("--hz", type=float, default=30.0)

    parser.add_argument("--quick-settle-sec", type=float, default=0.15)
    parser.add_argument("--quick-segment-sec", type=float, default=0.25)
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

    parser.add_argument("--quick-keys", default=",".join(DEFAULT_QUICK_KEYS))
    parser.add_argument("--random-restarts", type=int, default=3)
    parser.add_argument("--rng-seed", type=int, default=20260224)
    parser.add_argument("--max-local-steps", type=int, default=8)

    parser.add_argument("--init-rolling", default="1,1,1,1")
    parser.add_argument("--init-vx", default="1,1,1,1")
    parser.add_argument("--init-vy", default="1,1,1,1")
    parser.add_argument("--init-wz", default="1,1,1,1")

    parser.add_argument("--write-config", action="store_true")
    parser.add_argument("--config-path", default="")

    args = parser.parse_args()

    quick_keys = [token.strip() for token in args.quick_keys.split(",") if token.strip()]
    quick_cases = _cases_by_keys(quick_keys)
    if not quick_cases:
        raise RuntimeError("Quick case list is empty")

    init_candidate = {
        "rolling": _parse_sign_vector(args.init_rolling),
        "vx": _parse_sign_vector(args.init_vx),
        "vy": _parse_sign_vector(args.init_vy),
        "wz": _parse_sign_vector(args.init_wz),
    }

    rospy.init_node("auto_calibrate_sign_chain", anonymous=True)
    probe = MotionProbe(args.model, args.cmd_topic, args.state_topic)
    probe.wait_ready(args.ready_timeout_sec)
    reloader = ControllerReloader(args.controller_name)

    rng = random.Random(args.rng_seed)

    def eval_with_candidate(candidate: Dict[str, List[int]], full: bool) -> Dict:
        reloader.set_sign_params(candidate)
        if not reloader.reload():
            return {
                "score": -1,
                "pass_count": -1,
                "fail_count": 999,
                "failed_keys": [],
                "results": [],
            }
        reloader.reset_model(args.model)
        return _evaluate_candidate(
            probe,
            FULL_CASES if full else quick_cases,
            args.full_settle_sec if full else args.quick_settle_sec,
            args.full_segment_sec if full else args.quick_segment_sec,
            args.full_release_sec if full else args.quick_release_sec,
            args.hz,
            args.linear_pos_thr,
            args.linear_zero_thr,
            args.yaw_pos_thr,
            args.yaw_zero_thr,
            args.release_pos_thr,
            args.release_yaw_thr,
        )

    restart_candidates = [copy.deepcopy(init_candidate)]
    for _ in range(args.random_restarts):
        restart_candidates.append(_random_candidate(rng))

    best_overall = None
    trials = []

    for restart_index, start_candidate in enumerate(restart_candidates):
        current_candidate = copy.deepcopy(start_candidate)
        current_quick = eval_with_candidate(current_candidate, full=False)
        seen = {tuple(_candidate_to_bits(current_candidate))}

        improved = True
        steps = 0
        while improved and steps < args.max_local_steps:
            improved = False
            best_neighbor = None
            best_neighbor_eval = None
            for bit_index in range(16):
                neighbor = _flip_bit(current_candidate, bit_index)
                neighbor_bits = tuple(_candidate_to_bits(neighbor))
                if neighbor_bits in seen:
                    continue
                neighbor_eval = eval_with_candidate(neighbor, full=False)
                if best_neighbor_eval is None or _better(neighbor_eval, best_neighbor_eval):
                    best_neighbor = neighbor
                    best_neighbor_eval = neighbor_eval
            if best_neighbor_eval is not None and _better(best_neighbor_eval, current_quick):
                current_candidate = best_neighbor
                current_quick = best_neighbor_eval
                improved = True
                steps += 1
                seen.add(tuple(_candidate_to_bits(current_candidate)))

        current_full = eval_with_candidate(current_candidate, full=True)
        trial = {
            "restart": restart_index,
            "start_candidate": start_candidate,
            "local_steps": steps,
            "candidate": current_candidate,
            "quick": current_quick,
            "full": current_full,
        }
        trials.append(trial)

        if best_overall is None or _better(current_full, best_overall["full"]):
            best_overall = trial

        if current_full["pass_count"] == len(FULL_CASES):
            break

    if best_overall is None:
        raise RuntimeError("No sign-chain candidate evaluated")

    best_candidate = best_overall["candidate"]
    reloader.set_sign_params(best_candidate)
    reloader.reload()
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
        _write_signs_to_config(config_path, best_candidate)

    output = {
        "meta": {
            "quick_keys": [case.key for case in quick_cases],
            "restarts": len(restart_candidates),
        },
        "best": best_overall,
        "trials": trials,
        "write_config": bool(args.write_config),
        "config_path": config_path,
    }
    print(json.dumps(output, indent=2))

    return 0 if best_overall["full"]["pass_count"] == len(FULL_CASES) else 2


if __name__ == "__main__":
    raise SystemExit(main())
