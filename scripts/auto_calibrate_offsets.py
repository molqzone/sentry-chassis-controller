#!/usr/bin/env python3
"""Auto-calibrate per-wheel steer_zero_offsets for teleop key behavior."""

import argparse
import json
import math
import os
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

DEFAULT_QUICK_KEYS = ["i", "j", "l", ",", ".", "J", "L", "<", ">"]


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


def _parse_sign_vector(raw: str) -> List[int]:
    values = [int(token.strip()) for token in raw.split(",") if token.strip()]
    if len(values) != 4:
        raise RuntimeError(f"Sign vector must contain 4 integers: {raw}")
    for value in values:
        if value not in (-1, 1):
            raise RuntimeError(f"Sign value must be -1 or 1: {raw}")
    return values


def _parse_steps(raw: str) -> List[float]:
    values = [float(token.strip()) for token in raw.split(",") if token.strip()]
    if not values:
        raise RuntimeError("step sizes must not be empty")
    return [abs(v) for v in values if abs(v) > 1e-9]


def _parse_seed_offsets(raw: str) -> List[List[float]]:
    seeds = []
    for chunk in raw.split(";"):
        chunk = chunk.strip()
        if not chunk:
            continue
        values = [float(token.strip()) for token in chunk.split(",") if token.strip()]
        if len(values) != 4:
            raise RuntimeError(f"Offset seed must contain 4 numbers: {chunk}")
        seeds.append(values)
    if not seeds:
        raise RuntimeError("offset seeds must not be empty")
    return seeds


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

    def set_sign_params(
        self,
        rolling: Sequence[int],
        vx: Sequence[int],
        vy: Sequence[int],
        wz: Sequence[int],
    ):
        rospy.set_param("/sentry_chassis_controller/wheel_rolling_signs", list(rolling))
        rospy.set_param("/sentry_chassis_controller/wheel_direction_signs/vx", list(vx))
        rospy.set_param("/sentry_chassis_controller/wheel_direction_signs/vy", list(vy))
        rospy.set_param("/sentry_chassis_controller/wheel_direction_signs/wz", list(wz))

    def set_offsets(self, offsets: Sequence[float]):
        rospy.set_param("/sentry_chassis_controller/steer_zero_offsets", list(offsets))

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


def _update_offsets_in_config(config_path: str, offsets: Sequence[float]):
    with open(config_path, "r", encoding="utf-8") as handle:
        content = handle.read()

    replacement = "\n".join([
        "  steer_zero_offsets:",
        *[f"  - {value:.9f}" for value in offsets],
    ])
    content, count = re.subn(
        r"^\s*steer_zero_offsets:\n(?:\s*-\s*[-+0-9eE\.]+\n){4}",
        replacement + "\n",
        content,
        flags=re.MULTILINE,
    )
    if count != 1:
        raise RuntimeError("Failed to update steer_zero_offsets in config")

    with open(config_path, "w", encoding="utf-8") as handle:
        handle.write(content)


def _is_better(lhs: Dict, rhs: Dict) -> bool:
    return (lhs["pass_count"], lhs["score"]) > (rhs["pass_count"], rhs["score"])


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="sentry")
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--state-topic", default="/gazebo/model_states")
    parser.add_argument("--controller-name", default="sentry_chassis_controller")
    parser.add_argument("--ready-timeout-sec", type=float, default=30.0)
    parser.add_argument("--hz", type=float, default=30.0)

    parser.add_argument("--quick-settle-sec", type=float, default=0.2)
    parser.add_argument("--quick-segment-sec", type=float, default=0.4)
    parser.add_argument("--quick-release-sec", type=float, default=0.5)
    parser.add_argument("--full-settle-sec", type=float, default=0.8)
    parser.add_argument("--full-segment-sec", type=float, default=1.2)
    parser.add_argument("--full-release-sec", type=float, default=1.5)

    parser.add_argument("--linear-pos-thr", type=float, default=0.002)
    parser.add_argument("--linear-zero-thr", type=float, default=0.03)
    parser.add_argument("--yaw-pos-thr", type=float, default=0.01)
    parser.add_argument("--yaw-zero-thr", type=float, default=0.18)
    parser.add_argument("--release-pos-thr", type=float, default=0.012)
    parser.add_argument("--release-yaw-thr", type=float, default=0.02)

    parser.add_argument("--base-rolling", default="1,1,1,1")
    parser.add_argument("--base-vx", default="1,1,1,1")
    parser.add_argument("--base-vy", default="1,1,1,1")
    parser.add_argument("--base-wz", default="1,1,1,1")

    parser.add_argument("--quick-keys", default=",".join(DEFAULT_QUICK_KEYS))
    parser.add_argument("--step-sizes", default="0.8,0.4,0.2,0.1,0.05")
    parser.add_argument(
        "--offset-seeds",
        default="0,0,0,0;2.34,-0.285,-0.259,2.33;-2.34,0.285,0.259,-2.33",
    )

    parser.add_argument("--write-config", action="store_true")
    parser.add_argument("--config-path", default="")

    args = parser.parse_args()

    base_rolling = _parse_sign_vector(args.base_rolling)
    base_vx = _parse_sign_vector(args.base_vx)
    base_vy = _parse_sign_vector(args.base_vy)
    base_wz = _parse_sign_vector(args.base_wz)
    step_sizes = _parse_steps(args.step_sizes)
    seed_offsets = _parse_seed_offsets(args.offset_seeds)
    quick_keys = [token.strip() for token in args.quick_keys.split(",") if token.strip()]

    quick_cases = _cases_by_keys(quick_keys)
    if not quick_cases:
        raise RuntimeError("quick cases are empty")

    rospy.init_node("auto_calibrate_offsets", anonymous=True)
    probe = MotionProbe(args.model, args.cmd_topic, args.state_topic)
    probe.wait_ready(args.ready_timeout_sec)
    reloader = ControllerReloader(args.controller_name)
    reloader.set_sign_params(base_rolling, base_vx, base_vy, base_wz)

    best_overall = None
    traces = []

    for seed in seed_offsets:
        current_offsets = [_normalize_angle(v) for v in seed]
        reloader.set_offsets(current_offsets)
        if not reloader.reload():
            continue
        reloader.reset_model(args.model)

        baseline_full_eval = _evaluate_candidate(
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
        baseline_offsets = list(current_offsets)

        current_eval = _evaluate_candidate(
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

        seed_trace = {
            "seed": seed,
            "steps": [],
            "quick_best": current_eval,
            "offsets": list(current_offsets),
        }

        for step in step_sizes:
            improved_any = False
            local_improved = True
            while local_improved:
                local_improved = False
                for idx in range(4):
                    trial_best = None
                    trial_offsets = None
                    for delta in (-step, step):
                        candidate_offsets = list(current_offsets)
                        candidate_offsets[idx] = _normalize_angle(candidate_offsets[idx] + delta)
                        reloader.set_offsets(candidate_offsets)
                        if not reloader.reload():
                            continue
                        reloader.reset_model(args.model)
                        candidate_eval = _evaluate_candidate(
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
                        if trial_best is None or _is_better(candidate_eval, trial_best):
                            trial_best = candidate_eval
                            trial_offsets = candidate_offsets

                    if trial_best is not None and _is_better(trial_best, current_eval):
                        current_eval = trial_best
                        current_offsets = trial_offsets
                        local_improved = True
                        improved_any = True

            seed_trace["steps"].append(
                {
                    "step": step,
                    "improved": improved_any,
                    "offsets": list(current_offsets),
                    "quick_best": current_eval,
                }
            )

        reloader.set_offsets(current_offsets)
        if not reloader.reload():
            continue
        reloader.reset_model(args.model)

        optimized_full_eval = _evaluate_candidate(
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

        if _is_better(optimized_full_eval, baseline_full_eval):
            full_eval = optimized_full_eval
            selected_offsets = list(current_offsets)
        else:
            full_eval = baseline_full_eval
            selected_offsets = baseline_offsets

        record = {
            "seed": seed,
            "offsets": selected_offsets,
            "quick": current_eval,
            "baseline_full": baseline_full_eval,
            "optimized_full": optimized_full_eval,
            "full": full_eval,
            "signs": {
                "rolling": list(base_rolling),
                "vx": list(base_vx),
                "vy": list(base_vy),
                "wz": list(base_wz),
            },
        }
        traces.append(record)

        if best_overall is None or _is_better(full_eval, best_overall["full"]):
            best_overall = record

        if full_eval["pass_count"] == len(FULL_CASES):
            break

    if best_overall is None:
        raise RuntimeError("No valid offset candidate evaluated.")

    reloader.set_sign_params(base_rolling, base_vx, base_vy, base_wz)
    reloader.set_offsets(best_overall["offsets"])
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
        _update_offsets_in_config(config_path, best_overall["offsets"])

    output = {
        "meta": {
            "quick_keys": [case.key for case in quick_cases],
            "step_sizes": step_sizes,
            "seed_count": len(seed_offsets),
        },
        "best": best_overall,
        "traces": traces,
        "write_config": bool(args.write_config),
        "config_path": config_path,
    }
    print(json.dumps(output, indent=2))

    return 0 if best_overall["full"]["pass_count"] == len(FULL_CASES) else 2


if __name__ == "__main__":
    raise SystemExit(main())
