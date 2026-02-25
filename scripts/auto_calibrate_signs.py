#!/usr/bin/env python3
"""Auto-calibrate wheel sign-chain parameters and verify teleop key behavior."""

import argparse
import itertools
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

QUICK_CASE_KEYS = ["i", ",", "J", "L", "j", "l", "U", "O"]


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


class MotionProbe:
    """Minimal evaluator for teleop key command outcomes."""

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
    """Reload controller with updated sign params."""

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


def _generate_candidates(
    base_vx: Sequence[int],
    base_vy: Sequence[int],
    base_wz: Sequence[int],
):
    candidates = []
    # Keep first rolling sign positive to remove global inversion duplicates.
    for rolling_tail in itertools.product([-1, 1], repeat=3):
        rolling = [1, rolling_tail[0], rolling_tail[1], rolling_tail[2]]
        for flip_vx, flip_vy, flip_wz in itertools.product([-1, 1], repeat=3):
            vx = [flip_vx * value for value in base_vx]
            vy = [flip_vy * value for value in base_vy]
            wz = [flip_wz * value for value in base_wz]
            candidates.append(
                {
                    "rolling": rolling,
                    "vx": vx,
                    "vy": vy,
                    "wz": wz,
                }
            )
    return candidates


def _write_signs_to_config(
    config_path: str,
    rolling: Sequence[int],
    vx: Sequence[int],
    vy: Sequence[int],
    wz: Sequence[int],
):
    with open(config_path, "r", encoding="utf-8") as handle:
        content = handle.read()

    replacements = [
        (
            r"(^\s*wheel_rolling_signs:\s*)\[[^\]]+\]",
            "\\1[" + ", ".join(str(value) for value in rolling) + "]",
        ),
        (
            r"(^\s*vx:\s*)\[[^\]]+\]",
            "\\1[" + ", ".join(str(value) for value in vx) + "]",
        ),
        (
            r"(^\s*vy:\s*)\[[^\]]+\]",
            "\\1[" + ", ".join(str(value) for value in vy) + "]",
        ),
        (
            r"(^\s*wz:\s*)\[[^\]]+\]",
            "\\1[" + ", ".join(str(value) for value in wz) + "]",
        ),
    ]

    for pattern, replacement in replacements:
        content, count = re.subn(
            pattern, replacement, content, count=1, flags=re.MULTILINE
        )
        if count != 1:
            raise RuntimeError(f"Failed to update config field by pattern: {pattern}")

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
    parser.add_argument("--quick-settle-sec", type=float, default=0.25)
    parser.add_argument("--quick-segment-sec", type=float, default=0.45)
    parser.add_argument("--quick-release-sec", type=float, default=0.4)
    parser.add_argument("--full-settle-sec", type=float, default=0.6)
    parser.add_argument("--full-segment-sec", type=float, default=1.0)
    parser.add_argument("--full-release-sec", type=float, default=1.2)
    parser.add_argument("--top-k", type=int, default=8)
    parser.add_argument("--linear-pos-thr", type=float, default=0.002)
    parser.add_argument("--linear-zero-thr", type=float, default=0.03)
    parser.add_argument("--yaw-pos-thr", type=float, default=0.008)
    parser.add_argument("--yaw-zero-thr", type=float, default=0.18)
    parser.add_argument("--release-pos-thr", type=float, default=0.012)
    parser.add_argument("--release-yaw-thr", type=float, default=0.02)
    parser.add_argument(
        "--base-vx",
        default="1,-1,-1,-1",
        help="Comma-separated base wheel_direction_signs/vx",
    )
    parser.add_argument(
        "--base-vy",
        default="-1,-1,-1,1",
        help="Comma-separated base wheel_direction_signs/vy",
    )
    parser.add_argument(
        "--base-wz",
        default="1,1,1,-1",
        help="Comma-separated base wheel_direction_signs/wz",
    )
    parser.add_argument(
        "--config-path",
        default="",
        help="Optional config path to persist best sign set",
    )
    parser.add_argument("--write-config", action="store_true")
    args = parser.parse_args()

    def _parse_vector(raw: str):
        values = [int(token.strip()) for token in raw.split(",") if token.strip()]
        if len(values) != 4:
            raise RuntimeError(f"Sign vector must contain 4 integers: {raw}")
        for value in values:
            if value not in (-1, 1):
                raise RuntimeError(f"Sign value must be -1 or 1: {raw}")
        return values

    base_vx = _parse_vector(args.base_vx)
    base_vy = _parse_vector(args.base_vy)
    base_wz = _parse_vector(args.base_wz)

    rospy.init_node("auto_calibrate_signs", anonymous=True)
    probe = MotionProbe(args.model, args.cmd_topic, args.state_topic)
    probe.wait_ready(args.ready_timeout_sec)
    reloader = ControllerReloader(args.controller_name)

    all_candidates = _generate_candidates(base_vx, base_vy, base_wz)

    quick_cases = [case for case in FULL_CASES if case.key in QUICK_CASE_KEYS]
    quick_rank = []
    for index, candidate in enumerate(all_candidates):
        reloader.set_sign_params(
            candidate["rolling"],
            candidate["vx"],
            candidate["vy"],
            candidate["wz"],
        )
        if not reloader.reload():
            continue
        reloader.reset_model(args.model)
        evaluation = _evaluate_candidate(
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
        quick_rank.append(
            {
                "candidate_index": index,
                "candidate": candidate,
                "quick": evaluation,
            }
        )

    quick_rank.sort(
        key=lambda item: (
            item["quick"]["pass_count"],
            item["quick"]["score"],
        ),
        reverse=True,
    )
    shortlist = quick_rank[: max(args.top_k, 1)]

    best = None
    for item in shortlist:
        candidate = item["candidate"]
        reloader.set_sign_params(
            candidate["rolling"],
            candidate["vx"],
            candidate["vy"],
            candidate["wz"],
        )
        if not reloader.reload():
            continue
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
        scored = {
            "candidate": candidate,
            "quick": item["quick"],
            "full": full_eval,
        }
        if best is None:
            best = scored
        else:
            best_key = (best["full"]["pass_count"], best["full"]["score"])
            new_key = (full_eval["pass_count"], full_eval["score"])
            if new_key > best_key:
                best = scored
        if full_eval["pass_count"] == len(FULL_CASES):
            break

    if best is None:
        raise RuntimeError("Calibration failed: no valid candidate evaluated.")

    best_candidate = best["candidate"]
    reloader.set_sign_params(
        best_candidate["rolling"],
        best_candidate["vx"],
        best_candidate["vy"],
        best_candidate["wz"],
    )
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
        _write_signs_to_config(
            config_path,
            best_candidate["rolling"],
            best_candidate["vx"],
            best_candidate["vy"],
            best_candidate["wz"],
        )

    output = {
        "meta": {
            "candidate_total": len(all_candidates),
            "quick_shortlist": len(shortlist),
            "quick_cases": [case.key for case in quick_cases],
            "full_cases": [case.key for case in FULL_CASES],
        },
        "best": best,
        "write_config": bool(args.write_config),
        "config_path": config_path,
    }
    print(json.dumps(output, indent=2))

    return 0 if best["full"]["pass_count"] == len(FULL_CASES) else 2


if __name__ == "__main__":
    raise SystemExit(main())
