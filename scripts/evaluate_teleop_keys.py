#!/usr/bin/env python3
"""Evaluate teleop_twist_keyboard key mappings against gazebo model motion."""

import argparse
import json
import math
import time
from dataclasses import dataclass
from typing import Dict, List

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist


@dataclass
class KeyCase:
    key: str
    vx: float
    vy: float
    wz: float
    vz: float = 0.0


TELEOP_KEY_CASES: List[KeyCase] = [
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
    KeyCase("t", 0.0, 0.0, 0.0, 0.5),
    KeyCase("b", 0.0, 0.0, 0.0, -0.5),
    KeyCase("k", 0.0, 0.0, 0.0),
]


def _yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _world_to_body(vx_world: float, vy_world: float, yaw: float):
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    vx_body = cos_yaw * vx_world + sin_yaw * vy_world
    vy_body = -sin_yaw * vx_world + cos_yaw * vy_world
    return vx_body, vy_body


def _pose_delta_body(x0: float, y0: float, yaw0: float, x1: float, y1: float):
    dx_world = x1 - x0
    dy_world = y1 - y0
    cos_yaw = math.cos(yaw0)
    sin_yaw = math.sin(yaw0)
    dx_body = cos_yaw * dx_world + sin_yaw * dy_world
    dy_body = -sin_yaw * dx_world + cos_yaw * dy_world
    return dx_body, dy_body


class KeyEvaluator:
    """Evaluate key-mapped commands with segmented cmd_vel replay."""

    def __init__(self, model_name: str, cmd_topic: str, state_topic: str):
        self._model_name = model_name
        self._state_message = None
        self._publisher = rospy.Publisher(cmd_topic, Twist, queue_size=10)
        self._subscriber = rospy.Subscriber(
            state_topic, ModelStates, self._state_callback, queue_size=1
        )
        self._model_index = -1

    def _state_callback(self, message: ModelStates):
        self._state_message = message

    def wait_ready(self, timeout_sec: float):
        start = time.time()
        while not rospy.is_shutdown():
            if (
                self._state_message is not None
                and self._model_name in self._state_message.name
            ):
                self._model_index = self._state_message.name.index(self._model_name)
                return
            if time.time() - start > timeout_sec:
                raise RuntimeError("Timeout waiting for gazebo model state.")
            rospy.sleep(0.05)
        raise RuntimeError("ROS shutdown before evaluator became ready.")

    def _state(self):
        if self._state_message is None or self._model_index < 0:
            raise RuntimeError("Evaluator not ready.")
        pose = self._state_message.pose[self._model_index]
        twist = self._state_message.twist[self._model_index]
        yaw = _yaw_from_quaternion(pose.orientation)
        return {
            "x": pose.position.x,
            "y": pose.position.y,
            "yaw": yaw,
            "vx_world": twist.linear.x,
            "vy_world": twist.linear.y,
            "wz": twist.angular.z,
        }

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
        segment_sec: float,
        release_sec: float,
        hz: float,
    ):
        start_state = self._state()
        rate = rospy.Rate(hz)
        samples = []

        command = Twist()
        command.linear.x = case.vx
        command.linear.y = case.vy
        command.linear.z = case.vz
        command.angular.z = case.wz

        begin = time.time()
        while time.time() - begin < segment_sec and not rospy.is_shutdown():
            self._publisher.publish(command)
            current = self._state()
            vx_body, vy_body = _world_to_body(
                current["vx_world"], current["vy_world"], current["yaw"]
            )
            samples.append((vx_body, vy_body, current["wz"]))
            rate.sleep()

        active_end = self._state()
        dx, dy = _pose_delta_body(
            start_state["x"],
            start_state["y"],
            start_state["yaw"],
            active_end["x"],
            active_end["y"],
        )
        dyaw = _normalize_angle(active_end["yaw"] - start_state["yaw"])
        avg_vx = sum(s[0] for s in samples) / len(samples) if samples else 0.0
        avg_vy = sum(s[1] for s in samples) / len(samples) if samples else 0.0
        avg_wz = sum(s[2] for s in samples) / len(samples) if samples else 0.0

        release_dx = 0.0
        release_dy = 0.0
        release_dyaw = 0.0
        release_max_wz = 0.0
        release_max_speed = 0.0
        if release_sec > 0.0:
            release_start = self._state()
            self.publish_zero(release_sec, hz)
            release_end = self._state()
            release_dx, release_dy = _pose_delta_body(
                release_start["x"],
                release_start["y"],
                release_start["yaw"],
                release_end["x"],
                release_end["y"],
            )
            release_dyaw = _normalize_angle(release_end["yaw"] - release_start["yaw"])
            release_max_wz = max(
                abs(release_start["wz"]),
                abs(release_end["wz"]),
            )
            release_max_speed = max(
                math.hypot(release_start["vx_world"], release_start["vy_world"]),
                math.hypot(release_end["vx_world"], release_end["vy_world"]),
            )

        return {
            "key": case.key,
            "command": {
                "vx": case.vx,
                "vy": case.vy,
                "wz": case.wz,
                "vz": case.vz,
            },
            "delta_body": {
                "dx": dx,
                "dy": dy,
                "dyaw": dyaw,
            },
            "avg_body_twist": {
                "vx": avg_vx,
                "vy": avg_vy,
                "wz": avg_wz,
            },
            "release": {
                "dx": release_dx,
                "dy": release_dy,
                "dyaw": release_dyaw,
                "max_abs_wz": release_max_wz,
                "max_linear_speed": release_max_speed,
            },
        }


def _axis_ok(value: float, target: float, pos_thr: float, zero_thr: float) -> bool:
    if target > 1e-9:
        return value > pos_thr
    if target < -1e-9:
        return value < -pos_thr
    return abs(value) < zero_thr


def _evaluate_pass(
    result: Dict,
    case: KeyCase,
    linear_pos_thr: float,
    linear_zero_thr: float,
    yaw_pos_thr: float,
    yaw_zero_thr: float,
    release_pos_thr: float,
    release_yaw_thr: float,
):
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
    return expectation_ok, release_ok, all(expectation_ok.values()) and release_ok


def _pick_cases(case_filter: str) -> List[KeyCase]:
    if not case_filter:
        return TELEOP_KEY_CASES
    selected = set([token.strip() for token in case_filter.split(",") if token.strip()])
    return [case for case in TELEOP_KEY_CASES if case.key in selected]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="sentry")
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--state-topic", default="/gazebo/model_states")
    parser.add_argument("--segment-sec", type=float, default=1.2)
    parser.add_argument("--settle-sec", type=float, default=0.8)
    parser.add_argument("--release-sec", type=float, default=1.5)
    parser.add_argument("--hz", type=float, default=30.0)
    parser.add_argument("--ready-timeout-sec", type=float, default=40.0)
    parser.add_argument("--cases", default="")
    parser.add_argument("--linear-pos-thr", type=float, default=0.002)
    parser.add_argument("--linear-zero-thr", type=float, default=0.03)
    parser.add_argument("--yaw-pos-thr", type=float, default=0.01)
    parser.add_argument("--yaw-zero-thr", type=float, default=0.18)
    parser.add_argument("--release-pos-thr", type=float, default=0.012)
    parser.add_argument("--release-yaw-thr", type=float, default=0.02)
    parser.add_argument("--require-all-pass", action="store_true")
    args = parser.parse_args()

    rospy.init_node("teleop_key_evaluator", anonymous=True)
    evaluator = KeyEvaluator(args.model, args.cmd_topic, args.state_topic)
    evaluator.wait_ready(args.ready_timeout_sec)

    cases = _pick_cases(args.cases)
    if not cases:
        print(
            json.dumps(
                {
                    "summary": {
                        "total": 0,
                        "pass": 0,
                        "fail": 0,
                        "failed_keys": [],
                    },
                    "results": [],
                },
                indent=2,
            )
        )
        return 0

    evaluator.publish_zero(args.settle_sec, args.hz)

    results = []
    for case in cases:
        if args.settle_sec > 0.0:
            evaluator.publish_zero(args.settle_sec, args.hz)
        result = evaluator.run_case(case, args.segment_sec, args.release_sec, args.hz)
        expectation_ok, release_ok, passed = _evaluate_pass(
            result,
            case,
            args.linear_pos_thr,
            args.linear_zero_thr,
            args.yaw_pos_thr,
            args.yaw_zero_thr,
            args.release_pos_thr,
            args.release_yaw_thr,
        )
        result["expectation_ok"] = expectation_ok
        result["release_ok"] = release_ok
        result["pass"] = passed
        results.append(result)

    evaluator.publish_zero(0.5, args.hz)

    summary = {
        "total": len(results),
        "pass": sum(1 for r in results if r["pass"]),
        "fail": sum(1 for r in results if not r["pass"]),
        "failed_keys": [r["key"] for r in results if not r["pass"]],
    }

    output = {
        "meta": {
            "model": args.model,
            "cmd_topic": args.cmd_topic,
            "state_topic": args.state_topic,
            "segment_sec": args.segment_sec,
            "settle_sec": args.settle_sec,
            "release_sec": args.release_sec,
            "hz": args.hz,
            "cases": [case.key for case in cases],
        },
        "summary": summary,
        "results": results,
    }
    print(json.dumps(output, indent=2))

    if args.require_all_pass and summary["fail"] > 0:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
