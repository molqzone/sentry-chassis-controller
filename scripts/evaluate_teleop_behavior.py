#!/usr/bin/env python3
"""Evaluate teleop-like cmd_vel behavior in Gazebo and print JSON metrics."""

import argparse
import json
import math
import time
from typing import Dict, List, Tuple

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist


def _yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _world_to_body(vx_world: float, vy_world: float, yaw: float) -> Tuple[float, float]:
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    vx_body = cos_yaw * vx_world + sin_yaw * vy_world
    vy_body = -sin_yaw * vx_world + cos_yaw * vy_world
    return vx_body, vy_body


def _pose_delta_body(x0: float, y0: float, yaw0: float, x1: float, y1: float) -> Tuple[float, float]:
    dx_world = x1 - x0
    dy_world = y1 - y0
    cos_yaw = math.cos(yaw0)
    sin_yaw = math.sin(yaw0)
    dx_body = cos_yaw * dx_world + sin_yaw * dy_world
    dy_body = -sin_yaw * dx_world + cos_yaw * dy_world
    return dx_body, dy_body


class TeleopEvaluator:
    """Runs segmented cmd_vel tests and scores motion directionality."""

    def __init__(self, model_name: str, cmd_topic: str, state_topic: str):
        self.model_name = model_name
        self.latest_model_states = None
        self.publisher = rospy.Publisher(cmd_topic, Twist, queue_size=10)
        self.subscriber = rospy.Subscriber(state_topic, ModelStates, self._state_callback, queue_size=1)

    def _state_callback(self, message: ModelStates) -> None:
        self.latest_model_states = message

    def wait_until_ready(self, timeout_sec: float) -> int:
        start = time.time()
        while not rospy.is_shutdown():
            if self.latest_model_states is not None and self.model_name in self.latest_model_states.name:
                return self.latest_model_states.name.index(self.model_name)
            if time.time() - start > timeout_sec:
                raise RuntimeError("Timeout waiting for Gazebo model state.")
            rospy.sleep(0.05)
        raise RuntimeError("ROS shutdown before evaluator became ready.")

    def get_state(self, model_index: int) -> Dict[str, float]:
        message = self.latest_model_states
        pose = message.pose[model_index]
        twist = message.twist[model_index]
        yaw = _yaw_from_quaternion(pose.orientation)
        return {
            "x": pose.position.x,
            "y": pose.position.y,
            "yaw": yaw,
            "vx_world": twist.linear.x,
            "vy_world": twist.linear.y,
            "wz": twist.angular.z,
        }

    def publish_zero_for(self, seconds: float, hz: float) -> None:
        rate = rospy.Rate(hz)
        end_time = time.time() + seconds
        zero = Twist()
        while time.time() < end_time and not rospy.is_shutdown():
            self.publisher.publish(zero)
            rate.sleep()

    def run_segment(self, model_index: int, name: str, vx: float, vy: float, wz: float, seconds: float, hz: float) -> Dict:
        start_state = self.get_state(model_index)
        rate = rospy.Rate(hz)
        samples: List[Tuple[float, float, float]] = []
        command = Twist()
        command.linear.x = vx
        command.linear.y = vy
        command.angular.z = wz

        begin = time.time()
        while time.time() - begin < seconds and not rospy.is_shutdown():
            self.publisher.publish(command)
            current = self.get_state(model_index)
            vx_body, vy_body = _world_to_body(current["vx_world"], current["vy_world"], current["yaw"])
            samples.append((vx_body, vy_body, current["wz"]))
            rate.sleep()

        end_state = self.get_state(model_index)
        dx_body, dy_body = _pose_delta_body(
            start_state["x"], start_state["y"], start_state["yaw"], end_state["x"], end_state["y"]
        )
        dyaw = _normalize_angle(end_state["yaw"] - start_state["yaw"])

        avg_vx = sum(sample[0] for sample in samples) / len(samples) if samples else 0.0
        avg_vy = sum(sample[1] for sample in samples) / len(samples) if samples else 0.0
        avg_wz = sum(sample[2] for sample in samples) / len(samples) if samples else 0.0

        return {
            "segment": name,
            "command": {"vx": vx, "vy": vy, "wz": wz, "duration_sec": seconds},
            "delta_body": {"dx": dx_body, "dy": dy_body, "dyaw": dyaw},
            "avg_body_twist": {"vx": avg_vx, "vy": avg_vy, "wz": avg_wz},
        }


def _build_checks(results: List[Dict]) -> Dict:
    strict = {}
    directional = {}
    for result in results:
        segment = result["segment"]
        delta = result["delta_body"]
        if segment == "forward":
            strict[segment] = bool(delta["dx"] > 0.10 and abs(delta["dy"]) < 0.06 and abs(delta["dyaw"]) < 0.15)
            directional[segment] = bool(delta["dx"] > 0.02 and abs(delta["dy"]) < 0.10)
        elif segment == "left":
            strict[segment] = bool(delta["dy"] > 0.10 and abs(delta["dx"]) < 0.06 and abs(delta["dyaw"]) < 0.15)
            directional[segment] = bool(delta["dy"] > 0.02 and abs(delta["dx"]) < 0.10)
        elif segment == "rotate":
            strict[segment] = bool(delta["dyaw"] > 0.25 and abs(delta["dx"]) < 0.06 and abs(delta["dy"]) < 0.06)
            directional[segment] = bool(delta["dyaw"] > 0.05 and abs(delta["dx"]) < 0.10 and abs(delta["dy"]) < 0.10)

    return {
        "strict": {"items": strict, "all_pass": all(strict.values()) if strict else False},
        "directional": {"items": directional, "all_pass": all(directional.values()) if directional else False},
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="sentry")
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--state-topic", default="/gazebo/model_states")
    parser.add_argument("--segment-sec", type=float, default=3.0)
    parser.add_argument("--idle-sec", type=float, default=1.0)
    parser.add_argument("--hz", type=float, default=50.0)
    parser.add_argument("--ready-timeout-sec", type=float, default=60.0)
    args = parser.parse_args()

    rospy.init_node("sentry_teleop_behavior_evaluator", anonymous=True)
    evaluator = TeleopEvaluator(args.model, args.cmd_topic, args.state_topic)
    model_index = evaluator.wait_until_ready(timeout_sec=args.ready_timeout_sec)

    evaluator.publish_zero_for(args.idle_sec, args.hz)
    results = []
    for name, vx, vy, wz in (
        ("forward", 0.6, 0.0, 0.0),
        ("left", 0.0, 0.6, 0.0),
        ("rotate", 0.0, 0.0, 0.8),
    ):
        result = evaluator.run_segment(
            model_index=model_index,
            name=name,
            vx=vx,
            vy=vy,
            wz=wz,
            seconds=args.segment_sec,
            hz=args.hz,
        )
        results.append(result)
        evaluator.publish_zero_for(args.idle_sec, args.hz)

    output = {
        "meta": {
            "model": args.model,
            "cmd_topic": args.cmd_topic,
            "state_topic": args.state_topic,
            "segment_sec": args.segment_sec,
            "idle_sec": args.idle_sec,
            "hz": args.hz,
        },
        "results": results,
        "checks": _build_checks(results),
    }
    print(json.dumps(output, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
