#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import os
import sys
import argparse
import rospy
import tf2_ros
import tf
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped


def strip_leading_slash(s: str) -> str:
    if not s:
        return s
    return s[1:] if s.startswith("/") else s


def should_publish_sensor_tf(sensor_tf: dict):
    if not isinstance(sensor_tf, dict):
        return False, "invalid entry"

    child = strip_leading_slash(sensor_tf.get("frame_id", ""))
    if not child:
        return False, "empty frame_id"

    sensor_type = strip_leading_slash(sensor_tf.get("type", "")).lower()

    if child.lower() == "odom" or sensor_type == "odom":
        return False, "dynamic odom frame must not be published as static tf"

    return True, ""


class TfNode:
    def __init__(self):
        self.tfs = []
        self.tf_broadcaster = StaticTransformBroadcaster()

    def send_sensors_tf(self):
        if not self.tfs:
            return
        stamp = rospy.Time.now()
        for t in self.tfs:
            t.header.stamp = stamp
        self.tf_broadcaster.sendTransform(self.tfs)

    def read_sensors_tf(self, cfg_file: str, base_frame: str):
        with open(cfg_file, "r") as f:
            data = json.load(f)

        sensors = data.get("sensors_tf", [])
        if not isinstance(sensors, list):
            raise RuntimeError("Invalid hw_config.json: sensors_tf is not a list")

        base_frame = strip_leading_slash(base_frame)

        seen_child = set()
        dup_child = []

        for sensor_tf in sensors:
            ok, skip_reason = should_publish_sensor_tf(sensor_tf)
            if not ok:
                if isinstance(sensor_tf, dict):
                    child = strip_leading_slash(sensor_tf.get("frame_id", ""))
                    sensor_type = strip_leading_slash(sensor_tf.get("type", ""))
                    print(f"[TF_STATIC][SKIP] frame={child or '<empty>'} "
                          f"type={sensor_type or '<empty>'} reason={skip_reason}")
                continue

            child = strip_leading_slash(sensor_tf.get("frame_id", ""))
            parent = strip_leading_slash(sensor_tf.get("parent_frame_id", base_frame))

            if child == parent:
                print(f"[TF_STATIC][SKIP] frame={child} reason=child equals parent")
                continue

            pos = sensor_tf.get("position", {}) or {}
            ori = sensor_tf.get("orientation", {}) or {}

            x = float(pos.get("x", 0.0))
            y = float(pos.get("y", 0.0))
            z = float(pos.get("z", 0.0))

            roll = float(ori.get("roll", 0.0))
            pitch = float(ori.get("pitch", 0.0))
            yaw = float(ori.get("yaw", 0.0))

            # rpy -> quaternion (严格按 rad)
            q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

            msg = TransformStamped()
            msg.header.frame_id = parent
            msg.child_frame_id = child
            msg.transform.translation.x = x
            msg.transform.translation.y = y
            msg.transform.translation.z = z
            msg.transform.rotation.x = q[0]
            msg.transform.rotation.y = q[1]
            msg.transform.rotation.z = q[2]
            msg.transform.rotation.w = q[3]

            if child in seen_child:
                dup_child.append(child)
            seen_child.add(child)

            print(f"[TF_STATIC] {parent} -> {child} "
                  f"xyz=({x:.6f},{y:.6f},{z:.6f}) rpy=({roll:.6f},{pitch:.6f},{yaw:.6f}) "
                  f"q=({q[0]:.6f},{q[1]:.6f},{q[2]:.6f},{q[3]:.6f})")

            self.tfs.append(msg)

        if dup_child:
            rospy.logwarn("Duplicate child_frame_id detected (may conflict): %s", sorted(set(dup_child)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="/opt/factory/hw_config.json", help="Path to hw_config.json")
    parser.add_argument("--base_frame", default="base_footprint",
                        help="Default parent frame if parent_frame_id missing")
    args = parser.parse_args(rospy.myargv()[1:])

    if not os.path.exists(args.config):
        print(f"[ERROR] config not exist: {args.config}")
        sys.exit(2)

    rospy.init_node("send_static_tf", anonymous=False)

    node = TfNode()
    node.read_sensors_tf(args.config, args.base_frame)

    node.send_sensors_tf()

    rospy.spin()


if __name__ == "__main__":
    main()
