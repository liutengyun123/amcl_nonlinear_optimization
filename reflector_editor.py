#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import copy
import json
import math
import os
import shutil
import threading

import rospy
from geometry_msgs.msg import PointStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
)


def as_bool(v):
    if isinstance(v, bool):
        return v
    if isinstance(v, (int, float)):
        return v != 0
    return str(v).strip().lower() in ("1", "true", "yes", "on")


def parse_click_mode(v):
    """
    CLICK_MODE:
      0 -> add
      1 -> delete

    兼容字符串:
      "0"/"add"    -> 0
      "1"/"delete" -> 1
    """
    s = str(v).strip().lower()
    if s in ("0", "add"):
        return 0
    if s in ("1", "delete"):
        return 1
    raise RuntimeError("~click_mode 只能是 0(新增) 或 1(删除)")


class ReflectorEditor:
    def __init__(self):
        self.json_path = rospy.get_param("~json_path", "")
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.marker_scale = float(rospy.get_param("~marker_scale", 0.18))
        self.text_scale = float(rospy.get_param("~text_scale", 0.12))
        self.auto_save = as_bool(rospy.get_param("~auto_save", True))

        # 0 -> add, 1 -> delete
        self.click_mode = parse_click_mode(rospy.get_param("~click_mode", 0))
        self.click_delete_radius = float(rospy.get_param("~click_delete_radius", 0.20))

        if not self.json_path:
            raise RuntimeError("~json_path 未设置")
        if not os.path.isfile(self.json_path):
            raise RuntimeError("JSON 文件不存在: {}".format(self.json_path))

        self.lock = threading.RLock()
        self.server = InteractiveMarkerServer("reflector_editor")
        self.menu_handlers = {}
        self.reflectors = []
        self.json_root = {}
        self.json_is_list = False
        self.next_id = 0
        self.backup_done = False

        self.load_json()
        self.rebuild_markers()

        # 既然 ENABLE_REFLECTOR_EDITOR 已控制是否启用编辑器，
        # 这里默认总是订阅 clicked_point，并由 click_mode 决定“新增 or 删除”
        self.clicked_sub = rospy.Subscriber(
            "/clicked_point", PointStamped, self.on_clicked_point, queue_size=1
        )

        rospy.on_shutdown(self.on_shutdown)

        rospy.loginfo("ReflectorEditor 已启动")
        rospy.loginfo("JSON: %s", self.json_path)
        rospy.loginfo("frame_id: %s", self.frame_id)
        rospy.loginfo("当前反光板数量: %d", len(self.reflectors))
        rospy.loginfo("click_mode: %d", self.click_mode)
        if self.click_mode == 0:
            rospy.loginfo("Publish Point: 点击地图 => 新增反光板")
        else:
            rospy.loginfo(
                "Publish Point: 点击地图 => 删除最近反光板（阈值 %.3f m）",
                self.click_delete_radius,
            )
        rospy.loginfo("普通编辑请使用 RViz 的 Interact 工具")

    def load_json(self):
        with self.lock:
            with open(self.json_path, "r", encoding="utf-8") as f:
                data = json.load(f)

            if isinstance(data, dict):
                self.json_root = copy.deepcopy(data)
                self.json_is_list = False
                raw_list = data.get("reflectors", [])
            elif isinstance(data, list):
                self.json_root = copy.deepcopy(data)
                self.json_is_list = True
                raw_list = data
            else:
                raise RuntimeError("JSON 根节点既不是 dict 也不是 list")

            self.reflectors = []
            max_id = -1

            for i, r in enumerate(raw_list):
                item = copy.deepcopy(r)
                item.setdefault("x", 0.0)
                item.setdefault("y", 0.0)
                item.setdefault("mean_intensity", 255)
                item.setdefault("observations", 0)
                item.setdefault("total_points", 0)
                item.setdefault("rms_radius", 0.0)
                item["_id"] = int(i)
                max_id = max(max_id, item["_id"])
                self.reflectors.append(item)

            self.next_id = max_id + 1

    def ensure_backup(self):
        if self.backup_done:
            return
        backup_path = self.json_path + ".bak"
        if not os.path.exists(backup_path):
            shutil.copy2(self.json_path, backup_path)
            rospy.loginfo("已创建 JSON 备份: %s", backup_path)
        self.backup_done = True

    def save_json(self):
        with self.lock:
            self.ensure_backup()

            out_list = []
            for r in self.reflectors:
                item = {}
                for k, v in r.items():
                    if k.startswith("_"):
                        continue
                    item[k] = v

                item["x"] = round(float(item.get("x", 0.0)), 6)
                item["y"] = round(float(item.get("y", 0.0)), 6)
                if "rms_radius" in item:
                    item["rms_radius"] = round(float(item.get("rms_radius", 0.0)), 6)

                out_list.append(item)

            if self.json_is_list:
                output_obj = out_list
            else:
                root = copy.deepcopy(self.json_root)
                root["reflectors"] = out_list
                output_obj = root

            tmp_path = self.json_path + ".tmp"
            with open(tmp_path, "w", encoding="utf-8") as f:
                json.dump(output_obj, f, ensure_ascii=False, indent=2)
            os.replace(tmp_path, self.json_path)

            rospy.loginfo("JSON 已保存: %s (共 %d 个反光板)", self.json_path, len(out_list))

    def rebuild_markers(self):
        with self.lock:
            self.server.clear()
            self.menu_handlers = {}

            for idx, ref in enumerate(self.reflectors):
                self.insert_one_marker(idx, ref)

            self.server.applyChanges()

    def insert_one_marker(self, idx, ref):
        name = "reflector_{}".format(ref["_id"])

        im = InteractiveMarker()
        im.header.frame_id = self.frame_id
        im.name = name
        im.description = "R{}".format(idx)
        im.scale = max(0.4, self.marker_scale * 3.0)
        im.pose.position.x = float(ref.get("x", 0.0))
        im.pose.position.y = float(ref.get("y", 0.0))
        im.pose.position.z = 0.0
        im.pose.orientation.w = 1.0

        ctrl = InteractiveMarkerControl()
        ctrl.name = "move_xy"
        ctrl.always_visible = True
        ctrl.interaction_mode = InteractiveMarkerControl.MOVE_PLANE

        # 归一化四元数，避免 RViz 警告
        s = 0.7071067811865476  # 1/sqrt(2)
        ctrl.orientation.x = 0.0
        ctrl.orientation.y = s
        ctrl.orientation.z = 0.0
        ctrl.orientation.w = s

        body = Marker()
        body.type = Marker.CYLINDER
        body.scale.x = self.marker_scale
        body.scale.y = self.marker_scale
        body.scale.z = max(0.03, self.marker_scale * 0.25)
        body.color.r = 1.0
        body.color.g = 0.2
        body.color.b = 0.2
        body.color.a = 0.95
        ctrl.markers.append(body)

        text = Marker()
        text.type = Marker.TEXT_VIEW_FACING
        text.pose.position.z = max(0.12, self.marker_scale * 0.9)
        text.scale.z = self.text_scale
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 0.95
        text.text = "{}".format(idx)
        ctrl.markers.append(text)

        im.controls.append(ctrl)

        self.server.insert(im, self.on_feedback)

        menu = MenuHandler()
        menu.insert("删除这个反光板", callback=self.on_menu_delete)
        menu.insert("打印这个反光板信息", callback=self.on_menu_print)
        menu.insert("立即保存 JSON", callback=self.on_menu_save)
        menu.apply(self.server, name)

        self.menu_handlers[name] = menu

    def find_reflector_by_marker_name(self, marker_name):
        if not marker_name.startswith("reflector_"):
            return None
        try:
            rid = int(marker_name.split("_")[-1])
        except Exception:
            return None

        for r in self.reflectors:
            if r["_id"] == rid:
                return r
        return None

    def delete_reflector_by_id(self, rid):
        old_n = len(self.reflectors)
        self.reflectors = [r for r in self.reflectors if r["_id"] != rid]
        return len(self.reflectors) != old_n

    def find_nearest_reflector(self, x, y):
        if not self.reflectors:
            return None, None

        best = None
        best_d2 = None
        for r in self.reflectors:
            dx = float(r.get("x", 0.0)) - x
            dy = float(r.get("y", 0.0)) - y
            d2 = dx * dx + dy * dy
            if best_d2 is None or d2 < best_d2:
                best_d2 = d2
                best = r

        return best, math.sqrt(best_d2)

    def log_reflector_info(self, prefix, ref):
        rospy.loginfo(
            "%s id=%d, x=%.6f, y=%.6f, mean_intensity=%s, observations=%s, total_points=%s, rms_radius=%s",
            prefix,
            int(ref.get("_id", -1)),
            float(ref.get("x", 0.0)),
            float(ref.get("y", 0.0)),
            str(ref.get("mean_intensity", "")),
            str(ref.get("observations", "")),
            str(ref.get("total_points", "")),
            str(ref.get("rms_radius", "")),
        )

    def on_feedback(self, feedback):
        ref = self.find_reflector_by_marker_name(feedback.marker_name)
        if ref is None:
            return

        if feedback.event_type in (
            InteractiveMarkerFeedback.POSE_UPDATE,
            InteractiveMarkerFeedback.MOUSE_UP,
        ):
            ref["x"] = float(feedback.pose.position.x)
            ref["y"] = float(feedback.pose.position.y)

            if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
                rospy.loginfo(
                    "已移动 %s => x=%.6f, y=%.6f",
                    feedback.marker_name,
                    ref["x"],
                    ref["y"],
                )
                if self.auto_save:
                    self.save_json()

    def on_menu_delete(self, feedback):
        with self.lock:
            ref = self.find_reflector_by_marker_name(feedback.marker_name)
            if ref is None:
                return

            self.log_reflector_info("右键删除反光板:", ref)

            ok = self.delete_reflector_by_id(ref["_id"])
            if ok:
                self.rebuild_markers()
                if self.auto_save:
                    self.save_json()

    def on_menu_print(self, feedback):
        ref = self.find_reflector_by_marker_name(feedback.marker_name)
        if ref is None:
            return

        self.log_reflector_info("反光板信息:", ref)

    def on_menu_save(self, feedback):
        self.save_json()

    def on_clicked_point(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.frame_id:
            rospy.logwarn(
                "clicked_point 的 frame_id=%s，但编辑器 frame_id=%s，已忽略本次点击",
                msg.header.frame_id,
                self.frame_id,
            )
            return

        x = float(msg.point.x)
        y = float(msg.point.y)

        with self.lock:
            if self.click_mode == 0:
                new_ref = {
                    "_id": self.next_id,
                    "x": x,
                    "y": y,
                    "mean_intensity": 255,
                    "observations": 0,
                    "total_points": 0,
                    "rms_radius": 0.0,
                }
                self.next_id += 1
                self.reflectors.append(new_ref)

                self.log_reflector_info("通过 Publish Point 新增反光板:", new_ref)

                self.rebuild_markers()
                if self.auto_save:
                    self.save_json()

            else:
                ref, dist = self.find_nearest_reflector(x, y)
                if ref is None:
                    rospy.logwarn("当前没有可删除的反光板")
                    return

                if dist > self.click_delete_radius:
                    rospy.logwarn(
                        "点击位置附近 %.3f m 内没有反光板，最近距离为 %.3f m，未删除",
                        self.click_delete_radius,
                        dist,
                    )
                    return

                rid = ref["_id"]
                self.log_reflector_info("通过 Publish Point 删除最近反光板:", ref)
                rospy.loginfo("点击位置到该反光板距离: %.6f m", dist)

                ok = self.delete_reflector_by_id(rid)
                if ok:
                    self.rebuild_markers()
                    if self.auto_save:
                        self.save_json()

    def on_shutdown(self):
        try:
            if self.auto_save:
                self.save_json()
        except Exception as e:
            rospy.logwarn("关闭时保存 JSON 失败: %s", str(e))


if __name__ == "__main__":
    rospy.init_node("reflector_editor")
    try:
        editor = ReflectorEditor()
        rospy.spin()
    except Exception as e:
        rospy.logerr("ReflectorEditor 启动失败: %s", str(e))
