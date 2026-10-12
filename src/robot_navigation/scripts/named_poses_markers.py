#!/usr/bin/env python3
# 把 named_poses 點位發布成 MarkerArray，在 Foxglove / RViz 疊在 /map 上看。
#
#   ros2 run robot_navigation named_poses_markers.py --ros-args -p field_side:=mirror
#   （沒 build 也可以直接：python3 <本檔路徑> --ros-args -p field_side:=mirror）
#
# Foxglove 3D panel：Display frame 選 world，勾 /map 和 /named_poses_markers。
# 實時更新：每 poll_period 秒檢查 named_poses yaml 的修改時間，存檔後自動
# 重新解析並發布（yaml 寫到一半解析失敗時保留上一版，不會清掉畫面）。
#
# 參數：
#   field_side        normal / mirror / dit（空 = field_config.yaml 的 default_side）
#   named_poses_file  直接指定點位檔（空 = 用 field_side 對應的檔案）
#   frame_id          點位座標系，預設 world（場地左下角）
#   axis_length       world 原點 x/y 軸箭頭長度（m）
#   poll_period       檢查 yaml 有沒有被修改的週期（s）

import os

import rclpy
import yaml
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from named_poses_common import group_coincident, load_named_poses, load_side

POSE_COLOR = ColorRGBA(r=1.0, g=0.55, b=0.0, a=1.0)
LABEL_COLOR = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
X_COLOR = ColorRGBA(r=0.9, g=0.1, b=0.1, a=1.0)
Y_COLOR = ColorRGBA(r=0.1, g=0.8, b=0.1, a=1.0)
ORIGIN_COLOR = ColorRGBA(r=0.2, g=0.5, b=1.0, a=1.0)


class NamedPosesMarkers(Node):
    def __init__(self):
        super().__init__('named_poses_markers')
        field_side = self.declare_parameter('field_side', '').value
        named_poses_file = self.declare_parameter('named_poses_file', '').value
        self.frame_id = self.declare_parameter('frame_id', 'world').value
        self.axis_length = float(self.declare_parameter('axis_length', 1.0).value)
        poll_period = float(self.declare_parameter('poll_period', 0.5).value)

        side = load_side(field_side or None)
        self.named_poses_file = named_poses_file or side['named_poses_file']
        self.get_logger().info(f"field_side={side['side']}  watching {self.named_poses_file}")

        # transient_local 給晚連上的 subscriber；Foxglove 不一定用 transient_local
        # 訂閱，所以再每秒補發一次。
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, 'named_poses_markers', qos)
        self.msg = self._build([])  # 第一次讀檔失敗時至少有原點 / 座標軸
        self.mtime = None
        self.missing_warned = False
        self._reload_if_changed()
        self.create_timer(poll_period, self._reload_if_changed)
        self.create_timer(1.0, lambda: self.pub.publish(self.msg))

    def _reload_if_changed(self):
        try:
            mtime = os.stat(self.named_poses_file).st_mtime_ns
        except OSError as e:
            # 有些編輯器存檔是先刪再寫，短暫找不到檔案很正常，只警告一次
            if not self.missing_warned:
                self.get_logger().warn(f'cannot stat named_poses file: {e}')
                self.missing_warned = True
            return
        self.missing_warned = False
        if mtime == self.mtime:
            return
        self.mtime = mtime

        try:
            poses, errors = load_named_poses(self.named_poses_file)
        except (OSError, yaml.YAMLError, AttributeError, TypeError, ValueError) as e:
            self.get_logger().warn(f'reload failed, keeping previous markers: {e}')
            return
        for e in errors:
            self.get_logger().error(e)
        self.get_logger().info(f'loaded {len(poses)} poses')
        for name, x, y, yaw, rel in poses:
            self.get_logger().info(
                f'  {name:28s} x={x:7.3f} y={y:7.3f} yaw={yaw:6.3f}'
                f"{'  (relative)' if rel else ''}")

        self.msg = self._build(poses)
        self.pub.publish(self.msg)

    def _marker(self, ns, mid, mtype, color):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.ns = ns
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.color = color
        m.frame_locked = True
        return m

    def _text(self, ns, mid, text, x, y, z, size, color):
        m = self._marker(ns, mid, Marker.TEXT_VIEW_FACING, color)
        m.text = text
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        m.scale.z = size
        return m

    def _build(self, poses):
        markers = []
        # 先清掉舊的（重開 node 後點位變少時不會留下殘影）
        clear = Marker()
        clear.header.frame_id = self.frame_id
        clear.action = Marker.DELETEALL
        markers.append(clear)

        # world 原點 + x/y 軸
        L = self.axis_length
        for i, (color, end, label) in enumerate(
                [(X_COLOR, (L, 0.0), 'x'), (Y_COLOR, (0.0, L), 'y')]):
            a = self._marker('world_axes', i, Marker.ARROW, color)
            a.points = [Point(x=0.0, y=0.0, z=0.01), Point(x=end[0], y=end[1], z=0.01)]
            a.scale.x, a.scale.y, a.scale.z = 0.03, 0.07, 0.1  # shaft, head 直徑, head 長
            markers.append(a)
            markers.append(self._text('world_axes', 10 + i, label,
                                      end[0] * 1.08, end[1] * 1.08, 0.05, 0.15, color))
        o = self._marker('world_axes', 20, Marker.SPHERE, ORIGIN_COLOR)
        o.scale.x = o.scale.y = o.scale.z = 0.08
        markers.append(o)
        markers.append(self._text('world_axes', 21, f'{self.frame_id} (0,0)',
                                  -0.1, -0.12, 0.05, 0.1, ORIGIN_COLOR))

        # 點位：每個點一個圓柱，靠很近的點合併成一個標籤
        for i, (_name, x, y, _yaw, _rel) in enumerate(poses):
            p = self._marker('named_poses', i, Marker.CYLINDER, POSE_COLOR)
            p.pose.position.x, p.pose.position.y, p.pose.position.z = x, y, 0.01
            p.scale.x = p.scale.y = 0.06
            p.scale.z = 0.02
            markers.append(p)
        for i, (x, y, members) in enumerate(group_coincident(poses)):
            text = '\n'.join(name for name, _px, _py in members)
            markers.append(self._text('named_poses_labels', i, text,
                                      x, y + 0.05, 0.1, 0.07, LABEL_COLOR))

        return MarkerArray(markers=markers)


def main():
    rclpy.init()
    node = NamedPosesMarkers()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
