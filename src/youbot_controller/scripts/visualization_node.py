#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Узел визуализации миссии youBot в RViz.
# Публикует маркеры: старт, объект, цель(маркер), препятствия, запланированный путь.
# Координаты берутся из тех же значений, что использует trajectory_planner.

import rospy
import math
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

# Те же данные, что в trajectory_planner.py — держим синхронно
OBJECT_POS = (1.0, 1.0)
MARKER_POS = (4.0, 4.0)
OBSTACLES = [
    (1.5, 2.0, 0.4),
    (2.5, 1.5, 0.35),
    (3.0, 3.0, 0.45),
    (2.0, 3.5, 0.3),
    (3.5, 2.5, 0.25),
]
FRAME = "odom"   # система координат, в которой едет робот


class MissionViz:
    def __init__(self):
        rospy.init_node('mission_viz', anonymous=True)

        self.marker_pub = rospy.Publisher('/mission_markers', MarkerArray, queue_size=1, latch=True)
        self.path_pub = rospy.Publisher('/robot_path', Path, queue_size=1)

        # Стартовая позиция фиксируется по первой одометрии
        self.start_pos = None
        self.path = Path()
        self.path.header.frame_id = FRAME
        rospy.Subscriber('/odom', Odometry, self.odom_cb)

        # Маркеры публикуем периодически (на случай позднего запуска RViz)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.publish_markers)
        rospy.loginfo("Mission visualization node started.")

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.start_pos is None:
            self.start_pos = (x, y)
            rospy.loginfo("Старт зафиксирован: (%.2f, %.2f)", x, y)

        # Накапливаем реальный путь робота
        ps = PoseStamped()
        ps.header.frame_id = FRAME
        ps.header.stamp = rospy.Time.now()
        ps.pose = msg.pose.pose
        self.path.poses.append(ps)
        if len(self.path.poses) > 2000:
            self.path.poses.pop(0)
        self.path.header.stamp = rospy.Time.now()
        self.path_pub.publish(self.path)

    def _sphere(self, mid, pos, rgba, scale, ns="points"):
        m = Marker()
        m.header.frame_id = FRAME
        m.header.stamp = rospy.Time.now()
        m.ns = ns
        m.id = mid
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = pos[0]
        m.pose.position.y = pos[1]
        m.pose.position.z = 0.1
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = scale
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        return m

    def _cylinder(self, mid, cx, cy, r):
        m = Marker()
        m.header.frame_id = FRAME
        m.header.stamp = rospy.Time.now()
        m.ns = "obstacles"
        m.id = mid
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = cx
        m.pose.position.y = cy
        m.pose.position.z = 0.25
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = r * 2.0   # диаметр
        m.scale.z = 0.5
        m.color.r, m.color.g, m.color.b, m.color.a = (0.9, 0.2, 0.2, 0.6)
        return m

    def _text(self, mid, pos, text):
        m = Marker()
        m.header.frame_id = FRAME
        m.header.stamp = rospy.Time.now()
        m.ns = "labels"
        m.id = mid
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = pos[0]
        m.pose.position.y = pos[1]
        m.pose.position.z = 0.6
        m.pose.orientation.w = 1.0
        m.scale.z = 0.25
        m.color.r, m.color.g, m.color.b, m.color.a = (1.0, 1.0, 1.0, 1.0)
        m.text = text
        return m

    def publish_markers(self, event=None):
        arr = MarkerArray()
        mid = 0

        # Объект (синий) и цель/маркер (зелёный)
        arr.markers.append(self._sphere(mid, OBJECT_POS, (0.2, 0.4, 1.0, 0.9), 0.3)); mid += 1
        arr.markers.append(self._sphere(mid, MARKER_POS, (0.2, 1.0, 0.3, 0.9), 0.3)); mid += 1
        arr.markers.append(self._text(100, OBJECT_POS, "OBJECT (1,1)"))
        arr.markers.append(self._text(101, MARKER_POS, "GOAL (4,4)"))

        # Старт (жёлтый), если уже известен
        if self.start_pos is not None:
            arr.markers.append(self._sphere(mid, self.start_pos, (1.0, 0.9, 0.1, 0.9), 0.3)); mid += 1
            arr.markers.append(self._text(102, self.start_pos, "START / HOME"))

        # Препятствия (красные цилиндры)
        for (cx, cy, r) in OBSTACLES:
            arr.markers.append(self._cylinder(mid, cx, cy, r)); mid += 1

        self.marker_pub.publish(arr)


if __name__ == '__main__':
    try:
        node = MissionViz()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
