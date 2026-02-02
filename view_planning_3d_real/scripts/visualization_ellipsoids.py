#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion
import tf.transformations as tf

class EllipsoidVisualizer:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node("ellipsoid_visualizer", anonymous=True)
        
        # 创建 Marker 发布器
        self.publisher = rospy.Publisher("ellipsoid", Marker, queue_size=10)
        
        # # 定义椭球体数据
        # self.ellipsoids = [
        #     {"name": "黄桌子", "x": 1.5, "y": 1.95, "z": 0.472255, "roll": 0, "pitch": 0, "yaw": 0, "a": 1.4, "b": 1, "c": 1.1, "color": [1.0, 1.0, 0.0, 0.5]},  # 黄色
        #     {"name": "电视桌", "x": 4.0, "y": 2.1, "z": 0.8, "roll": 0, "pitch": 0, "yaw": 0, "a": 1.5, "b": 0.6, "c": 0.8, "color": [0.0, 0.0, 1.0, 0.5]},   # 蓝色
        #     {"name": "圆桌子", "x": 5.55, "y": 1.430089, "z": 0.58, "roll": 0, "pitch": 0, "yaw": 0, "a": 1.2, "b": 1.6, "c": 1.2, "color": [0.0, 1.0, 0.0, 0.5]},  # 绿色
        #     {"name": "沙发", "x": 4.2, "y": -0.5, "z": 0.530371, "roll": 0, "pitch": 0, "yaw": 0, "a": 1.9, "b": 1.2, "c": 1.15, "color": [1.0, 0.0, 0.0, 0.5]},  # 红色
        #     {"name": "床", "x": -5.1, "y": -2.5, "z": 0.520094, "roll": 0, "pitch": 0, "yaw": 0, "a": 1.8, "b": 1.9, "c": 1.4, "color": [1.0, 0.5, 0.0, 0.5]},  # 橙色
        # ]

        # 定义椭球体数据
        self.ellipsoids = [
            {"name": "黄桌子", "x": 1.5, "y": 1.95, "z": 0.472255, "roll": 0, "pitch": 0, "yaw": 0, "a": 1.4, "b": 1, "c": 1.1, "color": [1.0, 1.0, 0.0, 0.5]},  # 黄色
            {"name": "电视桌", "x": 4.0, "y": 2.1, "z": 0.8, "roll": 0, "pitch": 0, "yaw": 0, "a": 1.5, "b": 0.6, "c": 0.8, "color": [0.0, 0.0, 1.0, 0.5]},   # 蓝色
            {"name": "圆桌子", "x": 5.55, "y": 1.430089, "z": 0.58, "roll": 0, "pitch": 0, "yaw": 0, "a": 1.2, "b": 1.6, "c": 1.2, "color": [0.0, 1.0, 0.0, 0.5]},  # 绿色
            {"name": "沙发", "x": 4.2, "y": -0.5, "z": 0.530371, "roll": 0, "pitch": 0, "yaw": 0, "a": 1.9, "b": 1.2, "c": 1.15, "color": [1.0, 0.0, 0.0, 0.5]},  # 红色
            {"name": "床", "x": -5.1, "y": -2.5, "z": 0.520094, "roll": 0, "pitch": 0, "yaw": 0, "a": 1.8, "b": 1.9, "c": 1.4, "color": [1.0, 0.5, 0.0, 0.5]},  # 橙色
        ]

    def publish_ellipsoids(self, frame_id="world"):
        for i, ellipsoid in enumerate(self.ellipsoids):
            # 创建 Marker 消息
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = "world"
            marker.id = i
            marker.type = Marker.SPHERE  # 使用 SPHERE 类型表示椭球体
            marker.action = Marker.ADD

            # 设置椭球体的位置
            marker.pose.position.x = ellipsoid["x"]
            marker.pose.position.y = ellipsoid["y"]
            marker.pose.position.z = ellipsoid["z"]

            # 设置椭球体的比例尺寸
            marker.scale.x = ellipsoid["a"]  # x 轴长度
            marker.scale.y = ellipsoid["b"]  # y 轴长度
            marker.scale.z = ellipsoid["c"]  # z 轴长度

            # 设置颜色 (RGBA)
            marker.color.r = ellipsoid["color"][0]
            marker.color.g = ellipsoid["color"][1]
            marker.color.b = ellipsoid["color"][2]
            marker.color.a = ellipsoid["color"][3]

            # 设置旋转（Roll, Pitch, Yaw）
            quat = tf.quaternion_from_euler(ellipsoid["roll"], ellipsoid["pitch"], ellipsoid["yaw"])
            marker.pose.orientation = Quaternion(*quat)

            # 发布 Marker
            self.publisher.publish(marker)
            rospy.loginfo("Published marker for %s at (%.2f, %.2f, %.2f)", ellipsoid["name"], ellipsoid["x"], ellipsoid["y"], ellipsoid["z"])

def main():
    # 创建可视化器对象
    visualizer = EllipsoidVisualizer()

    # 设置发布频率
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        # 发布椭球体
        visualizer.publish_ellipsoids()
        rate.sleep()

if __name__ == "__main__":
    main()