#!/usr/bin/env python3
import rospy
import numpy as np
import time
from StepEgoMapPose_msgs.msg import StepEgoMapPose
from sensor_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

def publish_npz_data():
    # 1. 初始化 ROS 节点
    rospy.init_node('npz_data_publisher', anonymous=True)
    pub = rospy.Publisher('/step_ego_map_pose', StepEgoMapPose, queue_size=10)
    rate = rospy.Rate(10) # 10Hz 发布频率

    # 2. 读取 NPZ 文件
    npz_path = '/home/robotlab/dataset/semantic/semantic_datasets/data_slam/test/1/ruihai_livingroom.npz'
    print(f"Loading data from {npz_path}...")
    ep = np.load(npz_path)

    # 假设你的 NPZ 包含这些键 (根据你最初的 Dataset 类定义)
    # 调整为实际的 key，如果 key 名称不同请修改
    images = ep['step_ego_grid_27']  # [T, 27, 64, 64]
    poses = ep['virtual_robot_ground_poses'] # [T, 3]

    print(f"Total time steps: {len(images)}")

    # 3. 循环发布
    for i in range(len(images)):
        if rospy.is_shutdown():
            break
        
        # 构造自定义消息
        msg = StepEgoMapPose()
        msg.header = Header(stamp=rospy.Time.now(), frame_id="map")
        
        # 打包 OccupancyGrid (将 27x64x64 转为扁平列表)
        grid_data = images[i].flatten().tolist()
        msg.grid.data = grid_data
        msg.grid.info.width = 64
        msg.grid.info.height = 64
        
        # 打包 PoseStamped
        msg.pose.header = msg.header
        msg.pose.pose.position.x = poses[i][0]
        msg.pose.pose.position.y = poses[i][1]
        msg.pose.pose.orientation.z = poses[i][2] # 假设 theta 存储在 z
        
        # 发布
        pub.publish(msg)
        rate.sleep()

    print("Data transmission finished.")

if __name__ == '__main__':
    try:
        publish_npz_data()
    except rospy.ROSInterruptException:
        pass