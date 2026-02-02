#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class MoveItIkDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('moveit_ik_demo')
                
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('arm')
                
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()
                        
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'base_footprint'
        arm.set_pose_reference_frame(reference_frame)
                
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.01)
        arm.set_goal_orientation_tolerance(0.05)
        
<<<<<<< HEAD
        # 无用
        # target_pose1.pose.position.x = 0.004926
        # target_pose1.pose.position.y = -0.9705
        # target_pose1.pose.position.z = 0.62069
        # target_pose1.pose.orientation.x = 0.499
        # target_pose1.pose.orientation.y = 0.4998
        # target_pose1.pose.orientation.z = -0.500199
        # target_pose1.pose.orientation.w = 0.500199

        # target1
        target_pose1 = PoseStamped()
        target_pose1.header.frame_id = reference_frame
        target_pose1.header.stamp = rospy.Time.now()     
        target_pose1.pose.position.x = 0.468
        target_pose1.pose.position.y = -0.326
        target_pose1.pose.position.z = 0.950
        target_pose1.pose.orientation.x = 0.911822
        target_pose1.pose.orientation.y = -0.0269758
        target_pose1.pose.orientation.z = 0.285694
        target_pose1.pose.orientation.w = -0.293653

        
        # target3
        target_pose3 = PoseStamped()
        target_pose3.header.frame_id = reference_frame
        target_pose3.header.stamp = rospy.Time.now()     
        target_pose3.pose.position.x = 0.468
        target_pose3.pose.position.y = -0.326
        target_pose3.pose.position.z = 0.950
        # target_pose3.pose.orientation.x = 0.911822
        # target_pose3.pose.orientation.y = -0.0269758
        # target_pose3.pose.orientation.z = 0.285694
        # target_pose3.pose.orientation.w = -0.293653
        
=======
               
        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于base_link坐标系
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = 0.468
        target_pose.pose.position.y = -0.326
        target_pose.pose.position.z = 0.950
        target_pose.pose.orientation.x = 0.911822
        target_pose.pose.orientation.y = -0.0269758
        target_pose.pose.orientation.z = 0.285694
        target_pose.pose.orientation.w = -0.293653
>>>>>>> 08c1266182c2a3e9324b4c06027860246ab60b14
        
        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()
        
        # 设置机械臂终端运动的目标位姿
<<<<<<< HEAD
        arm.set_pose_target(target_pose3, end_effector_link)
        
        
        #选择 规划器
        # arm.set_planner_id("RRTConnectkConfigDefault")
        # arm.set_planner_id("RRT")
        arm.set_planner_id("RRTstar")
        

        # 规划运动路径
        arm.set_goal_position_tolerance(0.1)
        arm.set_goal_orientation_tolerance(0.01)
        traj = arm.plan()
        
        # 按照规划的运动路径控制机械臂运动
        arm.execute(traj)
        rospy.loginfo("1 success")
        rospy.sleep(1)


        # arm.shift_pose_target(0, 0.1, end_effector_link)
        # # arm.shift_pose_target(1, 0.1, end_effector_link)
        # arm.go()
        # rospy.loginfo("2 success")
        # rospy.sleep(1)
=======
        arm.set_pose_target(target_pose, end_effector_link)
        
        
        #选择 规划器
        arm.set_planner_id("RRTConnectkConfigDefault")
        # arm.set_planner_id("RRTstar")


        # 规划运动路径
        rospy.loginfo("1 success")
        traj = arm.plan()
        rospy.loginfo("2 success")
        
        # 按照规划的运动路径控制机械臂运动
        arm.execute(traj)
        rospy.loginfo("3 success")

        rospy.sleep(1)
>>>>>>> 08c1266182c2a3e9324b4c06027860246ab60b14
         
        # # 控制机械臂终端向右移动5cm
        # arm.shift_pose_target(1, -0.05, end_effector_link)
        # arm.go()
        # rospy.sleep(1)
  
        # # 控制机械臂终端反向旋转90度
        # arm.shift_pose_target(3, -1.57, end_effector_link)
        # arm.go()
        # rospy.sleep(1)
           


        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    MoveItIkDemo()

    
    
