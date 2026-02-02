#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from control_msgs.msg import GripperCommand
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import  PlanningScene, ObjectColor
from geometry_msgs.msg import PoseStamped, Pose


class MoveItFkDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('moveit_fk_demo', anonymous=True)
 
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('arm')

        
        # 设置机械臂和夹爪的允许误差值
        arm.set_goal_joint_tolerance(0.001)
        # arm.set_planner_id("RRTConnectkConfigDefault")
        arm.set_planner_id("RRTstar")

        reference_frame = 'odom'




        # 障碍物
        # 初始化场景对象
        scene = PlanningSceneInterface()
        
        # 创建一个发布场景变化信息的发布者
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=5)
        
        # 创建一个存储物体颜色的字典对象
        self.colors = dict()
        
        # 等待场景准备就绪
        rospy.sleep(1)
        # 设置场景物体的名称
        table_id = 'table'
        object_id = 'object'
        
        # 移除场景中之前运行残留的物体
        scene.remove_world_object(table_id)
        scene.remove_world_object(object_id)

        # 设置桌面的高度
        table_ground = 0.25
        
        # 设置table、box1和box2的三维尺寸
        table_size = [0.400311, 0.368233, 0.4]
        object_size = [0.368233, 0.400311, 0.245264]

        
        # 将三个物体加入场景当中
        # table_pose = PoseStamped()
        # table_pose.header.frame_id = reference_frame
        # table_pose.pose.position.x = 1.163990
        # table_pose.pose.position.y = -0.317092
        # table_pose.pose.position.z = 0.19
        # table_pose.pose.orientation.w = 1.0
        # scene.add_box(table_id, table_pose, table_size)
        
        box1_pose = PoseStamped()
        box1_pose.header.frame_id = reference_frame
        box1_pose.pose.position.x = 0.68888
        box1_pose.pose.position.y = -0.317092
        box1_pose.pose.position.z = 0.467195
        box1_pose.pose.orientation.w = 1.0   
        scene.add_box(object_id, box1_pose, object_size)
        
   
        
        # 将桌子设置成红色，两个box设置成橙色
        # self.setColor(table_id, 0.8, 0, 0, 1.0)
        self.setColor(object_id, 0.8, 0.4, 0, 1.0)

        
        # 将场景中的颜色设置发布
        self.sendColors()    








        # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）

        # 1
        joint_positions = [-1.210074720592524, 0.5214049999058119, 1.1192497150367222, 2.263123183509757, 1.3441201359633466, 1.1762948755187237, -2.367342719655495]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.loginfo("1 success")





        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)



    # 设置场景物体的颜色
    def setColor(self, name, r, g, b, a = 0.9):
        # 初始化moveit颜色对象
        color = ObjectColor()
        
        # 设置颜色值
        color.id = name
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a
        
        # 更新颜色字典
        self.colors[name] = color

    # 将颜色设置发送并应用到moveit场景当中
    def sendColors(self):
        # 初始化规划场景对象
        p = PlanningScene()

        # 需要设置规划场景是否有差异     
        p.is_diff = True
        
        # 从颜色字典中取出颜色设置
        for color in self.colors.values():
            p.object_colors.append(color)
        
        # 发布场景物体颜色设置
        self.scene_pub.publish(p)

if __name__ == "__main__":
    try:
        MoveItFkDemo()
    except rospy.ROSInterruptException:
        pass
