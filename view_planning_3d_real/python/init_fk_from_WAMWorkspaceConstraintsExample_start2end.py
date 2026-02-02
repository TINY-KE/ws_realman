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

        reference_frame = '/wam/base_link'

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
        # scene.add_box(object_id, box1_pose, object_size)
        
   
        
        # 将桌子设置成红色，两个box设置成橙色
        # self.setColor(table_id, 0.8, 0, 0, 1.0)
        self.setColor(object_id, 0.8, 0.4, 0, 1.0)

        
        # 将场景中的颜色设置发布
        self.sendColors()    








        # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）
        
        #gpmp规划结果 0, size:7, values: 
        # [-0.799999592529, -1.69999958617, 1.64000036225, 1.2900001101, 1.09999985802, -0.105999705866, 2.19999970525];
        # 设置joint values, 0
        # ^C^C规划失败
        joint_positions = [-0.799999592529, -1.69999958617, 1.64000036225, 1.2900001101, 1.09999985802, -0.105999705866, 2.19999970525]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.loginfo("0 success")


        # #gpmp规划结果 1, size:7, values: 
        # [-0.768871229128, -1.61628583614, 1.65771449603, 1.31773557176, 1.07470111634, -0.0949896927549, 2.14298743689];
        # 设置joint values, 1
        # 规划失败
        joint_positions = [-0.768871229128, -1.61628583614, 1.65771449603, 1.31773557176, 1.07470111634, -0.0949896927549, 2.14298743689]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.loginfo("1 success")


        # #gpmp规划结果 2, size:7, values: 
        # [-0.696596739084, -1.38722664484, 1.69131273321, 1.3944613834, 1.00625938754, -0.0781131763219, 1.98759505738];
        # 设置joint values, 2
        # 规划失败
        joint_positions = [-0.696596739084, -1.38722664484, 1.69131273321, 1.3944613834, 1.00625938754, -0.0781131763219, 1.98759505738]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.loginfo("2 success")


        # #gpmp规划结果 3, size:7, values: 
        # [-0.610587821012, -1.04633599698, 1.71286829409, 1.50667017655, 0.905277049848, -0.080229136145, 1.75706058947];
        # 设置joint values, 3
        # 规划失败
        joint_positions = [-0.610587821012, -1.04633599698, 1.71286829409, 1.50667017655, 0.905277049848, -0.080229136145, 1.75706058947]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.loginfo("3 success")


        # #gpmp规划结果 4, size:7, values: 
        # [-0.527071534331, -0.62874771292, 1.70632332474, 1.63302435227, 0.781508198367, -0.120334278383, 1.47483472182];
        # 设置joint values, 4
        # 规划失败
        joint_positions = [-0.527071534331, -0.62874771292, 1.70632332474, 1.63302435227, 0.781508198367, -0.120334278383, 1.47483472182]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.loginfo("4 success")

        # #gpmp规划结果 5, size:7, values: 
        # [-0.450062969226, -0.170682119864, 1.67368712016, 1.74933505199, 0.644351650068, -0.204699212895, 1.16513736401];
        # 设置joint values, 5
        # 规划失败
        # #gpmp规划结果 6, size:7, values: 
        # [-0.376309685246, 0.291658453117, 1.63049858651, 1.8385285277, 0.503564281913, -0.324452458541, 0.852930090116];
        # 设置joint values, 6
        # 规划失败
        # #gpmp规划结果 7, size:7, values: 
        # [-0.298368367969, 0.722062172125, 1.59839878872, 1.901296915, 0.369711117294, -0.452241728098, 0.562552890164];
        # 设置joint values, 7
        # 规划失败
        # #gpmp规划结果 8, size:7, values: 
        # [-0.219340060928, 1.08230991341, 1.59069281768, 1.94577849402, 0.255310609827, -0.557912691636, 0.315500753612];
        # 设置joint values, 8
        # 规划失败
        # #gpmp规划结果 9, size:7, values: 
        # [-0.274287305951, 1.3256463077, 1.5630144718, 1.86243960348, 0.182014967719, -0.742391834087, 0.132552109545];
        # 设置joint values, 9
        # 规划失败
        # #gpmp规划结果 10, size:7, values: 
        # [-0.534068492488, 1.40730090311, 1.48509807365, 1.59999744461, 0.168262480181, -1.04948803965, 0.0533745150475];
        # 设置joint values, 10
        # 规划失败


        

        



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
