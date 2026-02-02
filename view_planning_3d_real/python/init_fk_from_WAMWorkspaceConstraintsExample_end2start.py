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

        joint_positions = [2.58491646825e-08, 0.939844187586, 6.1410687169e-06, 1.59994486831, 4.40251864469e-06, -0.919023929322, 1.55000809572]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.loginfo("0 success")
        #rospy.sleep(2)

        joint_positions = [0.00240374550945, 0.736155427163, 0.0077994693087, 1.80445076903, 0.0076122594676, -0.976802949974, 1.54260457412]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.loginfo("1 success")
        #rospy.sleep(2)

        joint_positions = [0.00741193855667, 0.526404457734, 0.0349549284638, 2.24684323384, 0.0385518725741, -1.12308502897, 1.50702448397]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.loginfo("2 success")
        #rospy.sleep(2)

        joint_positions = [0.0041641582175, 0.211416763027, 0.131884410739, 2.67566050597, 0.153333208329, -1.25804812699, 1.48638912111]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.loginfo("3 success")
        #rospy.sleep(2)
        
        joint_positions = [-0.00754276356931, -0.0934838498441, 0.346747707186, 2.99689974866, 0.391821718418, -1.28401504952, 1.49780520074]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.loginfo("4 success")
        #rospy.sleep(2)
        
        joint_positions = [-0.0188323456761, -0.305138922035, 0.723568259709, 3.11112689863, 0.729090366975, -1.19039631799, 1.51988223506]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.loginfo("5 success")
        #rospy.sleep(2)
        
        joint_positions = [-0.110132157807, -0.449566248176, 1.17679851242, 3.04396039199, 1.10710714461, -1.03349654037, 1.49052471456]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.loginfo("6 success")
        #rospy.sleep(2)
        
        joint_positions = [-0.33257414009, -0.600464368301, 1.60156335408, 2.85204874872, 1.4341156613, -0.884174056183, 1.39563693176]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.loginfo("7 success")
        #rospy.sleep(2)
        
        joint_positions = [-0.662360276548, -0.791787175006, 1.91809232293, 2.50519696884, 1.60108925898, -0.77803833018, 1.31116862909]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.loginfo("8 success")
        #rospy.sleep(2)
        
        joint_positions = [-0.950587604592, -1.0162286692, 2.10870793499, 1.85969272056, 1.46585705611, -0.709051652543, 1.38843039434]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.loginfo("9 success")
        #rospy.sleep(2)
        
        joint_positions = [-0.926025904339, -1.40888544555, 2.32679521926, 1.2906711338, 1.19695295898, -0.639644340969, 1.57074794874]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.loginfo("10 success")
        #rospy.sleep(2)
        

        # # 桌面
        # joint_positions = [-2.018431036907354, -1.4999897089911451, 1.4046777996889483, -1.3707039693409548, -3.0999924865261397, -0.8560425214202461, 4.8622166345079165]
        # arm.set_joint_value_target(joint_positions)
        # arm.go()
        # rospy.loginfo("桌面 success")
        # #rospy.sleep(2)

        # # 抽屉                                                                 
        # joint_positions =  [-3.084730575741016, -1.763304599691998, 1.8552083929655296, 0.43301604856981246, -2.672461979658843, 0.46925065047728776, 4.000864693936108]
        # arm.set_joint_value_target(joint_positions)
        # arm.go()
        # rospy.loginfo("抽屉 success")
        # #rospy.sleep(2)
        

        



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
