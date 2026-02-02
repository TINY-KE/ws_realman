#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool

def publish_point(px,py,pz):
    msg = PointStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = ''
    msg.point.x = px
    msg.point.y = py
    msg.point.z = pz
    point_pub.publish(msg)
    rospy.loginfo("Published PointStamped message.")

def publish_bool():
    msg = Bool()
    msg.data = False
    bool_pub.publish(msg)
    rospy.loginfo("Published Bool message.")

if __name__ == '__main__':
    rospy.init_node('manual_topic_publisher')

    point_pub = rospy.Publisher('/object_centor', PointStamped, queue_size=10)
    bool_pub = rospy.Publisher('/stop_loop', Bool, queue_size=10)

    rospy.sleep(1)  # 等待连接建立

    print("请输入 1（发送 PointStamped） 或 2（发送 Bool）:")
    try:
        while not rospy.is_shutdown():
            user_input = input("输入指令（0/1/2/.../q退出）： ").strip()
            if user_input == '1':
                px = 1.5 
                py = 1.95
                pz = 0.5
                publish_point(px,py,pz)
            if user_input == '2':
                # 4 2.1 0.6
                px = 4 
                py = 2.1
                pz = 0.6
                publish_point(px,py,pz)
            if user_input == '3':
                # 5.55 1.630089 0.58 
                px = 5.55 
                py = 1.230089
                pz = 0.58
                publish_point(px,py,pz)
            if user_input == '4':
                # 4.2 -0.5 0.530371
                px = 4.35
                py = -0.5
                pz = 0.530371
                publish_point(px,py,pz)
            if user_input == '5':
                # -5.5 -2.5 0.620094
                px = -5.
                py = -2.8
                pz = 0.58
                publish_point(px,py,pz)

            elif user_input == '0':
                publish_bool()
            elif user_input.lower() == 'q':
                print("退出程序。")
                break
            else:
                print("无效输入，请输入 1、2 或 q。")
    except rospy.ROSInterruptException:
        pass