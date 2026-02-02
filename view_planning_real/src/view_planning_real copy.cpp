#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>

// moveit
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <rm_msgs/MoveJ.h>
#include <rm_msgs/Plan_State.h>

// 全局变量，用于控制循环状态
bool loop = false;  // 是否进入循环


/**
 * @brief 回调函数：用于接收控制进入循环的消息
 * 
 * @param msg 接收到的布尔值消息
 */
geometry_msgs::PointStamped object_centor;
void startCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    // 打印点的坐标值
    ROS_INFO("Received point in frame: %s", msg->header.frame_id.c_str());
    ROS_INFO("Point coordinates: x = %.2f, y = %.2f, z = %.2f",
             msg->point.x, msg->point.y, msg->point.z);
    object_centor = *msg;

    loop = true;
    ROS_INFO_STREAM("Received start signal: " << (loop ? "true" : "false"));
}


void stopCallback(const std_msgs::Bool::ConstPtr& msg) {
    loop = false;
    ROS_INFO_STREAM("Received stop signal: " << (loop ? "true" : "false"));
}

/**
 * @brief 获取 base_link 相对于 map 的坐标变换
 * 
 * @param tfBuffer TF2 缓存
 */
void getTransform(const tf2_ros::Buffer& tfBuffer) {
    try {
        // 获取坐标变换
        geometry_msgs::TransformStamped transformStamped;
        transformStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));

        // 打印变换信息
        // ROS_INFO_STREAM("Transform from map to base_link:");
        // ROS_INFO_STREAM("Translation: [" 
        //                 << transformStamped.transform.translation.x << ", "
        //                 << transformStamped.transform.translation.y << ", "
        //                 << transformStamped.transform.translation.z << "]");
        // ROS_INFO_STREAM("Rotation: [" 
        //                 << transformStamped.transform.rotation.x << ", "
        //                 << transformStamped.transform.rotation.y << ", "
        //                 << transformStamped.transform.rotation.z << ", "
        //                 << transformStamped.transform.rotation.w << "]");
    } catch (tf2::TransformException &ex) {
        ROS_WARN("Could not get transform: %s", ex.what());
    }
}

// 获取从 source_frame 坐标系到 target_frame 坐标系的变换。
// source_frame 是输入点所在的坐标系。
bool transformPoint(const tf2_ros::Buffer& tfBuffer, 
                    const std::string& target_frame, 
                    const std::string& source_frame, 
                    const geometry_msgs::PointStamped& input_point, 
                    geometry_msgs::PointStamped& transformed_point) {
    try {
        // 获取 TF 坐标变换
        geometry_msgs::TransformStamped transformStamped;
        transformStamped = tfBuffer.lookupTransform(target_frame, source_frame, ros::Time(0));

        // 执行坐标转换
        tf2::doTransform(input_point, transformed_point, transformStamped);

        return true; // 转换成功
    } catch (tf2::TransformException &ex) {
        ROS_WARN("Could not transform point from %s to %s: %s", source_frame.c_str(), target_frame.c_str(), ex.what());
        return false; // 转换失败
    }
}

// 全局状态标志
volatile bool plan_state_received = true;

void planStateCallback(const rm_msgs::Plan_State::ConstPtr& msg) {
    plan_state_received = true;  // 触发状态变更
}

int main(int argc, char** argv) {
    // 初始化 ROS 节点
    ros::init(argc, argv, "loop_transform_node");
    ros::NodeHandle nh;
    // 初始化 MoveIt 接口
    ros::AsyncSpinner spinner(1);
    spinner.start();
    moveit::planning_interface::MoveGroupInterface move_group("arm"); // 替换 "arm" 为你的机械臂规划组名称
    ROS_INFO("MoveIt interface initialized for planning group: %s", move_group.getName().c_str());

    // 创建 TF2 缓存和监听器
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    // 订阅两个话题 const geometry_msgs::PointStamped::ConstPtr& msg
    ros::Subscriber start_sub = nh.subscribe<geometry_msgs::PointStamped>("/object_centor", 10, startCallback);
    ros::Subscriber stop_sub = nh.subscribe<std_msgs::Bool>("/stop_loop", 10, stopCallback);
    // 创建订阅者 (监听PlanState反馈)
    ros::Subscriber plan_sub = nh.subscribe("/rm_driver/Plan_State", 10, planStateCallback);
    // 创建发布者 (发送MoveJ指令)
    ros::Publisher movej_pub = nh.advertise<rm_msgs::MoveJ>("/rm_driver/MoveJ_Cmd", 10);


    ros::Rate rate(5.0); // 循环频率为 10Hz

    ROS_INFO("Waiting for start signal...");

    // 主循环
    while (ros::ok()) {
        ros::spinOnce(); // 处理回调函数
        while (!plan_state_received ) {
            ros::spinOnce();   // 处理回调
            rate.sleep(); // 避免CPU空转
        }
        // 如果收到开始信号，则进入循环
        if (loop) {
            // 获取 base_link 相对于 map 的坐标变换
            // getTransform(tfBuffer);
            geometry_msgs::PointStamped object_centor_robot;
            transformPoint(tfBuffer, "base_link_wheeltec", "map", object_centor, object_centor_robot);

            // 提取点的 x 和 y 坐标
            double x = object_centor_robot.point.x;
            double y = object_centor_robot.point.y;
            
            // 计算夹角，atan2 返回的角度范围是 [-π, π]
            double angle = atan2(y, x);

            std::cout<<"angle: "<<angle/M_PI*180<<std::endl;
            
            rm_msgs::MoveJ cmd_msg;
            // 填充关节角度数据（示例值）[7](@ref)
            cmd_msg.joint = {angle, -0.7, 0.5, -0.0, 0.2, -0.0};  // 6轴机械臂典型位置
            cmd_msg.speed = 1;  
            movej_pub.publish(cmd_msg);

            // // 等待PlanState反馈
            // while (!plan_state_received && ros::ok()) {
            //     ros::spinOnce();   // 处理回调
            //     rate.sleep(); // 避免CPU空转
            // }
            plan_state_received = false; // 重置标志位
        }
        else{
            // 发布回正的信号
            ROS_INFO("Received stop signal. Exiting loop...");
            rm_msgs::MoveJ cmd_msg;
            // 填充关节角度数据（示例值）[7](@ref)
            cmd_msg.joint = {0, -0.7, 0.5, -0.0, 0.2, -0.0};  // 6轴机械臂典型位置
            cmd_msg.speed = 0.5;  
            rate.sleep(); // 避免CPU空转
            movej_pub.publish(cmd_msg);
            // 等待PlanState反馈
            plan_state_received = false; // 重置标志位
            ROS_INFO("Publish Zero");

        }
        ros::spinOnce(); // 处理回调函数
    }

    ROS_INFO("Node shutting down.");
    return 0;
}