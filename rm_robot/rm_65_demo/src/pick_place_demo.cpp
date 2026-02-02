//
// Created by ubuntu on 21-8-11.
// 修改：通过ROS话题接收物体中心位置
//
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <mutex>
#include <queue>
#include <cmath>

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
#include <rm_msgs/JointPos.h>
#include <rm_msgs/Plan_State.h>

//
// Created by ubuntu on 21-8-11.
// 修改：通过ROS话题接收物体中心位置
//
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <mutex>
#include <cmath>
#include <string>

// 全局变量
geometry_msgs::PointStamped object_center;
std::mutex center_mutex;
bool new_center_received = false;
bool execute_circular_path = false;
bool object_center_received = false;

// 函数声明
void objectCenterCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
void executeCommandCallback(const std_msgs::Bool::ConstPtr& msg);
bool transformToBaseFrame(geometry_msgs::PointStamped& point, 
                         const std::string& target_frame,
                         tf2_ros::Buffer& tf_buffer);
bool moveInCircularPath(moveit::planning_interface::MoveGroupInterface& group,
                       const geometry_msgs::Point& center_point,
                       double radius, 
                       double start_angle, 
                       double end_angle, 
                       int num_points,
                       bool clockwise = true,
                       bool maintain_orientation = true);
bool moveInCircularPathJointSpace(moveit::planning_interface::MoveGroupInterface& group,
                                 const geometry_msgs::Point& center_point,
                                 double radius,
                                 double start_angle,
                                 double end_angle,
                                 int num_points,
                                 bool clockwise = true);
bool moveToSafePosition(moveit::planning_interface::MoveGroupInterface& group,
                       const geometry_msgs::Point& center_point,
                       double safe_distance);

// 物体中心回调函数
void objectCenterCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(center_mutex);
    object_center = *msg;
    new_center_received = true;
    object_center_received = true;
    
    ROS_INFO("收到物体中心: 坐标系=%s, 位置=(%.3f, %.3f, %.3f)",
             msg->header.frame_id.c_str(),
             msg->point.x, msg->point.y, msg->point.z);
}

// 执行命令回调函数
void executeCommandCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data) {
        ROS_INFO("收到执行圆弧轨迹命令");
        execute_circular_path = true;
    }
}

// 转换坐标到基座坐标系
bool transformToBaseFrame(geometry_msgs::PointStamped& point, 
                         const std::string& target_frame,
                         tf2_ros::Buffer& tf_buffer)
{
    if (point.header.frame_id == target_frame) {
        return true;  // 已经在目标坐标系
    }
    
    try {
        geometry_msgs::PointStamped transformed_point;
        tf_buffer.transform(point, transformed_point, target_frame, ros::Duration(1.0));
        point = transformed_point;
        return true;
    } catch (tf2::TransformException& ex) {
        ROS_WARN("坐标转换失败: %s", ex.what());
        return false;
    }
}

// 关节空间规划圆弧轨迹
bool moveInCircularPathJointSpace(moveit::planning_interface::MoveGroupInterface& group,
                                 const geometry_msgs::Point& center_point,
                                 double radius,
                                 double start_angle,
                                 double end_angle,
                                 int num_points,
                                 bool clockwise)
{
    ROS_INFO("使用关节空间规划圆弧轨迹...");
    
    std::vector<geometry_msgs::Pose> waypoints;
    
    // 计算角度步长
    double angle_step = (end_angle - start_angle) / (num_points - 1);
    if (!clockwise) {
        angle_step = -angle_step;
    }
    
    // 获取当前末端姿态
    geometry_msgs::PoseStamped current_pose = group.getCurrentPose("Link6");
    geometry_msgs::Quaternion current_orientation = current_pose.pose.orientation;
    
    // 生成路径点
    for (int i = 0; i < num_points; i++) {
        double angle = start_angle + i * angle_step;
        
        geometry_msgs::Pose target_pose;
        target_pose.position.x = center_point.x + radius * cos(angle);
        target_pose.position.y = center_point.y + radius * sin(angle);
        target_pose.position.z = center_point.z;
        target_pose.orientation = current_orientation;
        
        waypoints.push_back(target_pose);
    }
    
    // 逐个点规划执行
    for (size_t i = 0; i < waypoints.size(); i++) {
        ROS_INFO("移动到点 %zu/%zu...", i+1, waypoints.size());
        
        group.setPoseTarget(waypoints[i], "Link6");
        
        // 设置规划参数
        group.setPlanningTime(5.0);
        group.setNumPlanningAttempts(3);
        
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        moveit::planning_interface::MoveItErrorCode success = group.plan(plan);
        
        if (success != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            ROS_WARN("点 %zu 规划失败，跳过", i+1);
            continue;
        }
        
        moveit::planning_interface::MoveItErrorCode execute_result = group.execute(plan);
        
        if (execute_result != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            ROS_WARN("点 %zu 执行失败，继续下一个点", i+1);
        }
        
        ros::Duration(0.05).sleep();
    }
    
    ROS_INFO("关节空间轨迹执行完成！");
    return true;
}

// 绕物体中心走圆弧轨迹
bool moveInCircularPath(moveit::planning_interface::MoveGroupInterface& group,
                       const geometry_msgs::Point& center_point,
                       double radius, 
                       double start_angle, 
                       double end_angle, 
                       int num_points,
                       bool clockwise,
                       bool maintain_orientation)
{
    ROS_INFO("开始规划圆弧轨迹...");
    ROS_INFO("物体中心: (%.3f, %.3f, %.3f)", 
             center_point.x, center_point.y, center_point.z);
    ROS_INFO("半径: %.3f米, 角度范围: %.1f° 到 %.1f°", 
             radius, start_angle*180.0/M_PI, end_angle*180.0/M_PI);
    
    std::vector<geometry_msgs::Pose> waypoints;
    
    // 计算角度步长
    double angle_step = (end_angle - start_angle) / (num_points - 1);
    if (!clockwise) {
        angle_step = -angle_step;
    }
    
    // 获取当前末端姿态
    geometry_msgs::PoseStamped current_pose = group.getCurrentPose("Link6");
    geometry_msgs::Quaternion current_orientation = current_pose.pose.orientation;
    
    // 生成圆弧上的点
    for (int i = 0; i < num_points; i++) {
        double angle = start_angle + i * angle_step;
        
        geometry_msgs::Pose target_pose;
        
        // 计算圆弧上的位置
        target_pose.position.x = center_point.x + radius * cos(angle);
        target_pose.position.y = center_point.y + radius * sin(angle);
        target_pose.position.z = center_point.z;  // 保持相同高度
        
        // 设置姿态
        if (maintain_orientation) {
            // 保持当前姿态
            target_pose.orientation = current_orientation;
        } else {
            // 计算指向物体中心的姿态
            tf2::Quaternion q;
            
            // 计算从当前位置指向物体中心的方向
            double dx = center_point.x - target_pose.position.x;
            double dy = center_point.y - target_pose.position.y;
            double dz = center_point.z - target_pose.position.z;
            
            // 计算偏航角（绕Z轴旋转）
            double yaw = atan2(dy, dx);
            
            // 计算俯仰角（绕Y轴旋转）
            double distance_xy = sqrt(dx*dx + dy*dy);
            double pitch = atan2(-dz, distance_xy);
            
            // 设置四元数
            q.setRPY(0, pitch, yaw);
            tf2::convert(q, target_pose.orientation);
        }
        
        waypoints.push_back(target_pose);
        
        ROS_INFO_NAMED("path_points", "点%d: 角度=%.1f°, 位置=(%.3f, %.3f, %.3f)", 
                       i, angle*180.0/M_PI,
                       target_pose.position.x,
                       target_pose.position.y,
                       target_pose.position.z);
    }
    
    // 使用笛卡尔路径规划
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    
    ROS_INFO("计算笛卡尔路径...");
    double fraction = group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    
    ROS_INFO("笛卡尔路径规划完成度: %.1f%%", fraction * 100.0);
    
    if (fraction < 0.8) {
        ROS_WARN("笛卡尔路径规划完成度较低，尝试关节空间规划...");
        return moveInCircularPathJointSpace(group, center_point, radius, 
                                          start_angle, end_angle, num_points, clockwise);
    }
    
    // 优化轨迹时间参数
    robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), "arm");
    rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory);
    
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    bool success = iptp.computeTimeStamps(rt, 0.1, 0.1);
    
    if (!success) {
        ROS_ERROR("轨迹时间参数化失败");
        return false;
    }
    
    // 创建并执行规划
    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
    rt.getRobotTrajectoryMsg(cartesian_plan.trajectory_);
    
    ROS_INFO("执行圆弧轨迹...");
    moveit::planning_interface::MoveItErrorCode execute_result = group.execute(cartesian_plan);
    
    if (execute_result == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        ROS_INFO("圆弧轨迹执行成功！");
        return true;
    } else {
        ROS_ERROR("圆弧轨迹执行失败！错误代码: %d", execute_result.val);
        return false;
    }
}

// 移动到安全位置
bool moveToSafePosition(moveit::planning_interface::MoveGroupInterface& group,
                       const geometry_msgs::Point& center_point,
                       double safe_distance)
{
    ROS_INFO("移动到安全位置...");
    
    geometry_msgs::PoseStamped current_pose = group.getCurrentPose("Link6");
    
    // 计算从物体中心指向当前点的方向
    double dx = current_pose.pose.position.x - center_point.x;
    double dy = current_pose.pose.position.y - center_point.y;
    double distance = sqrt(dx*dx + dy*dy);
    
    if (distance >= safe_distance) {
        ROS_INFO("已在安全距离外，不需要移动");
        return true;
    }
    
    // 计算安全位置
    double scale = safe_distance / distance;
    geometry_msgs::Pose safe_pose = current_pose.pose;
    safe_pose.position.x = center_point.x + dx * scale;
    safe_pose.position.y = center_point.y + dy * scale;
    
    group.setPoseTarget(safe_pose, "Link6");
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    
    if (success) {
        group.execute(plan);
        ROS_INFO("已移动到安全位置");
        return true;
    }
    
    return false;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rm65_circular_path_subscriber");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    // 获取参数
    double radius, angle_range;
    int num_points;
    private_nh.param("radius", radius, 0.15);
    private_nh.param("angle_range", angle_range, 2.0 * M_PI);
    private_nh.param("num_points", num_points, 30);
    
    // 创建异步spinner
    ros::AsyncSpinner spinner(2);  // 使用2个线程
    spinner.start();
    
    // 订阅物体中心话题
    ros::Subscriber center_sub = nh.subscribe<geometry_msgs::PointStamped>(
        "/object_center", 10, objectCenterCallback);
    
    // 订阅执行命令话题
    ros::Subscriber execute_sub = nh.subscribe<std_msgs::Bool>(
        "/execute_circular_path", 10, executeCommandCallback);
    
    // 等待MoveIt初始化
    ros::Duration(3.0).sleep();
    
    // 创建TF监听器
    tf2_ros::Buffer tf_buffer(ros::Duration(10));
    tf2_ros::TransformListener tf_listener(tf_buffer);
    
    // 初始化MoveIt
    moveit::planning_interface::MoveGroupInterface group("arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    
    // 设置规划参数
    group.setGoalPositionTolerance(0.01);
    group.setGoalOrientationTolerance(0.02);
    group.setMaxVelocityScalingFactor(0.3);
    group.setMaxAccelerationScalingFactor(0.3);
    
    ROS_INFO("机器人已初始化");
    ROS_INFO("等待接收物体中心位置...");
    ROS_INFO("发送命令: rostopic pub /execute_circular_path std_msgs/Bool \"data: true\"");
    ROS_INFO("轨迹参数: 半径=%.3fm, 点数=%d", radius, num_points);
    
    geometry_msgs::PointStamped current_center;
    bool has_valid_center = false;
    double safe_distance = 0.2;  // 安全距离
    
    // 主循环
    ros::Rate rate(10);  // 10Hz
    while (ros::ok()) {
        bool should_execute = false;
        
        // 检查是否有新的物体中心
        {
            std::lock_guard<std::mutex> lock(center_mutex);
            if (new_center_received) {
                current_center = object_center;
                new_center_received = false;
                has_valid_center = true;
                ROS_INFO("更新物体中心位置");
            }
            
            if (execute_circular_path) {
                should_execute = true;
                execute_circular_path = false;
            }
        }
        
        // 如果需要执行圆弧轨迹并且有有效的物体中心
        if (should_execute && has_valid_center) {
            ROS_INFO("开始执行圆弧轨迹...");
            
            // 转换坐标到基座坐标系
            geometry_msgs::PointStamped center_in_base = current_center;
            if (!transformToBaseFrame(center_in_base, group.getPlanningFrame(), tf_buffer)) {
                ROS_ERROR("无法转换物体中心到基座坐标系，使用原始坐标");
            }
            
            // 获取当前位置
            geometry_msgs::PoseStamped current_pose = group.getCurrentPose("Link6");
            
            // 计算当前位置到物体中心的距离
            double dx = current_pose.pose.position.x - center_in_base.point.x;
            double dy = current_pose.pose.position.y - center_in_base.point.y;
            double current_distance = sqrt(dx*dx + dy*dy);
            
            // 如果太近，先移动到安全距离
            if (current_distance < radius) {
                ROS_INFO("当前位置距离物体中心太近(%.3f米)，先移动到安全距离", current_distance);
                if (!moveToSafePosition(group, center_in_base.point, radius + 0.05)) {
                    ROS_WARN("移动到安全位置失败，尝试继续");
                }
                ros::Duration(1.0).sleep();
            }
            
            // 计算起始角度（从当前位置到物体中心的角度）
            double current_angle = atan2(current_pose.pose.position.y - center_in_base.point.y,
                                        current_pose.pose.position.x - center_in_base.point.x);
            
            // 执行圆弧轨迹
            bool success = moveInCircularPath(group, center_in_base.point,
                                            radius, current_angle,
                                            current_angle + angle_range,
                                            num_points, true, true);
            
            if (success) {
                ROS_INFO("圆弧轨迹执行完成！");
            } else {
                ROS_ERROR("圆弧轨迹执行失败");
            }
            
            // 短暂暂停
            ros::Duration(2.0).sleep();
            
            // 返回zero位置
            ROS_INFO("返回zero位置...");
            group.setNamedTarget("zero");
            group.move();
            
            ROS_INFO("任务完成，等待新的物体中心位置...");
        }
        
        rate.sleep();
    }
    
    spinner.stop();
    return 0;
}