#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float64MultiArray.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <visualization_msgs/Marker.h>

// 引入你自定义的头文件
#include "Viewer.h"
#include "MapObject.h"
#include "GenerateArm.h"
#include "ConverterTools.h"
#include "Converter.h"
#include "Map.h"
#include <thread>

using namespace std;

// ================= 全局变量 =================
bool loop = false;
geometry_msgs::PointStamped object_center_in_world;
ros::Publisher marker_pub;

// ================= 回调函数 =================
void ellipsoidCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    if (msg->data.size() < 9) return;
    object_center_in_world.header.stamp = ros::Time::now();
    object_center_in_world.header.frame_id = "map";
    object_center_in_world.point.x = msg->data[0];
    object_center_in_world.point.y = msg->data[1];
    object_center_in_world.point.z = msg->data[2];

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "ellipsoid";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = object_center_in_world.point;
    marker.scale.x = msg->data[6]; marker.scale.y = msg->data[7]; marker.scale.z = msg->data[8];
    marker.color.r = 0.1f; marker.color.g = 0.8f; marker.color.b = 0.2f; marker.color.a = 0.8f;
    marker_pub.publish(marker);
    loop = true;
}

void stopCallback(const std_msgs::Bool::ConstPtr& msg) {
    loop = false;
    ROS_INFO("Stop signal received.");
}

// ================= 视点生成逻辑：末端X轴指向物体 =================
std::vector<geometry_msgs::Pose> generateScanningViewpoints(const geometry_msgs::PointStamped& obj, int num, double h) {
    std::vector<geometry_msgs::Pose> viewpoints;
    double sweep_y = 0.20; // 左右 20cm
    double forward_x = 0.10; // 向前 10cm
    for (int i = 0; i < num; ++i) {
        double ratio = (num == 1) ? 0.5 : (double)i / (num - 1);
        geometry_msgs::Pose vp;
        vp.position.x = forward_x;
        vp.position.y = -sweep_y + (ratio * 2.0 * sweep_y);
        vp.position.z = h;

        tf2::Vector3 cam_pos(vp.position.x, vp.position.y, vp.position.z);
        tf2::Vector3 target_pos(obj.point.x, obj.point.y, obj.point.z);
        tf2::Vector3 x_axis = (target_pos - cam_pos).normalized(); // Link6 X轴指向物体
        tf2::Vector3 z_world(0, 0, 1);
        tf2::Vector3 y_axis = z_world.cross(x_axis).normalized();
        tf2::Vector3 z_axis = x_axis.cross(y_axis).normalized();

        tf2::Matrix3x3 rot(x_axis.x(), y_axis.x(), z_axis.x(),
                           x_axis.y(), y_axis.y(), z_axis.y(),
                           x_axis.z(), y_axis.z(), z_axis.z());
        tf2::Quaternion q;
        rot.getRotation(q);
        vp.orientation = tf2::toMsg(q);
        viewpoints.push_back(vp);
    }
    return viewpoints;
}

// ================= 主函数 =================
int main(int argc, char** argv) {
    ros::init(argc, argv, "object_centered_viewpoint_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // 1. 初始化机械臂与相机参数
    ArmModel *arm_model = generateArm("realman65");
    int CameraWidth = 640; int CameraHeight = 480;
    float fx = 554.254691191187; float fy = 554.254691191187;
    float cx = 320.5; float cy = 240.5;
    Eigen::Matrix3d Calib;
    Calib << fx, 0, cx, 0, fy, cy, 0, 0, 1;

    int FovDecrease = 20;
    if(argc > 1) FovDecrease = atoi(argv[1]);
    double FOVDepth = 3.5;

    // 2. 启动地图线程
    ObjectMap *map = new ObjectMap(nh);
    std::thread mptMap(&ObjectMap::Run, map);

    // 3. TF 与 MoveIt 初始化
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link_wheeltec");
    visual_tools.deleteAllMarkers();

    moveit::planning_interface::MoveGroupInterface group("arm");
    group.setPoseReferenceFrame("base_link_wheeltec");
    group.setGoalPositionTolerance(0.02);
    group.setGoalOrientationTolerance(0.05);
    group.setPlanningTime(1.5);
    group.setNumPlanningAttempts(1);
    group.setMaxVelocityScalingFactor(0.3);
    group.setMaxAccelerationScalingFactor(0.3);

    // 4. 启动可视化线程
    string default_frame = "base_link";
    Visualize_Arm_Tools vis_arm_tools(nh, *arm_model, group, CameraWidth, CameraHeight, Calib, default_frame);
    vis_arm_tools.setFOVDecrease(FovDecrease);
    vis_arm_tools.setFOVDepth(FOVDepth);
    std::thread mptVisualizeArmTools(&Visualize_Arm_Tools::Run, &vis_arm_tools);

    // 5. 订阅与发布
    ros::Subscriber ellipsoid_sub = nh.subscribe("/object_ellipsoid", 1, ellipsoidCallback);
    ros::Subscriber stop_sub = nh.subscribe("/stop_loop", 1, stopCallback);
    marker_pub = nh.advertise<visualization_msgs::Marker>("ellipsoid_marker", 1);

    ros::Rate rate(10.0);
    ROS_INFO("Waiting for object center...");

    while (ros::ok()) {
        if (!loop) {
            rate.sleep();
            continue;
        }

        geometry_msgs::PointStamped obj_in_base;
        try {
            geometry_msgs::TransformStamped ts = tfBuffer.lookupTransform("base_link_wheeltec", "map", ros::Time(0), ros::Duration(1.0));
            tf2::doTransform(object_center_in_world, obj_in_base, ts);
        } catch (...) { continue; }

        // 生成 5 个视点，高度设为 1.1m (0.7 + 0.4)
        auto viewpoints = generateScanningViewpoints(obj_in_base, 5, 1.1);

        for (size_t i = 0; i < viewpoints.size(); ++i) {
            if (!loop) break;
            
            // --- 你要求的 MoveIt 规划与执行判断逻辑 ---
            group.setStartState(*group.getCurrentState());
            group.clearPoseTargets();
            group.setPoseTarget(viewpoints[i], "Link6");

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            auto success = group.plan(plan);

            if (success != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
                ROS_WARN_STREAM("Planning failed for viewpoint " << i);
                continue;
            }

            success = group.execute(plan);
            if (success != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
                ROS_WARN_STREAM("Execution failed for viewpoint " << i);
                continue;
            }
            ros::Duration(0.5).sleep();
        }

        // 任务结束：回零点并停止本次循环
        group.setNamedTarget("zero");
        group.move();
        loop = false;
        ROS_INFO("Scanning complete. Waiting for next object...");
    }

    mptMap.join();
    mptVisualizeArmTools.join();
    return 0;
}