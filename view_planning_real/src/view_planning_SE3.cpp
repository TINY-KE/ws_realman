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
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <visualization_msgs/Marker.h>

// 视场可视化
#include "Viewer.h"
#include "MapObject.h"
#include "GenerateArm.h"
#include "ConverterTools.h"
#include "Converter.h"
#include "Map.h"
#include <thread>



// ================= 全局变量 =================
bool loop = false;
geometry_msgs::PointStamped object_center_in_world;
std_msgs::Float64MultiArray ellipsoid_in_world;
ros::Publisher marker_pub;


// ================= 回调函数 =================
void ellipsoidCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    if (msg->data.size() < 9)
    {
        ROS_WARN("Ellipsoid array size < 9");
        return;
    }

    // ✅ 保存椭球数组
    ellipsoid_in_world = *msg;

    // ✅ 提取中心点
    object_center_in_world.header.stamp = ros::Time::now();
    object_center_in_world.header.frame_id = "map";
    object_center_in_world.point.x = msg->data[0];
    object_center_in_world.point.y = msg->data[1];
    object_center_in_world.point.z = msg->data[2];

    // ✅ 构造 RViz 椭球 Marker
    visualization_msgs::Marker ellipsoid_marker;
    ellipsoid_marker.header.frame_id = "map";
    ellipsoid_marker.header.stamp = ros::Time::now();
    ellipsoid_marker.ns = "ellipsoid";
    ellipsoid_marker.id = 0;
    ellipsoid_marker.type = visualization_msgs::Marker::SPHERE;
    ellipsoid_marker.action = visualization_msgs::Marker::ADD;

    // 位置
    ellipsoid_marker.pose.position.x = msg->data[0];
    ellipsoid_marker.pose.position.y = msg->data[1];
    ellipsoid_marker.pose.position.z = msg->data[2];

    // 姿态（暂时只用 yaw，roll/pitch 以后可加）
    tf2::Quaternion q;
    q.setRPY(msg->data[3], msg->data[4], msg->data[5]);
    ellipsoid_marker.pose.orientation = tf2::toMsg(q);

    // ✅ 椭球尺寸（scale = 长宽高）
    ellipsoid_marker.scale.x = msg->data[6];
    ellipsoid_marker.scale.y = msg->data[7];
    ellipsoid_marker.scale.z = msg->data[8];

    // ✅ 颜色 & 透明度
    ellipsoid_marker.color.r = 0.1f;
    ellipsoid_marker.color.g = 0.8f;
    ellipsoid_marker.color.b = 0.2f;
    ellipsoid_marker.color.a = 1.0f;   // 半透明

    ellipsoid_marker.lifetime = ros::Duration(0); // 一直存在

    marker_pub.publish(ellipsoid_marker);

    loop = true;

    ROS_INFO("Ellipsoid received: "
             "pos(%.2f %.2f %.2f), size(%.2f %.2f %.2f)",
             msg->data[0], msg->data[1], msg->data[2],
             msg->data[6], msg->data[7], msg->data[8]);
}

void stopCallback(const std_msgs::Bool::ConstPtr& msg)
{
    loop = false;
    ROS_INFO("Stop signal received.");
}

// ================= 坐标变换（保持你原来的写法） =================
bool transformPoint(const tf2_ros::Buffer& tfBuffer,
                    const std::string& target_frame,
                    const geometry_msgs::PointStamped& input,
                    geometry_msgs::PointStamped& output)
{
    try {
        geometry_msgs::TransformStamped tf =
            tfBuffer.lookupTransform(
                target_frame,
                input.header.frame_id,
                ros::Time(0),
                ros::Duration(0.5));

        tf2::doTransform(input, output, tf);
        return true;
    }
    catch (tf2::TransformException& ex) {
        ROS_WARN("TF transform failed: %s", ex.what());
        return false;
    }
}

// ================= 生成扇形视点 =================
std::vector<geometry_msgs::Pose> generateFanViewpoints(
    const geometry_msgs::PointStamped& object_center_in_baselink,
    int num_viewpoints,
    double radius,
    double height,
    double angle_min,
    double angle_max)
{
    double x = object_center_in_baselink.point.x;
    double y = object_center_in_baselink.point.y;
    double z = object_center_in_baselink.point.z;

    tf2::Vector3 obj_pos(x, y, z);
    tf2::Vector3 dir_obj_to_robot = -obj_pos;
    double base_angle = atan2(dir_obj_to_robot.y(), dir_obj_to_robot.x());

    std::vector<geometry_msgs::Pose> viewpoints;
    viewpoints.reserve(num_viewpoints);

    for (int i = 0; i < num_viewpoints; ++i) {
        double ratio = (num_viewpoints == 1) ? 0.5 : double(i) / (num_viewpoints - 1);
        double theta = base_angle + angle_min + ratio * (angle_max - angle_min);

        geometry_msgs::Pose vp;
        vp.position.x = x + radius * cos(theta);
        vp.position.y = y + radius * sin(theta);
        vp.position.z = z + height;

        tf2::Vector3 x_cam(x - vp.position.x,
                           y - vp.position.y,
                           z - vp.position.z);
        x_cam.normalize();

        tf2::Vector3 z_world(0, 0, 1);
        tf2::Vector3 y_cam = z_world.cross(x_cam).normalized();
        tf2::Vector3 z_cam = x_cam.cross(y_cam).normalized();

        tf2::Matrix3x3 rot(
            x_cam.x(), y_cam.x(), z_cam.x(),
            x_cam.y(), y_cam.y(), z_cam.y(),
            x_cam.z(), y_cam.z(), z_cam.z());

        tf2::Quaternion q;
        rot.getRotation(q);
        vp.orientation = tf2::toMsg(q);

        viewpoints.push_back(vp);
    }
    return viewpoints;
}

// ================= 主函数 =================
int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_centered_viewpoint_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();



    //二、生成机械臂、相机参数
    //（1）生成机械臂
    ArmModel *arm_model = generateArm("realman65");
    //（2）生成相机参数
    int CameraWidth = 640;
    int CameraHeight = 480;
    float fx = 554.254691191187;
    float fy = 554.254691191187;
    float cx = 320.5;
    float cy = 240.5;
    Eigen::Matrix3d Calib;
    Calib << fx, 0, cx,
            0, fy, cy,
            0, 0, 1;
    // (3) 规划轨迹中差值的数量
    int circle_divides = 240;
    // （4）最佳视场的横向减小值
    int FovDecrease = 20;    //这里可能得设置为145.因为之前的程序一直没设置成功 
    if(argc>1)
        FovDecrease = atoi(argv[1]);
    // int FovDecrease = 20;  //为了可视化效果好，减小   
    double FOVDepth = 4.0; // 1.0用于截图， 6.0用于建图
    // 地图
    ObjectMap *map = new ObjectMap(nh);
    std::thread *mptMap;
    mptMap = new std::thread(&ObjectMap::Run, map);

    


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


    // 三、可视化线程
    string default_frame = "base_link";

    // Visualize_Tools *vis_tools = new Visualize_Tools(nh, map, default_frame);
    // std::thread *mptVisualizeTools;
    // mptVisualizeTools = new std::thread(&Visualize_Tools::Run, vis_tools);

    Visualize_Arm_Tools vis_arm_tools(nh, *arm_model, group, CameraWidth, CameraHeight, Calib, default_frame);
    vis_arm_tools.setFOVDecrease(FovDecrease);
    vis_arm_tools.setFOVDepth(FOVDepth);
    std::thread *mptVisualizeArmTools;
    mptVisualizeArmTools = new std::thread(&Visualize_Arm_Tools::Run, vis_arm_tools);







    ros::Subscriber ellipsoid_sub =
        nh.subscribe("/object_ellipsoid", 1, ellipsoidCallback);
    ros::Subscriber stop_sub =
        nh.subscribe("/stop_loop", 1, stopCallback);
    marker_pub =  nh.advertise<visualization_msgs::Marker>( "ellipsoid_marker", 1);

    ros::Rate rate(10.0);
    ROS_INFO("Waiting for object center...");

    while (ros::ok())
    {
        ros::spinOnce();
        if (!loop) {
            rate.sleep();
            continue;
        }

        geometry_msgs::PointStamped object_center_in_baselink;
        if (!transformPoint(tfBuffer,
                             "base_link_wheeltec",
                             object_center_in_world,
                             object_center_in_baselink))
        {
            rate.sleep();
            continue;
        }
        // ================= 视点参数 =================
        const int num_viewpoints = 5;
        const double radius = 1.0;
        const double height = 0.6;
        const double angle_min = -10.0 / 180.0 * M_PI;
        const double angle_max =  10.0 / 180.0 * M_PI;
        // 生成视点  
        std::vector<geometry_msgs::Pose> viewpoints = generateFanViewpoints(
            object_center_in_baselink,
            num_viewpoints,
            radius,
            height,
            angle_min,
            angle_max
        );


        visual_tools.deleteAllMarkers();

        for (size_t i = 0; i < viewpoints.size(); ++i)
        {
            visual_tools.publishSphere(
                viewpoints[i].position,
                rviz_visual_tools::BLUE,
                rviz_visual_tools::MEDIUM,
                "vp_sphere_" + std::to_string(i));

            visual_tools.publishAxisLabeled(
                viewpoints[i],
                "vp_" + std::to_string(i));
        }

        // ================= MoveIt 执行 =================
        for (size_t i = 0; i < viewpoints.size(); ++i)
        {
            ROS_INFO_STREAM("===== Executing viewpoint " << i << " =====");

            group.setStartState(*group.getCurrentState());
            group.clearPoseTargets();
            group.setPoseTarget(viewpoints[i], "Link6");

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            auto success = group.plan(plan);

            if (success != moveit::planning_interface::MoveItErrorCode::SUCCESS)
            {
                ROS_WARN_STREAM("Planning failed for viewpoint " << i);
                continue;
            }

            success = group.execute(plan);
            if (success != moveit::planning_interface::MoveItErrorCode::SUCCESS)
            {
                ROS_WARN_STREAM("Execution failed for viewpoint " << i);
                continue;
            }

            ros::Duration(0.4).sleep();
        }

        // 回零
        group.setStartState(*group.getCurrentState());
        group.setNamedTarget("zero");
        moveit::planning_interface::MoveGroupInterface::Plan home_plan;
        if (group.plan(home_plan))
            group.execute(home_plan);

        visual_tools.trigger();
        rate.sleep();
    }

    return 0;
}