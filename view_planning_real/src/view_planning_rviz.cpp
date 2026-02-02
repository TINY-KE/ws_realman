#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PointStamped.h>

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

// ================= 全局变量 =================
bool loop = false;
geometry_msgs::PointStamped object_center_in_world;

// ================= 回调函数 =================
void startCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    object_center_in_world = *msg;
    loop = true;

    ROS_INFO("Received object center in frame [%s]: (%.2f, %.2f, %.2f)",
             msg->header.frame_id.c_str(),
             msg->point.x, msg->point.y, msg->point.z);
}

void stopCallback(const std_msgs::Bool::ConstPtr& msg)
{
    loop = false;
    ROS_INFO("Stop signal received.");
}

// ================= 坐标变换 =================
bool transformPoint(const tf2_ros::Buffer& tfBuffer,
                    const std::string& target_frame,
                    const geometry_msgs::PointStamped& input,
                    geometry_msgs::PointStamped& output)
{
    try {
        // 1. 查找从输入坐标系到目标坐标系的变换
        geometry_msgs::TransformStamped tf =
            tfBuffer.lookupTransform(
                target_frame,           // 目标坐标系
                input.header.frame_id,  // 源坐标系（从输入点获取）
                ros::Time(0),           // 查询最新变换
                ros::Duration(0.5));    // 超时0.5秒

        // 2. 应用变换
        tf2::doTransform(input, output, tf);
        return true;
    }
    catch (tf2::TransformException& ex) {
        ROS_WARN("TF transform failed: %s", ex.what());
        return false;
    }
}



std::vector<geometry_msgs::Pose>  generateFanViewpoints(
                            const geometry_msgs::PointStamped& object_center_in_baselink,
                            const int num_viewpoints,
                            const double radius,
                            const double height,
                            const double angle_min,
                            const double angle_max
                        )
{
    double x = object_center_in_baselink.point.x;
    double y = object_center_in_baselink.point.y;
    double z = object_center_in_baselink.point.z;

    // ================= 几何建模 =================
    tf2::Vector3 obj_pos_in_baselink(x, y, z);


    // ✅ 扇形朝向：物体 → 机器人
    tf2::Vector3 dir_obj_to_robot =  - obj_pos_in_baselink;  // dir_obj_to_robot：物体 → 机器人
    // base_angle：扇形中心方向
    double base_angle = atan2(dir_obj_to_robot.y(),
                                dir_obj_to_robot.x());

    std::vector<geometry_msgs::Pose> viewpoints;
    viewpoints.reserve(num_viewpoints);

    // ================= 生成视点 =================
    for (int i = 0; i < num_viewpoints; ++i) {
        double ratio = (num_viewpoints == 1)
                            ? 0.5
                            : double(i) / (num_viewpoints - 1);

        double theta = base_angle +
                        angle_min +
                        ratio * (angle_max - angle_min);

        geometry_msgs::Pose vp;

        // ✅ 以物体为圆心
        vp.position.x = x + radius * cos(theta);
        vp.position.y = y + radius * sin(theta);
        vp.position.z = z + height;

        // ✅ 姿态：X 轴指向物体
        tf2::Vector3 x_cam(
            x - vp.position.x,
            y - vp.position.y,
            z - vp.position.z
        );
        x_cam.normalize();

        tf2::Vector3 z_world(0, 0, 1);
        tf2::Vector3 y_cam = z_world.cross(x_cam).normalized();
        tf2::Vector3 z_cam = x_cam.cross(y_cam).normalized();

        tf2::Matrix3x3 rot(
            x_cam.x(), y_cam.x(), z_cam.x(),
            x_cam.y(), y_cam.y(), z_cam.y(),
            x_cam.z(), y_cam.z(), z_cam.z()
        );

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

    // TF
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    // RViz 可视化工具
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link_wheeltec");
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();

    // MOVEIT
    moveit::planning_interface::MoveGroupInterface group("arm");
    group.setPoseReferenceFrame("base_link_wheeltec");
    group.setGoalPositionTolerance(0.02);
    group.setGoalOrientationTolerance(0.05);
    group.setPlanningTime(3.0);
    group.setMaxVelocityScalingFactor(0.3);
    group.setMaxAccelerationScalingFactor(0.3);

    // 订阅
    ros::Subscriber start_sub =
        nh.subscribe("/object_centor", 1, startCallback);
    ros::Subscriber stop_sub =
        nh.subscribe("/stop_loop", 1, stopCallback);

    ros::Rate rate(10.0);

    ROS_INFO("Waiting for object center...");

    while (ros::ok()) {
        ros::spinOnce();

        if (!loop) {
            rate.sleep();
            continue;
        }

        // ================= 视点参数 =================
        const int num_viewpoints = 5;
        const double radius = 1.0;
        const double height = 0.6;
        const double angle_min = -10.0 / 180.0 * M_PI;
        const double angle_max =  10.0 / 180.0 * M_PI;
        
        // ================= 坐标变换：map → base_link =================
        geometry_msgs::PointStamped object_center_in_baselink;
        if (!transformPoint(tfBuffer, "base_link_wheeltec",
                             object_center_in_world, object_center_in_baselink)) {
            rate.sleep();
            continue;
        }

        

        

        std::vector<geometry_msgs::Pose> viewpoints = generateFanViewpoints(
            object_center_in_baselink,
            num_viewpoints,
            radius,
            height,
            angle_min,
            angle_max
        );

        

        // ================= RViz 显示 =================
        visual_tools.deleteAllMarkers();

        for (size_t i = 0; i < viewpoints.size(); ++i) {
            visual_tools.publishSphere(
                viewpoints[i].position,
                rviz_visual_tools::BLUE,
                rviz_visual_tools::MEDIUM,
                "viewpoint_" + std::to_string(i)
            );

            // ✅ 正确 API：publishAxisLabeled
            visual_tools.publishAxisLabeled(
                viewpoints[i],    // 位姿（位置+方向）
                "vp_" + std::to_string(i),       // 文字标签
                rviz_visual_tools::MEDIUM            // 轴的大小
            );
        }


        // ================= MOVEIT 移动 =================
        for (size_t i = 0; i < viewpoints.size(); ++i)
        {
            ROS_INFO_STREAM("===== Executing viewpoint " << i << " =====");

            group.clearPoseTargets();
            group.setPoseTarget(viewpoints[i], "Link6");

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            moveit::planning_interface::MoveItErrorCode success = group.plan(plan);

            if (success != moveit::planning_interface::MoveItErrorCode::SUCCESS)
            {
                ROS_WARN_STREAM("Planning failed for viewpoint " << i);
                continue;   // ✅ 跳过不可达视点
            }

            success = group.execute(plan);

            if (success != moveit::planning_interface::MoveItErrorCode::SUCCESS)
            {
                ROS_WARN_STREAM("Execution failed for viewpoint " << i);
                continue;
            }

            ROS_INFO_STREAM("Viewpoint " << i << " executed successfully");

            ros::Duration(0.4).sleep();  // ✅ 给传感器/相机时间
        }
        group.setNamedTarget("zero");
        moveit::planning_interface::MoveGroupInterface::Plan home_plan;
        if (group.plan(home_plan))
        {
            group.execute(home_plan);
        }

        visual_tools.trigger();
        rate.sleep();
    }

    return 0;
}