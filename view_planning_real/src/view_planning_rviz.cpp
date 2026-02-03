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

// ============================================================
// ✅ 感知包络模型（新增，不依赖椭球）
// ============================================================
double effectiveTargetHeight(double theta)
{
    // 连续、非正弦、非对称
    return 0.30
         + 0.06 * cos(1.2 * theta)
         + 0.04 * sin(2.0 * theta)
         + 0.02 * cos(0.7 * theta);
}

// ============================================================
// ✅ 新版：感知 + FOV 驱动的扇形视点生成
//     （函数名、参数保持不变，主函数无需改）
// ============================================================
std::vector<geometry_msgs::Pose> generateFanViewpoints(
    const geometry_msgs::PointStamped& object_center_in_baselink,
    int num_viewpoints,
    double radius,
    double /* height 参数保留但不再直接使用 */,
    double angle_min,
    double angle_max)
{
    double x = object_center_in_baselink.point.x;
    double y = object_center_in_baselink.point.y;
    double z = object_center_in_baselink.point.z;

    // ✅ 相机垂直视场角（工程上合理的固定值）
    const double fov_vertical = 45.0 / 180.0 * M_PI;

    tf2::Vector3 obj_pos(x, y, z);
    tf2::Vector3 dir_obj_to_robot = -obj_pos;
    double base_angle = atan2(dir_obj_to_robot.y(), dir_obj_to_robot.x());

    std::vector<geometry_msgs::Pose> viewpoints;
    viewpoints.reserve(num_viewpoints);

    for (int i = 0; i < num_viewpoints; ++i)
    {
        double ratio =
            (num_viewpoints == 1)
            ? 0.5
            : double(i) / (num_viewpoints - 1);

        double theta =
            base_angle
          + angle_min
          + ratio * (angle_max - angle_min);

        // ✅ 感知包络 → 有效目标高度
        double h_eff = effectiveTargetHeight(theta);

        // ✅ FOV 约束 → 被动计算相机高度
        double cam_height =
            h_eff / tan(fov_vertical / 2.0) + 0.05;

        geometry_msgs::Pose vp;
        vp.position.x = x + radius * cos(theta);
        vp.position.y = y + radius * sin(theta);
        vp.position.z = z + cam_height;

        // ===== 姿态：始终看向目标点 =====
        tf2::Vector3 x_cam(
            x - vp.position.x,
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

    ros::Subscriber start_sub =
        nh.subscribe("/object_centor", 1, startCallback);
    ros::Subscriber stop_sub =
        nh.subscribe("/stop_loop", 1, stopCallback);

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

        auto viewpoints = generateFanViewpoints(
            object_center_in_baselink,
            5,                      // 视点数
            1.0,                    // 半径
            0.6,                    // 占位参数（不再直接使用）
           -10.0 / 180.0 * M_PI,
            10.0 / 180.0 * M_PI);

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