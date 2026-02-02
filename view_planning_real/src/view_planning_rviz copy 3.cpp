#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PointStamped.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// ================= 全局变量 =================
bool loop = false;
geometry_msgs::PointStamped object_center;

// ================= 回调函数 =================
void startCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    object_center = *msg;
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

// ================= 坐标变换工具 =================
bool transformPoint(const tf2_ros::Buffer& tfBuffer,
                    const std::string& target_frame,
                    const geometry_msgs::PointStamped& input,
                    geometry_msgs::PointStamped& output)
{
    try {
        geometry_msgs::TransformStamped tf =
            tfBuffer.lookupTransform(target_frame,
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

    // RViz 可视化
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link_wheeltec");
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();

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

        // ================= 坐标转换：map → base_link =================
        geometry_msgs::PointStamped obj_bl;
        if (!transformPoint(tfBuffer, "base_link_wheeltec",
                             object_center, obj_bl)) {
            rate.sleep();
            continue;
        }

        double x = obj_bl.point.x;
        double y = obj_bl.point.y;
        double z = obj_bl.point.z;

        // ================= 几何建模 =================
        tf2::Vector3 robot_pos(0, 0, 0);        // base_link
        tf2::Vector3 obj_pos(x, y, z);

        // ✅ 关键：物体 → 机器人方向（决定扇形朝向）
        tf2::Vector3 dir_obj_to_robot = robot_pos - obj_pos;
        double base_angle = atan2(dir_obj_to_robot.y(),
                                  dir_obj_to_robot.x());

        // ================= 视点参数 =================
        const int num_viewpoints = 5;
        const double radius = 0.6;
        const double height = 0.5;
        const double angle_min = -M_PI / 3;   // -60°
        const double angle_max =  M_PI / 3;   // +60°

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

            // ✅ 圆心在“物体”
            vp.position.x = x + radius * cos(theta);
            vp.position.y = y + radius * sin(theta);
            vp.position.z = z + height;

            // ✅ 姿态始终朝向物体
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

        // ================= RViz 显示 =================
        visual_tools.deleteAllMarkers();

        for (size_t i = 0; i < viewpoints.size(); ++i) {
            visual_tools.publishSphere(
                viewpoints[i].position,
                rviz_visual_tools::BLUE,
                rviz_visual_tools::SMALL,
                "viewpoint_" + std::to_string(i)
            );

            visual_tools.publishAxisLabeled(
                viewpoints[i],
                "vp_" + std::to_string(i),
                rviz_visual_tools::SMALL
            );
        }

        visual_tools.trigger();
        rate.sleep();
    }

    return 0;
}