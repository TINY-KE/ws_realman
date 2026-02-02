#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Geometry>

// 全局变量
geometry_msgs::PointStamped object_center;
bool visualize = false;

// 回调函数
void objectCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    object_center = *msg;
    visualize = true;
    ROS_INFO("Got object: (%.2f, %.2f, %.2f)", msg->point.x, msg->point.y, msg->point.z);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_viewpoint");
    ros::NodeHandle nh;
    
    // 可视化工具
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.deleteAllMarkers();
    
    // TF
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    
    // 订阅
    ros::Subscriber sub = nh.subscribe("/object_center", 1, objectCallback);
    
    ros::Rate rate(10);
    
    while (ros::ok()) {
        ros::spinOnce();
        
        if (visualize) {
            // 转换到base_link
            geometry_msgs::PointStamped target;
            try {
                tf_buffer.transform(object_center, target, "base_link", ros::Duration(1.0));
            } catch (...) {
                target = object_center;
            }
            
            // 生成5个视点
            std::vector<geometry_msgs::Pose> viewpoints;
            int n = 5;
            double radius = 0.6;
            double height = 0.3;
            
            for (int i = 0; i < n; i++) {
                double angle = 2.0 * M_PI * i / n;
                
                geometry_msgs::Pose vp;
                vp.position.x = target.point.x + radius * cos(angle);
                vp.position.y = target.point.y + radius * sin(angle);
                vp.position.z = target.point.z + height;
                
                // 朝向物体
                Eigen::Vector3d forward(
                    target.point.x - vp.position.x,
                    target.point.y - vp.position.y,
                    target.point.z - vp.position.z
                );
                forward.normalize();
                
                Eigen::Vector3d up(0,0,1);
                Eigen::Vector3d right = up.cross(forward).normalized();
                up = forward.cross(right).normalized();
                
                Eigen::Matrix3d rot;
                rot.col(0) = right;
                rot.col(1) = up;
                rot.col(2) = forward;
                
                Eigen::Quaterniond q(rot);
                vp.orientation.x = q.x();
                vp.orientation.y = q.y();
                vp.orientation.z = q.z();
                vp.orientation.w = q.w();
                
                viewpoints.push_back(vp);
            }
            
            // 可视化
            visual_tools.deleteAllMarkers();
            
            // 目标
            visual_tools.publishSphere(target.point, rviz_visual_tools::RED, 0.05, "target");
            
            // 视点
            for (size_t i = 0; i < viewpoints.size(); i++) {
                visual_tools.publishSphere(viewpoints[i].position, rviz_visual_tools::BLUE, 0.03, "vp_" + std::to_string(i));
                visual_tools.publishAxis(viewpoints[i], 0.1, "axis_" + std::to_string(i));
            }
            
            visual_tools.trigger();
            visualize = false;
        }
        
        rate.sleep();
    }
    
    return 0;
}