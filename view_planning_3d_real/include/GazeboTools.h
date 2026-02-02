//
// Created by robotlab on 24-11-5.
//

#ifndef GAZEBOTOOLS_H
#define GAZEBOTOOLS_H

#include <ros/ros.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/GetLinkState.h>
#include <gazebo_msgs/GetWorldProperties.h>
#include <gazebo_msgs/GetModelProperties.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "std_msgs/Float64MultiArray.h"
#include "MapObject.h"
#include "ConverterTools.h"



geometry_msgs::Pose  get_link_pose(ros::NodeHandle& n, std::string link_name, std::string reference_name = "world", bool output = false){
    ros::ServiceClient get_link_sate_client = n.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
    gazebo_msgs::GetLinkState srv;
    srv.request.link_name = link_name;
    srv.request.reference_frame = reference_name;
    geometry_msgs::Pose pose;
    if (get_link_sate_client.call(srv)) {
        // auto position = srv.response.link_state.pose.position;
        // auto orientation = srv.response.link_state.pose.orientation;
        // ROS_INFO("Position: x: %f, y: %f, z: %f", position.x, position.y, position.z);
        // ROS_INFO("Orientation: x: %f, y: %f, z: %f, w: %f", orientation.x, orientation.y, orientation.z, orientation.w);
        pose = srv.response.link_state.pose;
        if(output){
         ROS_INFO("Pose of [%s]: position(x: %f, y: %f, z: %f), orientation(x: %f, y: %f, z: %f, w: %f)",
             link_name.c_str(),
             pose.position.x, pose.position.y, pose.position.z,
             pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w);
        }
        return pose;

    } else {
        ROS_ERROR("Failed to call service get_link_state");
        return pose;
    }
}

void get_all_models_and_links_name(ros::NodeHandle& n){
    ros::ServiceClient get_world_properties_client = n.serviceClient<gazebo_msgs::GetWorldProperties>("/gazebo/get_world_properties");
    ros::ServiceClient get_model_properties_client = n.serviceClient<gazebo_msgs::GetModelProperties>("/gazebo/get_model_properties");

    ros::service::waitForService("/gazebo/get_world_properties");

    gazebo_msgs::GetWorldProperties world_srv;
    if (get_world_properties_client.call(world_srv)) {
        for (const auto& model_name : world_srv.response.model_names) {
            gazebo_msgs::GetModelProperties model_srv;
            model_srv.request.model_name = model_name;
            if (get_model_properties_client.call(model_srv)) {
                ROS_INFO("Model: %s", model_name.c_str());
                for (const auto& link_name : model_srv.response.body_names) {
                    ROS_INFO("  Link: %s", link_name.c_str());
                }
            } else {
                ROS_ERROR("Failed to get properties for model: %s", model_name.c_str());
            }
        }
    } else {
        ROS_ERROR("Failed to call service get_world_properties");
    }
}
geometry_msgs::Pose getPose(ros::NodeHandle& n, std::string name, bool output = false)
{
    ros::ServiceClient get_model_state_client = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    geometry_msgs::Pose pose;
    gazebo_msgs::GetModelState srv;
    srv.request.model_name = name;
    get_model_state_client.call(srv);
    if(output){
         ROS_INFO("Pose of [%s]: position(x: %f, y: %f, z: %f), orientation(x: %f, y: %f, z: %f, w: %f)",
             name.c_str(),
             srv.response.pose.position.x, srv.response.pose.position.y, srv.response.pose.position.z,
             srv.response.pose.orientation.x, srv.response.pose.orientation.y,
             srv.response.pose.orientation.z, srv.response.pose.orientation.w);
    }
    return srv.response.pose;
}

void setPose(ros::NodeHandle& n, std::string name, double x, double y, double z, tfScalar qw, tfScalar qx, tfScalar qy, tfScalar qz)
{
    ros::ServiceClient set_model_state_client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    gazebo_msgs::SetModelState object_state;

    object_state.request.model_state.model_name = name;


    object_state.request.model_state.pose.position.x = x;
    object_state.request.model_state.pose.position.y = y;
    object_state.request.model_state.pose.position.z = z;

    object_state.request.model_state.pose.orientation.w = qw;
    object_state.request.model_state.pose.orientation.x = qx;
    object_state.request.model_state.pose.orientation.y = qy;
    object_state.request.model_state.pose.orientation.z = qz;

    object_state.request.model_state.twist.linear.x = 0.0;
    object_state.request.model_state.twist.linear.y = 0.0;
    object_state.request.model_state.twist.linear.z = 0.0;
    object_state.request.model_state.twist.angular.x = 0.0;
    object_state.request.model_state.twist.angular.y = 0.0;
    object_state.request.model_state.twist.angular.z = 0.0;

    object_state.request.model_state.reference_frame = "world";

    set_model_state_client.call(object_state);
}

geometry_msgs::TransformStamped getTFTransform(std::string parent_name, std::string source_name, tf::Transform& out) {
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    geometry_msgs::TransformStamped transform_stamped;

    transform_stamped = tf_buffer.lookupTransform(parent_name, source_name, ros::Time(0), ros::Duration(1.0));

    // 将 TransformStamped 转换为 PoseStamped
    geometry_msgs::PoseStamped end_pose;
    end_pose.header.stamp = transform_stamped.header.stamp;
    end_pose.header.frame_id = transform_stamped.header.frame_id;

    // 设置位置
    end_pose.pose.position.x = transform_stamped.transform.translation.x;
    end_pose.pose.position.y = transform_stamped.transform.translation.y;
    end_pose.pose.position.z = transform_stamped.transform.translation.z;

    // 设置方向（四元数）
    end_pose.pose.orientation = transform_stamped.transform.rotation;

    // // 输出相机在 odom 坐标系下的位姿
    // ROS_INFO_STREAM("Pose of camera_rgb_frame in odom: \n" << end_pose);
    return transform_stamped;
}




#include <tf/transform_datatypes.h>

void rotate_90degrees_left(ros::NodeHandle &n, moveit::planning_interface::MoveGroupInterface &move_group,
                      geometry_msgs::Pose pose, bool first_stage, double rotate_divides = 30) {
    for(int i=0; i<rotate_divides; i++)
    {

        if (first_stage) {
            cout << "Rotate left 90 degrees" << endl;

            double angle_radians = M_PI_2 / rotate_divides;

            // 获取原来的姿态（四元数）
            tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

            // 将四元数转换为欧拉角
            // 将四元数转换为旋转矩阵
            tf::Matrix3x3 m(q);

            // 提取欧拉角（Yaw）
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            // 逆时针旋转
            yaw -= M_PI_2 - angle_radians*i;

            // 将新的欧拉角转换回四元数
            tf::Quaternion new_q = tf::createQuaternionFromRPY(roll, pitch, yaw);

            // 创建新的 pose
            geometry_msgs::Pose pose_new = pose;
            pose_new.orientation.x = new_q.x();
            pose_new.orientation.y = new_q.y();
            pose_new.orientation.z = new_q.z();
            pose_new.orientation.w = new_q.w();

            setPose(n, "mrobot", pose_new.position.x, pose_new.position.y, pose_new.position.z,
                    pose_new.orientation.w, pose_new.orientation.x, pose_new.orientation.y,
                    pose_new.orientation.z);

            std::vector<double> target_joint_group_positions = {0-angle_radians*i, 0, 0, 0.17, 0, 0, 0};;
            // std::cout<<" target_joint_group_positions:";
            // std::copy(target_joint_group_positions.begin(), target_joint_group_positions.end(), std::ostream_iterator<double>(std::cout, " "));
            // std::cout<<std::endl;
            move_group.setJointValueTarget(target_joint_group_positions);
            // plan 和 move
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if (success) {
                std::cout << "Rotate left 90 degrees -- Joint规划成功" << std::endl;
                move_group.execute(my_plan);
            } else
                std::cout << "Rotate left 90 degrees -- Joint规划失败" << std::endl;

        } else {
            cout << "Rotate right 90 degrees" << endl;

            double angle_radians = M_PI_2 / rotate_divides;

            // 获取原来的姿态（四元数）
            tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

            // 将四元数转换为欧拉角
            // 将四元数转换为旋转矩阵
            tf::Matrix3x3 m(q);

            // 提取欧拉角（Yaw）
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            // 逆时针旋转
            yaw += -1 * angle_radians*i;

            // 将新的欧拉角转换回四元数
            tf::Quaternion new_q = tf::createQuaternionFromRPY(roll, pitch, yaw);

            // 创建新的 pose
            geometry_msgs::Pose pose_new = pose;
            pose_new.orientation.x = new_q.x();
            pose_new.orientation.y = new_q.y();
            pose_new.orientation.z = new_q.z();
            pose_new.orientation.w = new_q.w();

            setPose(n, "mrobot", pose_new.position.x, pose_new.position.y, pose_new.position.z,
                    pose_new.orientation.w, pose_new.orientation.x, pose_new.orientation.y,
                    pose_new.orientation.z);
            std::vector<double> target_joint_group_positions = {angle_radians*i-M_PI_2, 0, 0, 0.17, 0, 0, 0};;
            move_group.setJointValueTarget(target_joint_group_positions);
            // std::cout<<" target_joint_group_positions:";
            // std::copy(target_joint_group_positions.begin(), target_joint_group_positions.end(), std::ostream_iterator<double>(std::cout, " "));
            // std::cout<<std::endl;
            move_group.setJointValueTarget(target_joint_group_positions);
            // plan 和 move
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if (success) {
                std::cout << "Rotate right 90 degrees -- Joint规划成功" << std::endl;
                move_group.execute(my_plan);
            } else
                std::cout << "Rotate right 90 degrees -- Joint规划失败" << std::endl;

        }

        // std::cout << "Press [any key] to continue ... " << std::endl;
        // std::cout << "*****************************" << std::endl;
        // char key = getchar();

    }
}

void rotate_90degrees_right(ros::NodeHandle &n, moveit::planning_interface::MoveGroupInterface &move_group,
                      geometry_msgs::Pose pose, bool first_stage, double rotate_divides = 30) {
    for(int i=0; i<rotate_divides; i++)
    {

        if (first_stage) {
            cout << "Rotate left 90 degrees" << endl;

            double angle_radians = M_PI_2 / rotate_divides;

            // 获取原来的姿态（四元数）
            tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

            // 将四元数转换为欧拉角
            // 将四元数转换为旋转矩阵
            tf::Matrix3x3 m(q);

            // 提取欧拉角（Yaw）
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            // 逆时针旋转
            yaw += M_PI_2 - angle_radians*i;

            // 将新的欧拉角转换回四元数
            tf::Quaternion new_q = tf::createQuaternionFromRPY(roll, pitch, yaw);

            // 创建新的 pose
            geometry_msgs::Pose pose_new = pose;
            pose_new.orientation.x = new_q.x();
            pose_new.orientation.y = new_q.y();
            pose_new.orientation.z = new_q.z();
            pose_new.orientation.w = new_q.w();

            setPose(n, "mrobot", pose_new.position.x, pose_new.position.y, pose_new.position.z,
                    pose_new.orientation.w, pose_new.orientation.x, pose_new.orientation.y,
                    pose_new.orientation.z);

            std::vector<double> target_joint_group_positions = {angle_radians*i, 0, 0, 0.17, 0, 0, 0};;
            // std::cout<<" target_joint_group_positions:";
            // std::copy(target_joint_group_positions.begin(), target_joint_group_positions.end(), std::ostream_iterator<double>(std::cout, " "));
            // std::cout<<std::endl;
            move_group.setJointValueTarget(target_joint_group_positions);
            // plan 和 move
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if (success) {
                std::cout << "Rotate left 90 degrees -- Joint规划成功" << std::endl;
                move_group.execute(my_plan);
            } else
                std::cout << "Rotate left 90 degrees -- Joint规划失败" << std::endl;

        } else {
            cout << "Rotate right 90 degrees" << endl;

            double angle_radians = M_PI_2 / rotate_divides;

            // 获取原来的姿态（四元数）
            tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

            // 将四元数转换为欧拉角
            // 将四元数转换为旋转矩阵
            tf::Matrix3x3 m(q);

            // 提取欧拉角（Yaw）
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            // 逆时针旋转
            yaw += angle_radians*i;

            // 将新的欧拉角转换回四元数
            tf::Quaternion new_q = tf::createQuaternionFromRPY(roll, pitch, yaw);

            // 创建新的 pose
            geometry_msgs::Pose pose_new = pose;
            pose_new.orientation.x = new_q.x();
            pose_new.orientation.y = new_q.y();
            pose_new.orientation.z = new_q.z();
            pose_new.orientation.w = new_q.w();

            setPose(n, "mrobot", pose_new.position.x, pose_new.position.y, pose_new.position.z,
                    pose_new.orientation.w, pose_new.orientation.x, pose_new.orientation.y,
                    pose_new.orientation.z);
            std::vector<double> target_joint_group_positions = {M_PI_2-angle_radians*i, 0, 0, 0.17, 0, 0, 0};;
            move_group.setJointValueTarget(target_joint_group_positions);
            // std::cout<<" target_joint_group_positions:";
            // std::copy(target_joint_group_positions.begin(), target_joint_group_positions.end(), std::ostream_iterator<double>(std::cout, " "));
            // std::cout<<std::endl;
            move_group.setJointValueTarget(target_joint_group_positions);
            // plan 和 move
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if (success) {
                std::cout << "Rotate right 90 degrees -- Joint规划成功" << std::endl;
                move_group.execute(my_plan);
            } else
                std::cout << "Rotate right 90 degrees -- Joint规划失败" << std::endl;

        }

        // std::cout << "Press [any key] to continue ... " << std::endl;
        // std::cout << "*****************************" << std::endl;
        // char key = getchar();

    }
}

#endif //GAZEBOTOOLS_H
