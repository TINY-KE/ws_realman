#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <vector>
#include <fstream>
#include <iomanip>
#include <string>
#include <iostream>

// 全局变量（移除TransformListener）
std::vector<Eigen::Matrix<double, 8, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 8, 1>>> baselink_traj_node;
std::string traj_save_name = "trajectory_from_rosbag.txt";

// 修改函数签名，添加listener参数
void add_baselink_traj_node(tf::TransformListener& listener) {
    tf::StampedTransform transform;
    try {
        listener.waitForTransform("/world", "/wam/base_link", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("/world", "/wam/base_link", ros::Time(0), transform);

        // 提取坐标和四元数
        double x = transform.getOrigin().x();
        double y = transform.getOrigin().y();
        double z = transform.getOrigin().z();
        tf::Quaternion q = transform.getRotation();
        
        // 获取ROS时间戳
        ros::Time timestamp = ros::Time::now();
        Eigen::Matrix<double, 8, 1> state;
        state << timestamp.toSec(), x, y, z, q.x(), q.y(), q.z(), q.w();
        
        baselink_traj_node.push_back(state);
    } catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
    }
}

// 保存函数保持不变
void save_baselink_traj_node(const std::string& traj_save_path) {
    std::string full_path = traj_save_path + "baselink_" + traj_save_name;
    std::cout << "\nSaving trajectory to " << full_path << "..." << std::endl;
    
    std::ofstream f(full_path, std::ios::app);
    if (!f) {
        std::cerr << "Failed to open file: " << full_path << std::endl;
        return;
    }
    
    f << std::fixed << std::setprecision(6);
    for (const auto& node : baselink_traj_node) {
        f << node[0] << " " << node[1] << " " << node[2] << " " << node[3]
          << " " << node[4] << " " << node[5] << " " << node[6] << " " << node[7] << "\n";
    }
    f.close();
    ROS_INFO("Trajectory saved.");
}

int main(int argc, char** argv) {
    // 正确初始化顺序
    ros::init(argc, argv, "baselink_traj_saver");
    ros::NodeHandle nh;
    
    // 在init之后创建TransformListener
    tf::TransformListener listener;
    
    const std::string traj_save_path = "/home/robotlab/ws_3d_vp/src/view_planning_3d/";
    ros::Rate rate(20.0);

    while (ros::ok()) {
        rate.sleep();
        
        // 传递listener到函数
        add_baselink_traj_node(listener);
        
        ROS_INFO("Nodes recorded: %lu", baselink_traj_node.size());
        
        if (baselink_traj_node.size() >= 10) {
            save_baselink_traj_node(traj_save_path);
            baselink_traj_node.clear();
        }
        
        ros::spinOnce();
    }

    // 退出前保存剩余数据
    if (!baselink_traj_node.empty()) {
        save_baselink_traj_node(traj_save_path);
    }
    
    return 0;
}