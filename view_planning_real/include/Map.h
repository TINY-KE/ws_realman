//
// Created by robotlab on 24-11-4.
//

#ifndef MAP_H
#define MAP_H

#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include "MapObject.h"
#include <ros/ros.h>
#include <std_msgs/String.h>  // 包含标准String消息类型
#include <std_msgs/Float32MultiArray.h>
#include <string>
#include <mutex>
#include <ctime>      // 用于获取当前系统时间
#include <sys/stat.h> // 用于检查文件是否存在

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/tf.h>
#include "tf/transform_datatypes.h"
#include <tf/transform_listener.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <fstream>

#include <unistd.h>

using namespace std;

class ObjectMap {
public:
    ObjectMap(ros::NodeHandle &nh) ;

    // 回调函数，用于接收话题消息
    void callback(const std_msgs::Float32MultiArray::ConstPtr &msg);


    // 主循环，定期处理物体信息
    void Run();

private:
    ros::Subscriber object_info_sub_; // 订阅器


public:
    std::vector<MapObject *> getMapObjects() ;

    std::vector<Eigen::Vector4d> getMapPlaneNormals() ;

    std::mutex mMutexMap;


private:
    std::vector<MapObject *> MapObjects;

    void addMapObject(MapObject *ob) ;

    void clearMapObjects() ;

    // std::vector<g2o::plane*> MapPlanes;
    std::vector<Eigen::Vector4d> MapPlaneNormals;

    void addMapPlaneNormals(Eigen::Vector4d plane);

    void clearMapPlaneNormals() ;



private:
    tf::TransformListener listener;

    std::string traj_save_name = "cam_traj.txt";

    std::vector<Eigen::Matrix<double, 8, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 8, 1>>> traj_node;
    std::vector<Eigen::Matrix<double, 8, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 8, 1>>> baselink_traj_node;


    std::string generateFileName() ;

    bool fileExists(const std::string &filename);

    void add_traj_node() ;
    void add_baselink_traj_node();

public:
    void save_traj_node(string traj_save_path) ;
    void save_baselink_traj_node(string traj_save_path) ;

private:
    bool isSameObject(const MapObject* obj1_center, const MapObject* obj2_center);
    double mDisThresh = 0.5;

};
#endif //MAP_H
