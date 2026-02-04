//
// Created by robotlab on 24-11-4.
//


#include "Map.h"

using namespace std;


ObjectMap::ObjectMap(ros::NodeHandle &nh) {
    // 初始化订阅器，订阅 "/object_info" 话题，回调函数为 ObjectInfoListener::callback
    object_info_sub_ = nh.subscribe("/objects_info", 10, &ObjectMap::callback, this);

    // 生成文件名
    traj_save_name = generateFileName();
}

// 回调函数，用于接收话题消息
void ObjectMap::callback(const std_msgs::Float32MultiArray::ConstPtr &msg) {
    int num_objects = msg->layout.dim[0].size; // 获取物体数量
    int num_features = msg->layout.dim[1].size; // 获取每个物体的特征数量

    if (msg->data.size() != num_objects * num_features) {
        ROS_WARN("Received data size does not match the expected matrix size.");
        return;
    }
    if (num_features != 9) {
        // 确保接收到的数组大小为9
        ROS_WARN("Received data size is not 9, ignoring...");
    }


    for (int i = 0; i < num_objects; ++i) {
        double x = msg->data[i * num_features + 0];
        double y = msg->data[i * num_features + 1];
        double z = msg->data[i * num_features + 2];
        double roll = msg->data[i * num_features + 3]; // no use
        double pitch = msg->data[i * num_features + 4]; // no use
        double yaw = msg->data[i * num_features + 5];
        double lenth = msg->data[i * num_features + 6];
        double width = msg->data[i * num_features + 7];
        double height = msg->data[i * num_features + 8];

        // 检查物体的长宽比是否小于1.25，如果是的话，则修改小的一方
        if(lenth/width > 1.25){
            width = lenth / 1.2;
        } else if(width/lenth > 1.25){
            lenth = width / 1.2;
        }


        // 遍历地图中有的物体，根据距离判断是否是同一个物体
        MapObject *ob = new MapObject();
        ob->Update_Twobj(x, y, z, yaw);
        ob->Update_object_size(lenth, width, height);
        ob->Update_corner();

        bool isnew = true;
        for(auto ob_old: MapObjects)
            if( isSameObject(ob, ob_old))
                isnew = false;

        if (isnew) {
            addMapObject(ob);
            ROS_INFO("Received One New Object ---------- ");
        }
    }
}


// 主循环，定期处理物体信息
void ObjectMap::Run() {
    ros::Rate rate(10); // 设置循环频率为10Hz
    while (ros::ok()) {
        add_traj_node();

        // 处理所有订阅的回调函数
        ros::spinOnce();

        // 控制循环频率
        rate.sleep();
    }
}

std::vector<MapObject *> ObjectMap::getMapObjects() {
    return MapObjects;
}

std::vector<Eigen::Vector4d> ObjectMap::getMapPlaneNormals() {
    return MapPlaneNormals;
}


void ObjectMap::addMapObject(MapObject *ob) {
    std::unique_lock<std::mutex> lock(mMutexMap);
    MapObjects.push_back(ob);
}

void ObjectMap::clearMapObjects() {
    MapPlaneNormals.clear();
}


void ObjectMap::addMapPlaneNormals(Eigen::Vector4d plane) {
    std::unique_lock<std::mutex> lock(mMutexMap);
    MapPlaneNormals.push_back(plane);
}

void ObjectMap::clearMapPlaneNormals() {
    MapPlaneNormals.clear();
}


std::string ObjectMap::generateFileName() {
    // 获取当前系统时间
    std::time_t now = std::time(0);
    char timestamp[80];

    // 格式化系统时间为年-月-日_时-分-秒
    std::strftime(timestamp, sizeof(timestamp), "%Y-%m-%d_%H-%M-%S", std::localtime(&now));

    // 生成文件名：cam_traj_当前时间.txt
    std::string filename = "cam_traj_" + std::string(timestamp) + ".txt";

    // 如果文件名已存在，确保不会覆盖
    int counter = 1;
    while (fileExists(filename)) {
        filename = "cam_traj_" + std::string(timestamp) + "_" + std::to_string(counter) + ".txt";
        counter++;
    }

    return filename;
}

bool ObjectMap::fileExists(const std::string &filename) {
    struct stat buffer;
    return (stat(filename.c_str(), &buffer) == 0);
}


void ObjectMap::add_traj_node() {
    // 利用tf获取
    tf::StampedTransform transform;
    try {
        // 等待直到可以获取到 /camera_rgb_optical_frame 相对于 /world 的变换
        listener.waitForTransform("/world", "/camera_rgb_optical_frame", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("/world", "/camera_rgb_optical_frame", ros::Time(0), transform);

        // 获取平移和旋转信息
        double x = transform.getOrigin().x();
        double y = transform.getOrigin().y();
        double z = transform.getOrigin().z();

        tf::Quaternion q = transform.getRotation();
        double qx = q.x();
        double qy = q.y();
        double qz = q.z();
        double qw = q.w();

        // 打印获取到的位姿信息
        // ROS_INFO("[add_traj_node] Translation: [%f, %f, %f]", x, y, z);
        // ROS_INFO("[add_traj_node] Rotation (Quaternion): [%f, %f, %f, %f]", qx, qy, qz, qw);

        ros::Time timestamp = ros::Time::now();
        double ros_timestamp = timestamp.toSec(); // 将时间戳转换为秒

        // 存储位姿 (平移 + 四元数) 到 Eigen::Matrix 中
        Eigen::Matrix<double, 8, 1> state;
        //  ICL-NUIM dataset:  timestamp tx ty tz qx qy qz qw
        state << ros_timestamp, x, y, z, qx, qy, qz, qw; // 这里只存储平移和四元数的前三个分量（qw 不需要存储在这里）

        traj_node.push_back(state);
    } catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
    }
    // printf("Save traj node\n");
}

void ObjectMap::add_baselink_traj_node() {
    // 利用tf获取
    tf::StampedTransform transform;
    try {
        // 等待直到可以获取到 /camera_rgb_optical_frame 相对于 /world 的变换
        listener.waitForTransform("/world", "/wam/base_link", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("/world", "/wam/base_link", ros::Time(0), transform);

        // 获取平移和旋转信息
        double x = transform.getOrigin().x();
        double y = transform.getOrigin().y();
        double z = transform.getOrigin().z();

        tf::Quaternion q = transform.getRotation();
        double qx = q.x();
        double qy = q.y();
        double qz = q.z();
        double qw = q.w();

        // 打印获取到的位姿信息
        // ROS_INFO("[add_traj_node] Translation: [%f, %f, %f]", x, y, z);
        // ROS_INFO("[add_traj_node] Rotation (Quaternion): [%f, %f, %f, %f]", qx, qy, qz, qw);

        ros::Time timestamp = ros::Time::now();
        double ros_timestamp = timestamp.toSec(); // 将时间戳转换为秒

        // 存储位姿 (平移 + 四元数) 到 Eigen::Matrix 中
        Eigen::Matrix<double, 8, 1> state;
        //  ICL-NUIM dataset:  timestamp tx ty tz qx qy qz qw
        state << ros_timestamp, x, y, z, qx, qy, qz, qw; // 这里只存储平移和四元数的前三个分量（qw 不需要存储在这里）

        baselink_traj_node.push_back(state);
    } catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
    }
    // printf("Save traj node\n");
}

void ObjectMap::save_traj_node(string traj_save_path) {
    // 将轨迹节点存储到文本文件中
    string full_path = traj_save_path + traj_save_name;
    std::cout << std::endl << "Saving keyframe trajectory to " << full_path << " ..." <<
            std::endl;

    std::ofstream f;
    f.open((traj_save_path + traj_save_name).c_str(), std::ios::out | std::ios::app);
    if (!f.is_open()) {
        std::cerr << "Failed to open file: " << full_path << std::endl;
        return;
    }

    f << std::fixed;

    for (const auto &node: traj_node) {
        // f << node.transpose().matrix() << std::endl;
        f << setprecision(6) << node[0] << setprecision(7) 
            << " " << node[1]
            << " " << node[2]
            << " " << node[3]
            << " " << node[4]
            << " " << node[5]
            << " " << node[6]
            << " " << node[7] << endl;
    }

    f.close();
    ROS_INFO("Saved Camera trajectory node.");
}


void ObjectMap::save_baselink_traj_node(string traj_save_path) {

    string baselink_traj_save_name = "baselink_" + traj_save_name;
    // 将轨迹节点存储到文本文件中
    string full_path = traj_save_path + baselink_traj_save_name;
    std::cout << std::endl << "Saving baselink trajectory to " << full_path << " ..." <<
            std::endl;

    std::ofstream f;
    f.open((traj_save_path + baselink_traj_save_name).c_str(), std::ios::out | std::ios::app);
    if (!f.is_open()) {
        std::cerr << "Failed to open file: " << full_path << std::endl;
        return;
    }

    f << std::fixed;

    for (const auto &node: baselink_traj_node) {
        // f << node.transpose().matrix() << std::endl;
        f << setprecision(6) << node[0] << setprecision(7) 
            << " " << node[1]
            << " " << node[2]
            << " " << node[3]
            << " " << node[4]
            << " " << node[5]
            << " " << node[6]
            << " " << node[7] << endl;
    }

    f.close();
    ROS_INFO("Saved Camera trajectory node.");
}


bool ObjectMap::isSameObject(const MapObject* obj1_center, const MapObject* obj2_center) {
    // 计算欧几里得距离
    double distance = std::sqrt(
        std::pow(obj2_center->mCuboid3D.cuboidCenter.x() - obj1_center->mCuboid3D.cuboidCenter.x(), 2)
        + std::pow(obj2_center->mCuboid3D.cuboidCenter.y() - obj1_center->mCuboid3D.cuboidCenter.y(), 2)
        + std::pow(obj2_center->mCuboid3D.cuboidCenter.z() - obj1_center->mCuboid3D.cuboidCenter.z(), 2)
    );

    // 输出距离以便调试
    // std::cout << "Distance between objects: " << distance << std::endl;

    // 如果距离小于或等于阈值，则认为是同一个物体
    return distance <= mDisThresh;
}