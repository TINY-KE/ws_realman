//
// Created by robotlab on 24-11-5.
//
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <vector>
#include <sstream>
    
int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "float32_multiarray_publisher");

    // 创建节点句柄
    ros::NodeHandle nh;

    // 创建一个发布者，发布 std_msgs::Float32MultiArray 类型消息
    ros::Publisher multiarray_pub = nh.advertise<std_msgs::Float32MultiArray>("/objects_info", 10);

    // 设置发布频率
    ros::Rate loop_rate(1);  // 1 Hz

    // 检查是否提供了足够的命令行参数
    if (argc < 2 || (argc - 2) % 9 != 0) {
        ROS_ERROR("Usage: %s <num_objects> <obj1_param1> <obj1_param2> ... <objN_param9>", argv[0]);
        return 1;
    }

    // 获取物体数量
    int num_objects = std::stoi(argv[1]);

    // 检查是否提供了正确数量的参数（每个物体9个参数）
    if ((argc - 2) != num_objects * 9) {
        ROS_ERROR("Error: Expected %d objects, but provided %d parameters.", num_objects, argc - 2);
        return 1;
    }

    // 准备一个 std_msgs::Float32MultiArray 消息
    std_msgs::Float32MultiArray multiarray_msg;

    // 设置消息的 layout，定义每行代表一个物体，包含9个参数
    multiarray_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    multiarray_msg.layout.dim[0].label = "objects";
    multiarray_msg.layout.dim[0].size = num_objects;       // 物体数量
    multiarray_msg.layout.dim[0].stride = num_objects * 9; // 每行9个参数
    multiarray_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    multiarray_msg.layout.dim[1].label = "parameters";
    multiarray_msg.layout.dim[1].size = 9;                 // 每个物体9个参数
    multiarray_msg.layout.dim[1].stride = 9;

    // 将命令行参数中的物体数据填入消息
    for (int i = 2; i < argc; ++i) {
        multiarray_msg.data.push_back(std::stof(argv[i]));
    }

    // 发布循环
    while (ros::ok())
    {
        // 发布消息
        multiarray_pub.publish(multiarray_msg);

        // 日志输出
        ROS_INFO("Published Float32MultiArray with %d objects", num_objects);

        // 休眠等待下一个循环
        loop_rate.sleep();
    }

    return 0;
}