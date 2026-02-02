//
// Created by robotlab on 24-11-6.
//

#ifndef TELEOPMOBILE_H
#define TELEOPMOBILE_H

/**
 * 创建时间：2024.1.13
 * 功能：使用键盘移动指定名称model，一般用于kinect相机
 * 使用方法：
 *    1. 修改 初始化值 和 model_name
 *    2. 上-前进，下-后退，左-左转，右-右转，w-相机仰，s-相机俯，a-向左平移，d-向右平移
 *    3. 空格-保存当前位置作为节点，Enter-演示目前所有节点对应的轨迹，c-删除当前节点并返回上一节点
*/


// 参考博客：https://www.jianshu.com/p/9c6adc3aeb02

#include <ros/ros.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

// #include <rclcpp/rclcpp.hpp>

#include <gazebo_msgs/SetModelState.h>
#include <math.h>
#include <assert.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/tf.h>
#include "tf/transform_datatypes.h"
#include <tf/transform_listener.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <iostream>

#include <fstream>

#include <unistd.h>

using namespace std;
// using namespace Eigen;

// a-61, b-62, c-63, d-64, e-65, f-66, g-67
// h-68, i-69, j-6A, k-6B, l-6C, m-6D, n-6E
// o-6F, p-70, q-71, r-72, s-73, t-74
// u-75, v-76, w-77, x-78, y-79, z-7A

#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
// #define KEYCODE_Esc 0x1B

#define KEYCODE_w 0x77  // w, to speed up linear velocity
#define KEYCODE_x 0x78  // x, to speed down linear velocity
#define KEYCODE_a 0x61  // a, to speed up angular velocity
#define KEYCODE_d 0x64  // d, to speed down angular velocity

#define KEYCODE_q 0x71  // w, to speed up linear velocity
#define KEYCODE_e 0x65  // x, to speed down linear velocity
#define KEYCODE_z 0x7A  // a, to speed up angular velocity
#define KEYCODE_c 0x63  // d, to speed down angular velocity

#define KEYCODE_s 0x73  // to save traj node

#define KEYCODE_j 0x6A  // to save traj node
#define KEYCODE_k 0x6B  // show traj node
#define KEYCODE_i 0x69  // clear traj node

#define KEYCODE_SPACE 0x20


// 用于 View Planning
#define KEYCODE_ENTER 0x0A  // to show traj


typedef Eigen::Matrix<double, 6, 1> Vector6d;

int kfd = 0;
struct termios cooked, raw;
// 定义两个termios结构体，跟Linux终端I/O相关，结构体中是对终端I/O的控制字段
//参看 https://www.jianshu.com/p/bb128a6b413e

void quit(int sig)  // 定义一个quit函数
{
    tcsetattr(kfd, TCSANOW, &cooked);
    //tcsetattr返回终端属性，中间参数表示可以指定在什么时候新的终端属性才起作用
    //TCSANOW：更改立即发生
    ros::shutdown();  //关闭节点
    exit(0); // exit（0）：正常运行程序并退出程序 ,exit（1）：非正常运行导致退出程序；
    // 参看：https://www.cnblogs.com/nufangrensheng/archive/2013/03/01/2938508.html
}


class TeleopMobile  //声明TeleopTurle类
{
public:
  TeleopMobile(ros::NodeHandle& nh_, double x_, double y_, double  z_, double roll_, double pitch_, double yaw_);  //构造函数
  void Run();
  void change_yaw(double angele);
  void set_xy(double x, double y);

private:
  void client_call(bool print_state = false);
  void load_traj_node();
  void clear_current_node();
  void set_state(Vector6d& state);
  
  void show_traj();
  ros::NodeHandle nh;
  double v_linear, v_angular, l_scale_, a_scale_;
  double x, y, z, roll, pitch, yaw;  //初始化的机器人坐标

  ros::Time start_time;

  string model_name;


  ros::Publisher vel_pub_;
  ros::ServiceClient client;

};




//int main(int argc, char** argv)
//{
//  ros::init(argc, argv, "teleop_turtle"); // 创建节点的第一步，初始化节点，节点名为："teleop_turtle"
//  TeleopTurtle teleop_turtle;  // 定义一个类
//  signal(SIGINT,quit);   // siganl 函数设置了某个信号的对于动作，再此是进入quit函数
//  teleop_turtle.keyLoop(); // 调用类中的方法
//  return(0);
//}


#endif //TELEOPMOBILE_H
