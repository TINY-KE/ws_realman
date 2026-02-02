//
// Created by robotlab on 24-11-6.
//


/**
 * 创建时间：2024.1.13
 * 功能：使用键盘移动指定名称model，一般用于kinect相机
 * 使用方法：
 *    1. 修改 初始化值 和 model_name
 *    2. 上-前进，下-后退，左-左转，右-右转，w-相机仰，s-相机俯，a-向左平移，d-向右平移
 *    3. 空格-保存当前位置作为节点，Enter-演示目前所有节点对应的轨迹，c-删除当前节点并返回上一节点
*/


// 参考博客：https://www.jianshu.com/p/9c6adc3aeb02

#include "TeleopMobile.h"

using namespace std;
// using namespace Eigen;

// a-61, b-62, c-63, d-64, e-65, f-66, g-67
// h-68, i-69, j-6A, k-6B, l-6C, m-6D, n-6E
// o-6F, p-70, q-71, r-72, s-73, t-74
// u-75, v-76, w-77, x-78, y-79, z-7A


TeleopMobile::TeleopMobile(ros::NodeHandle &nh_, double x_, double y_, double z_, double roll_, double pitch_,
                           double yaw_): //构造函数的定义，传递默认值
    nh(nh_), x(x_), y(y_), z(z_), roll(roll_), pitch(pitch_), yaw(yaw_),
    model_name("mrobot"),
    //   model_name("mobile_base"),
    v_linear(0.01),
    v_angular(0.01),
    l_scale_(2.0),
    a_scale_(2.0) {
    nh_.param("scale_angular", a_scale_, a_scale_); // param()和getParam()类似，但是允许指定参数的默认值
    // 参考：http://wiki.ros.org/cn/roscpp_tutorials/Tutorials/Parameters
    nh_.param("scale_linear", l_scale_, l_scale_); // 同上

    client = nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    gazebo_msgs::SetModelState objstate;
    objstate.request.model_state.model_name = model_name; //"acircles_pattern_0"  mobile_base;
    // objstate.request.model_state.model_name = "mobile_base";//"acircles_pattern_0"  mobile_base;

    client_call(true);



    // 注册一个话题，话题名为“turtle1/command_velocity”
    //消息类型为：turtlesim::Velocity，消息队列大小为1
}


void TeleopMobile::Run() {
    char c;
    bool dirty = false;

    // get the console in raw mode
    tcgetattr(kfd, &cooked); // 参看：https://www.jianshu.com/p/bb128a6b413e
    memcpy(&raw, &cooked, sizeof(struct termios)); // 将cooked中的内容拷贝到raw中
    raw.c_lflag &= ~(ICANON | ECHO);
    //将ICANON和ECHO按位或“|” 之后取反，再将raw.c_lflag和其相与，将结果赋值给raw.c_lflag
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    //raw.c_cc是控制字符  https://www.gwduan.com/web/computer/history/unix-prog/terminal.html#special-char
    tcsetattr(kfd, TCSANOW, &raw);

    // puts("Reading from keyboard"); // puts()函数用来向 标准输出设备 （屏幕）写字符串并换行
    // puts("---------------------------");
    // puts("Use arrow keys to move the turtle.");

    while (ros::ok()) // 循环一直执行 ，知道遇到退出语句
    {

        if (read(kfd, &c, 1) < 0) {
            perror("read():");
            //　perror ( )用 来 将 上 一 个 函 数 发 生 错 误 的 原 因 输 出 到 标 准 设备 (stderr)
            exit(-1);
        }

        // ROS_DEBUG("value: 0x%02X\n", c);
        // ROS_INFO("value: 0x%02X\n", c);

        // cout << "KEYCODE = " << c << endl;

        switch (c) // 根据键盘的按键值，给相应的参数赋值
        {
            case KEYCODE_a:
                ROS_DEBUG("LEFT");
                yaw += v_angular;
                dirty = true;
                break;
            case KEYCODE_d:
                ROS_DEBUG("RIGHT");
                yaw -= v_angular;
                dirty = true;
                break;
            case KEYCODE_w:
                x += v_linear * cos(yaw);
                y += v_linear * sin(yaw);
                dirty = true;
                break;
            case KEYCODE_x:
                x -= v_linear * cos(yaw);
                y -= v_linear * sin(yaw);
                dirty = true;
                break;


            case KEYCODE_q:
                // 左旋转前进：航向角增加，x, y 按新航向前进
                yaw += v_angular; // 增加航向角，向左旋转
                x += v_linear * cos(yaw); // 根据新的航向角前进
                y += v_linear * sin(yaw);
                dirty = true;
                break;

            case KEYCODE_e:
                // 右旋转前进：航向角减少，x, y 按新航向前进
                yaw -= v_angular; // 减少航向角，向右旋转
                x += v_linear * cos(yaw); // 根据新的航向角前进
                y += v_linear * sin(yaw);
                dirty = true;
                break;

            case KEYCODE_z:
                // 左旋转后退：航向角增加，x, y 按新航向后退
                yaw -= v_angular; // 增加航向角，向左旋转
                x -= v_linear * cos(yaw); // 根据新的航向角后退
                y -= v_linear * sin(yaw);
                dirty = true;
                break;

            case KEYCODE_c:
                // 右旋转后退：航向角减少，x, y 按新航向后退
                yaw += v_angular; // 减少航向角，向右旋转
                x -= v_linear * cos(yaw); // 根据新的航向角后退
                y -= v_linear * sin(yaw);
                dirty = true;
                break;

            // case KEYCODE_k:
            //     show_traj();
            //     break;
            // case KEYCODE_i:
            //     clear_current_node();
            //     break;
        }

        if (dirty == true) {
            client_call(false);
            dirty = false;
            ros::spinOnce();
        }

    }
    return;
}

void TeleopMobile::client_call(bool print_state) {
    gazebo_msgs::SetModelState objstate;
    tf::Quaternion q = tf::createQuaternionFromRPY(0, pitch, yaw);
    objstate.request.model_state.model_name = model_name; //"acircles_pattern_0"  mobile_base;
    // objstate.request.model_state.model_name = "mobile_base";//"acircles_pattern_0"  mobile_base;
    objstate.request.model_state.pose.position.x = x;
    objstate.request.model_state.pose.position.y = y;
    objstate.request.model_state.pose.position.z = z;

    objstate.request.model_state.pose.orientation.w = q.w();
    objstate.request.model_state.pose.orientation.x = q.x();
    objstate.request.model_state.pose.orientation.y = q.y();
    objstate.request.model_state.pose.orientation.z = q.z();

    if (print_state) {
        // ROS_INFO("Camera Pose: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f", x,y,z,roll,pitch,yaw);
        printf("Camera Pose: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, v_linear: %.2f, v_angular: %.2f\n",\
               x, y, z, roll, pitch, yaw, v_linear, v_angular);
    }

    // ros::Time current_time = ros::Time::now();
    // float relate_time = current_time.toSec() - start_time.toSec();
    // // ROS_INFO("Current time: %f", (current_time.toSec() - start_time.toSec()));
    //
    // printf("Time: %.6f, Camera Pose: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",\
    //             relate_time,x,y,z,q.x(),q.y(),q.z(),q.w());

    client.call(objstate);
}

void TeleopMobile::load_traj_node() {
    // ifstream f(traj_save_path.c_str());
    // if (!f.is_open()) {
    //     std::cerr << "Cannot open file: " << traj_save_path << std::endl;
    //     return;
    // }
    // traj_node.clear();
    // std::string line;
    // while (std::getline(f, line)) {
    //     if (line.empty()) {
    //         cout << "Empty Line." << endl;
    //         break; // 跳过空行
    //     }
    //     std::istringstream iss(line);
    //     std::string word;
    //     std::vector<std::string> words;
    //     while (iss >> word) {
    //         words.push_back(word); // 将每个单词加入向量
    //     }
    //
    //     if (words.size() != 6) {
    //         cout << "size != 6" << endl;
    //         break;
    //     }
    //
    //     Vector6d state;
    //
    //     state << std::stod(words[0]),
    //             std::stod(words[1]),
    //             std::stod(words[2]),
    //             std::stod(words[3]),
    //             std::stod(words[4]),
    //             std::stod(words[5]);
    //     traj_node.push_back(state);
    // }
    // f.close();
}


void TeleopMobile::clear_current_node() {
    // if (traj_node.size() == 1) {
    //     cout << "Only start node is left now." << endl;
    //     return;
    // }
    //
    // traj_node.resize(traj_node.size() - 1);
    // set_state(traj_node.back());
}

void TeleopMobile::set_state(Vector6d &state) {
    x = state(0);
    y = state(1);
    z = state(2);
    roll = state(3);
    pitch = state(4);
    yaw = state(5);

    client_call();
}

void TeleopMobile::change_yaw(double angele) {
    yaw += angele;
}

void TeleopMobile::set_xy(double x_, double y_) {
    x = x_;
    y = y_;
}

void TeleopMobile::show_traj() {
    // start_time = ros::Time::now();
    // int split_num = 1000;
    // // set_state(traj_node[0]);
    // for(int i_node = 0; i_node < traj_node.size() - 1; i_node++){
    //     auto& state1 = traj_node[i_node], state2 = traj_node[i_node+1];
    //     for(int i_split = 0; i_split < split_num - 1; i_split++) {
    //         double w2 = (double)i_split / split_num;
    //         double w1 = 1.0 - w2;
    //         Vector6d state_mid = w1 * state1 + w2 * state2;
    //         set_state(state_mid);
    //         usleep(1000); // us
    //     }
    // }
    //
    // set_state(traj_node.back());
}

//int main(int argc, char** argv)
//{
//  ros::init(argc, argv, "teleop_turtle"); // 创建节点的第一步，初始化节点，节点名为："teleop_turtle"
//  TeleopTurtle teleop_turtle;  // 定义一个类
//  signal(SIGINT,quit);   // siganl 函数设置了某个信号的对于动作，再此是进入quit函数
//  teleop_turtle.keyLoop(); // 调用类中的方法
//  return(0);
//}

