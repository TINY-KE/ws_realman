#include <ros/ros.h>
#include <rm_msgs/MoveJ.h>


int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "movej_commander");
    ros::NodeHandle nh;

    // 创建话题发布者[6](@ref)
    ros::Publisher movej_pub = nh.advertise<rm_msgs::MoveJ>(
        "/rm_driver/MoveJ_Cmd", 10);

    // 设置循环频率
    ros::Rate loop_rate(1);
    double angle = -1.5;
    while(ros::ok()) 
    {
        rm_msgs::MoveJ cmd_msg;
        
        // 填充关节角度数据（示例值）[7](@ref)
        cmd_msg.joint = {angle, 0, 00, 0.0, 0.0, 0};  // 6轴机械臂典型位置
        cmd_msg.speed = 5;  // 50%速度运行
        angle = angle + 0.1;
        // 发布消息[6](@ref)
        movej_pub.publish(cmd_msg);

        // 处理回调并保持循环频率
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}