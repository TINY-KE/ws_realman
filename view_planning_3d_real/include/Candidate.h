#ifndef VIEW_PLANNING_CANDIDATE_H
#define VIEW_PLANNING_CANDIDATE_H

#include <Converter.h>
#include <Eigen/Dense>
#include <vector>
#include "MapObject.h"
#include <tf/tf.h>
#include "tf/transform_datatypes.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "ConverterTools.h"
#include <Eigen/Dense>


class Candidate{
    public:
        double centor_x,centor_y,centor_z;
        double roll,pitch,yaw;
        Eigen::Vector3d start;
        Eigen::Vector3d end;
        Eigen::Quaterniond oritention_quaterniond;
        Eigen::Isometry3d  pose_isometry3d;

    public:
        // 构造函数
        Candidate(){
            centor_x = 0;
            centor_y = 0;
            centor_z = 0;
            roll = 0;
            pitch = 0;
            yaw = 0;
        }
        //This code creates a candidate trajectory for the robot to follow.
        Candidate(Eigen::Vector3d start_, Eigen::Vector3d end_){
            start = start_;
            end = end_;
            
            Eigen::Vector3d direction = end - start;
            direction.normalize();

            Eigen::Quaterniond quaternion;
            quaternion.setFromTwoVectors(Eigen::Vector3d::UnitX(), direction);
            oritention_quaterniond = quaternion;
            // std::cout<<"quaternion.setFromTwoVectors: x "<<quaterniond_.x()
            //                                         <<"  y "<<quaterniond_.y()
            //                                         <<"  z "<<quaterniond_.z()
            //                                         <<"  w "<<quaterniond_.w()<<std::endl;
            // 以下注销的部分是自行计算ypr。经过我的验证，和quaternion.setFromTwoVectors的计算结果一致
            // cv::Point3f v =  cv::Point3f(end.x(),  end.y(),  end.z()) - cv::Point3f(start.x(),  start.y(),  start.z());
            // yaw = std::atan2(v.y, v.x);  // 绕Z轴的旋转角度
            // pitch = std::atan2(-v.z, v.x);  // 绕y轴的旋转角度
            // // roll = std::atan2 因为要让相机视线（机械臂末端的x轴）与目标向量相同，有很多选择，因此我选择了限制roll=0
            // // 计算旋转矩阵
            // Eigen::Matrix3d rotation_matrix_;
            // rotation_matrix_ = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
            //                 * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
            // quaterniond_ = Eigen::Quaterniond(rotation_matrix_);
            // std::cout<<"Eigen::AngleAxisd: x "<<quaterniond_.x()
            //                             <<"  y "<<quaterniond_.y()
            //                             <<"  z "<<quaterniond_.z()
            //                             <<"  w "<<quaterniond_.w()<<std::endl;
            // std::cout<<std::endl;
            // std::cout<<std::endl;


            // //转换成  cvmat矩阵
            // cv::Mat rotate_mat = Converter::toCvMat(rotation_matrix);
            // cv::Mat t_mat = (cv::Mat_<float>(3, 1) << x, y, 0);
            // cv::Mat T_world_to_baselink = cv::Mat::eye(4, 4, CV_32F);
            // rotate_mat.copyTo(T_world_to_baselink.rowRange(0, 3).colRange(0, 3));
            // t_mat.copyTo(T_world_to_baselink.rowRange(0, 3).col(3));
            // cv::Mat Camera_mat = T_world_to_baselink * mT_basefootprint_cam;


            centor_x = start(0);
            centor_y = start(1);
            centor_z = start(2);
            pose_isometry3d = Eigen::Translation3d(centor_x, centor_y, centor_z) * oritention_quaterniond;

        }

        // cv::Mat GetPose(){
        //     // cv::Mat pose = cv::Mat::ones(4,4,CV_32F);
        //     // pose.at<float>(0,3) = x;
        //     // pose.at<float>(1,3) = y;
        //     // pose.at<float>(2,3) = z;
        //     // pose.at<float>(3,3) = 1;
            
        //     Eigen::AngleAxisd rotation_vector(q);
        //     Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        //     T.rotate(rotation_vector);
        //     T.pretranslate(Eigen::Vector3d(x, y, z));
        //     Eigen::Matrix4d Pose_eigen = T.matrix();

        //     cv::Mat cvMat(4,4,CV_32F);
        //     for(int i=0;i<4;i++)
        //         for(int j=0; j<4; j++)
        //             cvMat.at<float>(i,j)=Pose_eigen(i,j);

        //     return cvMat.clone();
        // }

        
        // cv::Mat GetCameraCenter(){
        //     cv::Mat o = (cv::Mat_<float>(4,1) << x, y, z, 1);
        //     return o.clone();   
        // }


};



struct Ellipse {
    double a; // 椭圆长轴
    double b; // 椭圆短轴
    double xc; // 椭圆中心的 x 坐标
    double yc; // 椭圆中心的 y 坐标
    double yaw; // 椭圆的偏航角（弧度）
};
struct Point {
    double x;
    double y;
};
// 计算两个向量的点积
double dotProduct(const Point& v1, const Point& v2) {
    return v1.x * v2.x + v1.y * v2.y;
}

// 计算向量的模长
double magnitude(const Point& v) {
    return std::sqrt(v.x * v.x + v.y * v.y);
}


// 计算椭圆上一点的切线与椭圆中心连线的夹角
double calculateAngle(const Ellipse& ellipse, const Point& A) {
    // 计算 AC 向量
    Point AC = { A.x - ellipse.xc, A.y - ellipse.yc };

    // 计算法线方向向量 (n)
    Point n = { (A.x - ellipse.xc) / (ellipse.a * ellipse.a),
                (A.y - ellipse.yc) / (ellipse.b * ellipse.b) };

    // 切线方向向量是法线方向的垂直向量
    Point t = { -n.y, n.x };

    // 计算 AC 向量和切线方向向量 t 之间的夹角
    double dot = dotProduct(AC, t);
    double magAC = magnitude(AC);
    double magT = magnitude(t);

    // 计算夹角的余弦值
    double cosTheta = dot / (magAC * magT);

    // 通过 acos 计算夹角，单位为弧度
    double theta = std::acos(cosTheta);

    return theta;
}


// 计算射线与椭圆的交点
Point calculateIntersection(const Ellipse& ellipse, double theta_rad) {

    // 如果theta_rad小于-PI或大于PI，则将其转换到[-PI, PI]范围内
    if (theta_rad < -M_PI) {
        theta_rad += 2 * M_PI;
    } else if (theta_rad > M_PI) {
        theta_rad -= 2 * M_PI;
    }
    
    double a = ellipse.a ;
    double b = ellipse.b;
    double xc = ellipse.xc;
    double yc = ellipse.yc;
    double yaw = ellipse.yaw;

    // 如果射线垂直（theta = 90 或 270 度）
    if (theta_rad == M_PI_2 || theta_rad == M_PI_2 * 3) {
        std::cout << "射线垂直于x轴，交点为: (0, " << ((theta_rad == M_PI_2) ? b : -b) << ")\n";
        return Point{ 0+xc, ((theta_rad == M_PI_2) ? b : -b )+yc};
    }


    // 计算 tan(theta)
    double tan_theta = std::tan(theta_rad);

    // 计算 x 坐标
    double x_abs = std::sqrt((a * a * b * b) / (b * b + a * a * tan_theta * tan_theta));

    // 计算 y 坐标
    double y_abs = x_abs * tan_theta;

    // // 根据 theta 的范围判断符号
    double x = std::abs(x_abs), y = std::abs(y_abs);
    if (theta_rad > M_PI_2 || theta_rad <= -1 * M_PI_2) x = -1 * x;  // 第二、三象限
    if (theta_rad < 0 ) y = -1 * y;                               // 第三、四象限

    // 输出结果
    std::cout << "角度为：" << theta_rad <<
    "时， 射线与椭圆的交点为: (" << x+xc << ", " << y+yc << ")\n";

    return Point{ x+xc, y+yc };
}

// 计算椭圆上一点的切线与 x 轴的夹角
double calculateAngleWithXAxis(const Ellipse& ellipse, const Point& A) {
    // 计算法线方向向量 (n)
    Point n = { (A.x - ellipse.xc) / (ellipse.a * ellipse.a),
                (A.y - ellipse.yc) / (ellipse.b * ellipse.b) };

    // 切线方向向量是法线方向的垂直向量
    Point t = { -n.y, n.x };

    // 计算切线与 x 轴的夹角
    double angleWithXAxis = std::atan2(t.y, t.x); // 使用 atan2 得到角度

    // 转换为度数
    double angleInDegrees = angleWithXAxis * 180.0 / M_PI;

    // 调整角度
    angleInDegrees -= 180.0; // 减去 180 度

    // 确保角度在 0 到 360 度之间
    if (angleInDegrees < 0) {
        angleInDegrees += 360.0;
    }

    return angleInDegrees; // 转换为度数
}



std::vector<geometry_msgs::Pose> GenerateCandidates_rotate( MapObject& sdf_object, std::vector<geometry_msgs::Pose> & RobotPoses, double radius=3 /*半径*/ , double camera_height = 1.0, double init_robot_x=0, double init_robot_y=0, bool forCamera = false , int rotate_divides = 30 ){

    std::vector<geometry_msgs::Pose> candidates;

    // 物体的位姿
    double object_x = sdf_object.mCuboid3D.cuboidCenter.x();
    double object_y = sdf_object.mCuboid3D.cuboidCenter.y();
    double object_z = sdf_object.mCuboid3D.cuboidCenter.z();
    // double object_z = 0;

    std::vector<Point> InterS;

    Ellipse ellipse = { radius, radius, sdf_object.mCuboid3D.cuboidCenter.x(), sdf_object.mCuboid3D.cuboidCenter.y(), sdf_object.mCuboid3D.rotY };

    double angle_init = std::atan2( init_robot_y-sdf_object.mCuboid3D.cuboidCenter.y() , init_robot_x-sdf_object.mCuboid3D.cuboidCenter.x() );

    double circle_start_x = radius*cos(angle_init)+object_x;
    double circle_start_y = radius*sin(angle_init)+object_y;


    // (2) 原地左旋转90度 部分
    // double rotate_mini = 5*M_PI/180;
    // int rotate_nums = M_PI_2/rotate_mini;
    int rotate_nums = rotate_divides;
    for(int i=rotate_nums; i>0; i--){

        double t = 1-static_cast<double>(i) / rotate_nums; // 归一化参数
        double delta_yaw = t*M_PI_2;
        double footprint_x = circle_start_x;
        double footprint_y = circle_start_y;
        double footprint_z = 0.0;
        tf::Quaternion q_footprint = tf::createQuaternionFromRPY(0, 0, angle_init+M_PI + delta_yaw);
        geometry_msgs::Pose T_world_footprint;
        T_world_footprint.position.x = footprint_x;
        T_world_footprint.position.y = footprint_y;
        T_world_footprint.position.z = footprint_z;
        T_world_footprint.orientation.x = q_footprint.x();
        T_world_footprint.orientation.y = q_footprint.y();
        T_world_footprint.orientation.z = q_footprint.z();
        T_world_footprint.orientation.w = q_footprint.w();
        RobotPoses.push_back(T_world_footprint);


        // 相机位姿
        double camera_x = footprint_x;
        double camera_y = footprint_y;
        double camera_z = camera_height;
        double deltaX = object_x - camera_x;
        double deltaY = object_y - camera_y;
        double deltaZ = object_z - camera_z;
        double delta_pitch = std::atan2(deltaZ, std::sqrt(deltaX*deltaX + deltaY*deltaY));

        // 计算candidate在foot_print中的位姿
        tf::Quaternion q_c = tf::createQuaternionFromRPY(0, -1*delta_pitch, -1*delta_yaw);
        geometry_msgs::Pose T_footprint_cameralink;
        T_footprint_cameralink.position.x = 0.0;
        T_footprint_cameralink.position.y = 0;
        T_footprint_cameralink.position.z = camera_z - footprint_z;
        T_footprint_cameralink.orientation.x = q_c.x();
        T_footprint_cameralink.orientation.y = q_c.y();
        T_footprint_cameralink.orientation.z = q_c.z();
        T_footprint_cameralink.orientation.w = q_c.w();

        candidates.push_back(T_footprint_cameralink );
    }

    return candidates;

}




std::vector<geometry_msgs::Pose> GenerateCandidates_circle_linear( MapObject& sdf_object, std::vector<geometry_msgs::Pose> & RobotPoses, int &linear_interpolation_nums, double radius=3 /*半径*/ , double camera_height = 1.0, double init_robot_x=0, double init_robot_y=0, bool forCamera = false , int rotate_divides = 30, int circle_divides = 240 ){

    std::vector<geometry_msgs::Pose> candidates;
    linear_interpolation_nums = 0;  //直线差值的数量
    // 物体的位姿
    double object_x = sdf_object.mCuboid3D.cuboidCenter.x();
    double object_y = sdf_object.mCuboid3D.cuboidCenter.y();
    double object_z = sdf_object.mCuboid3D.cuboidCenter.z();
    // double object_z = 0;

    std::vector<Point> InterS;

    Ellipse ellipse = { radius, radius, sdf_object.mCuboid3D.cuboidCenter.x(), sdf_object.mCuboid3D.cuboidCenter.y(), sdf_object.mCuboid3D.rotY };

    double angle_init = std::atan2( init_robot_y-sdf_object.mCuboid3D.cuboidCenter.y() , init_robot_x-sdf_object.mCuboid3D.cuboidCenter.x() );

    double circle_start_x = radius*cos(angle_init)+object_x;
    double circle_start_y = radius*sin(angle_init)+object_y;

    // (1) 直线部分
    // double line_end_x = circle_start_x;
    // double line_end_y = circle_start_y;
    // double line_distance_mini = 0.03;
    // int distance_nums = sqrt(line_end_x*line_end_x+line_end_y*line_end_y)/line_distance_mini;
    // Eigen::Vector2d start(init_robot_x, init_robot_y);  // 起点
    // Eigen::Vector2d end(line_end_x, line_end_y);  // 终点
    // Eigen::Vector2d direction = end - start;
    // for(int i=distance_nums; i>0; i--){
    //
    //     double t = 1-static_cast<double>(i) / distance_nums; // 归一化参数
    //     Eigen::Vector2d now_point = start + t * direction; // 插值计算
    //     double footprint_x = now_point.x();
    //     double footprint_y = now_point.y();
    //     double footprint_z = 0.0;
    //     tf::Quaternion q_footprint = tf::createQuaternionFromRPY(0, 0, angle_init+M_PI);
    //     geometry_msgs::Pose T_world_footprint;
    //     T_world_footprint.position.x = footprint_x;
    //     T_world_footprint.position.y = footprint_y;
    //     T_world_footprint.position.z = footprint_z;
    //     T_world_footprint.orientation.x = q_footprint.x();
    //     T_world_footprint.orientation.y = q_footprint.y();
    //     T_world_footprint.orientation.z = q_footprint.z();
    //     T_world_footprint.orientation.w = q_footprint.w();
    //     RobotPoses.push_back(T_world_footprint);
    //
    //
    //     // 相机位姿
    //     double camera_x = footprint_x;
    //     double camera_y = footprint_y;
    //     double camera_z = camera_height;
    //     double deltaX = object_x - camera_x;
    //     double deltaY = object_y - camera_y;
    //     double deltaZ = object_z - camera_z;
    //     double delta_pitch = std::atan2(deltaZ, std::sqrt(deltaX*deltaX + deltaY*deltaY));
    //
    //     // 计算candidate在foot_print中的位姿
    //     tf::Quaternion q_c = tf::createQuaternionFromRPY(0, -1*delta_pitch, 0);
    //     geometry_msgs::Pose T_footprint_cameralink;
    //     T_footprint_cameralink.position.x = 0.0;
    //     T_footprint_cameralink.position.y = 0;
    //     T_footprint_cameralink.position.z = camera_z - footprint_z;
    //     T_footprint_cameralink.orientation.x = q_c.x();
    //     T_footprint_cameralink.orientation.y = q_c.y();
    //     T_footprint_cameralink.orientation.z = q_c.z();
    //     T_footprint_cameralink.orientation.w = q_c.w();
    //
    //     candidates.push_back(T_footprint_cameralink );
    //
    //     linear_interpolation_nums ++;
    // }

    // (2) 原地左旋转90度 部分
    // double rotate_mini = 5*M_PI/180;
    // int rotate_nums = M_PI_2/rotate_mini;
    int rotate_nums = rotate_divides;
    for(int i=rotate_nums; i>0; i--){

        double t = 1-static_cast<double>(i) / rotate_nums; // 归一化参数
        double delta_yaw = t*M_PI_2;
        double footprint_x = circle_start_x;
        double footprint_y = circle_start_y;
        double footprint_z = 0.0;
        tf::Quaternion q_footprint = tf::createQuaternionFromRPY(0, 0, angle_init+M_PI + delta_yaw);
        geometry_msgs::Pose T_world_footprint;
        T_world_footprint.position.x = footprint_x;
        T_world_footprint.position.y = footprint_y;
        T_world_footprint.position.z = footprint_z;
        T_world_footprint.orientation.x = q_footprint.x();
        T_world_footprint.orientation.y = q_footprint.y();
        T_world_footprint.orientation.z = q_footprint.z();
        T_world_footprint.orientation.w = q_footprint.w();
        RobotPoses.push_back(T_world_footprint);


        // 相机位姿
        double camera_x = footprint_x;
        double camera_y = footprint_y;
        double camera_z = camera_height;
        double deltaX = object_x - camera_x;
        double deltaY = object_y - camera_y;
        double deltaZ = object_z - camera_z;
        double delta_pitch = std::atan2(deltaZ, std::sqrt(deltaX*deltaX + deltaY*deltaY));

        // 计算candidate在foot_print中的位姿
        tf::Quaternion q_c = tf::createQuaternionFromRPY(0, -1*delta_pitch, -1*delta_yaw);
        geometry_msgs::Pose T_footprint_cameralink;
        T_footprint_cameralink.position.x = 0.0;
        T_footprint_cameralink.position.y = 0;
        T_footprint_cameralink.position.z = camera_z - footprint_z;
        T_footprint_cameralink.orientation.x = q_c.x();
        T_footprint_cameralink.orientation.y = q_c.y();
        T_footprint_cameralink.orientation.z = q_c.z();
        T_footprint_cameralink.orientation.w = q_c.w();

        candidates.push_back(T_footprint_cameralink );
    }



    // (3) 绕圆部分
    int divide = circle_divides;
    double Max_angle_range = 2*M_PI;
    double miniA = Max_angle_range/divide;
    for(int i=divide; i>0; i--){

        double angle = angle_init + i*miniA;


        // 底盘的位置 通过圆计算
        double footprint_x = radius*cos(angle)+object_x;
        double footprint_y = radius*sin(angle)+object_y;
        Point intersection{footprint_x, footprint_y};
        double footprint_z = 0.0;
        double delta_yaw_footprint = calculateAngleWithXAxis(ellipse, intersection);  // 椭圆上切线与x轴的夹角
        tf::Quaternion q_footprint = tf::createQuaternionFromRPY(0, 0, delta_yaw_footprint/180*M_PI);
        geometry_msgs::Pose T_world_footprint;
        T_world_footprint.position.x = footprint_x;
        T_world_footprint.position.y = footprint_y;
        T_world_footprint.position.z = footprint_z;
        T_world_footprint.orientation.x = q_footprint.x();
        T_world_footprint.orientation.y = q_footprint.y();
        T_world_footprint.orientation.z = q_footprint.z();
        T_world_footprint.orientation.w = q_footprint.w();
        RobotPoses.push_back(T_world_footprint);


        // 相机位姿
        double camera_x = intersection.x;
        double camera_y = intersection.y;
        double camera_z = camera_height;
        double deltaX = object_x - camera_x;
        double deltaY = object_y - camera_y;
        double deltaZ = object_z - camera_z;
        double delta_pitch = std::atan2(deltaZ, std::sqrt(deltaX*deltaX + deltaY*deltaY));
        double yaw = calculateAngle(ellipse, intersection);
        double delta_yaw = yaw;  // -1*M_PI_2;


        // 计算candidate在foot_print中的位姿
        tf::Quaternion q_c = tf::createQuaternionFromRPY(0, -1*delta_pitch, -1* delta_yaw);
        geometry_msgs::Pose T_footprint_cameralink;
        T_footprint_cameralink.position.x = 0;
        T_footprint_cameralink.position.y = 0.5;
        T_footprint_cameralink.position.z = camera_z - footprint_z;
        T_footprint_cameralink.orientation.x = q_c.x();
        T_footprint_cameralink.orientation.y = q_c.y();
        T_footprint_cameralink.orientation.z = q_c.z();
        T_footprint_cameralink.orientation.w = q_c.w();

        candidates.push_back(T_footprint_cameralink );

    }

    return candidates;

}





std::vector<geometry_msgs::Pose> GenerateCandidates_circle( MapObject& sdf_object, std::vector<geometry_msgs::Pose> & RobotPoses, double radius=3 /*半径*/ , double camera_height = 1.0, double init_robot_x=0, double init_robot_y=0, bool forCamera = false , int divide_ = 240, bool Clockwise = true ){

    std::vector<geometry_msgs::Pose> candidates;

    // 物体的位姿
    double object_x = sdf_object.mCuboid3D.cuboidCenter.x();
    double object_y = sdf_object.mCuboid3D.cuboidCenter.y();
    double object_z = sdf_object.mCuboid3D.cuboidCenter.z();
    // double object_z = 0;

    std::vector<Point> InterS;

    Ellipse ellipse = { radius, radius, sdf_object.mCuboid3D.cuboidCenter.x(), sdf_object.mCuboid3D.cuboidCenter.y(), sdf_object.mCuboid3D.rotY };

    double angle_init = std::atan2( init_robot_y-sdf_object.mCuboid3D.cuboidCenter.y() , init_robot_x-sdf_object.mCuboid3D.cuboidCenter.x() );



    // (3) 绕圆部分
    int divide = divide_;
    double Max_angle_range = 2*M_PI;
    double miniA = Max_angle_range/divide;
    for(int i=divide; i>0; i--){

        // if(!Clockwise)
        //     i = divide - i; // 逆时针

        double angle = angle_init + i*miniA;
        if(!Clockwise)
            angle = angle_init - i*miniA; // 逆时针

        // 底盘的位置 通过圆计算
        double footprint_x = radius*cos(angle)+object_x;
        double footprint_y = radius*sin(angle)+object_y;
        Point intersection{footprint_x, footprint_y};
        double footprint_z = 0.0;
        double delta_yaw_footprint = calculateAngleWithXAxis(ellipse, intersection);  // 椭圆上切线与x轴的夹角
        tf::Quaternion q_footprint = tf::createQuaternionFromRPY(0, 0, delta_yaw_footprint/180*M_PI);
        if(!Clockwise)  // 逆时针
            q_footprint = tf::createQuaternionFromRPY(0, 0, delta_yaw_footprint/180*M_PI + M_PI);
        geometry_msgs::Pose T_world_footprint;
        T_world_footprint.position.x = footprint_x;
        T_world_footprint.position.y = footprint_y;
        T_world_footprint.position.z = footprint_z;
        T_world_footprint.orientation.x = q_footprint.x();
        T_world_footprint.orientation.y = q_footprint.y();
        T_world_footprint.orientation.z = q_footprint.z();
        T_world_footprint.orientation.w = q_footprint.w();
        RobotPoses.push_back(T_world_footprint);


        // 相机位姿
        double camera_x = intersection.x;
        double camera_y = intersection.y;
        double camera_z = camera_height;
        double deltaX = object_x - camera_x;
        double deltaY = object_y - camera_y;
        double deltaZ = object_z - camera_z;
        double delta_pitch = std::atan2(deltaZ, std::sqrt(deltaX*deltaX + deltaY*deltaY));
        double yaw = calculateAngle(ellipse, intersection);
        double delta_yaw = yaw;  // -1*M_PI_2;



        // [重要] 以下计算的Candidate位姿，并没有用到。当前程序直接用的机械臂关节角度，作为优化初始值
        // 计算candidate在foot_print中的位姿
        tf::Quaternion q_c = tf::createQuaternionFromRPY(0, -1*delta_pitch, -1* delta_yaw);
        if(!Clockwise) { // 逆时针c
            q_c = tf::createQuaternionFromRPY(0, -1*delta_pitch, M_PI - delta_yaw);
            // std::cout<<"calculateRelativePose: "<< M_PI - delta_yaw     <<std::endl;
        }
        // if(!Clockwise)  // 逆时针c
        // {
        //     tf::Quaternion q_fix;
        //     q_fix.setRPY(0, 0, M_PI);  // 额外绕 Z 轴旋转 180°
        //     q_c = q_c * q_fix;  // 组合旋转
        // }
        geometry_msgs::Pose T_footprint_cameralink;
        T_footprint_cameralink.position.x = 0;
        T_footprint_cameralink.position.y = 0.5;
        T_footprint_cameralink.position.z = camera_z - footprint_z;
        T_footprint_cameralink.orientation.x = q_c.x();
        T_footprint_cameralink.orientation.y = q_c.y();
        T_footprint_cameralink.orientation.z = q_c.z();
        T_footprint_cameralink.orientation.w = q_c.w();



        // std::cout<<"calculateRelativePose"
        //     <<", delta_roll: 0"
        //     <<", delta_yaw:"<< -1* delta_yaw/M_PI*180
        //     <<", delta_pitch:"<< -1*delta_pitch/M_PI*180
        //     <<", qx"<< T_footprint_cameralink.orientation.x
        //     <<", qy"<< T_footprint_cameralink.orientation.y
        //     <<", qz"<< T_footprint_cameralink.orientation.z
        //     <<", qw"<< T_footprint_cameralink.orientation.w
        //     <<std::endl;

        bool use_CameraOpticalFrame = false;

        if(!use_CameraOpticalFrame){
            candidates.push_back(T_footprint_cameralink );
        }
        else{
            Eigen::Matrix4d T_footprint_CameraLink_matrix = Converter::geometryPosetoMatrix4d(T_footprint_cameralink);
            Eigen::Matrix4d T_CameraLink_to_CameraOpticalFrame_matrix;
            T_CameraLink_to_CameraOpticalFrame_matrix << 0, 0, 1, 0.02,
                                            -1, 0, 0, -0.013,
                                            0, -1, 0, 0.0,  //实际为0.13，改为0.07
                                            0, 0, 0, 1;

            geometry_msgs::Pose T_footprint_CameraOpticalFrame = Converter::Matrix4dtoGeometryPose(T_footprint_CameraLink_matrix*T_CameraLink_to_CameraOpticalFrame_matrix);
            candidates.push_back(T_footprint_CameraOpticalFrame );
        }
        
    }
    
    return candidates;

}


std::vector<double> GenerateCandidates_circle_onlyfordirect( MapObject& sdf_object, std::vector<geometry_msgs::Pose> & RobotPoses, double radius=3 /*半径*/ , double camera_height = 1.0, double init_robot_x=0, double init_robot_y=0, bool forCamera = false , int divide_ = 240, bool Clockwise = true ){

    std::vector<double> direct_yaws;

    // 物体的位姿
    double object_x = sdf_object.mCuboid3D.cuboidCenter.x();
    double object_y = sdf_object.mCuboid3D.cuboidCenter.y();
    double object_z = sdf_object.mCuboid3D.cuboidCenter.z();
    // double object_z = 0;

    std::vector<Point> InterS;

    Ellipse ellipse = { radius, radius, sdf_object.mCuboid3D.cuboidCenter.x(), sdf_object.mCuboid3D.cuboidCenter.y(), sdf_object.mCuboid3D.rotY };

    double angle_init = std::atan2( init_robot_y-sdf_object.mCuboid3D.cuboidCenter.y() , init_robot_x-sdf_object.mCuboid3D.cuboidCenter.x() );



    // (3) 绕圆部分
    int divide = divide_;
    double Max_angle_range = 2*M_PI;
    double miniA = Max_angle_range/divide;
    for(int i=divide; i>0; i--){

        // if(!Clockwise)
        //     i = divide - i; // 逆时针

        double angle = angle_init + i*miniA;
        if(!Clockwise)
            angle = angle_init - i*miniA; // 逆时针

        // 底盘的位置 通过圆计算
        double footprint_x = radius*cos(angle)+object_x;
        double footprint_y = radius*sin(angle)+object_y;
        Point intersection{footprint_x, footprint_y};
        double footprint_z = 0.0;
        double delta_yaw_footprint = calculateAngleWithXAxis(ellipse, intersection);  // 椭圆上切线与x轴的夹角
        tf::Quaternion q_footprint = tf::createQuaternionFromRPY(0, 0, delta_yaw_footprint/180*M_PI);
        if(!Clockwise)  // 逆时针
            q_footprint = tf::createQuaternionFromRPY(0, 0, delta_yaw_footprint/180*M_PI + M_PI);
        geometry_msgs::Pose T_world_footprint;
        T_world_footprint.position.x = footprint_x;
        T_world_footprint.position.y = footprint_y;
        T_world_footprint.position.z = footprint_z;
        T_world_footprint.orientation.x = q_footprint.x();
        T_world_footprint.orientation.y = q_footprint.y();
        T_world_footprint.orientation.z = q_footprint.z();
        T_world_footprint.orientation.w = q_footprint.w();
        RobotPoses.push_back(T_world_footprint);


        // 相机位姿
        double camera_x = intersection.x;
        double camera_y = intersection.y;
        double camera_z = camera_height;
        double deltaX = object_x - camera_x;
        double deltaY = object_y - camera_y;
        double deltaZ = object_z - camera_z;
        double delta_pitch = std::atan2(deltaZ, std::sqrt(deltaX*deltaX + deltaY*deltaY));
        double yaw = calculateAngle(ellipse, intersection);
        double delta_yaw = yaw;  // -1*M_PI_2;
        direct_yaws.push_back(delta_yaw);


        
    }
    
    return direct_yaws;

}



std::vector<geometry_msgs::Pose> GenerateCandidates_ellipse( MapObject& sdf_object, std::vector<geometry_msgs::Pose> & RobotPoses, double radius=3 /*短轴*/ , double camera_height = 1.0, bool forCamera = false , int divide_ = 240 ){

    std::vector<geometry_msgs::Pose> candidates;
    // 长轴和短轴
    double a = radius;
    double b = radius; //+0.5; //*sdf_object.mCuboid3D.lenth/sdf_object.mCuboid3D.width;

    // 物体的位姿
    double object_x = sdf_object.mCuboid3D.cuboidCenter.x();
    double object_y = sdf_object.mCuboid3D.cuboidCenter.y();
    double object_z = sdf_object.mCuboid3D.cuboidCenter.z();
    // double object_z = 0;

    std::vector<Point> InterS;
    
    Ellipse ellipse = { a, b, sdf_object.mCuboid3D.cuboidCenter.x(), sdf_object.mCuboid3D.cuboidCenter.y(), sdf_object.mCuboid3D.rotY };

    int divide = divide_;
    double Max_angle_range = 2*M_PI;
    double miniA = Max_angle_range/divide;
    for(int i=divide; i>0; i--){

        double angle = i*miniA - 3*M_PI/4;


        // 底盘的位置 通过圆计算
        Point intersection = calculateIntersection(ellipse, angle);
        double footprint_x = intersection.x;
        double footprint_y = intersection.y;
        double footprint_z = 0.0;
        double delta_yaw_footprint = calculateAngleWithXAxis(ellipse, intersection);  // 椭圆上切线与x轴的夹角
        tf::Quaternion q_footprint = tf::createQuaternionFromRPY(0, 0, delta_yaw_footprint/180*M_PI);
        geometry_msgs::Pose T_world_footprint;
        T_world_footprint.position.x = footprint_x;
        T_world_footprint.position.y = footprint_y;
        T_world_footprint.position.z = footprint_z;
        T_world_footprint.orientation.x = q_footprint.x();
        T_world_footprint.orientation.y = q_footprint.y();
        T_world_footprint.orientation.z = q_footprint.z();
        T_world_footprint.orientation.w = q_footprint.w();
        RobotPoses.push_back(T_world_footprint);


        // 相机位姿
        double camera_x = intersection.x;
        double camera_y = intersection.y;
        double camera_z = camera_height;
        double deltaX = object_x - camera_x;
        double deltaY = object_y - camera_y;
        double deltaZ = object_z - camera_z;
        double delta_pitch = std::atan2(deltaZ, std::sqrt(deltaX*deltaX + deltaY*deltaY));
        double yaw = calculateAngle(ellipse, intersection);
        double delta_yaw = yaw;  // -1*M_PI_2;


        // 计算candidate在foot_print中的位姿
        tf::Quaternion q_c = tf::createQuaternionFromRPY(0, -1*delta_pitch, -1* delta_yaw);
        geometry_msgs::Pose T_footprint_cameralink;
        T_footprint_cameralink.position.x = 0;
        T_footprint_cameralink.position.y = 0.5;
        T_footprint_cameralink.position.z = camera_z - footprint_z;
        T_footprint_cameralink.orientation.x = q_c.x();
        T_footprint_cameralink.orientation.y = q_c.y();
        T_footprint_cameralink.orientation.z = q_c.z();
        T_footprint_cameralink.orientation.w = q_c.w();



        std::cout<<"calculateRelativePose"
            <<", delta_roll: 0"
            <<", delta_yaw:"<< -1* delta_yaw/M_PI*180
            <<", delta_pitch:"<< -1*delta_pitch/M_PI*180
            <<", qx"<< T_footprint_cameralink.orientation.x
            <<", qy"<< T_footprint_cameralink.orientation.y
            <<", qz"<< T_footprint_cameralink.orientation.z
            <<", qw"<< T_footprint_cameralink.orientation.w

            <<std::endl;


        Eigen::Matrix4d T_footprint_cameralink_matrix = Converter::geometryPosetoMatrix4d(T_footprint_cameralink);
        Eigen::Matrix4d T_cameralink_to_camera_matrix;
        T_cameralink_to_camera_matrix << 0, 0, 1, 0.02,
                                        -1, 0, 0, -0.013,
                                        0, -1, 0, 0.0,  //实际为0.13，改为0.07
                                        0, 0, 0, 1;

        geometry_msgs::Pose T_footprint_camera = Converter::Matrix4dtoGeometryPose(T_footprint_cameralink_matrix*T_cameralink_to_camera_matrix);
        candidates.push_back(T_footprint_camera );

    }
    
    return candidates;

}


std::vector<geometry_msgs::Pose> GenerateCandidates_ellipse_by_circle( MapObject& sdf_object, std::vector<geometry_msgs::Pose> & RobotPoses, double radius=3 /*短轴*/ , double camera_height=1.0  , bool forCamera = false , int divide_ = 240){

    std::vector<geometry_msgs::Pose> candidates;
    // 长轴和短轴
    // double a = radius;
    // double b = radius+0.5; //radius*sdf_object.length/sdf_object.width;
    // radius += 0.2;
    double a = radius;
    double b = radius+0.8; //radius*sdf_object.length/sdf_object.width;
    radius += 0.4;

    // 物体的位姿
    double object_x = sdf_object.mCuboid3D.cuboidCenter.x();
    double object_y = sdf_object.mCuboid3D.cuboidCenter.y();
    double object_z = 0;

    std::vector<Point> InterS;
    
    Ellipse ellipse = { a, b, sdf_object.mCuboid3D.cuboidCenter.x(), sdf_object.mCuboid3D.cuboidCenter.y() };

    int divide = divide_;
    double Max_angle_range = 2*M_PI;
    double miniA = Max_angle_range/divide;
    for(int i=divide; i>0; i--){

        double angle = i*miniA - 3*M_PI/4;


        // 底盘的位置 通过圆计算
        double footprint_x = radius*cos(angle)+object_x;
        double footprint_y = radius*sin(angle)+object_y;
        double footprint_z = 0.0;   
        Ellipse circle = { a, a, sdf_object.mCuboid3D.cuboidCenter.x(), sdf_object.mCuboid3D.cuboidCenter.y() };
        Point intersection_circle{footprint_x,footprint_y};
        double delta_yaw_footprint = calculateAngleWithXAxis(circle, intersection_circle);  // 圆上切线与x轴的夹角
        tf::Quaternion q_footprint = tf::createQuaternionFromRPY(0, 0, delta_yaw_footprint/180*M_PI);
        geometry_msgs::Pose T_world_footprint;
        T_world_footprint.position.x = footprint_x;
        T_world_footprint.position.y = footprint_y;
        T_world_footprint.position.z = footprint_z;
        T_world_footprint.orientation.x = q_footprint.x();
        T_world_footprint.orientation.y = q_footprint.y();
        T_world_footprint.orientation.z = q_footprint.z();
        T_world_footprint.orientation.w = q_footprint.w();
        RobotPoses.push_back(T_world_footprint);


        // 相机的位置通过椭圆计算
        Point intersection = calculateIntersection(ellipse, angle);  // 计算射线与椭圆的交点
        InterS.push_back(intersection);
        double delta_yaw = calculateAngle(ellipse, intersection); // 计算切线与 AC 之间的夹角
        std::cout<<"切线夹角: "<<delta_yaw<<std::endl;
        // 相机位姿
        double camera_x = intersection.x;
        double camera_y = intersection.y;
        double camera_z = camera_height;
        double deltaX = object_x - camera_x;
        double deltaY = object_y - camera_y;
        double deltaZ = object_z - camera_z;
        double delta_pitch = std::atan2(deltaZ, std::sqrt(deltaX*deltaX + deltaY*deltaY));


        // 计算candidate在foot_print中的位姿
        tf::Quaternion q_c = tf::createQuaternionFromRPY(0, -1*delta_pitch, -1* delta_yaw);
        geometry_msgs::Pose T_footprint_cameralink;
        T_footprint_cameralink.position.x = 0;
        T_footprint_cameralink.position.y = std::sqrt(deltaX*deltaX + deltaY*deltaY) - std::sqrt((object_x-footprint_x)*(object_x-footprint_x) + (object_y-footprint_y)*(object_y-footprint_y));
        T_footprint_cameralink.position.z = camera_z - footprint_z;
        T_footprint_cameralink.orientation.x = q_c.x();
        T_footprint_cameralink.orientation.y = q_c.y();
        T_footprint_cameralink.orientation.z = q_c.z();
        T_footprint_cameralink.orientation.w = q_c.w();

        std::cout<<"calculateRelativePose"
            <<", delta_roll: 0"
            <<", delta_yaw:"<< delta_yaw
            <<", delta_pitch:"<< delta_pitch
            <<", qx"<< T_footprint_cameralink.orientation.x
            <<", qy"<< T_footprint_cameralink.orientation.y
            <<", qz"<< T_footprint_cameralink.orientation.z
            <<", qw"<< T_footprint_cameralink.orientation.w
            <<", y"<<T_footprint_cameralink.position.y

            <<std::endl;

        Eigen::Matrix4d T_footprint_cameralink_matrix = Converter::geometryPosetoMatrix4d(T_footprint_cameralink);
        Eigen::Matrix4d T_cameralink_to_camera_matrix;
        T_cameralink_to_camera_matrix << 0, 0, 1, 0.02,
                                        -1, 0, 0, -0.013,
                                        0, -1, 0, 0.0,  //实际为0.13，改为0.07
                                        0, 0, 0, 1;

        geometry_msgs::Pose T_footprint_camera = Converter::Matrix4dtoGeometryPose(T_footprint_cameralink_matrix*T_cameralink_to_camera_matrix);
        candidates.push_back(T_footprint_camera );

    }
    
    return candidates;

}


std::vector<geometry_msgs::Pose> GenerateCandidates_future( MapObject& sdf_object, Eigen::Matrix3f& robot_pose, double radius=3 ){

    std::vector<geometry_msgs::Pose> candidates;
    
    double angle = 0;
    // 物体的位姿
    double object_x = sdf_object.mCuboid3D.cuboidCenter.x();
    double object_y = sdf_object.mCuboid3D.cuboidCenter.y();
    double object_z = sdf_object.mCuboid3D.cuboidCenter.z();

    for(int i=0; i<18; i++){

        // 相机的目标位姿
        angle = i*10/180.0*M_PI ;
        double footprint_x = radius*cos(angle)+object_x;
        double footprint_y = radius*sin(angle)+object_y;
        double footprint_z = 0.0;

        double camera_x = footprint_x;
        double camera_y = footprint_y;
        double camera_z = 1.0;

        double deltaX = object_x - camera_x;
        double deltaY = object_y - camera_y;
        double delta_yaw = std::atan2(deltaY, deltaX);  // -1*M_PI_2;
        double deltaZ = object_z - camera_z;
        double delta_pitch = std::atan2(deltaZ, std::sqrt(deltaX*deltaX + deltaY*deltaY));

        // 相机的目标位姿[todo: 引入当前的机器人位姿]
        // double footprint_x = robot_footprint_pose.position.x;
        // double footprint_y = robot_footprint_pose.position.y;
        // double footprint_z = robot_footprint_pose.position.z;
        
        // double camera_x = footprint_x;
        // double camera_y = footprint_y;
        // double camera_z = 1.0;
        
        // 计算candidate在foot_print中的位姿
        // tf::Quaternion q = tf::createQuaternionFromRPY(0, -1*delta_pitch, delta_yaw);
        // 创建四元数
        Eigen::Quaterniond q;

        // 计算四元数
        q = Eigen::AngleAxisd(-1*delta_yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(-1*delta_pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());

        geometry_msgs::Pose T_footprint_camera;

        T_footprint_camera.position.x = camera_x - footprint_x;
        T_footprint_camera.position.y = camera_y - footprint_y;
        T_footprint_camera.position.z = camera_z - footprint_z;
        
        T_footprint_camera.orientation.x = q.x();
        T_footprint_camera.orientation.y = q.y(); 
        T_footprint_camera.orientation.z = q.z();
        T_footprint_camera.orientation.w = q.w(); 

        std::cout<<"calculateRelativePose"
            <<", delta_yaw:"<< delta_yaw
            <<", delta_pitch:"<< delta_pitch
            <<", x:"<<T_footprint_camera.position.x
            <<", y:"<<T_footprint_camera.position.y
            <<", z:"<<T_footprint_camera.position.z
            <<std::endl;
        

        candidates.push_back(T_footprint_camera);

    }
    
    return candidates;

}


std::vector<geometry_msgs::Pose> GenerateCandidates_real( MapObject& sdf_object, std::vector<geometry_msgs::Pose> & RobotPoses, double radius=3 /*半径*/ , double camera_height = 1.0, double init_robot_x=0, double init_robot_y=0, bool forCamera = false , int divide_ = 240, bool Clockwise = true ){

    std::vector<geometry_msgs::Pose> candidates;
    
    // 长轴和短轴
    double a = radius;
    double b = radius*sdf_object.mCuboid3D.lenth/sdf_object.mCuboid3D.width * 0.8;

    // 物体的位姿
    double object_x = sdf_object.mCuboid3D.cuboidCenter.x();
    double object_y = sdf_object.mCuboid3D.cuboidCenter.y();
    double object_z = sdf_object.mCuboid3D.cuboidCenter.z();
    // double object_z = 0;

    std::vector<Point> InterS;

    Ellipse ellipse = { a,b, sdf_object.mCuboid3D.cuboidCenter.x(), sdf_object.mCuboid3D.cuboidCenter.y(), sdf_object.mCuboid3D.rotY };

    double angle_init = std::atan2( init_robot_y-sdf_object.mCuboid3D.cuboidCenter.y() , init_robot_x-sdf_object.mCuboid3D.cuboidCenter.x() );



    // (3) 绕圆部分
    int divide = divide_;
    double Max_angle_range = 2*M_PI;
    double miniA = Max_angle_range/divide;
    for(int i=divide; i>0; i--){

        // if(!Clockwise)
        //     i = divide - i; // 逆时针

        // 顺时针
        double angle = angle_init + i*miniA;
        // 逆时针
        if(!Clockwise)
            angle = angle_init - i*miniA; // 逆时针

        // 底盘的位置 通过圆计算
        Point intersection = calculateIntersection(ellipse, angle);
        double footprint_x = intersection.x;
        double footprint_y = intersection.y;
        // double footprint_x = radius*cos(angle)+object_x;
        // double footprint_y = radius*sin(angle)+object_y;
        // Point intersection{footprint_x, footprint_y};
        double footprint_z = 0.0;
        double delta_yaw_footprint = calculateAngleWithXAxis(ellipse, intersection);  // 椭圆上切线与x轴的夹角
        tf::Quaternion q_footprint = tf::createQuaternionFromRPY(0, 0, delta_yaw_footprint/180*M_PI);
        if(!Clockwise)  // 逆时针
            q_footprint = tf::createQuaternionFromRPY(0, 0, delta_yaw_footprint/180*M_PI + M_PI);
        geometry_msgs::Pose T_world_footprint;
        T_world_footprint.position.x = footprint_x;
        T_world_footprint.position.y = footprint_y;
        T_world_footprint.position.z = footprint_z;
        T_world_footprint.orientation.x = q_footprint.x();
        T_world_footprint.orientation.y = q_footprint.y();
        T_world_footprint.orientation.z = q_footprint.z();
        T_world_footprint.orientation.w = q_footprint.w();
        RobotPoses.push_back(T_world_footprint);


        // 相机位姿
        double camera_x = intersection.x;
        double camera_y = intersection.y;
        double camera_z = camera_height;
        double deltaX = object_x - camera_x;
        double deltaY = object_y - camera_y;
        double deltaZ = object_z - camera_z;
        double delta_pitch = std::atan2(deltaZ, std::sqrt(deltaX*deltaX + deltaY*deltaY));
        double yaw = calculateAngle(ellipse, intersection);
        double delta_yaw = yaw;  // -1*M_PI_2;



        // [重要] 以下计算的Candidate位姿，并没有用到。当前程序直接用的机械臂关节角度，作为优化初始值
        // 计算candidate在foot_print中的位姿
        tf::Quaternion q_c = tf::createQuaternionFromRPY(0, -1*delta_pitch, -1* delta_yaw);
        if(!Clockwise) { // 逆时针c
            q_c = tf::createQuaternionFromRPY(0, -1*delta_pitch, M_PI - delta_yaw);
            // std::cout<<"calculateRelativePose: "<< M_PI - delta_yaw     <<std::endl;
        }
        // if(!Clockwise)  // 逆时针c
        // {
        //     tf::Quaternion q_fix;
        //     q_fix.setRPY(0, 0, M_PI);  // 额外绕 Z 轴旋转 180°
        //     q_c = q_c * q_fix;  // 组合旋转
        // }
        geometry_msgs::Pose T_footprint_cameralink;
        T_footprint_cameralink.position.x = 0;
        T_footprint_cameralink.position.y = 0;
        T_footprint_cameralink.position.z = camera_z - footprint_z;
        T_footprint_cameralink.orientation.x = q_c.x();
        T_footprint_cameralink.orientation.y = q_c.y();
        T_footprint_cameralink.orientation.z = q_c.z();
        T_footprint_cameralink.orientation.w = q_c.w();



        // std::cout<<"calculateRelativePose"
        //     <<", delta_roll: 0"
        //     <<", delta_yaw:"<< -1* delta_yaw/M_PI*180
        //     <<", delta_pitch:"<< -1*delta_pitch/M_PI*180
        //     <<", qx"<< T_footprint_cameralink.orientation.x
        //     <<", qy"<< T_footprint_cameralink.orientation.y
        //     <<", qz"<< T_footprint_cameralink.orientation.z
        //     <<", qw"<< T_footprint_cameralink.orientation.w
        //     <<std::endl;

        bool use_CameraOpticalFrame = false;

        if(!use_CameraOpticalFrame){
            candidates.push_back(T_footprint_cameralink );
        }
        else{
            Eigen::Matrix4d T_footprint_CameraLink_matrix = Converter::geometryPosetoMatrix4d(T_footprint_cameralink);
            Eigen::Matrix4d T_CameraLink_to_CameraOpticalFrame_matrix;
            T_CameraLink_to_CameraOpticalFrame_matrix << 0, 0, 1, 0.02,
                                            -1, 0, 0, -0.013,
                                            0, -1, 0, 0.0,  //实际为0.13，改为0.07
                                            0, 0, 0, 1;

            geometry_msgs::Pose T_footprint_CameraOpticalFrame = Converter::Matrix4dtoGeometryPose(T_footprint_CameraLink_matrix*T_CameraLink_to_CameraOpticalFrame_matrix);
            candidates.push_back(T_footprint_CameraOpticalFrame );
        }
        
    }
    
    return candidates;

}


#endif // VIEW_PLANNING_CANDIDATE_H