#ifndef VIEW_PLANNING_CONVERTERTOOLS_H
#define VIEW_PLANNING_CONVERTERTOOLS_H

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/TransformStamped.h>

#include<Eigen/Dense>
#include <tf2/convert.h>
#include <geometry_msgs/Pose.h>
#include <tf2/transform_datatypes.h> // Include if needed
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include<opencv2/core/eigen.hpp>

#include <gtsam/base/Matrix.h>


// 用于计算Y轴水平、x轴指向目标点的坐标系(在我的工作中是相机Candidate坐标系)，在world中的位姿
geometry_msgs::Pose ZeroRoll_Pose_of_TwoPoints_in_world(const Eigen::Vector3d& start_point_in_world, const Eigen::Vector3d& end_point_in_world) {

    double object_x = end_point_in_world.x();
    double object_y = end_point_in_world.y();
    double object_z = end_point_in_world.z();

    double camera_x = start_point_in_world.x();
    double camera_y = start_point_in_world.y();
    double camera_z = start_point_in_world.z();

    double deltaX = object_x - camera_x;
    double deltaY = object_y - camera_y;
    double delta_yaw = std::atan2(deltaY, deltaX);
    double deltaZ = object_z - camera_z;
    double delta_pitch = std::atan2(deltaZ, std::sqrt(deltaX*deltaX + deltaY*deltaY));

    tf::Quaternion q = tf::createQuaternionFromRPY(0, -1*delta_pitch, delta_yaw);

    
    geometry_msgs::Pose T_world_camera;

    T_world_camera.position.x = camera_x;
    T_world_camera.position.y = camera_y;
    T_world_camera.position.z = camera_z;
    
    T_world_camera.orientation.x = q.x();
    T_world_camera.orientation.y = q.y(); 
    T_world_camera.orientation.z = q.z();
    T_world_camera.orientation.w = q.w(); 

    return T_world_camera;
}

geometry_msgs::Pose ZeroRoll_Pose_of_TwoPoints_in_object(const Eigen::Vector3d& start_point) {

    double camera_x = start_point.x();
    double camera_y = start_point.y();
    double camera_z = start_point.z();

    double deltaX = 0 - camera_x;
    double deltaY = 0 - camera_y;
    double delta_yaw = std::atan2(deltaY, deltaX);
    double deltaZ = 0 - camera_z;
    double delta_pitch = std::atan2(deltaZ, std::sqrt(deltaX*deltaX + deltaY*deltaY));

    tf::Quaternion q = tf::createQuaternionFromRPY(0, -1*delta_pitch, delta_yaw);

    geometry_msgs::Pose T_world_camera;

    T_world_camera.position.x = camera_x;
    T_world_camera.position.y = camera_y;
    T_world_camera.position.z = camera_z;
    
    T_world_camera.orientation.x = q.x();
    T_world_camera.orientation.y = q.y(); 
    T_world_camera.orientation.z = q.z();
    T_world_camera.orientation.w = q.w(); 

    return T_world_camera;
}


// geometry_msgs::Pose calculateRelativePose(const geometry_msgs::Pose& pose_of_parent, const geometry_msgs::Pose& pose_of_target) {
//     // Convert geometry_msgs::Pose to tf2::Transform
//     tf2::Transform parent_transform;
//     tf2::Transform target_transform;

//     tf2::fromMsg(pose_of_parent, parent_transform);
//     tf2::fromMsg(pose_of_target, target_transform);

//     // Calculate the relative transform
//     tf2::Transform relative_transform = parent_transform.inverse() * target_transform;

//     // Convert tf2::Transform to geometry_msgs::Pose
//     geometry_msgs::Pose relative_pose;
//     tf2::toMsg(relative_transform, relative_pose);

//     return relative_pose;
// }



// Eigen::Vector3d 到 geometry_msgs::Point 的转换函数
geometry_msgs::Point eigen_to_point(Eigen::Vector3d &v){
    geometry_msgs::Point p;
    p.x = v(0);
    p.y = v(1);
    p.z = v(2);
    return p;
}


// 组合两个位姿
geometry_msgs::Pose combinePoses(const geometry_msgs::Pose& pose_goal, const geometry_msgs::Pose& pose_reference)
{
    // 转换为tf2库中的数据类型
    tf2::Transform tf_pose_goal, tf_pose_reference, tf_combined;
    tf2::Quaternion tf_quat1, tf_quat2, tf_combined_quat;
    tf2::Vector3 tf_vec1, tf_vec2, tf_combined_vec;

    // 提取第一个位姿的旋转和平移部分
    tf_pose_goal.setRotation(tf2::Quaternion(pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z, pose_goal.orientation.w));
    tf_pose_goal.setOrigin(tf2::Vector3(pose_goal.position.x, pose_goal.position.y, pose_goal.position.z));

    // 提取第二个位姿的旋转和平移部分
    tf_pose_reference.setRotation(tf2::Quaternion(pose_reference.orientation.x, pose_reference.orientation.y, pose_reference.orientation.z, pose_reference.orientation.w));
    tf_pose_reference.setOrigin(tf2::Vector3(pose_reference.position.x, pose_reference.position.y, pose_reference.position.z));

    // 组合两个位姿
    tf_combined = tf_pose_reference.inverse() * tf_pose_goal;

    // 提取组合后的旋转和平移部分
    tf_combined_quat = tf_combined.getRotation();
    tf_combined_vec = tf_combined.getOrigin();

    // 转换回geometry_msgs中的数据类型
    geometry_msgs::Pose combined_pose;
    combined_pose.orientation.x = tf_combined_quat.x();
    combined_pose.orientation.y = tf_combined_quat.y();
    combined_pose.orientation.z = tf_combined_quat.z();
    combined_pose.orientation.w = tf_combined_quat.w();
    combined_pose.position.x = tf_combined_vec.x();
    combined_pose.position.y = tf_combined_vec.y();
    combined_pose.position.z = tf_combined_vec.z();

    return combined_pose;
}



geometry_msgs::Point corner_to_marker(geometry_msgs::Point& oldp){
        geometry_msgs::Point newp;

        // 坐标点P在A坐标系下的坐标
        double x_a = oldp.x;
        double y_a = oldp.y;
        double z_a = oldp.z;


        newp.x = x_a ;
        newp.y = y_a ;
        newp.z = z_a ;
        return newp;
}



geometry_msgs::Point corner_to_marker(geometry_msgs::Point& oldp, Eigen::Matrix4d& T_world_object){
        geometry_msgs::Point newp;
        
        // 坐标点P在A坐标系下的坐标
        double x_a = oldp.x;
        double y_a = oldp.y;
        double z_a = oldp.z;

        // 将点P从A坐标系变换到B坐标系
        Eigen::Vector4d P_a(x_a, y_a, z_a, 1.0);  // 注意点P_a需要补一个1，才能与矩阵T相乘
        Eigen::Vector4d P_b = T_world_object * P_a;

        // 输出结果
        double x_b = P_b(0);
        double y_b = P_b(1);
        double z_b = P_b(2);
        // std::cout << "Point P in A coordinate system: (" << x_a << ", " << y_a << ", " << z_a << ")" << std::endl;
        // std::cout << "Point P in B coordinate system: (" << x_b << ", " << y_b << ", " << z_b << ")" << std::endl;

        newp.x = x_b ;
        newp.y = y_b ;
        newp.z = z_b ;
        return newp;
}

// 函数：将 Eigen::Isometry3d 转换为 geometry_msgs::PoseStamped
geometry_msgs::PoseStamped eigenToPoseStamped(const Eigen::Isometry3d& eigen_pose, const std::string& frame_id = "world") {
    geometry_msgs::PoseStamped pose_stamped;
    
    // 设置 PoseStamped 的 header
    pose_stamped.header.frame_id = frame_id;
    pose_stamped.header.stamp = ros::Time::now();
    
    // 提取平移部分
    pose_stamped.pose.position.x = eigen_pose.translation().x();
    pose_stamped.pose.position.y = eigen_pose.translation().y();
    pose_stamped.pose.position.z = eigen_pose.translation().z();
    
    // 提取旋转部分并转换为四元数
    Eigen::Quaterniond quat(eigen_pose.rotation());
    pose_stamped.pose.orientation.x = quat.x();
    pose_stamped.pose.orientation.y = quat.y();
    pose_stamped.pose.orientation.z = quat.z();
    pose_stamped.pose.orientation.w = quat.w();
    
    return pose_stamped;
}


// 函数：将 std::vector<double> 转换为 gtsam::Vector
gtsam::Vector stdVectorToGtsamVector(const std::vector<double>& joint_values) {
    int num = joint_values.size();

    // 创建 gtsam::Vector (Eigen::VectorXd) 并赋值
    gtsam::Vector gtsamVector(num);
    for (size_t i = 0; i < num; ++i) {
        gtsamVector(i) = joint_values[i];
    }

    return gtsamVector;
}


// g2o::SE3Quat toSE3Quat(const cv::Mat &cvT)
// {
//     Eigen::Matrix<double,3,3> R;
//     R << cvT.at<float>(0,0), cvT.at<float>(0,1), cvT.at<float>(0,2),
//          cvT.at<float>(1,0), cvT.at<float>(1,1), cvT.at<float>(1,2),
//          cvT.at<float>(2,0), cvT.at<float>(2,1), cvT.at<float>(2,2);

//     Eigen::Matrix<double,3,1> t(cvT.at<float>(0,3), cvT.at<float>(1,3), cvT.at<float>(2,3));

//     return g2o::SE3Quat(R,t);
// }


//  【报错】以下来自ORBSLAM2  一直报错。

// std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors)
// {
//     std::vector<cv::Mat> vDesc;
//     vDesc.reserve(Descriptors.rows);
//     for (int j=0;j<Descriptors.rows;j++)
//         vDesc.push_back(Descriptors.row(j));

//     return vDesc;
// }



// cv::Mat toCvMat(const Eigen::Matrix<double,4,4> &m)
// {
//     cv::Mat cvMat(4,4,CV_32F);
//     for(int i=0;i<4;i++)
//         for(int j=0; j<4; j++)
//             cvMat.at<float>(i,j)=m(i,j);

//     return cvMat.clone();
//     //cv::Mat cv_mat_32f;
//     //cv::eigen2cv(GroundtruthPose_eigen, cv_mat_32f);
//     //cv_mat_32f.convertTo(mT_body_cam, CV_32F);
// }

// cv::Mat toCvMat(const Eigen::Matrix3d &m)
// {
//     cv::Mat cvMat(3,3,CV_32F);
//     for(int i=0;i<3;i++)
//         for(int j=0; j<3; j++)
//             cvMat.at<float>(i,j)=m(i,j);

//     return cvMat.clone();
// }

// cv::Mat toCvMat(const Eigen::Matrix<double,3,1> &m)
// {
//     cv::Mat cvMat(3,1,CV_32F);
//     for(int i=0;i<3;i++)
//             cvMat.at<float>(i)=m(i);

//     return cvMat.clone();
// }

// cv::Mat toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t)
// {
//     cv::Mat cvMat = cv::Mat::eye(4,4,CV_32F);
//     for(int i=0;i<3;i++)
//     {
//         for(int j=0;j<3;j++)
//         {
//             cvMat.at<float>(i,j)=R(i,j);
//         }
//     }
//     for(int i=0;i<3;i++)
//     {
//         cvMat.at<float>(i,3)=t(i);
//     }

//     return cvMat.clone();
// }

// Eigen::Matrix<double,3,1> toVector3d(const cv::Mat &cvVector)
// {
//     Eigen::Matrix<double,3,1> v;
//     v << cvVector.at<float>(0), cvVector.at<float>(1), cvVector.at<float>(2);

//     return v;
// }

// Eigen::Matrix<double,3,1> toVector3d(const cv::Point3f &cvPoint)
// {
//     Eigen::Matrix<double,3,1> v;
//     v << cvPoint.x, cvPoint.y, cvPoint.z;

//     return v;
// }

// Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3)
// {
//     Eigen::Matrix<double,3,3> M;

//     M << cvMat3.at<float>(0,0), cvMat3.at<float>(0,1), cvMat3.at<float>(0,2),
//          cvMat3.at<float>(1,0), cvMat3.at<float>(1,1), cvMat3.at<float>(1,2),
//          cvMat3.at<float>(2,0), cvMat3.at<float>(2,1), cvMat3.at<float>(2,2);

//     return M;
// }

// std::vector<float> toQuaternion(const cv::Mat &M)
// {
//     Eigen::Matrix<double,3,3> eigMat = toMatrix3d(M);
//     Eigen::Quaterniond q(eigMat);

//     std::vector<float> v(4);
//     v[0] = q.x();
//     v[1] = q.y();
//     v[2] = q.z();
//     v[3] = q.w();

//     return v;
// }

// //[active slam]
// float bboxOverlapratio(const cv::Rect& rect1, const cv::Rect& rect2)
// {
//     if (rect1.width == 0 || rect1.height == 0 || rect2.width == 0 || rect2.height == 0) {
//         return 0;
//     }
//     int overlap_area = (rect1&rect2).area();
//     return (float)overlap_area/((float)(rect1.area()+rect2.area()-overlap_area));
// }

// float bboxOverlapratioLatter(const cv::Rect& rect1, const cv::Rect& rect2)
// {
//     int overlap_area = (rect1&rect2).area();
//     return (float)overlap_area/((float)(rect2.area()));
// }

// float bboxOverlapratioFormer(const cv::Rect& rect1, const cv::Rect& rect2)
// {
//     int overlap_area = (rect1&rect2).area();
//     return (float)overlap_area/((float)(rect1.area()));
// }

//cv::Mat toCvMat(const Eigen::Matrix3d &m)
//{
//    cv::Mat cvMat(3,3,CV_32F);
//    for(int i=0;i<3;i++)
//        for(int j=0; j<3; j++)
//            cvMat.at<float>(i,j)=m(i,j);
//
//    return cvMat.clone();
//}

// Eigen::Matrix4d cvMattoMatrix4d(const cv::Mat &cvMat4) {
//     Eigen::Matrix4f eigenMat4f;
//     Eigen::Matrix4d eigenMat4d;
//     //std::cout<<"converter debug: "<<cvMat4<<std::endl;
//     //M << cvMat4.at<float>(0, 0), cvMat4.at<float>(0, 1), cvMat4.at<float>(0, 2), cvMat4.at<float>(0, 3),
//     //     cvMat4.at<float>(1, 0), cvMat4.at<float>(1, 1), cvMat4.at<float>(1, 2), cvMat4.at<float>(1, 3),
//     //     cvMat4.at<float>(2, 0), cvMat4.at<float>(2, 1), cvMat4.at<float>(2, 2), cvMat4.at<float>(2, 3),
//     //     cvMat4.at<float>(3, 0), cvMat4.at<float>(3, 1), cvMat4.at<float>(3, 2), cvMat4.at<float>(3, 3);
//     // cv::cv2eigen(cvMat4, eigenMat4f);
//     for (int i = 0; i < 4; ++i) {
//         for (int j = 0; j < 4; ++j) {
//             eigenMat4f(i, j) = cvMat4.at<float>(i, j);
//         }
//     }
//     eigenMat4d = eigenMat4f.cast<double>();
//     return eigenMat4d;
// }


// Eigen::Matrix4d Quation2Eigen(const double qx, const double qy, const double qz, const double qw, const double tx,
//                                  const double ty, const double tz) {

//     Eigen::Quaterniond quaternion(Eigen::Vector4d(qx, qy, qz, qw));
//     Eigen::AngleAxisd rotation_vector(quaternion);
//     Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
//     T.rotate(rotation_vector);
//     T.pretranslate(Eigen::Vector3d(tx, ty, tz));
//     Eigen::Matrix4d Pose_eigen = T.matrix();
//     return Pose_eigen;
// }

// cv::Mat Quation2CvMat(const double qx, const double qy, const double qz, const double qw, const double tx, const double ty, const double tz  ) {
//     return toCvMat(
//             Quation2Eigen(qx, qy, qz, qw, tx, ty, tz )
//     );
// }

// Eigen::Isometry3d  Matrix4dtoIsometry3d(const Eigen::Matrix4d &matrix) {
//     Eigen::Isometry3d Iso=Eigen::Isometry3d::Identity();                // 虽然称为3d，实质上是4＊4的矩阵
//     //不能直接赋值
//     //    T1<< 1.000000e+00, 1.197624e-11, 1.704639e-10, 3.214096e-14,
//     //            1.197625e-11, 1.197625e-11, 3.562503e-10, -1.998401e-15,
//     //            1.704639e-10, 3.562503e-10, 1.000000e+00, -4.041212e-14,
//     //                       0,            0,            0,              1;

//     //----1.对各个元素赋值----
//     Iso(0, 0) = matrix(0, 0), Iso(0, 1) = matrix(0, 1), Iso(0, 2) = matrix(0, 2), Iso(0, 3) = matrix(0, 3);
//     Iso(1, 0) = matrix(1, 0), Iso(1, 1) = matrix(1, 1), Iso(1, 2) = matrix(1, 2), Iso(1, 3) = matrix(1, 3);
//     Iso(2, 0) = matrix(2, 0), Iso(2, 1) = matrix(2, 1), Iso(2, 2) = matrix(2, 2), Iso(2, 3) = matrix(2, 3);
//     Iso(3, 0) = matrix(3, 0), Iso(3, 1) = matrix(3, 1), Iso(3, 2) = matrix(3, 2), Iso(3, 3) = matrix(3, 3);

//     return Iso;
// }

// Eigen::Matrix4d Isometry3dtoMatrix4d(const Eigen::Isometry3d &Iso ){
//     return Iso.matrix();
// }

// Eigen::Isometry3d cvMattoIsometry3d(const cv::Mat &cvMat4){
//     return Matrix4dtoIsometry3d(
//             cvMattoMatrix4d( cvMat4 )
//             );
// }


// Eigen::Quaterniond ExtractQuaterniond(const Eigen::Isometry3d &Iso){
//     Eigen::Quaterniond q = Eigen::Quaterniond(Iso.rotation());
//     return q;
// }

// Eigen::Quaterniond ExtractQuaterniond(const Eigen::Matrix4d &matrix ){
//     return ExtractQuaterniond(
//             Matrix4dtoIsometry3d(matrix)
//     );
// }

// Eigen::Quaterniond ExtractQuaterniond(const cv::Mat &mat ){
//     return ExtractQuaterniond(
//             cvMattoIsometry3d(mat)
//     );
// }





#endif //VIEW_PLANNING_CONVERTERTOOLS_H