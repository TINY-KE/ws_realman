//
// Created by zhjd on 11/8/22.
//

#include "MapObject.h"
#include "Converter.h"

void cmpute_corner(MapObject* object) {

        float x_min_obj = (-0.5)*object->mCuboid3D.lenth;
        float x_max_obj = (0.5)*object->mCuboid3D.lenth;
        float y_min_obj = (-0.5)*object->mCuboid3D.width;
        float y_max_obj = (0.5)*object->mCuboid3D.width;
        float z_min_obj = (-0.5)*object->mCuboid3D.height;
        float z_max_obj = (0.5)*object->mCuboid3D.height;

        // g2o::SE3Quat pose =  Converter::toSE3Quat( object->mCuboid3D.pose_mat);
        auto pose = Converter::cvMattoIsometry3d( object->mCuboid3D.pose_mat);
        object->mCuboid3D.corner_1 = pose * Eigen::Vector3d(x_min_obj, y_min_obj, z_min_obj) ;
        object->mCuboid3D.corner_2 = pose * Eigen::Vector3d(x_max_obj, y_min_obj, z_min_obj) ;
        object->mCuboid3D.corner_3 = pose * Eigen::Vector3d(x_max_obj, y_max_obj, z_min_obj) ;
        object->mCuboid3D.corner_4 = pose * Eigen::Vector3d(x_min_obj, y_max_obj, z_min_obj) ;
        object->mCuboid3D.corner_5 = pose * Eigen::Vector3d(x_min_obj, y_min_obj, z_max_obj) ;
        object->mCuboid3D.corner_6 = pose * Eigen::Vector3d(x_max_obj, y_min_obj, z_max_obj) ;
        object->mCuboid3D.corner_7 = pose * Eigen::Vector3d(x_max_obj, y_max_obj, z_max_obj) ;
        object->mCuboid3D.corner_8 = pose * Eigen::Vector3d(x_min_obj, y_max_obj, z_max_obj) ;

        object->mCuboid3D.x_max = std::max( std::max(object->mCuboid3D.corner_1[0],  object->mCuboid3D.corner_2[0]),
                                            std::max(object->mCuboid3D.corner_3[0],  object->mCuboid3D.corner_4[0]));
        object->mCuboid3D.x_min = std::min( std::min(object->mCuboid3D.corner_1[0],  object->mCuboid3D.corner_2[0]),
                                            std::min(object->mCuboid3D.corner_3[0],  object->mCuboid3D.corner_4[0]));
        object->mCuboid3D.y_max = std::max( std::max(object->mCuboid3D.corner_1[1],  object->mCuboid3D.corner_2[1]),
                                            std::max(object->mCuboid3D.corner_3[1],  object->mCuboid3D.corner_4[1]));
        object->mCuboid3D.y_min = std::min( std::min(object->mCuboid3D.corner_1[1],  object->mCuboid3D.corner_2[1]),
                                            std::min(object->mCuboid3D.corner_3[1],  object->mCuboid3D.corner_4[1]));
        object->mCuboid3D.z_max = object->mCuboid3D.corner_5[2];
        object->mCuboid3D.z_min = object->mCuboid3D.corner_1[2];
}



MapObject::MapObject(){

}

void MapObject::Update_Twobj()      //更新物体在世界下的坐标
{
    // Rotation matrix.
    float cp = cos(mCuboid3D.rotP);
    float sp = sin(mCuboid3D.rotP);
    float sr = sin(mCuboid3D.rotR);
    float cr = cos(mCuboid3D.rotR);
    float sy = sin(mCuboid3D.rotY);
    float cy = cos(mCuboid3D.rotY);
    Eigen::Matrix<double, 3, 3> REigen;
    REigen << cp * cy, (sr * sp * cy) - (cr * sy), (cr * sp * cy) + (sr * sy),
        cp * sy, (sr * sp * sy) + (cr * cy), (cr * sp * sy) - (sr * cy),
        -sp, sr * cp, cr * cp;
    cv::Mat Ryaw = Converter::toCvMat(REigen);

    // Transformation matrix.
    cv::Mat Twobj = cv::Mat::eye(4, 4, CV_32F);
    const cv::Mat Rcw = Twobj.rowRange(0, 3).colRange(0, 3);
    const cv::Mat tcw = Twobj.rowRange(0, 3).col(3);

    cv::Mat R_result = Rcw * Ryaw;

    Twobj.at<float>(0, 0) = R_result.at<float>(0, 0);
    Twobj.at<float>(0, 1) = R_result.at<float>(0, 1);
    Twobj.at<float>(0, 2) = R_result.at<float>(0, 2);
    Twobj.at<float>(0, 3) = mCuboid3D.cuboidCenter[0];

    Twobj.at<float>(1, 0) = R_result.at<float>(1, 0);
    Twobj.at<float>(1, 1) = R_result.at<float>(1, 1);
    Twobj.at<float>(1, 2) = R_result.at<float>(1, 2);
    Twobj.at<float>(1, 3) = mCuboid3D.cuboidCenter[1];

    Twobj.at<float>(2, 0) = R_result.at<float>(2, 0);
    Twobj.at<float>(2, 1) = R_result.at<float>(2, 1);
    Twobj.at<float>(2, 2) = R_result.at<float>(2, 2);
    Twobj.at<float>(2, 3) = mCuboid3D.cuboidCenter[2];

    Twobj.at<float>(3, 0) = 0;
    Twobj.at<float>(3, 1) = 0;
    Twobj.at<float>(3, 2) = 0;
    Twobj.at<float>(3, 3) = 1;

    this->mCuboid3D.pose_mat = Twobj;

}


void MapObject::Update_Twobj(double x, double y, double z, double yaw)      //更新物体在世界下的坐标
{
    mCuboid3D.rotY = yaw;
    mCuboid3D.cuboidCenter[0] = x;
    mCuboid3D.cuboidCenter[1] = y;
    mCuboid3D.cuboidCenter[2] = z;

    // Rotation matrix.
    float cp = cos(mCuboid3D.rotP);
    float sp = sin(mCuboid3D.rotP);
    float sr = sin(mCuboid3D.rotR);
    float cr = cos(mCuboid3D.rotR);
    float sy = sin(mCuboid3D.rotY);
    float cy = cos(mCuboid3D.rotY);

    Eigen::Matrix<double, 3, 3> REigen;
    REigen << cp * cy, (sr * sp * cy) - (cr * sy), (cr * sp * cy) + (sr * sy),
        cp * sy, (sr * sp * sy) + (cr * cy), (cr * sp * sy) - (sr * cy),
        -sp, sr * cp, cr * cp;
    cv::Mat Ryaw = Converter::toCvMat(REigen);

    // Transformation matrix.
    cv::Mat Twobj = cv::Mat::eye(4, 4, CV_32F);
    const cv::Mat Rcw = Twobj.rowRange(0, 3).colRange(0, 3);
    const cv::Mat tcw = Twobj.rowRange(0, 3).col(3);

    cv::Mat R_result = Rcw * Ryaw;

    Twobj.at<float>(0, 0) = R_result.at<float>(0, 0);
    Twobj.at<float>(0, 1) = R_result.at<float>(0, 1);
    Twobj.at<float>(0, 2) = R_result.at<float>(0, 2);
    Twobj.at<float>(0, 3) = mCuboid3D.cuboidCenter[0];

    Twobj.at<float>(1, 0) = R_result.at<float>(1, 0);
    Twobj.at<float>(1, 1) = R_result.at<float>(1, 1);
    Twobj.at<float>(1, 2) = R_result.at<float>(1, 2);
    Twobj.at<float>(1, 3) = mCuboid3D.cuboidCenter[1];

    Twobj.at<float>(2, 0) = R_result.at<float>(2, 0);
    Twobj.at<float>(2, 1) = R_result.at<float>(2, 1);
    Twobj.at<float>(2, 2) = R_result.at<float>(2, 2);
    Twobj.at<float>(2, 3) = mCuboid3D.cuboidCenter[2];

    Twobj.at<float>(3, 0) = 0;
    Twobj.at<float>(3, 1) = 0;
    Twobj.at<float>(3, 2) = 0;
    Twobj.at<float>(3, 3) = 1;

    this->mCuboid3D.pose_mat = Twobj;

}

// void BackgroudObject::computePose( double yaw ) {
//     float cp = cos(0.0);
//     float sp = sin(0.0);
//     float sr = sin(0.0);
//     float cr = cos(0.0);
//     float sy = sin(yaw);
//     float cy = cos(yaw);
//     Eigen::Matrix<double, 3, 3> REigen;
//     REigen << cp * cy, (sr * sp * cy) - (cr * sy), (cr * sp * cy) + (sr * sy),
//         cp * sy, (sr * sp * sy) + (cr * cy), (cr * sp * sy) - (sr * cy),
//         -sp, sr * cp, cr * cp;
//     cv::Mat Ryaw = Converter::toCvMat(REigen);

//     // Transformation matrix.
//     cv::Mat Twobj = cv::Mat::eye(4, 4, CV_32F);
//     const cv::Mat Rcw = Twobj.rowRange(0, 3).colRange(0, 3);
//     const cv::Mat tcw = Twobj.rowRange(0, 3).col(3);

//     cv::Mat R_result = Rcw * Ryaw;

//     Twobj.at<float>(0, 0) = R_result.at<float>(0, 0);
//     Twobj.at<float>(0, 1) = R_result.at<float>(0, 1);
//     Twobj.at<float>(0, 2) = R_result.at<float>(0, 2);
//     Twobj.at<float>(0, 3) = mean_x;

//     Twobj.at<float>(1, 0) = R_result.at<float>(1, 0);
//     Twobj.at<float>(1, 1) = R_result.at<float>(1, 1);
//     Twobj.at<float>(1, 2) = R_result.at<float>(1, 2);
//     Twobj.at<float>(1, 3) = mean_y;

//     Twobj.at<float>(2, 0) = R_result.at<float>(2, 0);
//     Twobj.at<float>(2, 1) = R_result.at<float>(2, 1);
//     Twobj.at<float>(2, 2) = R_result.at<float>(2, 2);
//     Twobj.at<float>(2, 3) = mean_z;

//     Twobj.at<float>(3, 0) = 0;
//     Twobj.at<float>(3, 1) = 0;
//     Twobj.at<float>(3, 2) = 0;
//     Twobj.at<float>(3, 3) = 1;

//     pose_mat = Twobj;
// }


void MapObject::Update_object_size(double lenth, double width, double height)      //更新物体在世界下的坐标
{
    mCuboid3D.lenth = lenth;
    mCuboid3D.width = width;
    mCuboid3D.height = height;
}

void MapObject::Update_corner() {

        float x_min_obj = (-0.5)*mCuboid3D.lenth;
        float x_max_obj = (0.5)*mCuboid3D.lenth;
        float y_min_obj = (-0.5)*mCuboid3D.width;
        float y_max_obj = (0.5)*mCuboid3D.width;
        float z_min_obj = (-0.5)*mCuboid3D.height;
        float z_max_obj = (0.5)*mCuboid3D.height;

        // g2o::SE3Quat pose =  Converter::toSE3Quat( mCuboid3D.pose_mat);
        auto pose = Converter::cvMattoIsometry3d( mCuboid3D.pose_mat);
        mCuboid3D.corner_1 = pose * Eigen::Vector3d(x_min_obj, y_min_obj, z_min_obj) ;
        mCuboid3D.corner_2 = pose * Eigen::Vector3d(x_max_obj, y_min_obj, z_min_obj) ;
        mCuboid3D.corner_3 = pose * Eigen::Vector3d(x_max_obj, y_max_obj, z_min_obj) ;
        mCuboid3D.corner_4 = pose * Eigen::Vector3d(x_min_obj, y_max_obj, z_min_obj) ;
        mCuboid3D.corner_5 = pose * Eigen::Vector3d(x_min_obj, y_min_obj, z_max_obj) ;
        mCuboid3D.corner_6 = pose * Eigen::Vector3d(x_max_obj, y_min_obj, z_max_obj) ;
        mCuboid3D.corner_7 = pose * Eigen::Vector3d(x_max_obj, y_max_obj, z_max_obj) ;
        mCuboid3D.corner_8 = pose * Eigen::Vector3d(x_min_obj, y_max_obj, z_max_obj) ;

        mCuboid3D.x_max = std::max( std::max(mCuboid3D.corner_1[0],  mCuboid3D.corner_2[0]),
                                            std::max(mCuboid3D.corner_3[0],  mCuboid3D.corner_4[0]));
        mCuboid3D.x_min = std::min( std::min(mCuboid3D.corner_1[0],  mCuboid3D.corner_2[0]),
                                            std::min(mCuboid3D.corner_3[0],  mCuboid3D.corner_4[0]));
        mCuboid3D.y_max = std::max( std::max(mCuboid3D.corner_1[1],  mCuboid3D.corner_2[1]),
                                            std::max(mCuboid3D.corner_3[1],  mCuboid3D.corner_4[1]));
        mCuboid3D.y_min = std::min( std::min(mCuboid3D.corner_1[1],  mCuboid3D.corner_2[1]),
                                            std::min(mCuboid3D.corner_3[1],  mCuboid3D.corner_4[1]));
        mCuboid3D.z_max = mCuboid3D.corner_5[2];
        mCuboid3D.z_min = mCuboid3D.corner_1[2];
}