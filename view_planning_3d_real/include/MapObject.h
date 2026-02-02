/**
* This file is part of https://github.com/JingwenWang95/DSP-SLAM
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

#ifndef MAPOBJECT_H
#define MAPOBJECT_H

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>


class KeyFrame;
class Frame;


struct Cuboid3D{
    //     7------6
    //    /|     /|
    //   / |    / |
    //  4------5  |
    //  |  3---|--2
    //  | /    | /
    //  0------1
    // lenth ：corner_2[0] - corner_1[0]
    // width ：corner_2[1] - corner_3[1]
    // height：corner_2[2] - corner_6[2]

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
    // 8 vertices.
    Eigen::Vector3d corner_1;
    Eigen::Vector3d corner_2;
    Eigen::Vector3d corner_3;
    Eigen::Vector3d corner_4;
    Eigen::Vector3d corner_5;
    Eigen::Vector3d corner_6;
    Eigen::Vector3d corner_7;
    Eigen::Vector3d corner_8;

    Eigen::Vector3d cuboidCenter;       // the center of the Cube, not the center of mass of the object
    float x_min, x_max, y_min, y_max, z_min, z_max;     // the boundary in XYZ direction.

    float lenth;
    float width;
    float height;
    float mfRMax;      // 中心点与角点的最大半径

    //g2o::SE3Quat pose ;                               // 6 dof pose.
    cv::Mat pose_mat = cv::Mat::eye(4, 4, CV_32F);      //cv::mat形式的 物体在世界坐标系下的位姿
    //g2o::SE3Quat pose_without_yaw;                    // 6 dof pose without rotation.
    cv::Mat pose_noyaw_mat = cv::Mat::eye(4, 4, CV_32F);
    // angle.
    float rotY = 0.0;
    float rotP = 0.0;
    float rotR = 0.0;

    // line.
    float mfErrorParallel;
    float mfErroeYaw;
};



class MapObject {
public:
    MapObject();

    // cuboid
    float w;
    float h;
    float l;

    Cuboid3D mCuboid3D;                 // cuboid.

    void Update_Twobj();                // 原UpdateObjPose  更新物体在世界下的坐标
    void Update_Twobj(double x, double y, double z, double yaw);                // 原UpdateObjPose  更新物体在世界下的坐标
    void Update_object_size(double lenth, double width, double height)   ;   //更新物体的尺寸
    void Update_corner();

    bool explored = false;

};


#endif //MAPOBJECT_H
