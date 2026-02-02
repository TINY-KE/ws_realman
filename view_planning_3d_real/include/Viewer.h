#ifndef VIEWER_H
#define VIEWER_H

#include <ros/ros.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/GetLinkState.h>
#include <gazebo_msgs/GetWorldProperties.h>
#include <gazebo_msgs/GetModelProperties.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "std_msgs/Float64MultiArray.h"
#include "MapObject.h"
#include "ConverterTools.h"

#include "Map.h"

class Visualize_Tools{
    public:
        Visualize_Tools(ros::NodeHandle& nh, ObjectMap *map_, std::string default_frame ):
                    mpMap(map_),
                    default_frame_(default_frame){
            candidate_quaterniond_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(
                "candidate_pose_quaterniond", 1);
            pub_field = nh.advertise<visualization_msgs::Marker>("field", 1);
            pub_sdf = nh.advertise<visualization_msgs::Marker>("sdf", 1);
            joint_values_pub = nh.advertise<std_msgs::Float64MultiArray>("joint_values_gpmp", 10);
            candidate_pub = nh.advertise<visualization_msgs::Marker>("/candidate", 10);
            publisher_object = nh.advertise<visualization_msgs::Marker>("object", 1000);
            publisher_ellipsoid = nh.advertise<visualization_msgs::Marker>("ellipsoid", 1000);
            point_pub = nh.advertise<visualization_msgs::Marker>("/nbv_point", 10);
            bbox_plane_pub = nh.advertise<visualization_msgs::Marker>("bbox_plane", 1);
            normal_plane_pub = nh.advertise<visualization_msgs::Marker>("normal_plane", 1);
        }

        void visualize_geometry_pose(geometry_msgs::Pose pose, std::string frame_id = "world",  double id_num = 1,  std::string name = "no-name", bool output = false);
        // void visualize_MapObject(SdfObject& ob, std::string frame_id);
        void visualize_ellipsoid(MapObject* ob, std::string frame_id, double id);
        void visualize_ellipsoid(double x, double y, double z, double a, double b, double c, double roll, double pitch, double yaw, std::string frame_id, double id);
        void visualize_point(Eigen::Vector3d& p , std::string frame_id, double id, double lifetime = 10);
        void visualize_plane_triangle_bypoint(std::vector<geometry_msgs::Point>& points, int id, std::string frame_id = "world");
        void visualize_plane_rectangle(Eigen::Vector4d plane_param, int id, std::string frame_id = "world");
        void visualize_gbv_lbv(std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& arrows_starts, std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& arrows_ends, std::string frame_id);
        void clean_visualize_gbv_lbv(std::string frame_id);
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> arrows_starts, arrows_ends;
        void add_arrows_starts_arrows_ends(std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& arrows_starts, std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& arrows_ends);
        // void clean_arrows_starts_arrows_ends(std::string frame_id);


        void Run();
        
    private:
        ros::Publisher candidate_quaterniond_pub;
        ros::Publisher pub_field;
        ros::Publisher pub_sdf;
        ros::Publisher joint_values_pub;
        ros::Publisher candidate_pub;
        ros::Publisher publisher_object;
        ros::Publisher publisher_ellipsoid;
        ros::Publisher point_pub;
        ros::Publisher bbox_plane_pub;
        ros::Publisher normal_plane_pub;
        // Visualize_Arm_Tools arm_pub;

        std::string default_frame_ = "wam/base_link";

        ObjectMap *mpMap;

    public:
        //std::vector<std::vector<geometry_msgs::Point>> BboxPlanesTrianglePointsInWorld;



};




#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "std_msgs/Float64MultiArray.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

 // gpmp2 机械臂可视化
 #include <gtsam/base/Matrix.h>
 #include <gtsam/base/Vector.h>
 #include <gtsam/geometry/Point3.h>
 #include <gtsam/nonlinear/NonlinearFactor.h>
 #include <gtsam/geometry/Pose3.h>
 #include <gpmp2/kinematics/ArmModel.h>
 #include <gpmp2/kinematics/Arm.h>

 using namespace gpmp2;
 using namespace gtsam;



 typedef gpmp2::ArmModel Robot;
 class Visualize_Arm_Tools {
 public:
     Visualize_Arm_Tools(ros::NodeHandle& nh, const Robot& robot, moveit::planning_interface::MoveGroupInterface& move_group, int width, int height, Eigen::Matrix3d calib, std::string default_frame );

 private:
     ros::Publisher collision_spheres_pub;
     ros::Publisher arm_link_spheres_pub;
     ros::Publisher bbox_plane_pub;

     // arm: planar one, all alpha = 0
     const Robot& robot_;
     std::string default_frame_;
     moveit::planning_interface::MoveGroupInterface& move_group_;
     // 图像宽度和高度
     int miImageCols; // = Config::Get<int>("Camera.width");
     int miImageRows; // = Config::Get<int>("Camera.height");
     Eigen::Matrix3d mCalib;
     double mDepth = 6.0;   //预期的相机视场长度  1.0用于截图
     double mFOV_decrease = 145;
     Eigen::Matrix4f mRobotPose;


public:
     void setRobotPose(Eigen::Matrix4f& robot_pose) {
         mRobotPose = robot_pose;
     }
     void setFOVDepth(double mDepth_) {
         mDepth = mDepth_;
     }
     void setFOVDecrease(double mFOV_decrease_) {
         mFOV_decrease = mFOV_decrease_;
     }

 public:
     void Run();

 //发布机械臂的球体
 private:
     void publish_collision_spheres(const Eigen::Matrix<double, 7, 1>& conf) ;


     void publish_arm_link_spheres(const Eigen::Matrix<double, 7, 1>& conf);


//发布相机视场
private:
        geometry_msgs::Point pixelToCamera(const Eigen::Vector2d& pixel) ;

        geometry_msgs::Point transformPointToWorld(const Eigen::Matrix4f& T_world_camera, const geometry_msgs::Point& point_camera) ;

        std::vector<std::vector<geometry_msgs::Point>> GenerateBboxPlanesTrianglePoints(const typename Robot::Pose& conf) ;


     // 以三角的形式显示平面
     void visualize_plane_triangle_bypoint(const typename Robot::Pose& conf);


 };




#endif //VIEWER_H