//
// Created by robotlab on 24-11-4.
//

#ifndef VIEWPLANNING_H
#define VIEWPLANNING_H

#include <iostream>
#include <cmath>
#include <algorithm>
#include <CppUnitLite/TestHarness.h>

// gtsam插件
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gpmp2/planner/TrajUtils.h>


// 优化因子
#include "gtsam/BboxEllipsoidFactor.h"
#include "gtsam/BboxPlaneArmLinkFactor.h"
// #include "gtsam_quadrics/geometry/BboxCameraFactor.h"
// #include "gtsam_quadrics/geometry/InnerBboxCameraFactor.h"
// #include "gtsam_quadrics/geometry/HeightCameraFactor.h"
// #include "gtsam_quadrics/geometry/BoundingBoxFactor.h"
#include "gtsam/CameraXaxisHorizontal.h"
// #include "gtsam/CentorEllipsoidFactor.h"
#include <gpmp2/gp/GaussianProcessInterpolatorLinear.h>
// #include <gpmp2/kinematics/GaussianPriorWorkspaceOrientationArm.h>
// #include <gpmp2/kinematics/GaussianPriorWorkspacePoseArm.h>
#include <gpmp2/gp/GaussianProcessPriorLinear.h>   // GP prior


// gpmp的封装优化器
#include <gpmp2/planner/TrajOptimizerSetting.h>
#include <gpmp2/planner/BatchTrajOptimizer.h>

#include "Candidate.h"
#include "MapObject.h"

#include <nav_msgs/OccupancyGrid.h>

class ViewPlanningTool {

    enum opt_type {
        LM = 1, //LEVENBERG_MARQUARDT
        GN = 2, //GaussNewton
        DOGLEG = 3
    };

private:
    // 差值的数量
    // double mRotateDivides = 90;
    double mCircleDivides = 240;

    // 相机位置的参数
    double mCameraPoseHeight = 1.0;

    // 像素平面
    double mFovDecrease = 120;
    double mCameraWidth = 640, mCameraHeight = 480;
    Eigen::Matrix3d mCalib;

    ArmModel *mpArmModel;
    Visualize_Tools* vis_tools;

    std::vector<BboxEllipsoidFactor<ArmModel> > bbox_factors;
    std::vector<BboxPlaneArmLinkFactor<ArmModel> > lowplane_factors;


public:
    ViewPlanningTool(ArmModel *arm_model_, int circle_divides_, int CameraWidth_, int CameraHeight_,
                 double FovDecrease_, Eigen::Matrix3d Calib_, Visualize_Tools* vis_tools_): mpArmModel(arm_model_),
                                                               // mRotateDivides(rotate_divides_),
                                                               mCircleDivides(circle_divides_),
                                                               mCameraWidth(CameraWidth_),
                                                               mCameraHeight(CameraHeight_),
                                                               mFovDecrease(FovDecrease_),
                                                               mCalib(Calib_),
                                                               vis_tools(vis_tools_) {
    }

    void visualize(int i, Vector &joint_group) {
        bbox_factors[i].visulize(joint_group, "adjust bbox");
    }

    void planning(MapObject *mp_target_obj, double robot_pose_x, double robot_pose_y, Values &return_arm_results, std::vector<geometry_msgs::Pose> & return_FootPrints) {
        // 计算视场角
        float fx = mCalib(0, 0);
        float fy = mCalib(1, 1);
        float cx = mCalib(0, 2);
        float cy = mCalib(1, 2);
        double theta_x_rad = 2 * std::atan((mCameraWidth - mFovDecrease * 2) / (2 * fx));
        double theta_y_rad = 2 * std::atan((mCameraHeight - 2 * mFovDecrease * mCameraHeight / mCameraWidth) / (2 * fy));
        //  只用横向视场角度计算
        double FOV_radius = (mp_target_obj->mCuboid3D.width + mp_target_obj->mCuboid3D.lenth) / 4.0
                            / sin(theta_x_rad / 2.0);

        std::vector<geometry_msgs::Pose> FootPrints; //FootPrints候选位姿
        bool Clockwise = false;  //绕着物体顺时针旋转，还是逆时针旋转

        // 相比于planning_for_direct，旋转半径要更小
        double scale = 0.85;
        std::vector<geometry_msgs::Pose> Candidates =
                // GenerateCandidates_ellipse_by_circle(*ob, FootPrints, radius, camera_height, false, 300);
                // GenerateCandidates_ellipse(*ob, FootPrints, radius, camera_height, false, 300);
                GenerateCandidates_circle(*mp_target_obj, FootPrints, FOV_radius*scale, mCameraPoseHeight, robot_pose_x,
                                          robot_pose_y, false, mCircleDivides, Clockwise);
        // GenerateCandidates_circle_linear(*ob, FootPrints, linear_interpolation_nums, radius, camera_height, 0, 0, false, rotate_divides, circle_divides);
        return_FootPrints = FootPrints;


        // 六. 构建图
        // % algo settings
        bbox_factors.clear();
        lowplane_factors.clear();


        // 七、优化
        int type_start = 0;
        gtsam::Vector start_conf, end_conf;

        // start_conf = (Vector(7) << 2.43078, -0.627707, 0.13426, 1.70191, -0.10452, -1.27042, 2.40398).finished();
        // end_conf = (Vector(7) << 2.43078, -0.627707, 0.13426, 1.70191, -0.10452, -1.27042, 2.40398).finished();

        // 潜在可行
        // start_conf = (Vector(7) << -1.838, -0.526, 0.010, 1.421, -0.016, -0.743, 0.230).finished();
        // end_conf = (Vector(7) << -1.838, -0.526, 0.010, 1.421, -0.016, -0.743, 0.230).finished();

        // 最可行，都在中间
        // start_conf = (Vector(7) << -1.1334936224161147, -0.2191269354635086, 0.11841710938086258,  0.5149487492216238,   -1.1956554103433579,  0.30335829217330623, 0.6029216146087082).finished();
        // end_conf = (Vector(7) << -1.1334936224161147, -0.2191269354635086, 0.11841710938086258,  0.5149487492216238,   -1.1956554103433579,  0.30335829217330623, 0.6029216146087082).finished();

        // start_conf = (Vector(7) << 0,0,0,0,0,0,0).finished();
        // end_conf = (Vector(7) << 0,0,0,0,0,0,0).finished();

        // 用零优化
        if(Clockwise){
            start_conf = (Vector(7) << -1.59, 0, 0, 0.3, 0, 0, 0).finished();
            end_conf = (Vector(7) << -1.59, 0, 0, 0.3, 0, 0, 0).finished();
        }
        else{
            start_conf = (Vector(7) << 1.59, 0, 0, 0.3, 0, 0, 0).finished();
            end_conf = (Vector(7) << 1.59, 0, 0, 0.3, 0, 0, 0).finished();  //0.17
        }
        


        double total_time_step = FootPrints.size() - 1;
        double total_time_sec = 1 * (FootPrints.size() - 1);
        double check_inter = 5;
        // 使用直线插值
        gtsam::Values init_values = initArmTrajStraightLine(start_conf, end_conf, total_time_step);

        // 八、优化的参数
        // 视场和机械臂本体
        double obs_sigma = 0.05; // 障碍物因子的权重
        double epsilon_dist = 0.05;
        //初始位姿
        double pose_sigma = 1e-4; //固定的位姿，包括初始的位姿
        double orien_sigma = 1e-2; //过程中的方向。
        // 机械臂之间的关节尽可能的小
        double delta_t = total_time_sec / total_time_step;
        Eigen::MatrixXd Qc = 1 * Eigen::MatrixXd::Identity(mpArmModel->dof(), mpArmModel->dof());
        noiseModel::Gaussian::shared_ptr Qc_model = noiseModel::Gaussian::Covariance(Qc);
        // bbox和二次曲线
        double s = mFovDecrease;
        gtsam_quadrics::AlignedBox2 gtsam_bbox(0 + s, 0 + s * mCameraHeight / mCameraWidth, mCameraWidth - s,
                                               mCameraHeight - s * mCameraHeight / mCameraWidth); //预期的物体检测框
        double bbox_sigma = 0.1; // bbox的权重
        // x轴水平
        double xaxis_sigma = 0.0001;

        NonlinearFactorGraph graph;
        for (int i = 0; i < FootPrints.size(); i++) {
            // for (int i = 0; i < linear_interpolation_nums; i++) {
            Key key_pos = symbol('x', i);
            Key key_vel = symbol('v', i);



            Eigen::Matrix4f RobotPose = Eigen::Matrix4f::Identity();
            RobotPose = Converter::geometryPosetoMatrix4d(FootPrints[i]).cast<float>();
            BboxEllipsoidFactor<ArmModel> factor_ellipsoid_factor(key_pos, *mpArmModel,
                                                                  bbox_sigma,
                                                                  gtsam_bbox,
                                                                  mp_target_obj,
                                                                  RobotPose,
                                                                  mCameraWidth, mCameraHeight,
                                                                  mCalib);
            graph.add(factor_ellipsoid_factor);
            bbox_factors.push_back(factor_ellipsoid_factor);



            BboxPlaneArmLinkFactor<ArmModel> factor_planearm(
                key_pos,
                *mpArmModel,
                obs_sigma,
                epsilon_dist,
                mCameraWidth, mCameraHeight,
                mCalib
            );
            graph.add(factor_planearm);


            CameraXaxisHorizontal<ArmModel> factor_xaxis(
                key_pos,
                *mpArmModel,
                xaxis_sigma
            );
            graph.add(factor_xaxis);


            if (i > 0) {
                // 初始化变量
                Key key_pos1 = symbol('x', i - 1);
                Key key_pos2 = symbol('x', i);
                Key key_vel1 = symbol('v', i - 1);
                Key key_vel2 = symbol('v', i);

                graph.add(GaussianProcessPriorLinear(key_pos1, key_vel1, key_pos2, key_vel2, delta_t, Qc_model));
            }
        }


        int opt_type = LM;
        Values results;
        if (opt_type == LM) {
            LevenbergMarquardtParams parameters;
            parameters.setVerbosity("ERROR"); // SILENT, TERMINATION, ERROR, VALUES, DELTA, LINEAR
            parameters.setAbsoluteErrorTol(1e-4);
            parameters.setlambdaInitial(1000.0);
            LevenbergMarquardtOptimizer optimizer(graph, init_values, parameters);
            // LevenbergMarquardtOptimizer类的定义
            // #include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
            // virtual class LevenbergMarquardtOptimizer : gtsam::NonlinearOptimizer {
            //     LevenbergMarquardtOptimizer(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& initialValues);
            //     LevenbergMarquardtOptimizer(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& initialValues, const gtsam::LevenbergMarquardtParams& params);
            //     double lambda() const;
            //     void print(string str) const;
            results = optimizer.optimize();
        } else if (opt_type == GN) {
            GaussNewtonParams parameters;
            parameters.setVerbosity("ERROR");
            //setVerbosity("TERMINATION"); //.setVerbosity("ERROR"); SILENT, TERMINATION, ERROR, VALUES, DELTA, LINEAR
            GaussNewtonOptimizer optimizer(graph, init_values, parameters);
            results = optimizer.optimize();
        } else if (opt_type == DOGLEG) {
            DoglegParams parameters;
            parameters.setVerbosity("ERROR"); //setVerbosity("TERMINATION"); //.setVerbosity("ERROR");
            DoglegOptimizer optimizer(graph, init_values, parameters);
            results = optimizer.optimize();
            // cout_results = optimizer.values();
        }

        return_arm_results = results;
        Eigen::Vector3d nbv(FootPrints[0].position.x,FootPrints[0].position.y,FootPrints[0].position.z);
        vis_tools->visualize_point(nbv, "world", 0, 100);
    }



    void planning_for_moveit(MapObject *mp_target_obj, double robot_pose_x, double robot_pose_y, Values &return_arm_results, std::vector<geometry_msgs::Pose> & return_FootPrints, std::vector<geometry_msgs::Pose> & return_CameraLink_Candidates) {
        // 计算视场角
        float fx = mCalib(0, 0);
        float fy = mCalib(1, 1);
        float cx = mCalib(0, 2);
        float cy = mCalib(1, 2);
        double theta_x_rad = 2 * std::atan((mCameraWidth - mFovDecrease * 2) / (2 * fx));
        double theta_y_rad = 2 * std::atan((mCameraHeight - 2 * mFovDecrease * mCameraHeight / mCameraWidth) / (2 * fy));
        //  只用横向视场角度计算
        double FOV_radius = (mp_target_obj->mCuboid3D.width + mp_target_obj->mCuboid3D.lenth) / 4.0
                            / sin(theta_x_rad / 2.0);

        std::vector<geometry_msgs::Pose> FootPrints; //FootPrints候选位姿
        bool Clockwise = false;  //绕着物体顺时针旋转，还是逆时针旋转
        std::vector<geometry_msgs::Pose> CameraLink_Candidates =
                // GenerateCandidates_ellipse_by_circle(*ob, FootPrints, radius, camera_height, false, 300);
                // GenerateCandidates_ellipse(*ob, FootPrints, radius, camera_height, false, 300);
                GenerateCandidates_circle(*mp_target_obj, FootPrints, FOV_radius, mCameraPoseHeight, robot_pose_x,
                                          robot_pose_y, false, mCircleDivides, Clockwise);
        // GenerateCandidates_circle_linear(*ob, FootPrints, linear_interpolation_nums, radius, camera_height, 0, 0, false, rotate_divides, circle_divides);
        return_FootPrints = FootPrints;
        return_CameraLink_Candidates = CameraLink_Candidates;
        Eigen::Vector3d nbv(FootPrints[0].position.x,FootPrints[0].position.y,FootPrints[0].position.z);
        vis_tools->visualize_point(nbv, "world", 0, 100);
    }


    void planning_for_direct(MapObject *mp_target_obj, double robot_pose_x, double robot_pose_y, Values &return_arm_results, std::vector<geometry_msgs::Pose> & return_FootPrints, std::vector<double> & return_direct_yaws) {
        // 计算视场角
        float fx = mCalib(0, 0);
        float fy = mCalib(1, 1);
        float cx = mCalib(0, 2);
        float cy = mCalib(1, 2);
        double theta_x_rad = 2 * std::atan((mCameraWidth - mFovDecrease * 2) / (2 * fx));
        double theta_y_rad = 2 * std::atan((mCameraHeight - 2 * mFovDecrease * mCameraHeight / mCameraWidth) / (2 * fy));
        //  只用横向视场角度计算
        double FOV_radius = (mp_target_obj->mCuboid3D.width + mp_target_obj->mCuboid3D.lenth) / 4.0
                            / sin(theta_x_rad / 2.0);

        double scale = 0.95;
        std::vector<geometry_msgs::Pose> FootPrints; //FootPrints候选位姿
        bool Clockwise = false;  //绕着物体顺时针旋转，还是逆时针旋转
        std::vector<double> direct_yaws =
                GenerateCandidates_circle_onlyfordirect(*mp_target_obj, FootPrints, FOV_radius*scale, mCameraPoseHeight, robot_pose_x,
                                          robot_pose_y, false, mCircleDivides, Clockwise);
        return_FootPrints = FootPrints;
        return_direct_yaws = direct_yaws;
        Eigen::Vector3d nbv(FootPrints[0].position.x,FootPrints[0].position.y,FootPrints[0].position.z);
        vis_tools->visualize_point(nbv, "world", 0, 100);
    }


    Eigen::Vector3d calculateEndPoint(
    const Eigen::Vector3d& start, 
    const Eigen::Vector3d& goal, 
    double length = 0.3) 
    {
        // 计算方向向量并归一化
        Eigen::Vector3d direction = (goal - start).normalized();
        
        // 计算终点
        Eigen::Vector3d end = start + length * direction;
        
        return end;
    }
    
    // 世界坐标转栅格坐标
    void worldToGrid(double world_x, double world_y, const nav_msgs::OccupancyGrid& map_data_, int& grid_x, int& grid_y) {
        const auto& info = map_data_.info;
        
        // 计算相对于地图原点的偏移量
        double dx = world_x - info.origin.position.x;
        double dy = world_y - info.origin.position.y;
        
        // 转换为栅格坐标（注意Y轴方向）
        grid_x = static_cast<int>(dx / info.resolution);
        grid_y = static_cast<int>(dy / info.resolution);
    }
    
    // 判断点是否在空闲区域
    bool isPointInFreeSpace(double world_x, double world_y, const nav_msgs::OccupancyGrid& map_data_) {
        // if (!map_received_) {
        //     ROS_WARN("Map data not available!");
        //     return false;
        // }

        int grid_x = 0, grid_y = 0;  // 必须先定义可修改的变量
        // 1. 世界坐标转栅格坐标
        worldToGrid(world_x, world_y, map_data_, grid_x, grid_y);
        
        // 2. 检查坐标是否在有效范围内  
        if (grid_x < 0 || grid_x >= map_data_.info.width || 
            grid_y < 0 || grid_y >= map_data_.info.height) {
            ROS_DEBUG("Point outside map boundaries");
            return false;
        }
        // if (grid_x < 0/map_data_.info.resolution || grid_x >=  7/map_data_.info.resolution || 
        //     grid_y < 0/map_data_.info.resolution || grid_y >= 2/map_data_.info.resolution) {
        //         std::cout<<"grid_x:"<<grid_x<<" grid_y:"<<grid_y<<std::endl;
        //         std::cout<<"范围:"<<-7/map_data_.info.resolution<<" "<<7/map_data_.info.resolution<<" "<<-5/map_data_.info.resolution<<" "<<2/map_data_.info.resolution<<std::endl;
        //     ROS_DEBUG("Point outside map boundaries");
        //     return false;
        // }


        // 3. 获取该栅格的值（注意行优先存储）
        int index = grid_y * map_data_.info.width + grid_x;
        int cell_value = map_data_.data[index];
        
        // 4. 判断状态（0=空闲，-1=未知，其他=障碍）
        return (cell_value == 0);
    }

    bool isAreaFree(double world_x, double world_y,
               const nav_msgs::OccupancyGrid& map_data_,
               double radius = 0.25)
    {
        // 参数有效性检查
        if(radius <= 0 || map_data_.data.empty()) {
            ROS_WARN("Invalid parameters");
            return false;
        }

        // 获取中心栅格坐标
        int center_x, center_y;
        worldToGrid(world_x, world_y, map_data_, center_x, center_y);

        if (center_x < 0 || center_x >= map_data_.info.width || 
            center_y < 0 || center_y >= map_data_.info.height) {
            ROS_DEBUG("Point outside map boundaries");
            return false;
        }

        // 计算检测半径（栅格单位）
        const int radius_cells = static_cast<int>(
            std::ceil(radius / map_data_.info.resolution));
        
        // 遍历检测区域[7](@ref)
        for(int dx = -radius_cells; dx <= radius_cells; ++dx) {
            for(int dy = -radius_cells; dy <= radius_cells; ++dy) {
                // 当前检测位置
                const int x = center_x + dx;
                const int y = center_y + dy;
                
                // 边界检查
                if(x < 0 || x >= map_data_.info.width ||
                y < 0 || y >= map_data_.info.height) 
                {
                    continue;
                }

                // 精确距离过滤（圆形检测区域）
                const double actual_dist = map_data_.info.resolution * 
                                        std::hypot(dx, dy);
                if(actual_dist > radius) continue;

                // 获取栅格值
                const int index = y * map_data_.info.width + x;
                const int value = map_data_.data[index];

                // 障碍物判断标准[2](@ref)
                if(value != 0) { // 0:空闲, -1:未知, >0:障碍
                    return false;
                }
            }
        }
        return true;
    }

    void planning_for_real(MapObject *mp_target_obj, nav_msgs::OccupancyGrid map_data, double robot_pose_x, double robot_pose_y, Values &return_arm_results, std::vector<geometry_msgs::Pose> & return_FootPrints, std::vector<geometry_msgs::Pose> & return_CameraLink_Candidates) {
        // 计算视场角
        float fx = mCalib(0, 0);
        float fy = mCalib(1, 1);
        float cx = mCalib(0, 2);
        float cy = mCalib(1, 2);
        double theta_x_rad = 2 * std::atan((mCameraWidth - mFovDecrease * 2) / (2 * fx));
        double theta_y_rad = 2 * std::atan((mCameraHeight - 2 * mFovDecrease * mCameraHeight / mCameraWidth) / (2 * fy));
        //  只用横向视场角度计算
        double FOV_radius = (mp_target_obj->mCuboid3D.width + mp_target_obj->mCuboid3D.lenth) / 4.0
                            / sin(theta_x_rad / 2.0);

        double scale = 0.95;
        std::vector<geometry_msgs::Pose> FootPrints; //FootPrints候选位姿
        bool Clockwise = false;  //绕着物体顺时针旋转，还是逆时针旋转
        // std::vector<double> direct_yaws =
        //         GenerateCandidates_circle_onlyfordirect(*mp_target_obj, FootPrints, FOV_radius*scale, mCameraPoseHeight, robot_pose_x,
        //                                   robot_pose_y, false, mCircleDivides, Clockwise);
        std::vector<geometry_msgs::Pose> CameraLink_Candidates =
                // GenerateCandidates_ellipse_by_circle(*ob, FootPrints, radius, camera_height, false, 300);
                // GenerateCandidates_ellipse(*ob, FootPrints, radius, camera_height, false, 300);
                GenerateCandidates_real(*mp_target_obj, FootPrints, FOV_radius*scale, 1.2, robot_pose_x,
                                          robot_pose_y, false, mCircleDivides, Clockwise);
        // return_FootPrints = FootPrints;
        // return_CameraLink_Candidates = CameraLink_Candidates;

        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> arrows_starts, arrows_ends;
        for(int i=-0; i<FootPrints.size(); i++) {
            std::cout<<"FootPrints["<<i<<"]:"<<FootPrints[i].position.x<<","<<FootPrints[i].position.y<<std::endl;
            auto p = FootPrints[i];
            // if(isPointInFreeSpace(p.position.x, p.position.y, map_data)) {
            if(isAreaFree(p.position.x, p.position.y, map_data)) {
                std::cout<<"FootPrints in free space!"<<std::endl;
                Eigen::Vector3d start(p.position.x, p.position.y, 0.7);
                Eigen::Vector3d goal(mp_target_obj->mCuboid3D.cuboidCenter[0], mp_target_obj->mCuboid3D.cuboidCenter[1], mp_target_obj->mCuboid3D.cuboidCenter[2]);
                Eigen::Vector3d end = calculateEndPoint(start, goal, 0.3);
                arrows_starts.push_back(start);
                arrows_ends.push_back(end);
                return_FootPrints.push_back(FootPrints[i]); 
                return_CameraLink_Candidates.push_back(CameraLink_Candidates[i]);
            }
            else
            {
                std::cout<<"FootPrints in occupied space!"<<std::endl;
            }
            
        }
        // vis_tools->clean_arrows_starts_arrows_ends();
        vis_tools->add_arrows_starts_arrows_ends(arrows_starts, arrows_ends);
    }

};



#endif //VIEWPLANNING_H
