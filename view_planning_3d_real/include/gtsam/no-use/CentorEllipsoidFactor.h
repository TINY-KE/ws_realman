/**
 *  @file   CentorEllipsoidFactor.h
 *  @brief  Gaussian prior defined on the workspace pose of any joint of a robot
 *          given its state in configuration space
 *  @author zhang jiadong
 *  @date   Jan 8, 2018
 **/
#ifndef CentorEllipsoidFactor_H
#define CentorEllipsoidFactor_H

#pragma once

#include <gpmp2/kinematics/RobotModel.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/PinholeCamera.h>

#include "MapObject.h"
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam_quadrics/geometry/AlignedBox2.h>
#include <gtsam_quadrics/geometry/ConstrainedDualQuadric.h>
#include <gtsam_quadrics/geometry/BoundingBoxFactor.h>
#include <gtsam_quadrics/geometry/QuadricCamera.h>
#include <gtsam_quadrics/base/QuadricProjectionException.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/bind/bind.hpp>

#include "Converter.h"

#include <opencv2/opencv.hpp>
        

typedef Eigen::Matrix<double, 3, 1> Vector3d;
typedef Eigen::Matrix<double, 9, 1> Vector9d;
typedef Eigen::Matrix<double, 9, 9> Matrix9d;
typedef Eigen::Matrix<double, 5, 5> Matrix5d;
typedef Eigen::Matrix<double, 3, 8> Matrix38d;
typedef Eigen::Matrix<double, 10, 1> Vector10d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 5, 1> Vector5d;
typedef Eigen::Matrix<double, 2, 1> Vector2d;
typedef Eigen::Matrix<double, 4, 1> Vector4d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 3, 3> Matrix3d;

// #define NUMERICAL_DERIVATIVE false

using namespace gtsam_quadrics;

namespace gpmp2 {
    /**
     * Gaussian prior defined on the workspace pose
     */
    template<class ROBOT>
    class CentorEllipsoidFactor
            : public gtsam::NoiseModelFactorN<typename ROBOT::Pose> {
    public:
        enum MeasurementModel {
            STANDARD,
            TRUNCATED
        };

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef ROBOT Robot;
        typedef typename Robot::Pose Pose;

    private:
        typedef CentorEllipsoidFactor This;
        typedef gtsam::NoiseModelFactorN<Pose> Base;

        const Robot &robot_; // Robot

        int miImageCols, miImageRows;


        // 目标物体。物体的位姿实在world坐标系中
        ConstrainedDualQuadric Quadric_;

        //用于 bbox与Pose3的误差
        AlignedBox2 measured_; ///< measured bounding box
        boost::shared_ptr<gtsam::Cal3_S2> gtsam_calibration_;
        // = boost::make_shared<gtsam::Cal3_S2>(fx, fy, 0.0, cx, cy);
        MeasurementModel measurementModel_;

    public:
        /* Default constructor */
        CentorEllipsoidFactor() {
        }

        /// Constructor
        CentorEllipsoidFactor(gtsam::Key poseKey, const Robot &robot,
                            double cost_sigma,
                            const AlignedBox2& measured,
                            MapObject *ob_,
                            Eigen::Matrix4f mRobotPose_,
                            int width, int height, Matrix3d calib)
            : Base(gtsam::noiseModel::Isotropic::Sigma(1,cost_sigma),
                   poseKey),
              robot_(robot),
              measured_(measured),
              miImageCols(width),
              miImageRows(height) {
            // 生成世界坐标系下的目标物体
            GenerateConstrainedDualQuadric(ob_,mRobotPose_);
            // 生成 内参
            GenerateCalibration(calib);
            // 生成 目标bbox
            // GenerateMeasureBbox(measured);
            //
            measurementModel_ = STANDARD;

        }

        virtual ~CentorEllipsoidFactor() {
        }


        /// factor error function
        /// 输入参数： pose —— robot pose in config space
        /// numerical jacobians / analytic jacobians from cost function
        // zhjd: 此处的Robot::Pose对应的是gtsam::Vector。   根据 ForwardKinematics<gtsam::Vector, gtsam::Vector>。 因此这里的pose就是关节角度。
        gtsam::Vector evaluateError(
            const typename Robot::Pose &conf,
            gtsam::OptionalMatrixType H1 = nullptr
            ) const override;

        gtsam::Vector evaluateError_fornumericalDerivative(
            const Vector7d & conf, gtsam::OptionalMatrixType H1) const;


        void visulize(
            const typename Robot::Pose &conf,
            const string name = "Bounding Boxes"
        ) const;

        

        /// @return a deep copy of this factor
        virtual gtsam::NonlinearFactor::shared_ptr clone() const {
            return std::static_pointer_cast<gtsam::NonlinearFactor>(
                gtsam::NonlinearFactor::shared_ptr(new This(*this)));
        }

        /** print contents */
        void print(const std::string &s = "",
                   const gtsam::KeyFormatter &keyFormatter =
                           gtsam::DefaultKeyFormatter) const {
            std::cout << s << "CentorEllipsoidFactor :" << std::endl;
            Base::print("", keyFormatter);
        }

    private:
#ifdef GPMP2_ENABLE_BOOST_SERIALIZATION
        /** Serialization function */
        friend class boost::serialization::access;
        template <class ARCHIVE>
        void serialize(ARCHIVE& ar, const unsigned int version) {
            ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
        }
#endif

    public:
        void GenerateConstrainedDualQuadric(MapObject *object, Eigen::Matrix4f& T_world_baselink) {
            // 一、将物体的旋转矩阵，转为四元数
            auto matrix = object->mCuboid3D.pose_mat;
            Eigen::Matrix4d T_world_object = Converter::cvMattoMatrix4d(matrix);
            Eigen::Matrix4d T_baselink_object = T_world_baselink.cast<double>().inverse() * T_world_object;

            // 二、物体的尺寸
            double lenth = object->mCuboid3D.lenth;
            double width = object->mCuboid3D.width;
            double height = object->mCuboid3D.height;

            // 三、转为ConstrainedDualQuadric
            //                Eigen::Matrix4d matrix_4x4;
            //                matrix_4x4 << 1, 0, 0, x,
            //                              0, 1, 0, y,
            //                              0, 0, 1, z,
            //                              0, 0, 0, 1;
            //                ConstrainedDualQuadric Quadric(gtsam::Pose3(matrix_4x4), gtsam::Vector3(lenth/2.0,width/2.0,height/2.0));
            Quadric_ = ConstrainedDualQuadric(gtsam::Pose3(T_baselink_object),
                                                              gtsam::Vector3(lenth / 2.0, width / 2.0, height / 2.0));
        }

        void GenerateCalibration(Matrix3d &calib) {
            //            Eigen::Matrix3d Calib;
            //            Calib << fx,  0,  cx,
            //                      0,  fy,  cy,
            //                      0,   0,   1;
            //            boost::shared_ptr<gtsam::Cal3_S2> gtsam_calib = boost::make_shared<gtsam::Cal3_S2>(fx, fy, 0.0, cx, cy);
            gtsam_calibration_ = boost::make_shared<gtsam::Cal3_S2>(calib(0, 0), calib(1, 1), 0.0, calib(0, 2),
                                                                    calib(1, 2));
        }


    };
}

Vector7d convertGtsamToEigen_4(const gtsam::Vector& gtsamVec) {
    // 检查 gtsam::Vector 的大小
    if (gtsamVec.size() != 7) {
        throw std::invalid_argument("gtsam::Vector must be of size 7.");
    }

    // 创建 Eigen::Matrix<double, 7, 1> 并填充数据
    Vector7d eigenVec;
    eigenVec = Eigen::Map<const Vector7d>(gtsamVec.data());

    return eigenVec;
}


#include <gpmp2/obstacle/ObstacleCost.h>
#include <vector>
using namespace std;
using namespace gtsam;

namespace gpmp2 {
    /* ************************************************************************** */


    template<class ROBOT>
    gtsam::Vector CentorEllipsoidFactor<ROBOT>::evaluateError(
        const typename Robot::Pose &conf, gtsam::OptionalMatrixType H1) const {

            Vector error;

            Vector7d eigenVec = convertGtsamToEigen_4(conf);

            if (H1) {
                error = evaluateError_fornumericalDerivative(eigenVec, H1);
                return error;
            }
            else {
                error = evaluateError_fornumericalDerivative(eigenVec, {});
                return error;
            }
            // calculate derivative of error wrt quadric
    }


    template<class ROBOT>
    gtsam::Vector CentorEllipsoidFactor<ROBOT>::evaluateError_fornumericalDerivative(
        const Vector7d & conf, gtsam::OptionalMatrixType H1) const{
        
        // 一, 获取机械臂末端位姿（相对于baselink），获取dex_djoint偏导
        std::vector<Pose3> joint_pos;
        std::vector<gtsam::Matrix> dex_djoint;
        robot_.fk_model().forwardKinematics(conf, {}, joint_pos, {}, &dex_djoint);
        gtsam::Pose3 T_baselink_to_endlink = joint_pos[joint_pos.size()-1];
        Eigen::Matrix4d T_endlink_to_c;
        T_endlink_to_c <<   0, 0, 1, 0.02,
                            -1, 0, 0, -0.013,
                            0, -1, 0, 0.07, //实际为0.13(用于ros)，改为0.07(用于gpmp2 forwardKinematics)
                            0, 0, 0, 1;
        Eigen::Matrix4d T_baselink_to_camera = T_baselink_to_endlink.matrix() * T_endlink_to_c;
        gtsam::Pose3 T_endlink_to_camera(T_endlink_to_c);
        gtsam::Pose3 camera_pose(T_baselink_to_camera);

        // 二、点的坐标
        // 定义相机的外参（相机的位姿：旋转和平移）
        // gtsam::Pose3 cameraPose(T_baselink_to_camera);  // 相机位于 (0, 0, 0)，无旋转
        // // 创建一个 PinholeCamera 实例
        // gtsam::PinholeCamera<gtsam::Cal3_S2> camera(camera_pose, *gtsam_calibration_);
        // // 定义一个 3D 点 (X, Y, Z)
        // gtsam::Point3 point3D(Quadric_.pose().x(), Quadric_.pose().y(), Quadric_.pose().z());  // 3D 点的坐标
        // // 投影 3D 点到 2D 图像平面
        // gtsam::Point2 point2D = camera.project(point3D);
        // gtsam_calibration_->print("evaluateError_fornumericalDerivative: gtsam_calibration_ ");
        // camera_pose.print("evaluateError_fornumericalDerivative: camera_pose ");
        // std::cout<<"evaluateError_fornumericalDerivative: object x:"<<point3D.x()<<", y:"<<point3D.y()<<", z:"<<point3D.z()<<std::endl;
        // std::cout<<"evaluateError_fornumericalDerivative: point x:"<<point2D.x()<<", y:"<<point2D.y()<<std::endl;

        //
        std::cout<<"evaluateError_fornumericalDerivative: camera_link-pose: "<<endl<<T_baselink_to_endlink<<endl;
        std::cout<<"evaluateError_fornumericalDerivative: camera_pose: "<<endl<<T_baselink_to_camera<<endl;
        // 将点转换为齐次坐标
        Eigen::Vector4d point_baselink_homogeneous(Quadric_.pose().x(), Quadric_.pose().y(), Quadric_.pose().z(), 1.0);
        // 进行变换
        Eigen::Vector4d point_camera_homogeneous = T_baselink_to_camera.inverse() * point_baselink_homogeneous;
        // 将结果转换回非齐次坐标
        Eigen::Vector3d point3D(point_camera_homogeneous(0) / point_camera_homogeneous(3),
                                      point_camera_homogeneous(1) / point_camera_homogeneous(3),
                                      point_camera_homogeneous(2) / point_camera_homogeneous(3));
        // 相机内参
        float fx = 554.254691191187;
        float fy = 554.254691191187;
        float cx = 320.5;
        float cy = 240.5;
        // 计算 2D 点
        double z = point3D(2);
        Eigen::Vector2d point2D(
            (fx * point3D(0) / z) + cx,
            (fy * point3D(1) / z) + cy
        );
        std::cout<<"evaluateError_fornumericalDerivative: world object x:"<<Quadric_.pose().x()<<", y:"<<Quadric_.pose().y()<<", z:"<<Quadric_.pose().z()<<std::endl;
        std::cout<<"evaluateError_fornumericalDerivative: camera object x:"<<point3D(0)<<", y:"<<point3D(1)<<", z:"<<point3D(2)<<std::endl;
        std::cout<<"evaluateError_fornumericalDerivative: point x:"<<point2D.x()<<", y:"<<point2D.y()<<std::endl;

        // 三、计算误差和雅可比
        double delta_x = point2D.x()- miImageCols/2.0;
        double delta_y = point2D.y()- miImageRows/2.0;
        double distance = sqrt(delta_x*delta_x + delta_y*delta_y);
        // std::cout<< "[debug] CameraXaxisHorizontal::evaluateError_forNum   error_angle: \n" << error_angle/M_PI_2*90 << std::endl;
        Vector error(1);
        error[0] = distance;

        // 5. 计算雅可比矩阵：
        // calculate derivative of error wrt joints
        std::function<gtsam::Vector(const Vector7d &)> funPtr(
        boost::bind(&CentorEllipsoidFactor<ROBOT>::evaluateError_fornumericalDerivative, this,
                    boost::placeholders::_1, nullptr));

        if (H1) {
            Eigen::Matrix<double, 1, 7> de_dr =
                    gtsam::numericalDerivative11(funPtr, conf, 1e-6);
            *H1 = de_dr;
        }

        return error;

    }




    template<class ROBOT>
    void CentorEllipsoidFactor<ROBOT>::visulize(const typename Robot::Pose &conf, const string name) const {
        double board = 200;
        cv::Mat image = cv::Mat::zeros(miImageRows+board*2, miImageCols+board*2, CV_8UC3);
        cv::Rect FOV_cvmat(board, board, miImageCols, miImageRows);
    	Eigen::Vector4d measure_bbox = getNewMeasureBounds ( conf );
        // std::cout<<"[debug] measure_bbox: "<<measure_bbox.transpose()<<std::endl;
        cv::Rect measure_bbox_cvmat(measure_bbox[0]+board, measure_bbox[1]+board, measure_bbox[2]-measure_bbox[0], measure_bbox[3]-measure_bbox[1]);
    	Eigen::Vector4d predict_bbox = getPredictedBounds ( conf );
        // std::cout<<"[debug] predict_bbox: "<<predict_bbox.transpose()<<std::endl;
        cv::Rect predict_bbox_cvmat(predict_bbox[0]+board, predict_bbox[1]+board, predict_bbox[2]-predict_bbox[0], predict_bbox[3]-predict_bbox[1]);
        cv::rectangle(image, FOV_cvmat, cv::Scalar(255, 255, 255), 2);
        cv::rectangle(image, measure_bbox_cvmat, cv::Scalar(255, 0, 0), 2);
        cv::rectangle(image, predict_bbox_cvmat, cv::Scalar(0, 255, 0), 2);
        auto error = evaluateError(conf);
        // std::cout<<"[debug] error: "<<error.transpose()<<std::endl;
        cv::imshow(name, image);
        cv::waitKey(100);
    }

    
} // namespace gpmp2

#endif
