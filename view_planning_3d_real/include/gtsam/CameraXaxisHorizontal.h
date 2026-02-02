/**
 *  @file   CameraXaxisHorizontal.h
 *  @brief  Gaussian prior defined on the workspace pose of any joint of a robot
 *          given its state in configuration space
 *  @author zhang jiadong
 *  @date   Jan 8, 2018
 **/
#ifndef CameraXaxisHorizontal_H
#define CameraXaxisHorizontal_H

#pragma once

#include <gpmp2/kinematics/RobotModel.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "MapObject.h"
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam_quadrics/geometry/AlignedBox2.h>
#include <gtsam_quadrics/geometry/ConstrainedDualQuadric.h>
#include <gtsam_quadrics/geometry/BoundingBoxFactor.h>
#include <gtsam_quadrics/geometry/QuadricCamera.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/bind/bind.hpp>

#include "Converter.h"

#include <opencv2/opencv.hpp>
#include <cmath>
#include <algorithm>
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
    class CameraXaxisHorizontal
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
        typedef CameraXaxisHorizontal This;
        typedef gtsam::NoiseModelFactorN<Pose> Base;

        const Robot &robot_; // Robot

    public:
        /* Default constructor */
        CameraXaxisHorizontal() {
        }

        /// Constructor
        CameraXaxisHorizontal(gtsam::Key poseKey, const Robot &robot,
                              double cost_sigma
        )
            : Base(gtsam::noiseModel::Isotropic::Sigma(1, cost_sigma),
                   poseKey),
              robot_(robot) {

        }

        virtual ~CameraXaxisHorizontal() {
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


        

        /// @return a deep copy of this factor
        virtual gtsam::NonlinearFactor::shared_ptr clone() const {
            return std::static_pointer_cast<gtsam::NonlinearFactor>(
                gtsam::NonlinearFactor::shared_ptr(new This(*this)));
        }

        /** print contents */
        void print(const std::string &s = "",
                   const gtsam::KeyFormatter &keyFormatter =
                           gtsam::DefaultKeyFormatter) const {
            std::cout << s << "CameraXaxisHorizontal :" << std::endl;
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




    };
}

Vector7d convertGtsamToEigen_3(const gtsam::Vector& gtsamVec) {
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
    gtsam::Vector CameraXaxisHorizontal<ROBOT>::evaluateError(
        const typename Robot::Pose &conf, gtsam::OptionalMatrixType H1) const {

        // 一, 获取机械臂末端位姿（相对于baselink），获取dex_djoint偏导
        std::vector<Pose3> joint_pos;
        std::vector<gtsam::Matrix> dex_djoint;
        robot_.fk_model().forwardKinematics(conf, {}, joint_pos, {}, &dex_djoint);
        gtsam::Pose3 T_baselink_to_endlink = joint_pos[joint_pos.size()-1];

        Eigen::Matrix4d T_baselink_to_endlink_eigen = T_baselink_to_endlink.matrix() ;
        Matrix3d rotation_matrix = T_baselink_to_endlink_eigen.topLeftCorner<3, 3>();

        // 三、 机械臂末端的朝向用 rpy表示， 如何计算机械臂末端y轴与水平面的夹角, 计算Error，
        Eigen::Vector3d Y = rotation_matrix * Eigen::Vector3d(0, 1, 0); // 获取 Y 轴方向
        Vector3d H(0, 0, 1); // 水平面法向量

        double cos_theta = Y.dot(H) / (Y.norm() * H.norm());
        // 使用条件判断限制余弦值范围
        if (cos_theta < -1.0) {
            cos_theta = -1.0;
        } else if (cos_theta > 1.0) {
            cos_theta = 1.0;
        }
        double error_angle = fabs(acos(cos_theta) - M_PI_2); // 返回夹角（弧度）
        Vector err(1);
        err[0] = error_angle;



        // 5. 计算雅可比矩阵：
        // calculate derivative of error wrt joints
        if (H1) {
            Vector7d eigenVec = convertGtsamToEigen_3(conf);
            evaluateError_fornumericalDerivative(eigenVec, H1);
        }

        // calculate derivative of error wrt quadric



        return err;

    }


    template<class ROBOT>
    gtsam::Vector CameraXaxisHorizontal<ROBOT>::evaluateError_fornumericalDerivative(
        const Vector7d & conf, gtsam::OptionalMatrixType H1) const{


        // 一, 获取机械臂末端位姿（相对于baselink），获取dex_djoint偏导
        std::vector<Pose3> joint_pos;
        std::vector<gtsam::Matrix> dex_djoint;
        robot_.fk_model().forwardKinematics(conf, {}, joint_pos, {}, &dex_djoint);
        gtsam::Pose3 T_baselink_to_endlink = joint_pos[joint_pos.size()-1];

        Eigen::Matrix4d T_baselink_to_endlink_eigen = T_baselink_to_endlink.matrix() ;
        Matrix3d rotation_matrix = T_baselink_to_endlink_eigen.topLeftCorner<3, 3>();

        // 三、 机械臂末端的朝向用 rpy表示， 如何计算机械臂末端y轴与水平面的夹角, 计算Error，
        Eigen::Vector3d Y = rotation_matrix * Eigen::Vector3d(0, 1, 0); // 获取 Y 轴方向
        Vector3d H(0, 0, 1); // 水平面法向量

        double cos_theta = Y.dot(H) / (Y.norm() * H.norm());
        // 使用条件判断限制余弦值范围
        // if (cos_theta < -1.0) {
        //     cos_theta = -1.0;
        // } else if (cos_theta > 1.0) {
        //     cos_theta = 1.0;
        // }
        double error_angle = fabs(acos(cos_theta) - M_PI_2); // 返回夹角（弧度）
        // std::cout<< "[debug] CameraXaxisHorizontal::evaluateError_forNum   error_angle: \n" << error_angle/M_PI_2*90 << std::endl;
        Vector err(1);
        err[0] = error_angle;



        // 5. 计算雅可比矩阵：
        // calculate derivative of error wrt joints
        std::function<gtsam::Vector(const Vector7d &)> funPtr(
        boost::bind(&CameraXaxisHorizontal<ROBOT>::evaluateError_fornumericalDerivative, this,
                    boost::placeholders::_1, nullptr));

        if (H1) {
            Eigen::Matrix<double, 1, 7> de_dr =
                    gtsam::numericalDerivative11(funPtr, conf, 1e-6);
            *H1 = de_dr;
            // std::cout<< "[debug] CameraXaxisHorizontal::evaluateError_forNum   H1: \n" << *H1 << std::endl;
        }

        return err;
    }




    
} // namespace gpmp2

#endif
