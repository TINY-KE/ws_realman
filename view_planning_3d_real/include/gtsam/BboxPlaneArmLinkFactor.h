/**
 *  @file  BboxPlaneArmLinkFactor.h
 *  @brief Obstacle avoidance cost factor, using 3D signed distance field
 *  @author Jing Dong
 *  @date  May 11, 2016
 **/

#ifndef GTSAM_BBOX_PLANE_ARM_LINK_H
#define GTSAM_BBOX_PLANE_ARM_LINK_H

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>

#include <iostream>
#include <vector>

#include "plane/Plane.h"
#include <gtsam/base/numericalDerivative.h>


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

#define use

using namespace gtsam;

namespace gpmp2 {
    /**
     * unary factor for obstacle avoidance
     * template robot model version
     */
    template<class ROBOT>
    class BboxPlaneArmLinkFactor
            : public gtsam::NoiseModelFactorN<typename ROBOT::Pose> {
    public:
        // typedefs
        typedef ROBOT Robot;
        typedef typename Robot::Pose Pose;

    private:
        // typedefs
        typedef BboxPlaneArmLinkFactor This;
        typedef gtsam::NoiseModelFactorN<Pose> Base;

        // obstacle cost settings
        double epsilon_; // distance from object that start non-zero cost

        // arm: planar one, all alpha = 0
        const Robot &robot_;

        // 图像宽度和高度
        int miImageCols; // = Config::Get<int>("Camera.width");
        int miImageRows; // = Config::Get<int>("Camera.height");
        Matrix3d mCalib;
        g2o::plane *mPlaneLow;

    public:
        /// shorthand for a smart pointer to a factor
        typedef std::shared_ptr<This> shared_ptr;

        /* Default constructor */
        BboxPlaneArmLinkFactor() {
        }

        /**
         * Constructor
         * @param cost_model cost function covariance, should to identity model
         * @param field      signed distance field
         * @param nn_index   nearest neighbour index of signed distance field
         */
        BboxPlaneArmLinkFactor(gtsam::Key poseKey, const Robot &robot,
                               double cost_sigma,
                               double epsilon,
                               int width, int height, Matrix3d calib)
            : Base(gtsam::noiseModel::Isotropic::Sigma(robot.nr_body_spheres(),
                                                       cost_sigma),
                   poseKey),
              epsilon_(epsilon),
              robot_(robot),
              miImageCols(width),
              miImageRows(height),
              mCalib(calib) {
            GenerateBboxBlowPlane_g2o();
        }

        virtual ~BboxPlaneArmLinkFactor() {
        }

        /// error function
        /// numerical jacobians / analytic jacobians from cost function
        // zhjd: 此处的Robot::Pose对应的是gtsam::Vector。   根据 ForwardKinematics<gtsam::Vector, gtsam::Vector>。 因此这里的pose就是关节角度。
        gtsam::Vector evaluateError(
            const typename Robot::Pose &conf,
            gtsam::OptionalMatrixType H1 = nullptr) const override;

        gtsam::Vector evaluateError_fornumericalDerivative(
            const Vector7d &conf,
            gtsam::OptionalMatrixType H1 = nullptr) const;

        g2o::plane *computeplane(
            const typename Robot::Pose &conf, bool output = false);

        /// @return a deep copy of this factor
        virtual gtsam::NonlinearFactor::shared_ptr clone() const {
            return std::static_pointer_cast<gtsam::NonlinearFactor>(
                gtsam::NonlinearFactor::shared_ptr(new This(*this)));
        }

        /** print contents */
        void print(const std::string &s = "",
                   const gtsam::KeyFormatter &keyFormatter =
                           gtsam::DefaultKeyFormatter) const {
            std::cout << s << "BboxPlaneArmLinkFactor :" << std::endl;
            Base::print("", keyFormatter);
        }

#ifdef GPMP2_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int version) {
    ar& boost::serialization::make_nvp(
        "NoiseModelFactor1", boost::serialization::base_object<Base>(*this));
  }
#endif

    private:
        Eigen::Matrix3Xd generateProjectionMatrix();

        Eigen::MatrixXd fromDetectionsToLines();

        Eigen::MatrixXd GenerateBboxBlowPlane();

        void GenerateBboxBlowPlane_g2o();
    };
} // namespace gpmp2


double hingeLossFovCost(
    const gtsam::Point3 &point, g2o::plane *plane_low, double eps,
    gtsam::OptionalJacobian<1, 3> H_point = nullptr) {
    gtsam::Vector3 field_gradient; //距离对Point3的导数，
    double dist_signed; //Point3与最低视场平面之间的最近距离

    //（注意平面的normal是向上，因此带入平面方程后的距离结果要取负数）
    dist_signed = -1 * plane_low->distanceToPoint(point, true);

    if (dist_signed > eps) {
        // faraway no error
        if (H_point) *H_point = gtsam::Matrix13::Zero();
        return 0.0;
    } else {
        // outside but < eps or inside object
        if (H_point) {
            // ERROR增加的方向是靠近障碍物的方向，与平面的normal方向相同。
            field_gradient = plane_low->normal();
            field_gradient.normalize();
            *H_point = field_gradient.transpose();
        }
        return eps - dist_signed;
    }
}

Vector7d convertGtsamToEigen(const gtsam::Vector& gtsamVec) {
    // 检查 gtsam::Vector 的大小
    if (gtsamVec.size() != 7) {
        throw std::invalid_argument("gtsam::Vector must be of size 7.");
    }

    // 创建 Eigen::Matrix<double, 7, 1> 并填充数据
    Vector7d eigenVec;
    eigenVec = Eigen::Map<const Vector7d>(gtsamVec.data());

    return eigenVec;
}


using namespace std;
using namespace gtsam;

namespace gpmp2 {
    /* ************************************************************************** */

    template<class ROBOT>
    g2o::plane *BboxPlaneArmLinkFactor<ROBOT>::computeplane(
        const typename Robot::Pose &conf, bool output) {
        // 更改平面的位置
        using namespace gtsam;
        // 相机相对于endlink的位姿
        Eigen::Matrix4f T_endlink_to_c;
        T_endlink_to_c << 0, 0, 1, 0.02,
                -1, 0, 0, -0.013,
                0, -1, 0, 0.07, //实际为0.13，改为0.07
                0, 0, 0, 1;
        // 机械臂endlink的位姿
        std::vector<Pose3> joint_pos; //  link poses in 3D work space
        std::vector<gtsam::Matrix> J_jpx_jp; //  et al. optional Jacobians
        robot_.fk_model().forwardKinematics(conf, {}, joint_pos);
        Pose3 pose_end_link = joint_pos[joint_pos.size() - 1];
        if (output)
            pose_end_link.print("[zhjd-debug] pose_end_link: \n");
        // 将 gtsam::Pose3 转换为 Eigen::Matrix4f
        Eigen::Matrix4f T_baselink_endlink = Eigen::Matrix4f::Identity(); // 创建 4x4 单位矩阵
        // 获取 gtsam::Pose3 的 3x3 旋转矩阵并赋值到 eigenMatrix 的左上角
        T_baselink_endlink.block<3, 3>(0, 0) = pose_end_link.rotation().matrix().cast<float>();
        // 获取 gtsam::Pose3 的 3x1 平移向量并赋值到 eigenMatrix 的右侧
        T_baselink_endlink.block<3, 1>(0, 3) << pose_end_link.x(), pose_end_link.y(), pose_end_link.z();
        if (output)
            std::cout << "[zhjd-debug] T_baselink_endlink: " << std::endl << T_baselink_endlink << std::endl;

        Eigen::Matrix4f T_baselink_2_c = T_baselink_endlink * T_endlink_to_c;


        // 将平面变到baselink坐标系
        g2o::plane *pl_in_baselink = new g2o::plane(*mPlaneLow);

        pl_in_baselink->transform(T_baselink_2_c);

        return pl_in_baselink;
    }


    // template <class ROBOT>
    // gtsam::Vector BboxPlaneArmLinkFactor<ROBOT>::evaluateError(
    //        const typename Robot::Pose& conf, gtsam::OptionalMatrixType H1) const {
    //
    //      // 更改平面的位置
    //      using namespace gtsam;
    //      // 相机相对于endlink的位姿
    //      Eigen::Matrix4f T_endlink_to_c;
    //      T_endlink_to_c << 0, 0, 1, 0.02,
    //                        -1, 0, 0, -0.013,
    //                        0, -1, 0, 0.07,  //实际为0.13，改为0.07
    //                        0, 0, 0, 1;
    //      // 机械臂endlink的位姿
    //      std::vector<Pose3> joint_pos;   //  link poses in 3D work space
    //      std::vector<gtsam::Matrix> J_jpx_jp;   //  et al. optional Jacobians
    //      robot_.fk_model().forwardKinematics(conf, {}, joint_pos);
    //      Pose3 pose_end_link = joint_pos[joint_pos.size()-1];
    //      // 将 gtsam::Pose3 转换为 Eigen::Matrix4f
    //      Eigen::Matrix4f T_baselink_endlink = Eigen::Matrix4f::Identity();  // 创建 4x4 单位矩阵
    //      // 获取 gtsam::Pose3 的 3x3 旋转矩阵并赋值到 eigenMatrix 的左上角
    //      T_baselink_endlink.block<3, 3>(0, 0) = pose_end_link.rotation().matrix().cast<float>();
    //      // 获取 gtsam::Pose3 的 3x1 平移向量并赋值到 eigenMatrix 的右侧
    //      T_baselink_endlink.block<3, 1>(0, 3) << pose_end_link.x(), pose_end_link.y(), pose_end_link.z();
    //
    //      Eigen::Matrix4f T_baselink_2_c = T_baselink_endlink * T_endlink_to_c;
    //
    //      // 将平面变到baselink坐标系
    //      g2o::plane* pl_in_baselink = new g2o::plane(*mPlaneLow);
    //      pl_in_baselink->transform(T_baselink_2_c);
    //
    //      // if Jacobians used, initialize as zeros
    //      // size: arm_nr_points_ * DOF
    //      if (H1) *H1 = gtsam::Matrix::Zero(robot_.nr_body_spheres(), robot_.dof());
    //
    //      // run forward kinematics of this configuration
    //      vector<Point3> sph_centers;
    //      vector<gtsam::Matrix> J_px_jp;
    //      if (H1)
    //       robot_.sphereCenters(conf, sph_centers, &J_px_jp);
    //      else
    //       robot_.sphereCenters(conf, sph_centers);
    //
    //      // allocate cost vector
    //      Vector err(robot_.nr_body_spheres());
    //
    //      // for each point on arm stick, get error
    //      for (size_t sph_idx = 0; sph_idx < robot_.nr_body_spheres(); sph_idx++) {
    //           const double total_eps = robot_.sphere_radius(sph_idx) + epsilon_;
    //
    //           if (H1) {
    //            Matrix13 Jerr_point;
    //            err(sph_idx) = hingeLossFovCost(sph_centers[sph_idx], pl_in_baselink,
    //                                                 total_eps, Jerr_point);
    //
    //            // chain rules
    //            H1->row(sph_idx) = Jerr_point * J_px_jp[sph_idx];
    //
    //           } else {
    //            err(sph_idx) =
    //                hingeLossFovCost(sph_centers[sph_idx], pl_in_baselink, total_eps);
    //           }
    //      }
    //
    //      return err;
    // }


    template<class ROBOT>
    gtsam::Vector BboxPlaneArmLinkFactor<ROBOT>::evaluateError(
        const typename Robot::Pose& conf, gtsam::OptionalMatrixType H1) const {
        // 更改平面的位置
        using namespace gtsam;
        // 相机相对于endlink的位姿
        Eigen::Matrix4f T_endlink_to_c;
        T_endlink_to_c << 0, 0, 1, 0.02,
                -1, 0, 0, -0.013,
                0, -1, 0, 0.07, //实际为0.13，改为0.07
                0, 0, 0, 1;
        // 机械臂endlink的位姿
        std::vector<Pose3> joint_pos; //  link poses in 3D work space
        std::vector<gtsam::Matrix> J_jpx_jp; //  et al. optional Jacobians
        robot_.fk_model().forwardKinematics(conf, {}, joint_pos);
        Pose3 pose_end_link = joint_pos[joint_pos.size() - 1];
        // 将 gtsam::Pose3 转换为 Eigen::Matrix4f
        Eigen::Matrix4f T_baselink_endlink = Eigen::Matrix4f::Identity(); // 创建 4x4 单位矩阵
        // 获取 gtsam::Pose3 的 3x3 旋转矩阵并赋值到 eigenMatrix 的左上角
        T_baselink_endlink.block<3, 3>(0, 0) = pose_end_link.rotation().matrix().cast<float>();
        // 获取 gtsam::Pose3 的 3x1 平移向量并赋值到 eigenMatrix 的右侧
        T_baselink_endlink.block<3, 1>(0, 3) << pose_end_link.x(), pose_end_link.y(), pose_end_link.z();

        Eigen::Matrix4f T_baselink_2_c = T_baselink_endlink * T_endlink_to_c;

        // 将平面变到baselink坐标系
        g2o::plane *pl_in_baselink = new g2o::plane(*mPlaneLow);
        pl_in_baselink->transform(T_baselink_2_c);

        // if Jacobians used, initialize as zeros
        // size: arm_nr_points_ * DOF


        // run forward kinematics of this configuration
        vector<Point3> sph_centers;
        robot_.sphereCenters(conf, sph_centers);

        // allocate cost vector
        Vector err(robot_.nr_body_spheres());

        // for each point on arm stick, get error
        for (size_t sph_idx = 0; sph_idx < robot_.nr_body_spheres(); sph_idx++) {
            const double total_eps = robot_.sphere_radius(sph_idx) + epsilon_;

            err(sph_idx) =
                    hingeLossFovCost(sph_centers[sph_idx], pl_in_baselink, total_eps);
        }

        if(H1) {
            Vector7d eigenVec = convertGtsamToEigen(conf);
            evaluateError_fornumericalDerivative(eigenVec, H1);
        }

        return err;
    }

    template<class ROBOT>
    gtsam::Vector BboxPlaneArmLinkFactor<ROBOT>::evaluateError_fornumericalDerivative(
        const Vector7d & conf, gtsam::OptionalMatrixType H1) const{
        
        // std::cout<< "[debug] BboxPlaneArmLinkFactor::evaluateError_forNum   conf: " << conf.transpose() << std::endl;

        // 更改平面的位置
        using namespace gtsam;
        // 相机相对于endlink的位姿
        Eigen::Matrix4f T_endlink_to_c;
        T_endlink_to_c << 0, 0, 1, 0.02,
                -1, 0, 0, -0.013,
                0, -1, 0, 0.07, //实际为0.13，改为0.07
                0, 0, 0, 1;
        // 机械臂endlink的位姿
        std::vector<Pose3> joint_pos; //  link poses in 3D work space
        std::vector<gtsam::Matrix> J_jpx_jp; //  et al. optional Jacobians
        robot_.fk_model().forwardKinematics(conf, {}, joint_pos);
        Pose3 pose_end_link = joint_pos[joint_pos.size() - 1];
        // 将 gtsam::Pose3 转换为 Eigen::Matrix4f
        Eigen::Matrix4f T_baselink_endlink = Eigen::Matrix4f::Identity(); // 创建 4x4 单位矩阵
        // 获取 gtsam::Pose3 的 3x3 旋转矩阵并赋值到 eigenMatrix 的左上角
        T_baselink_endlink.block<3, 3>(0, 0) = pose_end_link.rotation().matrix().cast<float>();
        // 获取 gtsam::Pose3 的 3x1 平移向量并赋值到 eigenMatrix 的右侧
        T_baselink_endlink.block<3, 1>(0, 3) << pose_end_link.x(), pose_end_link.y(), pose_end_link.z();

        Eigen::Matrix4f T_baselink_2_c = T_baselink_endlink * T_endlink_to_c;

        // 将平面变到baselink坐标系
        g2o::plane *pl_in_baselink = new g2o::plane(*mPlaneLow);
        pl_in_baselink->transform(T_baselink_2_c);

        // run forward kinematics of this configuration
        vector<Point3> sph_centers;
        robot_.sphereCenters(conf, sph_centers);

        // allocate cost vector
        Vector err(9);

        // for each point on arm stick, get error
        for (size_t sph_idx = 0; sph_idx < 9; sph_idx++) {
            const double total_eps = robot_.sphere_radius(sph_idx) + epsilon_;
            err(sph_idx) =
                    hingeLossFovCost(sph_centers[sph_idx], pl_in_baselink, total_eps);
        }

        std::function<gtsam::Vector(const Vector7d &)> funPtr(
            boost::bind(&BboxPlaneArmLinkFactor<ROBOT>::evaluateError_fornumericalDerivative, this,
                        boost::placeholders::_1, nullptr));

        if (H1) {
            // 避障球有9个
            Eigen::Matrix<double, 9, 7> de_dr =
                    gtsam::numericalDerivative11(funPtr, conf, 1e-6);
            *H1 = de_dr;
            // std::cout<< "[debug] BboxPlaneArmLinkFactor::evaluateError_forNum   H1: \n" << *H1 << std::endl;
        }

        // std::cout<< "[debug] END !!!!!    BboxPlaneArmLinkFactor::evaluateError_forNum " << std::endl;
        return err;
    }

    template<class ROBOT>
    Eigen::Matrix3Xd BboxPlaneArmLinkFactor<ROBOT>::generateProjectionMatrix() {
        Eigen::Matrix3Xd identity_lefttop;
        identity_lefttop.resize(3, 4);
        identity_lefttop.col(3) = Vector3d(0, 0, 0);
        identity_lefttop.topLeftCorner<3, 3>() = Matrix3d::Identity(3, 3);

        Eigen::Matrix3Xd proj_mat = mCalib * identity_lefttop; //维度是 3x4

        // g2o::SE3Quat campose_wc = g2o::SE3Quat();
        // g2o::SE3Quat campose_cw = campose_wc.inverse();
        Eigen::Matrix4d campose_wc = Eigen::Matrix4d::Identity();
        // proj_mat = proj_mat * campose_cw.to_homogeneous_matrix();
        proj_mat = proj_mat * campose_wc.inverse();

        return proj_mat;
    }

    template<class ROBOT>
    Eigen::MatrixXd BboxPlaneArmLinkFactor<ROBOT>::fromDetectionsToLines() {
        bool flag_openFilter = false; // filter those lines lying on the image boundary

        double x1 = 0;
        double y1 = 0;
        double x2 = miImageCols;
        double y2 = miImageRows;

        // line4表示一条底部水平线的平面方程，其方程为： y = y2
        // 通过向量形式表示为： 0*x + 1*y - y2 = 0
        Eigen::Vector3d line4(0, 1, -y2);

        // those lying on the image boundary have been marked -1
        Eigen::MatrixXd line_selected(3, 0);
        Eigen::MatrixXd line_selected_none(3, 0);

        int config_border_pixel = 10;

        // if (!flag_openFilter || (x1 > config_border_pixel && x1 < miImageCols - config_border_pixel)) {
        //     line_selected.conservativeResize(3, line_selected.cols() + 1);
        //     line_selected.col(line_selected.cols() - 1) = line1;
        // }
        // if (!flag_openFilter || (y1 > config_border_pixel && y1 < miImageRows - config_border_pixel)) {
        //     line_selected.conservativeResize(3, line_selected.cols() + 1);
        //     line_selected.col(line_selected.cols() - 1) = line2;
        // }
        // if (!flag_openFilter || (x2 > config_border_pixel && x2 < miImageCols - config_border_pixel)) {
        //     line_selected.conservativeResize(3, line_selected.cols() + 1);
        //     line_selected.col(line_selected.cols() - 1) = line3;
        // }
        // if (!flag_openFilter || (y2 > config_border_pixel && y2 < miImageRows - config_border_pixel)) {
        // 将其列数增加 1。
        line_selected.conservativeResize(3, line_selected.cols() + 1);
        // 将向量 line4 赋值给矩阵 line_selected 的最后一列。
        line_selected.col(line_selected.cols() - 1) = line4;
        // }

        return line_selected;
    }

    template<class ROBOT>
    Eigen::MatrixXd BboxPlaneArmLinkFactor<ROBOT>::GenerateBboxBlowPlane() {
        Eigen::MatrixXd planes_all(4, 0);
        // std::cout << " [debug] calib : \n " << calib << std::endl;
        // get projection matrix

        Eigen::MatrixXd P = generateProjectionMatrix();

        Eigen::MatrixXd low_line = fromDetectionsToLines();
        Eigen::MatrixXd low_plane = P.transpose() * low_line;

        // 相机z轴方向，各个平面的normal应该与camera_direction 夹角小于90度
        Eigen::Vector3d camera_direction(0, 0, 1); // Z轴方向

        // add to matrix
        for (int m = 0; m < low_plane.cols(); m++) {
            // 获取平面的法向量 (a, b, c)
            Eigen::Vector3d normal = low_plane.block<3, 1>(0, m);

            // 检查法向量与z轴朝向相同，如果不是则反转法向量
            if (normal.dot(camera_direction) < 0) {
                // 反转法向量方向
                low_plane.col(m) = -low_plane.col(m);
            }

            planes_all.conservativeResize(planes_all.rows(), planes_all.cols() + 1);
            planes_all.col(planes_all.cols() - 1) = low_plane.col(m);
        }

        return planes_all;
    }


    template<class ROBOT>
    void BboxPlaneArmLinkFactor<ROBOT>::GenerateBboxBlowPlane_g2o() {
        Eigen::MatrixXd mPlanesParamLocal_Col = GenerateBboxBlowPlane(); // attention: store as 列
        Eigen::MatrixXd mPlanesParamLocal = mPlanesParamLocal_Col.transpose();
        Eigen::Vector4d vec = mPlanesParamLocal.row(0);
        mPlaneLow = new g2o::plane(vec.head(4));
    }
} // namespace gpmp2


#endif
