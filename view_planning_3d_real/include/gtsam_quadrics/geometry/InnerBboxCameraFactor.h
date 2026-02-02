/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision,
 Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file BoundingBoxFactor.h
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief factor between Pose3 and ConstrainedDualQuadric
 */

#pragma once

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam_quadrics/geometry/AlignedBox2.h>
#include <gtsam_quadrics/geometry/ConstrainedDualQuadric.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/bind/bind.hpp>

#include "MapObject.h"
#include "Converter.h"
#include <opencv2/opencv.hpp>


namespace gtsam_quadrics {
    /**
     * @class BoundingBoxFactor
     * AlignedBox3 factor between Pose3 and ConstrainedDualQuadric
     * Projects the quadric at the current pose estimates,
     * Calculates the bounds of the dual conic,
     * and compares this to the measured bounding box.
     */
    class InnerBboxCameraFactor
            : public gtsam::NoiseModelFactorN<gtsam::Pose3> {
    public:
        enum MeasurementModel {
            STANDARD,
            TRUNCATED
        }; ///< enum to declare which error function to use

    protected:
        AlignedBox2 measured_; ///< measured bounding box
        typedef NoiseModelFactorN<gtsam::Pose3> Base; ///< base class has keys and noisemodel as private members
        MeasurementModel measurementModel_;

        int miImageCols, miImageRows;

        boost::shared_ptr<gtsam::Cal3_S2> gtsam_calibration_;

        // 目标物体。物体的位姿实在world坐标系中
        ConstrainedDualQuadric Quadric_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /// @name Constructors and named constructors
        /// @{

        /** Default constructor */
        InnerBboxCameraFactor()
            : measured_(0., 0., 0., 0.), measurementModel_(STANDARD) {
        };

        /** Constructor from measured box, calbration, dimensions and posekey,
         * quadrickey, noisemodel */
        InnerBboxCameraFactor(gtsam::Key poseKey,
                         double cost_sigma,
                         const AlignedBox2 &measured,
                         MapObject *ob_,
                         Eigen::Matrix4f mRobotPose_,
                         int width, int height, Eigen::Matrix3d calib)
            : Base(gtsam::noiseModel::Isotropic::Sigma(4, cost_sigma), poseKey),
              measured_(measured),
              miImageCols(width),
              miImageRows(height)
        {
            // 生成世界坐标系下的目标物体
            GenerateConstrainedDualQuadric(ob_, mRobotPose_);
            // 生成 内参
            GenerateCalibration(calib);
            // 生成 目标bbox
            // GenerateMeasureBbox(measured);
            //
            measurementModel_ = STANDARD;
        };


        /// @}
        /// @name Class accessors
        /// @{

        /** Returns the measured bounding box */
        AlignedBox2 measurement() const { return AlignedBox2(measured_.vector()); }

        /** Returns the pose key */
        gtsam::Key poseKey() const { return key1(); }

        /// @}
        /// @name Class methods
        /// @{

        /**
         * Evaluate the error between a quadric and 3D pose
         * @param pose the 6DOF camera position
         * @param quadric the constrained dual quadric
         * @param H1 the derivative of the error wrt camera pose (4x6)
         * @param H2 the derivative of the error wrt quadric (4x9)
         */
        gtsam::Vector evaluateError(
            const gtsam::Pose3 &pose,
            gtsam::OptionalMatrixType H1 = nullptr) const;

        gtsam::Vector getPredictedBounds(
            const gtsam::Pose3 &pose,
            gtsam::OptionalMatrixType H1 = nullptr) const;

        gtsam::Vector getMeasureBounds(
            const gtsam::Pose3 &pose,
            gtsam::OptionalMatrixType H1 = nullptr) const;


        /// @}
        /// @name Testable group traits
        /// @{

        /** Prints the boundingbox factor with optional string */
        void print(const std::string &s = "",
                   const gtsam::KeyFormatter &keyFormatter =
                           gtsam::DefaultKeyFormatter) const override;

        /** Returns true if equal keys, measurement, noisemodel and calibration */
        bool equals(const InnerBboxCameraFactor &other, double tol = 1e-9) const;

    private:
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

        void GenerateCalibration(Eigen::Matrix3d &calib) {
            //            Eigen::Matrix3d Calib;
            //            Calib << fx,  0,  cx,
            //                      0,  fy,  cy,
            //                      0,   0,   1;
            //            boost::shared_ptr<gtsam::Cal3_S2> gtsam_calib = boost::make_shared<gtsam::Cal3_S2>(fx, fy, 0.0, cx, cy);
            gtsam_calibration_ = boost::make_shared<gtsam::Cal3_S2>(calib(0, 0), calib(1, 1), 0.0, calib(0, 2),
                                                                    calib(1, 2));
        }

        gtsam_quadrics::AlignedBox2 AdjustingBBox(const AlignedBox2& predicted_bbox, const AlignedBox2& measured_bbox) const{
            // gtsam_quadrics::AlignedBox2 gtsam_bbox(0+s, 0+s, CameraWidth-s, CameraHeight-s);   //预期的物体检测框
            // measured_ = measured;
            double predicted_width = predicted_bbox.xmax() - predicted_bbox.xmin();
            double predicted_height = predicted_bbox.ymax() - predicted_bbox.ymin();

            double measured_width = measured_bbox.xmax() - measured_bbox.xmin();
            double measured_height = measured_bbox.ymax() - measured_bbox.ymin();

            if( predicted_width/predicted_height > measured_width/measured_height) {

                double new_height = measured_width * predicted_height / predicted_width;
                double new_ymin = measured_bbox.ymin() + (measured_height - new_height) / 2;
                double new_ymax = new_ymin + new_height;
                gtsam_quadrics::AlignedBox2 new_measured_bbox (measured_bbox.xmin(), new_ymin, measured_bbox.xmax(), new_ymax);
                std::cout<<"[debug] new_measured_bbox: "<<new_measured_bbox.vector().transpose()<<std::endl;
                return new_measured_bbox;
            } else {
                double new_width = measured_height * predicted_width / predicted_height;
                double new_xmin = measured_bbox.xmin() + (measured_width - new_width) / 2;
                double new_xmax = new_xmin + new_width;
                gtsam_quadrics::AlignedBox2 new_measured_bbox (new_xmin, measured_bbox.ymin(), new_xmax, measured_bbox.ymax());
                std::cout<<"[debug] new_measured_bbox: "<<new_measured_bbox.vector().transpose()<<std::endl;
                return new_measured_bbox;
            }

        }
    public:
        void visulize(const  gtsam::Pose3 &conf) const {
            double board = 200;
            cv::Mat image = cv::Mat::zeros(miImageRows+board*2, miImageCols+board*2, CV_8UC3);
            cv::Rect FOV_cvmat(board, board, miImageCols, miImageRows);
            Eigen::Vector4d measure_bbox = getMeasureBounds ( conf );
            std::cout<<"[debug] measure_bbox: "<<measure_bbox.transpose()<<std::endl;
            cv::Rect measure_bbox_cvmat(measure_bbox[0]+board, measure_bbox[1]+board, measure_bbox[2]-measure_bbox[0], measure_bbox[3]-measure_bbox[1]);
            Eigen::Vector4d predict_bbox = getPredictedBounds ( conf );
            std::cout<<"[debug] predict_bbox: "<<predict_bbox.transpose()<<std::endl;
            cv::Rect predict_bbox_cvmat(predict_bbox[0]+board, predict_bbox[1]+board, predict_bbox[2]-predict_bbox[0], predict_bbox[3]-predict_bbox[1]);
            cv::rectangle(image, FOV_cvmat, cv::Scalar(255, 255, 255), 2);
            cv::rectangle(image, measure_bbox_cvmat, cv::Scalar(255, 0, 0), 2);
            cv::rectangle(image, predict_bbox_cvmat, cv::Scalar(0, 255, 0), 2);
            auto error = evaluateError(conf);
            std::cout<<"[debug] error: "<<error.transpose()<<std::endl;
            cv::imshow("Bounding Boxes", image);
            cv::waitKey(100);
        }


        gtsam::Vector4 computeBoundaryDifferences(const gtsam::Vector4& predictedBounds, const gtsam::Vector4& measured_) const{
            // Extract the bounds for better readability
            double x_min_pred = predictedBounds[0];
            double y_min_pred = predictedBounds[1];
            double x_max_pred = predictedBounds[2];
            double y_max_pred = predictedBounds[3];

            double x_min_meas = measured_[0];
            double y_min_meas = measured_[1];
            double x_max_meas = measured_[2];
            double y_max_meas = measured_[3];

            // Compute the differences
            double x_min_diff = (x_min_pred > x_min_meas) ? (x_min_meas - x_min_pred) : 0.0;
            double y_min_diff = (y_min_pred > y_min_meas) ? (y_min_meas - y_min_pred) : 0.0;
            double x_max_diff = (x_max_pred < x_max_meas) ? (x_max_pred - x_max_meas) : 0.0;
            double y_max_diff = (y_max_pred < y_max_meas) ? (y_max_pred - y_max_meas) : 0.0;

            // Output the differences
            gtsam::Vector4 error;
            error[0] = x_min_diff;
            error[1] = y_min_diff;
            error[2] = x_max_diff;
            error[3] = y_max_diff;
            return error;
        }

    };
} // namespace gtsam_quadrics

/** \cond PRIVATE */
// Add to testable group
// template <>
// struct gtsam::traits<gtsam_quadrics::InnerBboxCameraFactor>
//     : public gtsam::Testable<gtsam_quadrics::InnerBboxCameraFactor> {};
/** \endcond */
