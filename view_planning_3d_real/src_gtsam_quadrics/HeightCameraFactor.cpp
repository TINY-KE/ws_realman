/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision,
 Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file BoundingBoxFactor.cpp
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief factor between Pose3 and ConstrainedDualQuadric
 */

#include <gtsam/base/numericalDerivative.h>
#include <gtsam_quadrics/base/QuadricProjectionException.h>
#include <gtsam_quadrics/geometry/HeightCameraFactor.h>
#include <gtsam_quadrics/geometry/QuadricCamera.h>

#include <boost/bind/bind.hpp>

#define NUMERICAL_DERIVATIVE false

using namespace std;

namespace gtsam_quadrics {
    /* ************************************************************************* */
    // gtsam::Vector HeightCameraFactor::evaluateError(
    //     const gtsam::Pose3 &pose,
    //     gtsam::OptionalMatrixType H1) const {
    //
    //     // calculate derivative of error wrt pose
    //     if (H1) {
    //         // combine partial derivatives
    //         Eigen::Matrix<double, 1, 6> normal;
    //         normal << 0, 0, 0, 0, 0, 1;   // y p r x y z
    //         *H1 = normal;
    //     }
    //
    //     gtsam::Vector1 error = gtsam::Vector1::Zero();
    //     double dis = height_ - pose.z();
    //     if (dis < 0)
    //         error[0] = -1 * dis * height_scale_;
    //     return error;
    // }

    gtsam::Vector HeightCameraFactor::evaluateError(
        const gtsam::Pose3 &pose,
        gtsam::OptionalMatrixType H1) const {
        // calculate derivative of error wrt pose
        double delta_x = pose.x() - mRobotPose_(0, 3);
        double delta_y = pose.y() - mRobotPose_(1, 3);
        double delta_z = pose.z() - mRobotPose_(2, 3);

        if (H1) {
            Eigen::Vector3d normal_1(delta_x, delta_y, delta_z);
            normal_1.normalize();
            // combine partial derivatives
            Eigen::Matrix<double, 1, 6> normal;
            normal << 0, 0, 0, normal_1.x(), normal_1.y(), normal_1.z(); // y p r x y z
            *H1 = normal;
        }

        gtsam::Vector1 error = gtsam::Vector1::Zero();
        double dis = height_ - sqrt(delta_x*delta_x + delta_y*delta_y + delta_z*delta_z);
        if (dis < 0)
            error[0] = -1 * dis * height_scale_;
        return error;
    }


    /* ************************************************************************* */
    void HeightCameraFactor::print(const std::string &s,
                                   const gtsam::KeyFormatter &keyFormatter) const {
        cout << s << "HeightCameraFactor(" << keyFormatter(key1()) << endl;

        cout << "    NoiseModel: ";
        noiseModel()->print();
        cout << endl;
    }

    /* ************************************************************************* */
    bool HeightCameraFactor::equals(const HeightCameraFactor &other,
                                    double tol) const {
        bool equal = height_ == other.height_;;
        return equal;
    }
} // namespace gtsam_quadrics
