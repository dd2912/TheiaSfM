// Copyright (C) 2014 The Regents of the University of California (Regents).
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials provided
//       with the distribution.
//
//     * Neither the name of The Regents or University of California nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Please contact the author of this library if you have any questions.
// Author: Chris Sweeney (cmsweeney@cs.ucsb.edu)

#include "theia/sfm/estimators/estimate_rotation_with_no_translation.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <limits>
#include <memory>
#include <vector>

#include "theia/matching/feature_correspondence.h"
#include "theia/sfm/create_and_initialize_ransac_variant.h"
#include "theia/sfm/pose/essential_matrix_utils.h"
#include "theia/sfm/pose/two_point_rotation_with_no_translation.h"
#include "theia/sfm/pose/util.h"
#include "theia/sfm/triangulation/triangulation.h"
#include "theia/solvers/estimator.h"
#include "theia/solvers/sample_consensus_estimator.h"
#include "theia/util/util.h"

namespace theia {
namespace {

using Eigen::Matrix3d;
using Eigen::Vector3d;

// An estimator for computing the relative pose from 5 feature
// correspondences. The feature correspondences should be normalized
// by the focal length with the principal point at (0, 0).
class EstimateOptimalRotation: public Estimator<FeatureCorrespondence, RelativePose> {
 public:
  EstimateOptimalRotation() {}

 private:
};

}  // namespace

bool EstimateOptimalRotation(const std::vector<FeatureCorrespondence>& normalized_correspondences, RelativePose* relative_pose) {

    Eigen::Matrix3Xd A(normalized_correspondences.size(), 3); // First set of 3D vectors
    Eigen::Matrix3Xd B(normalized_correspondences.size(), 3); // Second set of 3D vectors

    for(int i = 0; i < normalized_correspondences.size(); i++) {

        Eigen::Vector3d feature1(normalized_correspondences[i].feature1[0], normalized_correspondences[i].feature1[1], 1);
        Eigen::Vector3d feature2(normalized_correspondences[i].feature2[0], normalized_correspondences[i].feature2[1], 1);

        feature1.normalize();
        feature2.normalize();

        // Fill A and B with your 3D vector data

        A.col(i) = feature1;
        B.col(i) = feature2;

    }

    // Compute the covariance matrix
    Eigen::Matrix3d H = A * B.transpose();

    // Perform SVD on the covariance matrix
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);

    // Compute the optimal rotation matrix
    double d = (svd.matrixV() * (svd.matrixU().transpose())).determinant();

    Matrix3d I = Matrix3d::Identity();

    if (d > 0) {
        I(2, 2) = 1.0;
    } else {
        I(2, 2) = -1.0;
    }

    Eigen::Matrix3d R =  svd.matrixV() * I * svd.matrixU().transpose();

    relative_pose->rotation = R;

//    std::cout << "++++++++++++++++++" << std::endl;
//
////    std::cout << "relative_pose->rotation: " << typeid(relative_pose->rotation).name() << std::endl;
//    // Convert the rotation matrix to angle-axis representation
//    Eigen::AngleAxisd angle_axis_gt(relative_pose->rotation);
//    // Get the angle (in degrees) and axis
//    double angle_degrees_gt = angle_axis_gt.angle() * (180.0 / M_PI);
//    Eigen::Vector3d axis_gt = angle_axis_gt.axis();
//    // Output the rotation matrix
//    std::cout << "ground truth: " << axis_gt * angle_degrees_gt << std::endl;
//
////    std::cout << "estimation: " << typeid(R).name() << std::endl;
//    Eigen::AngleAxisd angle_axis_est(R);
//    // Get the angle (in degrees) and axis
//    double angle_degrees_est = angle_axis_est.angle() * (180.0 / M_PI);
//    Eigen::Vector3d axis_est = angle_axis_est.axis();
//    std::cout << "estimation: " << axis_est * angle_degrees_est << std::endl;
//
//    std::cout << "------------------" << std::endl;
//
//    exit(1);

    return 0;

}

}  // namespace theia
