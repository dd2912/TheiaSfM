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
class RotationWithNoTranslationEstimator: public Estimator<FeatureCorrespondence, RelativePose> {
 public:
  RotationWithNoTranslationEstimator() {}

  // 2 correspondences are needed to determine a rotation matrix
  double SampleSize() const { return 2; }

  // Estimates candidate relative poses from correspondences.
  bool EstimateModel(const std::vector<FeatureCorrespondence>& correspondences,
                     std::vector<RelativePose>* relative_poses) const {

    std::vector<Eigen::Vector2d> image1_points, image2_points;
    image1_points.reserve(correspondences.size());
    image2_points.reserve(correspondences.size());
    for (int i = 0; i < correspondences.size(); i++) {
      image1_points.emplace_back(correspondences[i].feature1);
      image2_points.emplace_back(correspondences[i].feature2);
    }

    std::vector<Matrix3d> rotation_matrices;
    if (!TwoPointRotationWithNoTranslation(image1_points, image2_points, &rotation_matrices)) {
        return false;
    }

    relative_poses->reserve(rotation_matrices.size() * 4);
    for (const Eigen::Matrix3d& rotation_matrix : rotation_matrices) {
        RelativePose relative_pose;
        relative_pose.rotation = rotation_matrix;
//        Eigen::AngleAxisd angle_axis(rotation_matrix);
        relative_poses->push_back(relative_pose);
    }
    return relative_poses->size() > 0;
  }

  // The error for a correspondences given a model. This is the squared sampson
  // error.
  double Error(const FeatureCorrespondence& correspondence,
               const RelativePose& relative_pose) const {

//    std::cout << "ROTATION " << relative_pose.rotation << std::endl;
//    std::cout << "correspondence.feature1 " << correspondence.feature1 << std::endl;
//    std::cout << "correspondence.feature2 " << correspondence.feature2 << std::endl;


    Eigen::Vector3d feature1(correspondence.feature1[0], correspondence.feature1[1], 1);
    Eigen::Vector3d feature2(correspondence.feature2[0], correspondence.feature2[1], 1);

    feature1.normalize();
    feature2.normalize();

//    if (IsTriangulatedPointInFrontOfCameras(correspondence,
//                                            relative_pose.rotation,
//                                            relative_pose.position)) {
//      return SquaredSampsonDistance(relative_pose.rotation,
//                                    correspondence.feature1,
//                                    correspondence.feature2);
//    }

    return 1 - CosSimRotatedVectors(relative_pose.rotation, feature1, feature2);

//    std::cout << "cossim" << cossim << std::endl;


//    Eigen::Matrix3d identity_matrix;
//    identity_matrix.setIdentity();
//    cossim = CosSimRotatedVectors(identity_matrix, feature1, feature2);
//    std::cout << "cossim identity" << cossim << std::endl;
//    return std::numeric_limits<double>::max();
  }

 private:
  DISALLOW_COPY_AND_ASSIGN(RotationWithNoTranslationEstimator);
};

}  // namespace

bool EstimateRotationWithNoTranslation(
    const RansacParameters& ransac_params,
    const RansacType& ransac_type,
    const std::vector<FeatureCorrespondence>& normalized_correspondences,
    RelativePose* relative_pose,
    RansacSummary* ransac_summary) {

//    std::cout << "IN EstimateRotationWithNoTranslation " << std::endl;

  RotationWithNoTranslationEstimator rotation_with_no_translation_estimator;
  std::unique_ptr<SampleConsensusEstimator<RotationWithNoTranslationEstimator> > ransac =
      CreateAndInitializeRansacVariant(ransac_type,
                                       ransac_params,
                                       rotation_with_no_translation_estimator);
  // Estimate the relative pose.
  return ransac->Estimate(normalized_correspondences,
                          relative_pose,
                          ransac_summary);
}

}  // namespace theia
