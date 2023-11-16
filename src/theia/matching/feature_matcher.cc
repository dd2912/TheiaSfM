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

#include "theia/matching/feature_matcher.h"

#include <glog/logging.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <fstream>
#include <chrono>
using namespace std::chrono;

#include "theia/image/keypoint_detector/keypoint.h"

#include "theia/matching/feature_correspondence.h"
#include "theia/matching/feature_matcher_options.h"
#include "theia/matching/features_and_matches_database.h"
#include "theia/matching/image_pair_match.h"
#include "theia/matching/keypoints_and_descriptors.h"
#include "theia/sfm/camera_intrinsics_prior.h"
#include "theia/sfm/two_view_match_geometric_verification.h"

#include "theia/util/map_util.h"
#include "theia/util/threadpool.h"
#include "theia/util/util.h"

namespace theia {
namespace {
void SelectAllPairs(
    const std::vector<std::string>& image_names,
    std::vector<std::pair<std::string, std::string>>* pairs_to_match) {
  // Compute the total number of potential matches.
  const int num_pairs_to_match =
      image_names.size() * (image_names.size() - 1) / 2;

  pairs_to_match->reserve(num_pairs_to_match);
  // Create a list of all possible image pairs.
  for (int i = 0; i < image_names.size(); i++) {
    for (int j = i + 1; j < image_names.size(); j++) {
      pairs_to_match->emplace_back(image_names[i], image_names[j]);
    }
  }
}
}  // namespace

FeatureMatcher::~FeatureMatcher() {}

FeatureMatcher::FeatureMatcher(
    const FeatureMatcherOptions& options,
    FeaturesAndMatchesDatabase* feature_and_matches_db)
    : options_(options), feature_and_matches_db_(feature_and_matches_db) {}

void FeatureMatcher::AddImage(const std::string& image_name) {
  image_names_.push_back(image_name);
}

void FeatureMatcher::AddImages(const std::vector<std::string>& image_names) {
  image_names_.reserve(image_names.size() + image_names_.size());

  for (int i = 0; i < image_names.size(); ++i) {
    AddImage(image_names[i]);
  }
}

void FeatureMatcher::SetImagePairsToMatch(const std::vector<std::pair<std::string, std::string>>& pairs_to_match) {

      pairs_to_match_ = pairs_to_match;

      /**** DEBUG ****/
//      for(auto &p : pairs_to_match) {
//        if(p.first == "0001.jpg"){
//            std::cout << p.first << " " << p.second << std::endl;
//        }
//        if(p.first == "0002.jpg"){
//            std::cout << p.first << " " << p.second << std::endl;
//         }
//      }
//
//      exit(1);
      /*****   *****/
}

void FeatureMatcher::MatchImages() {

    // If SetImagePairsToMatch has not been called, match all image-to-image
    // pairs.
    std::cout << "Start of MatchImages " << pairs_to_match_.size() << std::endl;
    if (pairs_to_match_.empty()) {
        std::cout << "PAIRS TO MATCH IS EMPTY" << std::endl;
        SelectAllPairs(image_names_, &pairs_to_match_);
    }

    // Add workers for matching. It is more efficient to let each thread compute
    // multiple matches at a time than add each matching task to the pool. This is
    // sort of like OpenMP's dynamic schedule in that it is able to balance
    // threads fairly efficiently.
    const int num_matches = pairs_to_match_.size();

    const int num_threads = std::min(options_.num_threads, static_cast<int>(num_matches));
    std::unique_ptr<ThreadPool> pool(new ThreadPool(num_threads));
    const int interval_step = std::min(this->kMaxThreadingStepSize_, num_matches / num_threads);
    for (int i = 0; i < num_matches; i += interval_step) {
        const int end_interval = std::min(num_matches, i + interval_step);
        pool->Add(&FeatureMatcher::MatchAndVerifyImagePairs, this, i, end_interval);
    }
    // Wait for all threads to finish.
    pool.reset(nullptr);

    VLOG(1) << "Matched " << feature_and_matches_db_->NumMatches()
          << " image pairs out of " << num_matches
          << " pairs selected for matching.";


    /****** DEBUG SAVE PAIRWISE *********/

//     std::cout << "options_.geometric_verification_options.estimate_twoview_info_options.geometry_verification " << options_.geometric_verification_options.estimate_twoview_info_options.geometry_verification << std::endl;

//     std::ofstream pairwise_results_file("pairwise_results_north1_1_" + options_.geometric_verification_options.estimate_twoview_info_options.geometry_verification + "_new.txt");
//     std::cout << "pairwise_results_" + options_.geometric_verification_options.estimate_twoview_info_options.geometry_verification + "_1000.txt" << std::endl;
//     if (!pairwise_results_file.is_open()) {
//             std::cerr << "Couldn't open the pairwise_results_file!" << std::endl;
//             exit(1);
//         }

//     for(auto pair_to_match : pairs_to_match_) {

// //        int image1_i = stoi(pair_to_match.first.substr(6).substr(0, pair_to_match.first.find_last_of(".")));
// //        int image1_i = stoi(pair_to_match.first.substr(0, pair_to_match.first.find_last_of(".")));
//         int image1_i = stoi(pair_to_match.first.substr(14).substr(0, pair_to_match.first.find_last_of(".")));
// //        int image2_i = stoi(pair_to_match.second.substr(6).substr(0, pair_to_match.second.find_last_of(".")));
// //        int image2_i = stoi(pair_to_match.second.substr(0, pair_to_match.second.find_last_of(".")));
//         int image2_i = stoi(pair_to_match.second.substr(14).substr(0, pair_to_match.second.find_last_of(".")));

//         Eigen::Vector3d rotation = feature_and_matches_db_->GetImagePairMatch(pair_to_match.first, pair_to_match.second).twoview_info.rotation_2;
//         Eigen::Vector3d position = feature_and_matches_db_->GetImagePairMatch(pair_to_match.first, pair_to_match.second).twoview_info.position_2;
//         std::cout << pair_to_match.first << " " << pair_to_match.second << std::endl;
//         std::cout << rotation << std::endl;
//         std::cout << "<<" << std::endl;
//         std::cout << feature_and_matches_db_->GetImagePairMatch(pair_to_match.first, pair_to_match.second).twoview_info.computation_time << std::endl;
//         std::cout << "<<" << std::endl;

//         pairwise_results_file << image1_i << " " << image2_i << " " << rotation[0] << " " << rotation[1] << " " << rotation[2] << " " << position[0] << " " << position[1] << " " << position[2] << " " << feature_and_matches_db_->GetImagePairMatch(pair_to_match.first, pair_to_match.second).twoview_info.computation_time << std::endl;
//     }

//     pairwise_results_file.close();


//     /****** END DEBUG SAVE PAIRWISE *********/

//     exit(1);
}

void FeatureMatcher::MatchAndVerifyImagePairs(const int start_index, const int end_index) {
  std::cout << "MatchAndVerifyImagePairs " << start_index << " " << end_index << std::endl;
  for (int i = start_index; i < end_index; i++) {
    const std::string image1_name = pairs_to_match_[i].first;
    const std::string image2_name = pairs_to_match_[i].second;

    // Match the image pair. If the pair fails to match then continue to the next match.
    ImagePairMatch image_pair_match;
    image_pair_match.image1 = image1_name;
    image_pair_match.image2 = image2_name;


    int image1_i = stoi(image1_name.substr(14).substr(0, image1_name.find_last_of(".")));
    int image2_i = stoi(image2_name.substr(14).substr(0, image2_name.find_last_of(".")));
    image_pair_match.twoview_info.distance_between_frames = abs(image1_i - image2_i);

    // Get the keypoints and descriptors from the db.
    const KeypointsAndDescriptors& features1 = feature_and_matches_db_->GetFeatures(image1_name);
    const KeypointsAndDescriptors& features2 = feature_and_matches_db_->GetFeatures(image2_name);

    // Compute the visual matches from feature descriptors.
    std::vector<IndexedFeatureMatch> putative_matches;
    if (!MatchImagePair(features1, features2, &putative_matches)) {
        std::cout << "Could not match a sufficient number of features between images " << image1_name << " and " << image2_name << std::endl;
        continue;
    }


    auto time_start = high_resolution_clock::now();

    // Perform geometric verification if applicable.
    if (options_.perform_geometric_verification) {
      // If geometric verification fails, do not add the match to the output.
      if (!GeometricVerification(features1, features2, putative_matches, &image_pair_match)) {
        VLOG(1) << "Geometric verification between images " << image1_name << " and " << image2_name << " failed.";
        continue;
      }
    } else {
      // If no geometric verification is performed then the putative matches are
      // output.
      image_pair_match.correspondences.reserve(putative_matches.size());
      for (int i = 0; i < putative_matches.size(); i++) {
        const Keypoint& keypoint1 = features1.keypoints[putative_matches[i].feature1_ind];
        const Keypoint& keypoint2 = features2.keypoints[putative_matches[i].feature2_ind];
        image_pair_match.correspondences.emplace_back(
            Feature(keypoint1.x(), keypoint1.y()),
            Feature(keypoint2.x(), keypoint2.y()));
      }
    }

    // Log information about the matching results.
    std::cout << "Images " << image1_name << " and " << image2_name
            << " were matched withwere matched with " << image_pair_match.correspondences.size()
            << " verified matches and "
            << image_pair_match.twoview_info.num_homography_inliers
            << " homography matches out of " << putative_matches.size()
            << " putative matches." << std::endl;


//     std::cout << image1_name << " " << image2_name << std::endl;
//     std::cout << feature_and_matches_db_->GetImagePairMatch(image1_name, image2_name).twoview_info.distance_between_frames << std::endl;

    auto time_stop = high_resolution_clock::now();
    std::chrono::duration<double, std::milli> computation_time = time_stop - time_start;
    std::cout << computation_time.count() << std::endl;
    image_pair_match.twoview_info.distance_between_frames = abs(image1_i - image2_i);
    image_pair_match.twoview_info.computation_time = computation_time.count();


    // This operation is thread safe.
    feature_and_matches_db_->PutImagePairMatch(
        image1_name, image2_name, image_pair_match);

  }

}

bool FeatureMatcher::GeometricVerification(
    const KeypointsAndDescriptors& features1,
    const KeypointsAndDescriptors& features2,
    const std::vector<IndexedFeatureMatch>& putative_matches,
    ImagePairMatch* image_pair_match) {

    CameraIntrinsicsPrior intrinsics1, intrinsics2;

    // Load camera intrinsics if they are available.
    if (feature_and_matches_db_->ContainsCameraIntrinsicsPrior(features1.image_name)) {
        intrinsics1 = feature_and_matches_db_->GetCameraIntrinsicsPrior(features1.image_name);
    }

    if (feature_and_matches_db_->ContainsCameraIntrinsicsPrior(features2.image_name)) {
        intrinsics2 = feature_and_matches_db_->GetCameraIntrinsicsPrior(features2.image_name);
    }

    //  std::cout << "options_.geometric_verification_options " << options_.geometric_verification_options << std::endl;
    //  std::cout << "options_.geometric_verification_options.geometry_verification " << options_.geometric_verification_options.geometry_verification << std::endl;

    TwoViewMatchGeometricVerification geometric_verification(
        options_.geometric_verification_options,
        intrinsics1,
        intrinsics2,
        features1,
        features2,
        putative_matches);



    // Return whether geometric verification succeeds.
    return geometric_verification.VerifyMatches(&image_pair_match->correspondences, &image_pair_match->twoview_info);

}

}  // namespace theia
