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

#ifndef THEIA_MATCHING_SPASM_FEATURE_MATCHER_H_
#define THEIA_MATCHING_SPASM_FEATURE_MATCHER_H_

#include <vector>

#include "theia/matching/feature_matcher.h"
#include "theia/matching/features_and_matches_database.h"
#include "theia/util/util.h"

namespace theia {
struct FeatureMatcherOptions;
struct IndexedFeatureMatch;
struct KeypointsAndDescriptors;

// Performs features matching between two sets of features using a brute force
// matching method.
class SpasmFeatureMatcher : public FeatureMatcher {
 public:
  SpasmFeatureMatcher(const FeatureMatcherOptions& options,
                      FeaturesAndMatchesDatabase* features_and_matches_database)
                      : FeatureMatcher(options, features_and_matches_database) {}

  ~SpasmFeatureMatcher() {}

  void SetImagePairsToMatch(
      const std::vector<std::pair<std::string, std::string> >& pairs_to_match);

 private:
  bool MatchImagePair(
      const KeypointsAndDescriptors& features1,
      const KeypointsAndDescriptors& features2,
      std::vector<IndexedFeatureMatch>* matched_featuers) override;

  DISALLOW_COPY_AND_ASSIGN(SpasmFeatureMatcher);
};
}  // namespace theia

#endif  // THEIA_MATCHING_SPASM_FEATURE_MATCHER_H_
