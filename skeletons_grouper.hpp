#pragma once
#include "is/msgs/camera.pb.h"
#include "skeletons.pb.h"

using namespace is::vision;

class SkeletonsGrouper {
 public:
  SkeletonsGrouper(std::unordered_map<int64_t, CameraCalibration> calibrations, int64_t const& referencial,
                   double max_mean_d);

  Skeletons group(std::unordered_map<int64_t, Skeletons>& sks_2d);

 private:
  std::unordered_map<int64_t, CameraCalibration> calibrations;
  int64_t referencial;
  double max_mean_d;
};