#include "skeletons_grouper.hpp"

SkeletonsGrouper::SkeletonsGrouper(std::unordered_map<int64_t, CameraCalibration> calibrations,
                                   int64_t const& referencial, double max_mean_d = 50.0)
    : calibrations(calibrations), referencial(referencial), max_mean_d(max_mean_d) {
}

Skeletons SkeletonsGrouper::group(std::unordered_map<int64_t, Skeletons>& sks_2d) {
  Skeletons skeletons;
  return skeletons;
}