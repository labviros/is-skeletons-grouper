#pragma once

#include <armadillo>
#include <unordered_map>
#include <vector>
#include "is/msgs/camera.pb.h"
#include "boost/optional.hpp"

boost::optional<is::vision::CameraCalibration> load_calib(std::string const& path);

std::unordered_map<int64_t, is::vision::CameraCalibration> load_calibs(
    std::string const& basedir);

std::unordered_map<int64_t, is::vision::CameraCalibration> load_calibs(std::string const& basedir,
                                                                             std::vector<int64_t> const& cameras);