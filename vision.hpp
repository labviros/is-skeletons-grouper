#pragma once

#include <armadillo>
#include <unordered_map>
#include <vector>
#include "boost/optional.hpp"
#include "is/msgs/camera.pb.h"
#include "is/msgs/common.pb.h"

boost::optional<is::vision::CameraCalibration> load_calib(std::string const& path);

std::unordered_map<int64_t, is::vision::CameraCalibration> load_calibs(std::string const& basedir);

std::unordered_map<int64_t, is::vision::CameraCalibration> load_calibs(std::string const& basedir,
                                                                       std::vector<int64_t> const& cameras);

arma::mat arma_view(is::common::Tensor* tensor);

arma::mat get_extrinsic(is::vision::CameraCalibration& calib, int64_t const& from);

arma::mat skew(arma::vec const& vec);

arma::mat fundamental_matrix(arma::mat const& K0, arma::mat const& RT0, arma::mat const& K1, arma::mat const& RT1);

std::unordered_map<int64_t, std::unordered_map<int64_t, arma::mat>> compute_fundamentals_matrix(
    std::unordered_map<int64_t, is::vision::CameraCalibration>& calibrations, int64_t const& referencial);

arma::mat epipolar_line(arma::mat const& points, arma::mat const& F);