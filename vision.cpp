#include "vision.hpp"

#include "boost/filesystem.hpp"
#include "boost/range.hpp"
#include "is/msgs/io.hpp"
#include "is/wire/core/logger.hpp"

boost::optional<is::vision::CameraCalibration> load_calib(std::string const& path) {
  boost::optional<is::vision::CameraCalibration> maybe_calibration;
  is::vision::CameraCalibration calibration;

  auto status = is::load(path, &calibration);
  if (status.code() == is::wire::StatusCode::OK) {
    maybe_calibration = calibration;
  } else {
    is::warn("Failed to load file: {}", status.why());
  }
  return maybe_calibration;
}

std::unordered_map<int64_t, is::vision::CameraCalibration> load_calibs(std::string const& basedir) {
  namespace fs = boost::filesystem;

  std::unordered_map<int64_t, is::vision::CameraCalibration> calibrations;
  if (!fs::is_directory(basedir)) {
    is::warn("Unable to load calibrations, path \"{}\" is not a directory", basedir);
    return calibrations;
  }

  for (auto& entry : boost::make_iterator_range(fs::directory_iterator(basedir), {})) {
    auto maybe_calibration = load_calib(entry.path().string());
    if (maybe_calibration) {
      calibrations[maybe_calibration->id()] = *maybe_calibration;
      is::info("[CameraCalibration] Loaded: {}", maybe_calibration->id());
    }
  }
  return calibrations;
}

std::unordered_map<int64_t, is::vision::CameraCalibration> load_calibs(std::string const& basedir,
                                                                       std::vector<int64_t> const& cameras) {
  namespace fs = boost::filesystem;

  std::unordered_map<int64_t, is::vision::CameraCalibration> calibrations;
  if (!fs::is_directory(basedir)) {
    is::warn("Unable to load calibrations, path \"{}\" is not a directory", basedir);
    return calibrations;
  }

  for (auto& camera : cameras) {
    auto entry = fs::path(basedir) / fs::path(fmt::format("{}.json", camera));
    auto maybe_calibration = load_calib(entry.string());
    if (maybe_calibration) {
      calibrations[maybe_calibration->id()] = *maybe_calibration;
      is::info("[CameraCalibration] Loaded: {}", maybe_calibration->id());
    }
  }
  return calibrations;
}

arma::mat arma_view(is::common::Tensor* tensor) {
  auto shape = tensor->shape();
  int rows = 0;
  int cols = 0;

  if (shape.dims_size() == 1) {
    rows = 1;
    cols = shape.dims(0).size();
  } else if (shape.dims_size() == 2) {
    rows = shape.dims(0).size();
    cols = shape.dims(1).size();
  } else
    return arma::mat();
  // arma library uses column major, then cols and rows were swapped and output matrix was transposed
  if (tensor->type() == is::common::DataType::DOUBLE_TYPE)
    return arma::dmat(tensor->mutable_doubles()->mutable_data(), cols, rows, false, true).t();
  return arma::mat();
}

arma::mat get_extrinsic(is::vision::CameraCalibration& calib, int64_t const& from) {
  auto camera = calib.id();
  auto begin = calib.extrinsic().begin();
  auto end = calib.extrinsic().end();
  auto predicate = [&](auto& ext) { return ext.from() == from && ext.to() == camera; };
  auto pos = std::find_if(begin, end, predicate);
  if (pos == end) {
    is::warn("Camera with ID {} doesn't have extrinsic with \'from\' equals {}", camera, from);
    return arma::mat();
  }
  auto index = std::distance(begin, pos);
  return arma_view(calib.mutable_extrinsic(index)->mutable_tf());
}

arma::mat skew(arma::vec const& vec) {
  assert(vec.size() == 3);
  return arma::mat({{0.0, -vec[2], vec[1]}, {vec[2], 0.0, -vec[0]}, {-vec[1], vec[0], 0.0}});
}

arma::mat fundamental_matrix(arma::mat const& K0, arma::mat const& RT0, arma::mat const& K1, arma::mat const& RT1) {
  arma::mat RT10 = RT1 * RT0.i();
  arma::mat R10 = RT10.submat(0, 0, 2, 2);
  arma::vec t10 = RT10.col(3).rows(0, 2);
  arma::mat E10 = skew(t10) * R10;
  arma::mat F10 = K1.i().t() * E10 * K0.i();
  return F10;
}

std::unordered_map<int64_t, std::unordered_map<int64_t, arma::mat>> compute_fundamentals_matrix(
    std::unordered_map<int64_t, is::vision::CameraCalibration>& calibrations, int64_t const& referencial) {
  std::unordered_map<int64_t, std::unordered_map<int64_t, arma::mat>> F;
  for (auto& kv0 : calibrations) {
    auto& cam0 = kv0.first;
    auto& calib0 = kv0.second;
    arma::mat K0 = arma_view(calib0.mutable_intrinsic());
    arma::mat RT0 = get_extrinsic(calib0, referencial);
    if (RT0.is_empty()) continue;
    for (auto& kv1 : calibrations) {
      auto& cam1 = kv1.first;
      auto& calib1 = kv1.second;
      arma::mat K1 = arma_view(calib1.mutable_intrinsic());
      arma::mat RT1 = get_extrinsic(calib1, referencial);
      if (RT1.is_empty()) continue;
      F[cam1][cam0] = fundamental_matrix(K0, RT0, K1, RT1);
    }
  }
  return F;
}

arma::mat epipolar_line(arma::mat const& points, arma::mat const& F, arma::mat const& sc0, arma::mat const& sc1) {
  arma::mat l = sc0.i() * F * sc1.i() * points;
  arma::rowvec k = arma::sqrt(arma::square(l.row(0)) + arma::square(l.row(1)));
  l.each_row() /= k;
  // check if it's necessery check these values
  if (l.has_inf()) throw std::runtime_error("@epipolar_line: lines matrix has infinity value");
  if (l.has_inf()) throw std::runtime_error("@epipolar_line: lines matrix has 'not a number' value");
  return l;
}

arma::mat epipolar_line(arma::mat const& points, arma::mat const& F) {
  return epipolar_line(points, F, arma::eye(3, 3), arma::eye(3, 3));
}

double mean_distance(arma::mat const& points, arma::mat const& lines, arma::urowvec const& parts) {
  return arma::mean(arma::vectorise(arma::abs(arma::sum(points.cols(parts) % lines.cols(parts), 0))));
}

arma::mat intrinsic_scale_matrix(is::vision::Resolution const& image_res, is::vision::Resolution const& camera_res) {
  auto sx = static_cast<double>(image_res.width()) / static_cast<double>(camera_res.width());
  auto sy = static_cast<double>(image_res.height()) / static_cast<double>(camera_res.height());
  arma::mat sc = {
      {sx, 0.0, 0.0}, {0.0, sy, 0.0}, {0.0, 0.0, 1.0},
  };
  return sc;
}