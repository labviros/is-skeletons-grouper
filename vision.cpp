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
    if (maybe_calibration) { calibrations[maybe_calibration->id()] = *maybe_calibration; }
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
    if (maybe_calibration) { calibrations[maybe_calibration->id()] = *maybe_calibration; }
  }

  return calibrations;
}