#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <regex>
#include "boost/filesystem.hpp"
#include "is/wire/core/logger.hpp"
#include "skeletons.pb.h"
#include "skeletons_group.pb.h"
#include "skeletons_grouper.hpp"
#include "stream_pb.hpp"
#include "vision.hpp"

namespace fs = boost::filesystem;

auto const base_folder = "/home/felippe/panoptic-dataset-poses";
auto const dataset = "160422_haggling1";
auto const dataset_folder = fs::path(base_folder) / fs::path(dataset);
auto const calibrs_folder = dataset_folder / fs::path("calibrations");
auto const detections_file = dataset_folder / fs::path(fmt::format("coco_pose_2d{}", ""));
auto const images_folder = fs::path("/home/felippe/panoptic-images") / fs::path(dataset) / fs::path("hdImgs");
std::vector<int64_t> const cameras {1, 2, 4, 6, 7};

auto const id_regex = std::regex("\\d\\d[_]\\d(\\d)");
std::vector<int64_t> const sequence_ids{500, 1000, 1500};

template <class Container, class T>
bool find_id(Container c, T value) {
  return std::find(c.begin(), c.end(), value) != c.end();
}

std::unordered_map<int64_t, cv::Mat> load_images(std::string const& folder,
                                                 std::vector<int64_t> const& cameras,
                                                 int64_t sequence_id) {
  std::unordered_map<int64_t, cv::Mat> images;
  for (auto& camera : cameras) {
    auto filename = fs::path(fmt::format("00_0{}_{:08d}.jpg", camera, sequence_id));
    auto path = fs::path(folder) / fs::path(fmt::format("00_0{}", camera)) / filename;
    images[camera] = cv::imread(path.string(), cv::IMREAD_COLOR);
  }
  return images;
}

int main() {
  auto calibrations = load_calibs(calibrs_folder.string(), cameras);
  SkeletonsGrouper grouper(calibrations, 9999, 50.0);

  ProtobufReader reader(detections_file.string());  
  for (;;) {
    auto sk_group = reader.next<SkeletonsGroup>();
    if (!sk_group) break;

    if (!find_id(sequence_ids, sk_group->sequence_id())) continue;

    std::unordered_map<int64_t, is::vision::Skeletons> sks_2d;
    for (auto& key_value : sk_group->detections()) {
      std::smatch matches;
      int64_t camera;

      if (std::regex_match(key_value.first, matches, id_regex))
        camera = std::stoi(matches[1]);
      else
        continue;

      if (!find_id(cameras, camera)) continue;

      auto sks_id = key_value.second;
      for (auto& sk_id : sks_id.skeletons()) {
        *sks_2d[camera].add_skeletons() = sk_id.skeleton();
      }
      sks_2d[camera].set_model(sk_group->model());
    }

    auto images = load_images(images_folder.string(), cameras, sk_group->sequence_id());
    auto sks_3d = grouper.group(sks_2d, images);
  }

  return 0;
}