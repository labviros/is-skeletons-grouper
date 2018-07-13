#include <regex>
#include "boost/filesystem.hpp"
#include "is/wire/core/logger.hpp"
#include "skeletons.pb.h"
#include "skeletons_group.pb.h"
#include "stream_pb.hpp"
#include "vision.hpp"
#include "skeletons_grouper.hpp"

namespace fs = boost::filesystem;

auto const base_folder = "/home/felippe/panoptic-dataset-poses";
auto const dataset = "160422_haggling1";
auto const dataset_folder = fs::path(base_folder) / fs::path(dataset);
auto const calibrs_folder = dataset_folder / fs::path("calibrations");
auto const detections_file = dataset_folder / fs::path(fmt::format("coco_pose_2d{}", ""));
std::vector<int64_t> const cameras {1, 2, 4, 6, 7};

auto const id_regex = std::regex("\\d\\d[_]\\d(\\d)");
std::vector<int64_t> const sequence_ids {500, 1000, 1500};

template<class Container, class T>
bool find_id(Container c, T value) {
  return std::find(c.begin(), c.end(), value) != c.end();
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

      if (!find_id(cameras, camera))
        continue;
      
      auto sks_id = key_value.second;
      for (auto& sk_id : sks_id.skeletons()) {
        *sks_2d[camera].add_skeletons() = sk_id.skeleton();
      }
      sks_2d[camera].set_model(sk_group->model());
    }
    
    auto sks_3d = grouper.group(sks_2d);
  }

  return 0;
}