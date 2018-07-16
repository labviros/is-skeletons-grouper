#include <chrono>
#include <regex>
#include "boost/filesystem.hpp"
#include "is/wire/core/logger.hpp"
#include "skeletons.pb.h"
#include "skeletons_group.pb.h"
#include "skeletons_grouper.hpp"
#include "stream_pb.hpp"
#include "vision.hpp"

namespace fs = boost::filesystem;
using namespace std::chrono;

auto const base_folder = fs::path("/home/felippe/panoptic-dataset-poses");
auto const id_regex = std::regex("\\d\\d[_]\\d(\\d)");
// std::vector<int64_t> const cameras{0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

template <class Container, class T>
bool find_id(Container c, T value) {
  return std::find(c.begin(), c.end(), value) != c.end();
}

int main(int argc, char** argv) {
  if (argc < 4 || argc > 5)
    is::critical("Enter a dataset name: ./group_panoptic <MODEL[mpii/coco]> <DATASET_NAME> <REFERENCIAL> <SUFIX>");
  auto const model = std::string(argv[1]);
  auto const dataset = std::string(argv[2]);
  auto const referencial = std::stoi(argv[3]);
  auto const sufix = argc == 4 ? "" : std::string(argv[4]);
  auto const dataset_folder = base_folder / fs::path(dataset);
  auto const calibrs_folder = dataset_folder / fs::path("calibrations");

  auto calibrations = load_calibs(calibrs_folder.string());
  SkeletonsGrouper grouper(calibrations, referencial, 50.0);

  auto const detections_file = dataset_folder / fs::path(fmt::format("{}_pose_2d{}", model, sufix));
  ProtobufReader reader(detections_file.string());
  auto const output_file = dataset_folder / fs::path(fmt::format("{}_pose_3d{}_grouped", model, sufix));
  ProtobufWriter writer(output_file.string());
  auto const time_output_file = dataset_folder / fs::path(fmt::format("{}_duration{}_grouped", model, sufix));
  ProtobufWriter time_writer(time_output_file.string());

  is::info("[IN][Detections] {}", detections_file.string());
  is::info("[OUT][Detections] {}", output_file.string());
  is::info("[OUT][Durations] {}", time_output_file.string());
  
  for (;;) {
    auto sk_group = reader.next<SkeletonsGroup>();
    if (!sk_group) break;

    std::unordered_map<int64_t, is::vision::Skeletons> sks_2d;
    for (auto& key_value : sk_group->detections()) {
      std::smatch matches;
      int64_t camera;

      if (std::regex_match(key_value.first, matches, id_regex)) camera =
          std::stoi(matches[1]);
      else if (dataset == "hpn-001")
        camera = std::stoi(key_value.first);
      else
        continue;

      // if (!find_id(cameras, camera)) continue;

      auto sks_id = key_value.second;
      for (auto& sk_id : sks_id.skeletons()) {
        *sks_2d[camera].add_skeletons() = sk_id.skeleton();
      }
      sks_2d[camera].set_model(sk_group->model());
    }

    auto t0 = system_clock::now();
    auto sks_3d = grouper.group(sks_2d);
    auto tf = system_clock::now();
    auto dt_ms = duration_cast<microseconds>(tf - t0).count() / 1000.0;

    if (sk_group->sequence_id() % 100 == 0) {
      is::info("[{:>5d}] took {:.2f}ms and detect {} skeletons", sk_group->sequence_id(), dt_ms, sks_3d.skeletons().size()); 
    }

    writer.insert(sks_3d);
    time_writer.insert(is::to_duration(tf - t0));
  }

  return 0;
}