#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <regex>
#include "boost/filesystem.hpp"
#include "is/msgs/image.pb.h"
#include "is/wire/core/logger.hpp"
#include "skeletons.pb.h"
#include "skeletons_group.pb.h"
#include "skeletons_grouper.hpp"
#include "stream_pb.hpp"
#include "vision.hpp"

namespace fs = boost::filesystem;
using namespace is::vision;

auto const base_folder = "/home/felippe/panoptic-dataset-poses";
auto const dataset = "160422_haggling1";
auto const dataset_folder = fs::path(base_folder) / fs::path(dataset);
auto const calibrs_folder = dataset_folder / fs::path("calibrations");
auto const detections_file = dataset_folder / fs::path(fmt::format("coco_pose_2d{}", ""));
auto const images_folder = fs::path("/home/felippe/panoptic-images") / fs::path(dataset) / fs::path("hdImgs");
std::vector<int64_t> const cameras{1, 2, 4, 6, 7};
std::vector<int64_t> const scaled_cameras{2, 7};
float scale_x = 0.5;
float scale_y = 0.75;

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

void scale_images(std::unordered_map<int64_t, cv::Mat>& images,
                  std::vector<int64_t> const& scaled_cameras,
                  float scale_x,
                  float scale_y) {
  for (auto& cam : scaled_cameras) {
    auto pos = images.find(cam);
    if (pos == images.end()) continue;
    cv::resize(images[cam], images[cam], cv::Size(0, 0), scale_x, scale_y);
  }
}

ObjectAnnotations to_object_annotations(sks::Skeletons const& skeletons,
                                        int64_t width,
                                        int64_t height,
                                        float scale_x = 1.0,
                                        float scale_y = 1.0) {
  const auto* descriptor = HumanKeypoints_descriptor();
  ObjectAnnotations objs;
  for (auto& skeleton : skeletons.skeletons()) {
    auto obj = objs.add_objects();
    for (auto& part : skeleton.parts()) {
      auto type = part.type();
      if (type == sks::SkeletonPartType::BACKGROUND) continue;
      auto keypoint = obj->add_keypoints();
      auto type_str = SkeletonPartType_Name(type);
      auto id = descriptor->FindValueByName(type_str)->number();
      keypoint->set_id(id);
      keypoint->set_score(part.score());
      auto position = keypoint->mutable_position();
      position->set_x(scale_x * part.x());
      position->set_y(scale_y * part.y());
    }
    obj->set_id(ObjectLabels::HUMAN_SKELETON);
    obj->set_label("human_skeleton");
  }
  auto resolution = objs.mutable_resolution();
  resolution->set_width(static_cast<uint32_t>(scale_x * width));
  resolution->set_height(static_cast<uint32_t>(scale_y * height));
  return objs;
}

int main() {
  auto calibrations = load_calibs(calibrs_folder.string(), cameras);
  SkeletonsGrouper grouper(calibrations, 9999, 50.0);

  ProtobufReader reader(detections_file.string());
  for (;;) {
    auto sk_group = reader.next<SkeletonsGroup>();
    if (!sk_group) break;

    if (!find_id(sequence_ids, sk_group->sequence_id())) continue;

    std::unordered_map<int64_t, ObjectAnnotations> sks_2d;
    for (auto& key_value : sk_group->detections()) {
      std::smatch matches;
      int64_t camera;

      if (std::regex_match(key_value.first, matches, id_regex))
        camera = std::stoi(matches[1]);
      else
        continue;

      if (!find_id(cameras, camera)) continue;

      auto sks_id = key_value.second;
      sks::Skeletons sks;
      for (auto& sk_id : sks_id.skeletons()) {
        *sks.add_skeletons() = sk_id.skeleton();
      }
      auto sc_x = find_id(scaled_cameras, camera) ? scale_x : 1.0;
      auto sc_y = find_id(scaled_cameras, camera) ? scale_y : 1.0;
      sks_2d[camera] = to_object_annotations(sks, 1920, 1080, sc_x, sc_y);
    }
    auto images = load_images(images_folder.string(), cameras, sk_group->sequence_id());
    scale_images(images, scaled_cameras, scale_x, scale_y);
    auto sks_3d = grouper.group(sks_2d, images);
    sks_3d.PrintDebugString();
    std::cin.ignore();
  }

  return 0;
}