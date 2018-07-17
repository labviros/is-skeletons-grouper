#include <chrono>
#include <regex>
#include "is/wire/core.hpp"
#include "is/msgs/utils.hpp"
#include "is/msgs/camera.pb.h"
#include "options.pb.h"
#include "skeletons.pb.h"
#include "google/protobuf/timestamp.pb.h"
#include "skeletons_grouper.hpp"

using namespace std::chrono;

auto get_id(std::string const& topic) {
  auto const id_regex = std::regex("Skeletons.(\\d+).Detections");
  std::smatch matches;
  if (std::regex_match(topic, matches, id_regex))
    return std::stoi(matches[1]);
  else
    throw std::runtime_error(fmt::format("Wrong fromat topic: \'{}\'", topic));
}

auto count_detections(std::unordered_map<int64_t, is::vision::Skeletons>& sks_group) {
  std::vector<int64_t> cameras;
  std::transform(sks_group.begin(), sks_group.end(), std::back_inserter(cameras), [](auto& kv) { return kv.first; });
  if (cameras.empty()) return std::string("");
  std::sort(cameras.begin(), cameras.end());
  auto formatter = [&](auto& c) { return fmt::format("{}[{}]", c, sks_group[c].skeletons().size()); };
  auto per_camera =
      std::accumulate(std::next(cameras.begin()), cameras.end(), formatter(cameras[0]), [&](std::string a, auto& c) {
        return a + ',' + formatter(c);
      });
  auto total = std::count_if(sks_group.begin(), sks_group.end(), [](auto& kv) { return kv.second.skeletons().size(); });
  return fmt::format("{} => {}", per_camera, total);
}

int main(int argc, char** argv) {
  std::string filename = (argc == 2) ? argv[1] : "options.json";
  is::SkeletonsGrouperOptions options;
  auto status = is::load(filename, &options);
  if (status.code() != is::wire::StatusCode::OK) is::critical("{}", status);

  auto channel = is::Channel(options.broker_uri());
  auto subscription = is::Subscription(channel);

  is::vision::GetCalibrationRequest calibs_request;
  *calibs_request.mutable_ids() = options.cameras_ids();
  auto request = is::Message(calibs_request);
  request.set_topic("FrameConversion.GetCalibration");
  request.set_reply_to(subscription);
  channel.publish(request);

  is::info("Waiting for 5 seconds to get calibrations");
  auto reply = channel.consume_for(seconds(5));
  if (!reply) is::critical("Failed to get cameras calibrations");
  auto maybe_calibrations = reply->unpack<is::vision::GetCalibrationReply>();
  if (!maybe_calibrations) is::critical("Invalid \'GetCalibrationReply\' reply");

  std::unordered_map<int64_t, is::vision::CameraCalibration> calibrations;
  for (auto& mc : maybe_calibrations->calibrations()) {
    calibrations[mc.id()] = mc;
  }

  SkeletonsGrouper grouper(calibrations, options.referencial(), options.min_error());
  for (auto& camera : options.cameras_ids()) {
    subscription.subscribe(fmt::format("Skeletons.{}.Detections", camera));
  }

  std::unordered_map<int64_t, is::vision::Skeletons> sks_group;
  for (;;) {
    auto message = channel.consume();
    auto skeletons = message.unpack<is::vision::Skeletons>();
    if (skeletons) {
      auto id = get_id(message.topic());
      sks_group[id] = *skeletons;

      auto t0 = system_clock::now();
      auto sks_3d = grouper.group(sks_group);
      auto tf = system_clock::now();

      auto sks_message = is::Message(sks_3d);
      sks_message.set_topic(fmt::format("Skeletons.{}.Detections", options.referencial()));
      channel.publish(sks_message);

      auto dt_ms = duration_cast<microseconds>(tf - t0).count() / 1000.0;
      auto n_skeletons = sks_3d.skeletons().size();
      auto detections_info = count_detections(sks_group);
      is::info("[{} 3D skeletons from {} detections][{:>6.2f}ms]", n_skeletons, detections_info, dt_ms);
    } else {
      is::warn("Can't unpack message");
    }
  }
  return 0;
}