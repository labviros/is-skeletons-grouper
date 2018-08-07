#include <chrono>
#include <regex>
#include "is/wire/core.hpp"
#include "is/msgs/utils.hpp"
#include "is/msgs/camera.pb.h"
#include "is/msgs/image.pb.h"
#include "is/msgs/validate.hpp"
#include "options.pb.h"
#include "google/protobuf/timestamp.pb.h"
#include "skeletons_grouper.hpp"
#include "zipkin/opentracing.h"

using namespace std::chrono;
using namespace zipkin;
using namespace opentracing;

auto get_id(std::string const& topic) {
  auto const id_regex = std::regex("Skeletons.(\\d+).Detection");
  std::smatch matches;
  if (std::regex_match(topic, matches, id_regex))
    return std::stoi(matches[1]);
  else
    throw std::runtime_error(fmt::format("Wrong fromat topic: \'{}\'", topic));
}

auto make_frame_transformation(std::string const& topic, is::common::Tensor const& tf) {
  boost::optional<is::vision::FrameTransformation> maybe_transformation;
  auto const id_regex = std::regex("FrameTransformation.(\\d+).(\\d+)");
  std::smatch matches;
  if (std::regex_match(topic, matches, id_regex)) {
    if (matches.size() == 3) {
      is::vision::FrameTransformation transformation;
      transformation.set_from(std::stoi(matches[1]));
      transformation.set_to(std::stoi(matches[2]));
      *transformation.mutable_tf() = tf;
      maybe_transformation = transformation;
    }
  }
  return maybe_transformation;
}

auto get_cameras(std::unordered_map<int64_t, is::vision::ObjectAnnotations>& sks) {
  std::vector<int64_t> cameras;
  std::transform(sks.begin(), sks.end(), std::back_inserter(cameras), [](auto& kv) { return kv.first; });
  std::sort(cameras.begin(), cameras.end());
  return cameras;
}

auto count_detections(std::unordered_map<int64_t, is::vision::ObjectAnnotations>& sks) {
  auto cameras = get_cameras(sks);
  if (cameras.empty()) return std::string("");
  auto formatter = [&](auto& c) { return fmt::format("{}({})", c, sks[c].objects().size()); };
  auto per_camera =
      std::accumulate(std::next(cameras.begin()), cameras.end(), formatter(cameras[0]), [&](std::string a, auto& c) {
        return a + ',' + formatter(c);
      });
  auto total =
      std::accumulate(sks.begin(), sks.end(), 0, [](auto& a, auto& kv) { return a + kv.second.objects().size(); });
  return fmt::format("{} => {}", per_camera, total);
}

int main(int argc, char** argv) {
  std::string filename = (argc == 2) ? argv[1] : "options.json";
  is::SkeletonsGrouperOptions options;
  auto status = is::load(filename, &options);
  if (status.code() != is::wire::StatusCode::OK) is::critical("{}", status);
  auto validate_status = is::validate_message(options);
  if (validate_status.code() != is::wire::StatusCode::OK) is::critical("{}", validate_status);

  auto channel = is::Channel(options.broker_uri());
  auto subscription = is::Subscription(channel);

  ZipkinOtTracerOptions zp_options;
  zp_options.service_name = "Skeletons.Localization";
  zp_options.collector_host = options.zipkin_host();
  zp_options.collector_port = options.zipkin_port();
  auto tracer = makeZipkinOtTracer(zp_options);
  channel.set_tracer(tracer);
  auto span_name = "localization";

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
  // check for referential on calibrations
  std::vector<int64_t> without_ref;
  for (auto& kv : calibrations) {
    auto begin = kv.second.extrinsic().begin();
    auto end = kv.second.extrinsic().end();
    auto pos = std::find_if(begin, end, [&](auto& ext) { return ext.from() == options.referential(); });
    if (pos == end) {
      auto camera = kv.first;
      without_ref.push_back(camera);
      auto tf_topic = fmt::format("FrameTransformation.{}.{}", options.referential(), camera);
      subscription.subscribe(tf_topic);
    }
  }

  is::info("Waiting for 5 seconds to necessary transformations");
  auto consume_deadline = system_clock::now() + seconds(5);
  while (!without_ref.empty()) {
    auto msg = channel.consume_until(consume_deadline);
    if (!msg) break;
    auto maybe_tensor = msg->unpack<is::common::Tensor>();
    if (!maybe_tensor) continue;
    auto maybe_transformation = make_frame_transformation(msg->topic(), *maybe_tensor);
    if (!maybe_transformation) continue;
    auto camera = maybe_transformation->to();
    *calibrations[camera].add_extrinsic() = *maybe_transformation;
    without_ref.erase(std::remove(without_ref.begin(), without_ref.end(), camera), without_ref.end());
    subscription.unsubscribe(msg->topic());
  }
  if (!without_ref.empty()) { is::critical("Can't get all necessary transformations."); }

  SkeletonsGrouper grouper(calibrations, options.referential(), options.min_error(), options.min_score());
  std::unordered_map<int64_t, int64_t> not_received;
  for (auto& camera : options.cameras_ids()) {
    subscription.subscribe(fmt::format("Skeletons.{}.Detection", camera));
    not_received[camera] = 0;
  }

  std::unordered_map<int64_t, is::vision::ObjectAnnotations> sks_group;
  std::unique_ptr<opentracing::SpanContext> last_ctx;
  auto period_ms = milliseconds(options.period_ms());
  auto deadline = system_clock::now() + period_ms;
  for (;;) {
    bool has_ctx = false;
    std::for_each(not_received.begin(), not_received.end(), [](auto& kv) { kv.second++; });

    while (true) {
      auto message = channel.consume_until(deadline);
      if (!message) break;

      auto skeletons = message->unpack<is::vision::ObjectAnnotations>();
      if (skeletons) {
        auto camera = get_id(message->topic());
        sks_group[camera] = *skeletons;
        not_received[camera] = 0;
        auto maybe_ctx = message->extract_tracing(tracer);
        if (maybe_ctx) {
          has_ctx = true;
          last_ctx = std::move(maybe_ctx.value());
        }
      } else {
        is::warn("Can't unpack message from \'{}\'", message->topic());
      }
    }

    for (auto& kv : not_received) {
      if (kv.second > options.release_samples()) {
        auto camera = kv.first;
        auto pos = sks_group.find(camera);
        if (pos != sks_group.end()) {
          if (sks_group.erase(pos->first) > 0) {
            auto dt = options.release_samples() * options.period_ms();
            is::warn("Didn't received any detections from camera \'{}\' in the last {}ms", camera, dt);
          }
        }
      }
    }

    auto span = has_ctx ? tracer->StartSpan(span_name, {ChildOf(last_ctx.get())}) : tracer->StartSpan(span_name);
    auto t0 = system_clock::now();
    auto sks_3d = grouper.group(sks_group);
    auto tf = system_clock::now();

    auto sks_message = is::Message(sks_3d);
    sks_message.set_topic("Skeletons.Localization");
    sks_message.inject_tracing(tracer, span->context());
    channel.publish(sks_message);

    auto n_skeletons = sks_3d.objects().size();
    auto detections_info = count_detections(sks_group);
    span->SetTag("Localizations", n_skeletons);
    span->SetTag("Camera(Detection)", detections_info);
    span->Finish();

    auto dt_ms = duration_cast<microseconds>(tf - t0).count() / 1000.0;
    is::info("[(2D) {}][(3D) {:2d}][{:>4.2f}ms]", detections_info, n_skeletons, dt_ms);
    deadline += period_ms;
  }
  return 0;
}