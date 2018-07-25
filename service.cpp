#include <chrono>
#include <regex>
#include "is/wire/core.hpp"
#include "is/msgs/utils.hpp"
#include "is/msgs/camera.pb.h"
#include "is/msgs/image.pb.h"
#include "options.pb.h"
#include "google/protobuf/timestamp.pb.h"
#include "skeletons_grouper.hpp"
#include "zipkin/opentracing.h"

using namespace std::chrono;
using namespace zipkin;
using namespace opentracing;

auto get_id(std::string const& topic) {
  auto const id_regex = std::regex("Skeletons.(\\d+).Detections");
  std::smatch matches;
  if (std::regex_match(topic, matches, id_regex))
    return std::stoi(matches[1]);
  else
    throw std::runtime_error(fmt::format("Wrong fromat topic: \'{}\'", topic));
}

auto count_detections(std::unordered_map<int64_t, is::vision::ObjectAnnotations>& sks) {
  std::vector<int64_t> cameras;
  std::transform(sks.begin(), sks.end(), std::back_inserter(cameras), [](auto& kv) { return kv.first; });
  if (cameras.empty()) return std::string("");
  std::sort(cameras.begin(), cameras.end());
  auto formatter = [&](auto& c) { return fmt::format("{}({})", c, sks[c].objects().size()); };
  auto per_camera =
      std::accumulate(std::next(cameras.begin()), cameras.end(), formatter(cameras[0]), [&](std::string a, auto& c) {
        return a + ',' + formatter(c);
      });
  auto total = std::accumulate(sks.begin(), sks.end(), 0, [](auto& a, auto& kv) { return a + kv.second.objects().size(); });
  return fmt::format("{} => {}", per_camera, total);
}

int main(int argc, char** argv) {
  std::string filename = (argc == 2) ? argv[1] : "options.json";
  is::SkeletonsGrouperOptions options;
  auto status = is::load(filename, &options);
  if (status.code() != is::wire::StatusCode::OK) is::critical("{}", status);

  auto channel = is::Channel(options.broker_uri());
  auto subscription = is::Subscription(channel);

  ZipkinOtTracerOptions zp_options;
  zp_options.service_name = "skeletons_grouper";
  zp_options.collector_host = options.zipkin_host();
  zp_options.collector_port = options.zipkin_port();
  auto tracer = makeZipkinOtTracer(zp_options);
  channel.set_tracer(tracer);
  auto span_name = fmt::format("group_{}", options.referencial());

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

  std::unordered_map<int64_t, is::vision::ObjectAnnotations> sks_group;
  std::unique_ptr<opentracing::SpanContext> last_ctx;
  auto period_ms = milliseconds(options.period_ms());
  auto deadline = system_clock::now() + period_ms;
  for (;;) {
    bool has_ctx = false;
    while (true) {
      auto message = channel.consume_until(deadline);
      if (!message) break;

      auto skeletons = message->unpack<is::vision::ObjectAnnotations>();
      if (skeletons) {
        sks_group[get_id(message->topic())] = *skeletons;
        auto maybe_ctx = message->extract_tracing(tracer);
        if (maybe_ctx) {
          has_ctx = true;
          last_ctx = std::move(maybe_ctx.value());
        }
      } else {
        is::warn("Can't unpack message from \'{}\'", message->topic());
      }
    }

    auto span = has_ctx ? tracer->StartSpan(span_name, {ChildOf(last_ctx.get())}) : tracer->StartSpan(span_name);
    auto t0 = system_clock::now();
    auto sks_3d = grouper.group(sks_group);
    auto tf = system_clock::now();

    auto sks_message = is::Message(sks_3d);
    sks_message.set_topic(fmt::format("Skeletons.{}.Detections", options.referencial()));
    sks_message.inject_tracing(tracer, span->context());
    channel.publish(sks_message);
    
    auto n_skeletons = sks_3d.objects().size();
    span->SetTag("Detections", n_skeletons);
    span->Finish();

    auto dt_ms = duration_cast<microseconds>(tf - t0).count() / 1000.0;
    auto detections_info = count_detections(sks_group);
    is::info("[{} (3D)][{} (2D)][{:>4.2f}ms]", n_skeletons, detections_info, dt_ms);
    deadline += period_ms;
  }
  return 0;
}