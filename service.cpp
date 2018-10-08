#include <chrono>
#include "is/wire/core.hpp"
#include "is/wire/core/status.hpp"
#include "is/wire/rpc.hpp"
#include "is/wire/rpc/log-interceptor.hpp"
#include "is/msgs/image.pb.h"
#include "google/protobuf/timestamp.pb.h"
#include "google/protobuf/struct.pb.h"
#include "google/protobuf/empty.pb.h"
#include "utils/skeletons_grouper.hpp"
#include "utils/utils.hpp"

using namespace std::chrono;

int main(int argc, char** argv) {
  auto options = load_options(argc, argv);

  std::vector<int64_t> cameras;
  std::transform(options.cameras().begin(), options.cameras().end(), std::back_inserter(cameras), [](auto& kv) {
    return kv.first;
  });

  auto channel = is::Channel(options.broker_uri());
  auto provider = is::ServiceProvider(channel);
  provider.add_interceptor(is::LogInterceptor());
  auto subscription = is::Subscription(channel);
  auto tracer = make_tracer(options, "Skeletons.Localization");
  channel.set_tracer(tracer);

  auto calibrations = request_calibrations(channel, subscription, cameras);
  update_extrinsics(channel, subscription, calibrations, options.referential());
  SkeletonsGrouper grouper(
      calibrations, options.referential(), options.min_error(), options.min_score(), options.max_distance());

  auto cams = std::accumulate(
      std::next(cameras.begin()), cameras.end(), std::to_string(*cameras.begin()), [](auto& a, auto& b) {
        return fmt::format("{}.{}", a, b);
      });
  auto endpoint = fmt::format("Skeletons.Localization.{}.Config", cams);
  provider.delegate<google::protobuf::Struct, google::protobuf::Empty>(
      endpoint, [&](is::Context*, google::protobuf::Struct const& request, google::protobuf::Empty*) {
        auto fields = request.fields();
        if (fields.find("max_error") != fields.end()) {
          auto value = static_cast<double>(fields.at("max_error").number_value());
          grouper.set_max_error(value);
        }
        if (fields.find("min_score") != fields.end()) {
          auto value = static_cast<double>(fields.at("min_score").number_value());
          if (value < 0.0 || value > 1.0) {
            return is::make_status(is::wire::StatusCode::OUT_OF_RANGE,
                                   "\'min_score\' must be greater or equal 0.0 and less and equal 1.0");
          }
          grouper.set_max_error(value);
        }
        return is::make_status(is::wire::StatusCode::OK);
      });

  std::unordered_map<int64_t, int64_t> not_received;
  for (auto& camera : cameras) {
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
      if (provider.serve(*message)) continue;

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

    auto span_name = "localization";
    auto span = has_ctx ? tracer->StartSpan(span_name, {ChildOf(last_ctx.get())}) : tracer->StartSpan(span_name);
    auto t0 = system_clock::now();
    filter_by_region(sks_group, options.cameras());
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