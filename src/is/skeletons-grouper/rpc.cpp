#include "is/wire/core.hpp"
#include "is/wire/core/status.hpp"
#include "is/wire/rpc.hpp"
#include "is/wire/rpc/log-interceptor.hpp"
#include "is/msgs/image.pb.h"
#include "utils/skeletons-grouper.hpp"
#include "utils/utils.hpp"
#include "grouper_request.pb.h"

int main(int argc, char** argv) {
  auto options = load_options(argc, argv);

  auto begin = options.cameras().begin();
  auto end = options.cameras().end();
  std::vector<int64_t> cameras;
  std::transform(begin, end, std::back_inserter(cameras), [](auto& kv) { return kv.first; });
  std::sort(cameras.begin(), cameras.end());

  auto channel = is::Channel(options.broker_uri());
  auto subscription = is::Subscription(channel);
  auto provider = is::ServiceProvider(channel);
  provider.add_interceptor(is::LogInterceptor());
  auto tracer = make_tracer(options, "SkeletonsGrouper");
  channel.set_tracer(tracer);

  auto calibrations = request_calibrations(channel, subscription, cameras);
  update_extrinsics(channel, subscription, calibrations, options.referential());

  SkeletonsGrouper grouper(
      calibrations, options.referential(), options.min_error(), options.min_score(), options.max_distance());

  auto endpoint = "SkeletonsGrouper.Localize";
  provider.delegate<MultipleObjectAnnotations, ObjectAnnotations>(
      endpoint, [&](is::Context*, MultipleObjectAnnotations const& request, ObjectAnnotations* reply) {
        try {
          std::unordered_map<int64_t, is::vision::ObjectAnnotations> sks_group;
          for (auto& objects : request.list()) {
            sks_group[objects.frame_id()] = objects;
          }
          *reply = grouper.group(sks_group);
          return is::make_status(is::wire::StatusCode::OK);
        } catch (...) { return is::make_status(is::wire::StatusCode::INTERNAL_ERROR); }
      });

  provider.run();
}