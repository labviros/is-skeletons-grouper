#include "utils.hpp"

using namespace std::chrono;

void filter_by_region(std::unordered_map<int64_t, is::vision::ObjectAnnotations>& sks_group,
                      google::protobuf::Map<int64_t, is::vision::BoundingPoly> configs) {
  for (auto& kv : sks_group) {
    auto camera = kv.first;
    if (configs.find(camera) == configs.end()) continue;
    auto config = configs.at(camera);
    if (config.vertices().size() != 2) continue;
    auto w = kv.second.resolution().width();
    auto h = kv.second.resolution().height();
    auto min_w = config.vertices(0).x() * w;
    auto max_w = config.vertices(1).x() * w;
    auto min_h = config.vertices(0).y() * h;
    auto max_h = config.vertices(1).y() * h;
    auto objs = kv.second.mutable_objects();
    auto pos = std::remove_if(objs->begin(), objs->end(), [&](auto& obj) {
      auto any = [&](auto predicate) { return std::any_of(obj.keypoints().begin(), obj.keypoints().end(), predicate); };
      if (any([&](auto& kp) { return kp.position().x() < min_w; })) return true;
      if (any([&](auto& kp) { return kp.position().x() > max_w; })) return true;
      if (any([&](auto& kp) { return kp.position().y() < min_h; })) return true;
      if (any([&](auto& kp) { return kp.position().y() > max_h; })) return true;
      return false;
    });
    objs->erase(pos, objs->end());
  }
}

int64_t get_id(std::string const& topic) {
  auto const id_regex = std::regex("SkeletonsDetector.(\\d+).Detection");
  std::smatch matches;
  if (std::regex_match(topic, matches, id_regex))
    return std::stoi(matches[1]);
  else
    throw std::runtime_error(fmt::format("Wrong format topic: \'{}\'", topic));
}

std::vector<int64_t> get_cameras(std::unordered_map<int64_t, is::vision::ObjectAnnotations>& sks) {
  std::vector<int64_t> cameras;
  std::transform(sks.begin(), sks.end(), std::back_inserter(cameras), [](auto& kv) { return kv.first; });
  std::sort(cameras.begin(), cameras.end());
  return cameras;
}

std::string detections_info(std::unordered_map<int64_t, is::vision::ObjectAnnotations>& sks) {
  auto cameras = get_cameras(sks);
  if (cameras.empty()) return std::string("");
  auto formatter = [&](auto& c) { return fmt::format(" \'{}\': {}", c, sks[c].objects().size()); };
  auto begin = cameras.begin();
  auto end = cameras.end();
  auto second = std::next(begin);
  return std::accumulate(second, end, formatter(*begin), [&](auto& a, auto& c) { return a + ',' + formatter(c); });
}

std::unordered_map<int64_t, is::vision::CameraCalibration> request_calibrations(is::Channel& channel,
                                                                                is::Subscription& subscription,
                                                                                std::vector<int64_t>& cameras) {
  is::vision::GetCalibrationRequest calibs_request;
  *calibs_request.mutable_ids() = {cameras.begin(), cameras.end()};
  auto request = is::Message(calibs_request);
  request.set_topic("FrameTransformation.GetCalibration");
  request.set_reply_to(subscription);
  channel.publish(request);

  is::info("Waiting for 5 seconds to get calibrations");
  auto reply = channel.consume_for(seconds(5));
  if (!reply) is::critical("Failed to get cameras calibrations");
  auto maybe_calibrations = reply->unpack<is::vision::GetCalibrationReply>();
  if (!maybe_calibrations) is::critical("Invalid \'GetCalibrationReply\' reply");

  std::unordered_map<int64_t, is::vision::CameraCalibration> calibrations;
  for (auto& mc : maybe_calibrations->calibrations()) {
    is::info("Calibration from camera {} received.", mc.id());
    calibrations[mc.id()] = mc;
  }
  return calibrations;
}

void update_extrinsics(is::Channel& channel,
                       is::Subscription& subscription,
                       std::unordered_map<int64_t, is::vision::CameraCalibration>& calibrations,
                       int64_t const& referential) {
  // check for referential on calibrations
  std::vector<int64_t> without_ref;
  for (auto& kv : calibrations) {
    auto begin = kv.second.extrinsic().begin();
    auto end = kv.second.extrinsic().end();
    auto pos = std::find_if(begin, end, [&](auto& ext) { return ext.from() == referential; });
    if (pos == end) {
      auto camera = kv.first;
      without_ref.push_back(camera);
      auto tf_topic = fmt::format("FrameTransformation.{}.{}", referential, camera);
      subscription.subscribe(tf_topic);
    }
  }
  if (without_ref.empty()) return;

  is::info("Waiting for 5 seconds to necessary transformations");
  auto consume_deadline = system_clock::now() + seconds(5);
  while (!without_ref.empty()) {
    auto msg = channel.consume_until(consume_deadline);
    if (!msg) break;
    auto maybe_transformation = msg->unpack<is::vision::FrameTransformation>();
    if (!maybe_transformation) continue;
    auto camera = maybe_transformation->to();
    *calibrations[camera].add_extrinsic() = *maybe_transformation;
    without_ref.erase(std::remove(without_ref.begin(), without_ref.end(), camera), without_ref.end());
    subscription.unsubscribe(msg->topic());
  }
  if (!without_ref.empty()) { is::critical("Can't get all necessary transformations."); }
}

is::SkeletonsGrouperOptions load_options(int argc, char** argv) {
  std::string filename = (argc == 2) ? argv[1] : "options.json";
  is::SkeletonsGrouperOptions options;
  try {
    is::load(filename, &options);
    is::validate_message(options);
  } catch (std::exception& e) { is::critical("{}", e.what()); }
  // validate vertices
  auto not_unit = [](float const& c) { return c < 0.0 || c > 1.0; };
  for (auto& kv : options.cameras()) {
    auto camera = kv.first;
    auto vertices = kv.second.vertices();
    if (std::any_of(vertices.begin(), vertices.end(), [&](auto& v) { return not_unit(v.x()) || not_unit(v.y()); })) {
      is::critical("One of the vertices from camera \'{}\' isn't unitary: {}", camera, kv.second);
    }
  }
  return options;
}

std::shared_ptr<opentracing::Tracer> make_tracer(is::SkeletonsGrouperOptions& options,
                                                 std::string const& service_name) {
  ZipkinOtTracerOptions zp_options;
  zp_options.service_name = service_name;
  zp_options.collector_host = options.zipkin_host();
  zp_options.collector_port = options.zipkin_port();
  return makeZipkinOtTracer(zp_options);
}