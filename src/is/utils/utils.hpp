#pragma once

#include <unordered_map>
#include <regex>
#include <chrono>
#include <string>
#include "is/msgs/common.pb.h"
#include "is/msgs/camera.pb.h"
#include "is/msgs/image.pb.h"
#include "is/msgs/utils.hpp"
#include "is/wire/core.hpp"
#include "boost/optional.hpp"
#include "options.pb.h"
#include "zipkin/opentracing.h"

using namespace zipkin;
using namespace opentracing;

void filter_by_region(std::unordered_map<int64_t, is::vision::ObjectAnnotations>& sks_group,
                      google::protobuf::Map<int64_t, is::vision::BoundingPoly> configs);

int64_t get_id(std::string const& topic);

boost::optional<is::vision::FrameTransformation> make_frame_transformation(std::string const& topic,
                                                                           is::common::Tensor const& tf);

std::vector<int64_t> get_cameras(std::unordered_map<int64_t, is::vision::ObjectAnnotations>& sks);

std::string detections_info(std::unordered_map<int64_t, is::vision::ObjectAnnotations>& sks);

std::unordered_map<int64_t, is::vision::CameraCalibration> request_calibrations(is::Channel& channel,
                                                                                is::Subscription& subscription,
                                                                                std::vector<int64_t>& cameras);
void update_extrinsics(is::Channel& channel,
                       is::Subscription& subscription,
                       std::unordered_map<int64_t, is::vision::CameraCalibration>& calibrations,
                       int64_t const& referential);

is::SkeletonsGrouperOptions load_options(int argc, char** argv);

std::shared_ptr<opentracing::Tracer> make_tracer(is::SkeletonsGrouperOptions& options, std::string const& service_name);