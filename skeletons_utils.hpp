#pragma once

#include <map>
#include <vector>
#include <utility>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "skeletons.pb.h"

using namespace is::vision;

std::vector<std::pair<SkeletonPartType, SkeletonPartType>> coco_links{
    std::make_pair(SkeletonPartType::NECK, SkeletonPartType::LEFT_SHOULDER),
    std::make_pair(SkeletonPartType::LEFT_SHOULDER, SkeletonPartType::LEFT_ELBOW),
    std::make_pair(SkeletonPartType::LEFT_ELBOW, SkeletonPartType::LEFT_WRIST),
    std::make_pair(SkeletonPartType::NECK, SkeletonPartType::LEFT_HIP),
    std::make_pair(SkeletonPartType::LEFT_HIP, SkeletonPartType::LEFT_KNEE),
    std::make_pair(SkeletonPartType::LEFT_KNEE, SkeletonPartType::LEFT_ANKLE),
    std::make_pair(SkeletonPartType::NECK, SkeletonPartType::RIGHT_SHOULDER),
    std::make_pair(SkeletonPartType::RIGHT_SHOULDER, SkeletonPartType::RIGHT_ELBOW),
    std::make_pair(SkeletonPartType::RIGHT_ELBOW, SkeletonPartType::RIGHT_WRIST),
    std::make_pair(SkeletonPartType::NECK, SkeletonPartType::RIGHT_HIP),
    std::make_pair(SkeletonPartType::RIGHT_HIP, SkeletonPartType::RIGHT_KNEE),
    std::make_pair(SkeletonPartType::RIGHT_KNEE, SkeletonPartType::RIGHT_ANKLE),
    std::make_pair(SkeletonPartType::NOSE, SkeletonPartType::LEFT_EYE),
    std::make_pair(SkeletonPartType::LEFT_EYE, SkeletonPartType::LEFT_EAR),
    std::make_pair(SkeletonPartType::NOSE, SkeletonPartType::RIGHT_EYE),
    std::make_pair(SkeletonPartType::RIGHT_EYE, SkeletonPartType::RIGHT_EAR)};

void render_skeletons(cv::Mat& image, is::vision::Skeletons const& skeletons, bool render_links);