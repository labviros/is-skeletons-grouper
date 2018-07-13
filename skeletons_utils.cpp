#include "skeletons_utils.hpp"

void render_skeletons(cv::Mat& image, is::vision::Skeletons const& skeletons, bool render_lins = true) {
  for (auto& sk : skeletons.skeletons()) {
    std::map<SkeletonPartType, std::pair<double, double>> joints;
    for (auto& part : sk.parts()) {
      joints[part.type()] = std::make_pair(part.x(), part.y());
    }

    if (render_lins) {
      // TODO
    }

    for (auto& joint : joints) {
      cv::Point p(joint.second.first, joint.second.second);
      cv::circle(image, p, 3, cv::Scalar(255, 0, 0), 3);
    }
  }
}