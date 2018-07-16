#pragma once

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <armadillo>
#include <boost/bimap.hpp>
#include "is/msgs/camera.pb.h"
#include "skeletons.pb.h"
#include "vision.hpp"

using namespace is::vision;
using namespace boost::bimaps;

struct col_index {};
struct sk_type {};
typedef bimap<tagged<SkeletonPartType, sk_type>, tagged<arma::uword, col_index>> SkeletonPartIndex;

struct HSkeleton {
  HSkeleton(Skeleton* sk, int64_t const& camera, unsigned int id, SkeletonPartIndex& part_index_map);
  Skeleton* skeleton;
  int64_t camera;
  unsigned int id;
  std::set<arma::uword> parts;
  arma::mat points;
};

typedef std::shared_ptr<HSkeleton> HSkeleton_ptr;

class SkeletonsData {
 public:
  void update(std::unordered_map<int64_t, Skeletons>& sks_2d);
  std::map<int64_t, std::vector<HSkeleton_ptr>>& by_cam();
  std::map<unsigned int, HSkeleton_ptr>& by_id();
  std::vector<int64_t>& get_cameras();
  SkeletonModel& get_model();
  unsigned int n_skeletons();
  void set_index_map(std::map<SkeletonModel, SkeletonPartIndex> const& part_index_map);

 private:
  std::map<SkeletonModel, SkeletonPartIndex> part_index_map;
  std::map<int64_t, std::vector<HSkeleton_ptr>> sks_2d_cam;
  std::map<unsigned int, HSkeleton_ptr> sks_2d_id;
  std::vector<int64_t> cameras;
  SkeletonModel model;
  unsigned int n_sks;
  void clear();
};

class SkeletonsGrouper {
 public:
  SkeletonsGrouper(std::unordered_map<int64_t, CameraCalibration> calibrations,
                   int64_t const& referencial,
                   double max_mean_d);

  Skeletons group(std::unordered_map<int64_t, Skeletons>& sks_2d);

 private:
  std::unordered_map<int64_t, CameraCalibration> calibrations;
  int64_t referencial;
  double max_mean_d;
  SkeletonsData data;
  std::map<SkeletonModel, SkeletonPartIndex> part_index_map;
  std::unordered_map<int64_t /* destination camera */, std::unordered_map<int64_t /* reference camera */, arma::mat>> F;

 private:
  std::vector<std::pair<int, int>> find_matches(int64_t cam0, int64_t cam1);
  /*
    Based on algorith to group connected components in an undirected graph
    https://www.geeksforgeeks.org/connected-components-in-an-undirected-graph/
  */
  std::vector<std::vector<int>> group_matches(std::vector<std::vector<int>>& matches);

  Skeletons make_3d_skeletons(std::vector<std::vector<int>>& groups, SkeletonModel& model);
  SkeletonPart make_3d_part(arma::uword const& part,
                            std::vector<unsigned int>& skeletons,
                            SkeletonPartIndex& part_index_map);
};

void render_skeletons(cv::Mat& image, HSkeleton_ptr& sk_data, std::vector<arma::uword>& common_parts);
void render_epipolar_lines(cv::Mat& image, arma::mat& lines, std::vector<arma::uword>& common_parts);