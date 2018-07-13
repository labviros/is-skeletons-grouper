#pragma once

#include <armadillo>
#include <boost/bimap.hpp>
#include "is/msgs/camera.pb.h"
#include "skeletons.pb.h"

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
  SkeletonsGrouper(std::unordered_map<int64_t, CameraCalibration> calibrations, int64_t const& referencial,
                   double max_mean_d);

  Skeletons group(std::unordered_map<int64_t, Skeletons>& sks_2d);

 private:
  std::unordered_map<int64_t, CameraCalibration> calibrations;
  int64_t referencial;
  double max_mean_d;
  SkeletonsData data;
  std::map<SkeletonModel, SkeletonPartIndex> part_index_map;
};