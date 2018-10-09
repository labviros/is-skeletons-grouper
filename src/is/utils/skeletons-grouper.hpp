#pragma once

#include <armadillo>
#include <boost/bimap.hpp>
#include "is/msgs/camera.pb.h"
#include "is/msgs/image.pb.h"
#include "vision.hpp"

using namespace is::vision;
using namespace boost::bimaps;

struct col_index {};
struct human_keypoint {};
typedef bimap<tagged<int64_t /* HumanKeypoints */, human_keypoint>, tagged<arma::uword, col_index>> HumanKeypointIndex;

struct HSkeleton {
  HSkeleton(ObjectAnnotation* sk,
            int64_t const& camera,
            unsigned int id,
            arma::mat const& scale_intrinsic,
            HumanKeypointIndex& part_index_map);
  ObjectAnnotation* skeleton;
  int64_t camera;
  unsigned int id;
  std::set<arma::uword> parts;
  arma::mat points;
  arma::mat scale_intrinsic;
};

typedef std::shared_ptr<HSkeleton> HSkeleton_ptr;

class SkeletonsData {
 public:
  void update(std::unordered_map<int64_t, ObjectAnnotations>& sks_2d);
  std::map<int64_t, std::vector<HSkeleton_ptr>>& by_cam();
  std::map<unsigned int, HSkeleton_ptr>& by_id();
  std::vector<int64_t>& get_cameras();
  unsigned int n_skeletons();
  void set_index_map(HumanKeypointIndex const& part_index_map);
  void set_calibrations(std::unordered_map<int64_t, CameraCalibration> const& calibs);

 private:
  std::map<int64_t, std::vector<HSkeleton_ptr>> sks_2d_cam;
  std::map<unsigned int, HSkeleton_ptr> sks_2d_id;
  HumanKeypointIndex part_index_map;
  std::unordered_map<int64_t, CameraCalibration> calibs;
  std::vector<int64_t> cameras;
  unsigned int n_sks;
  void clear();
};

class SkeletonsGrouper {
 public:
  SkeletonsGrouper(std::unordered_map<int64_t, CameraCalibration> calibrations,
                   int64_t const& referencial,
                   double max_mean_d,
                   double min_score,
                   double max_distance);

  ObjectAnnotations group(std::unordered_map<int64_t, ObjectAnnotations>& sks_2d);
  void set_max_error(double const& max_error);
  void set_min_score(double const& min_score);
  void set_max_distance(double const& max_distance);

 private:
  std::unordered_map<int64_t, CameraCalibration> calibrations;
  int64_t referencial;
  double max_mean_d;
  double min_score;
  double max_distance;
  SkeletonsData data;
  HumanKeypointIndex part_index_map;
  std::unordered_map<int64_t /* destination camera */, std::unordered_map<int64_t /* reference camera */, arma::mat>> F;
  std::vector<std::pair<int64_t, int64_t>> links;

 private:
  std::vector<std::pair<int, int>> find_matches(int64_t cam0, int64_t cam1);
  /*
    Based on algorithm to group connected components in an undirected graph
    https://www.geeksforgeeks.org/connected-components-in-an-undirected-graph/
  */
  std::vector<std::vector<int>> group_matches(std::vector<std::vector<int>>& matches);

  ObjectAnnotations make_3d_skeletons(std::vector<std::vector<int>>& groups);
  PointAnnotation make_3d_part(arma::uword const& part, std::vector<unsigned int>& skeletons);
  void filter_by_score(std::unordered_map<int64_t, ObjectAnnotations>& sks_2d);
};