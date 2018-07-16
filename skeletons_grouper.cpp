#include "skeletons_grouper.hpp"

HSkeleton::HSkeleton(Skeleton* sk, int64_t const& camera, unsigned int id, SkeletonPartIndex& part_index_map)
    : skeleton(sk), camera(camera), id(id), points(3, part_index_map.size(), arma::fill::ones) {
  std::for_each(this->skeleton->parts().begin(), this->skeleton->parts().end(), [&](auto& part) {
    auto p = part_index_map.by<sk_type>().at(part.type());
    this->parts.insert(p);
    this->points.at(0, p) = part.x();
    this->points.at(1, p) = part.y();
  });
}

std::map<int64_t, std::vector<HSkeleton_ptr>>& SkeletonsData::by_cam() {
  return this->sks_2d_cam;
}

std::map<unsigned int, HSkeleton_ptr>& SkeletonsData::by_id() {
  return this->sks_2d_id;
}

std::vector<int64_t>& SkeletonsData::get_cameras() {
  return this->cameras;
}

SkeletonModel& SkeletonsData::get_model() {
  return this->model;
}

unsigned int SkeletonsData::n_skeletons() {
  return this->n_sks;
}

void SkeletonsData::set_index_map(std::map<SkeletonModel, SkeletonPartIndex> const& part_index_map) {
  this->part_index_map = part_index_map;
}

void SkeletonsData::update(std::unordered_map<int64_t, Skeletons>& sks_2d) {
  this->clear();
  for (auto& ss : sks_2d) {
    auto camera = ss.first;
    auto& sks = ss.second;
    this->model = sks.model();
    for (auto k = 0; k < sks.skeletons_size(); ++k) {
      auto sk = sks.mutable_skeletons(k);
      HSkeleton_ptr hs = std::make_shared<HSkeleton>(sk, camera, this->n_sks, this->part_index_map[model]);
      this->sks_2d_cam[camera].push_back(hs);
      this->sks_2d_id[this->n_sks] = hs;
      this->cameras.push_back(camera);
      this->n_sks++;
    }
  }
  std::sort(this->cameras.begin(), this->cameras.end());
  auto last = std::unique(this->cameras.begin(), this->cameras.end());
  this->cameras.erase(last, this->cameras.end());
}

void SkeletonsData::clear() {
  this->n_sks = 0;
  this->cameras.clear();
  this->sks_2d_cam.clear();
  this->sks_2d_id.clear();
}

SkeletonsGrouper::SkeletonsGrouper(std::unordered_map<int64_t, CameraCalibration> calibrations,
                                   int64_t const& referencial,
                                   double max_mean_d = 50.0)
    : calibrations(calibrations), referencial(referencial), max_mean_d(max_mean_d) {
  SkeletonPartIndex part_index_mpii;
  part_index_mpii.insert(SkeletonPartIndex::value_type(SkeletonPartType::NECK, 0));
  part_index_mpii.insert(SkeletonPartIndex::value_type(SkeletonPartType::HEAD, 1));
  part_index_mpii.insert(SkeletonPartIndex::value_type(SkeletonPartType::CHEST, 2));
  part_index_mpii.insert(SkeletonPartIndex::value_type(SkeletonPartType::LEFT_SHOULDER, 3));
  part_index_mpii.insert(SkeletonPartIndex::value_type(SkeletonPartType::LEFT_ELBOW, 4));
  part_index_mpii.insert(SkeletonPartIndex::value_type(SkeletonPartType::LEFT_WRIST, 5));
  part_index_mpii.insert(SkeletonPartIndex::value_type(SkeletonPartType::LEFT_HIP, 6));
  part_index_mpii.insert(SkeletonPartIndex::value_type(SkeletonPartType::LEFT_KNEE, 7));
  part_index_mpii.insert(SkeletonPartIndex::value_type(SkeletonPartType::LEFT_ANKLE, 8));
  part_index_mpii.insert(SkeletonPartIndex::value_type(SkeletonPartType::RIGHT_SHOULDER, 9));
  part_index_mpii.insert(SkeletonPartIndex::value_type(SkeletonPartType::RIGHT_ELBOW, 10));
  part_index_mpii.insert(SkeletonPartIndex::value_type(SkeletonPartType::RIGHT_WRIST, 11));
  part_index_mpii.insert(SkeletonPartIndex::value_type(SkeletonPartType::RIGHT_HIP, 12));
  part_index_mpii.insert(SkeletonPartIndex::value_type(SkeletonPartType::RIGHT_KNEE, 13));
  part_index_mpii.insert(SkeletonPartIndex::value_type(SkeletonPartType::RIGHT_ANKLE, 14));
  this->part_index_map[SkeletonModel::MPII] = part_index_mpii;

  SkeletonPartIndex part_index_coco;
  part_index_coco.insert(SkeletonPartIndex::value_type(SkeletonPartType::NECK, 0));
  part_index_coco.insert(SkeletonPartIndex::value_type(SkeletonPartType::NOSE, 1));
  part_index_coco.insert(SkeletonPartIndex::value_type(SkeletonPartType::BACKGROUND, 2));
  part_index_coco.insert(SkeletonPartIndex::value_type(SkeletonPartType::LEFT_SHOULDER, 3));
  part_index_coco.insert(SkeletonPartIndex::value_type(SkeletonPartType::LEFT_ELBOW, 4));
  part_index_coco.insert(SkeletonPartIndex::value_type(SkeletonPartType::LEFT_WRIST, 5));
  part_index_coco.insert(SkeletonPartIndex::value_type(SkeletonPartType::LEFT_HIP, 6));
  part_index_coco.insert(SkeletonPartIndex::value_type(SkeletonPartType::LEFT_KNEE, 7));
  part_index_coco.insert(SkeletonPartIndex::value_type(SkeletonPartType::LEFT_ANKLE, 8));
  part_index_coco.insert(SkeletonPartIndex::value_type(SkeletonPartType::RIGHT_SHOULDER, 9));
  part_index_coco.insert(SkeletonPartIndex::value_type(SkeletonPartType::RIGHT_ELBOW, 10));
  part_index_coco.insert(SkeletonPartIndex::value_type(SkeletonPartType::RIGHT_WRIST, 11));
  part_index_coco.insert(SkeletonPartIndex::value_type(SkeletonPartType::RIGHT_HIP, 12));
  part_index_coco.insert(SkeletonPartIndex::value_type(SkeletonPartType::RIGHT_KNEE, 13));
  part_index_coco.insert(SkeletonPartIndex::value_type(SkeletonPartType::RIGHT_ANKLE, 14));
  part_index_coco.insert(SkeletonPartIndex::value_type(SkeletonPartType::LEFT_EYE, 15));
  part_index_coco.insert(SkeletonPartIndex::value_type(SkeletonPartType::LEFT_EAR, 16));
  part_index_coco.insert(SkeletonPartIndex::value_type(SkeletonPartType::RIGHT_EYE, 17));
  part_index_coco.insert(SkeletonPartIndex::value_type(SkeletonPartType::RIGHT_EAR, 18));
  this->part_index_map[SkeletonModel::COCO] = part_index_coco;

  this->data.set_index_map(this->part_index_map);

  this->F = compute_fundamentals_matrix(this->calibrations, this->referencial);
}

Skeletons SkeletonsGrouper::group(std::unordered_map<int64_t, Skeletons>& sks_2d) {
  this->data.update(sks_2d);
  if (this->data.n_skeletons() < 2) return Skeletons();

  std::vector<std::vector<int>> matches(this->data.n_skeletons());
  auto& cameras = this->data.get_cameras();
  for (auto& cam0 : cameras) {
    for (auto& cam1 : cameras) {
      if (cam0 == cam1) continue;
      auto matches = this->find_matches(cam0, cam1);
    }
  }

  return Skeletons();
}

std::vector<std::pair<unsigned int, unsigned int>> SkeletonsGrouper::find_matches(int64_t cam0, int64_t cam1) {
  /*
    'cam0' -> reference camera
    'cam1' -> destination camera
  */
  struct SkeletonMatch {
    unsigned int id;
    double error;
  };
  std::map<unsigned int, std::vector<SkeletonMatch>> matches, rev_matches;

  for (auto& sk0_data : this->data.by_cam().at(cam0)) {
    for (auto& sk1_data : this->data.by_cam().at(cam1)) {
      auto& sk0_parts = sk0_data->parts;
      auto& sk1_parts = sk1_data->parts;

      std::vector<arma::uword> common_parts;
      std::set_intersection(
          sk0_parts.begin(), sk0_parts.end(), sk1_parts.begin(), sk1_parts.end(), std::back_inserter(common_parts));

      if (common_parts.empty()) continue;

      auto& sk0_points = sk0_data->points;
      auto& sk1_points = sk1_data->points;
      arma::urowvec parts(common_parts.data(), common_parts.size(), false, false);

      arma::mat lines0 = epipolar_line(sk1_points, this->F[cam0][cam1]);
      arma::mat lines1 = epipolar_line(sk0_points, this->F[cam1][cam0]);
    }
  }

  std::vector<std::pair<unsigned int, unsigned int>> final_matches;
  return final_matches;
}