#include "skeletons_grouper.hpp"

HSkeleton::HSkeleton(ObjectAnnotation* sk,
                     int64_t const& camera,
                     unsigned int id,
                     arma::mat const& scale_intrinsic,
                     HumanKeypointIndex& part_index_map)
    : skeleton(sk),
      camera(camera),
      id(id),
      points(3, part_index_map.size(), arma::fill::ones),
      scale_intrinsic(scale_intrinsic) {
  for (auto& part : sk->keypoints()) {
    auto p = part_index_map.by<human_keypoint>().at(part.id());
    this->parts.insert(p);
    this->points.at(0, p) = part.position().x();
    this->points.at(1, p) = part.position().y();
  }
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

unsigned int SkeletonsData::n_skeletons() {
  return this->n_sks;
}

void SkeletonsData::set_index_map(HumanKeypointIndex const& part_index_map) {
  this->part_index_map = part_index_map;
}

void SkeletonsData::set_calibrations(std::unordered_map<int64_t, CameraCalibration> const& calibs) {
  this->calibs = calibs;
}

void SkeletonsData::update(std::unordered_map<int64_t, ObjectAnnotations>& sks_2d) {
  this->clear();
  for (auto& ss : sks_2d) {
    auto camera = ss.first;
    auto& sks = ss.second;
    for (auto k = 0; k < sks.objects_size(); ++k) {
      auto sk = sks.mutable_objects(k);
      auto image_res = sks.resolution();
      auto calib_res = this->calibs[camera].resolution();
      arma::mat sc = intrinsic_scale_matrix(image_res, calib_res);
      HSkeleton_ptr hs = std::make_shared<HSkeleton>(sk, camera, this->n_sks, sc, this->part_index_map);
      this->sks_2d_cam[camera].push_back(hs);
      this->sks_2d_id[this->n_sks] = hs;
      this->cameras.push_back(camera);
      this->n_sks++;
    }
  }
  // remove repeated cameras
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
  const auto add_to_part_index_map = [&](std::string const& key_name, auto const& col_value) {
    const auto* descriptor = HumanKeypoints_descriptor();
    auto key_value = descriptor->FindValueByName(key_name)->number();
    this->part_index_map.insert(HumanKeypointIndex::value_type(key_value, col_value));
  };

  add_to_part_index_map("HEAD", 0);
  add_to_part_index_map("NOSE", 1);
  add_to_part_index_map("NECK", 2);
  add_to_part_index_map("RIGHT_SHOULDER", 3);
  add_to_part_index_map("RIGHT_ELBOW", 4);
  add_to_part_index_map("RIGHT_WRIST", 5);
  add_to_part_index_map("LEFT_SHOULDER", 6);
  add_to_part_index_map("LEFT_ELBOW", 7);
  add_to_part_index_map("LEFT_WRIST", 8);
  add_to_part_index_map("RIGHT_HIP", 9);
  add_to_part_index_map("RIGHT_KNEE", 10);
  add_to_part_index_map("RIGHT_ANKLE", 11);
  add_to_part_index_map("LEFT_HIP", 12);
  add_to_part_index_map("LEFT_KNEE", 13);
  add_to_part_index_map("LEFT_ANKLE", 14);
  add_to_part_index_map("RIGHT_EYE", 15);
  add_to_part_index_map("LEFT_EYE", 16);
  add_to_part_index_map("RIGHT_EAR", 17);
  add_to_part_index_map("LEFT_EAR", 18);
  add_to_part_index_map("CHEST", 19);

  this->data.set_index_map(this->part_index_map);
  this->data.set_calibrations(this->calibrations);

  this->F = compute_fundamentals_matrix(this->calibrations, this->referencial);
}

ObjectAnnotations SkeletonsGrouper::group(std::unordered_map<int64_t, ObjectAnnotations>& sks_2d) {
  // TODO: check if cameras received are available
  this->data.update(sks_2d);
  if (this->data.n_skeletons() < 2) return ObjectAnnotations();

  std::vector<std::vector<int>> matches(this->data.n_skeletons());
  auto& cameras = this->data.get_cameras();
  for (auto& cam0 : cameras) {
    for (auto& cam1 : cameras) {
      if (cam0 == cam1) continue;
      auto ms = this->find_matches(cam0, cam1);
      std::for_each(ms.begin(), ms.end(), [&](auto& m) {
        matches[m.first].push_back(m.second);
        matches[m.second].push_back(m.first);
      });
    }
  }

  // remove repeated matches to make group_matches faster
  for (auto& m : matches) {
    std::sort(m.begin(), m.end());
    auto last = std::unique(m.begin(), m.end());
    m.erase(last, m.end());
  }

  auto groups = group_matches(matches);
  return make_3d_skeletons(groups);
}

std::vector<std::pair<int, int>> SkeletonsGrouper::find_matches(int64_t cam0, int64_t cam1) {
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

      auto& sc0_intrinsic = sk0_data->scale_intrinsic;
      auto& sc1_intrinsic = sk1_data->scale_intrinsic;

      arma::mat lines0 = epipolar_line(sk1_points, this->F[cam0][cam1], sc0_intrinsic, sc1_intrinsic);
      arma::mat lines1 = epipolar_line(sk0_points, this->F[cam1][cam0], sc1_intrinsic, sc0_intrinsic);
      auto m_error0 = mean_distance(sk0_points, lines0, parts);
      auto m_error1 = mean_distance(sk1_points, lines1, parts);
      auto error = m_error0 + m_error1;
      if (error > this->max_mean_d) continue;

      auto& id0 = sk0_data->id;
      auto& id1 = sk1_data->id;
      matches[id0].push_back({.id = id1, .error = m_error0});
      rev_matches[id1].push_back({.id = id0, .error = m_error1});
    }
  }

  // keep on matches just matches with smaller error
  for (auto& rvm : rev_matches) {
    if (rvm.second.size() < 2) continue;
    auto id_match = rvm.first;
    auto& rv = rvm.second;
    auto best_match =
        std::min_element(rv.begin(), rv.end(), [](auto& lhs, auto& rhs) { return lhs.error < rhs.error; });
    for (auto it = rv.begin(); it != rv.end(); ++it) {
      if (it == best_match) continue;
      auto id_ref = it->id;
      std::vector<SkeletonMatch> new_matches;
      auto const copy_matches = [&](auto lid, auto rid) {
        std::copy_if(matches[lid].begin(), matches[lid].end(), std::back_inserter(new_matches), [&](auto& m) {
          return m.id != rid;
        });
      };
      copy_matches(id_ref, id_match);
      matches[id_ref] = new_matches;
      new_matches.clear();
      copy_matches(id_match, id_ref);
      matches[id_match] = new_matches;
    }
  }

  std::vector<std::pair<int, int>> final_matches;
  for (auto& m : matches) {
    auto& id_ref = m.first;
    if (m.second.empty()) continue;
    auto& id_match = m.second.front().id;
    final_matches.emplace_back(id_ref, id_match);
  }
  return final_matches;
}

std::vector<std::vector<int>> SkeletonsGrouper::group_matches(std::vector<std::vector<int>>& matches) {
  std::vector<std::vector<int>> groups;
  auto n_labels = matches.size();
  std::vector<bool> visited(n_labels, false);

  const std::function<void(int, std::vector<bool>&, std::vector<int>&)> traverse =
      [&](int n, std::vector<bool>& visited, std::vector<int>& connected) {
        visited[n] = true;
        connected.push_back(n);
        for (auto& k : matches[n]) {
          if (!visited[k]) traverse(k, visited, connected);
        }
      };

  for (size_t n = 0; n < n_labels; n++) {
    if (!visited[n]) {
      std::vector<int> connected;
      traverse(n, visited, connected);
      groups.push_back(connected);
    }
  }
  return groups;
}

ObjectAnnotations SkeletonsGrouper::make_3d_skeletons(std::vector<std::vector<int>>& groups) {
  ObjectAnnotations sks_3d;
  for (auto& group : groups) {
    if (group.size() < 2) continue;

    std::map<arma::uword /* part matrix column */, std::vector<unsigned int> /* skeleton id */> sk_parts;
    std::for_each(group.begin(), group.end(), [&](auto& sk_id) {
      auto& parts = this->data.by_id()[sk_id]->parts;
      std::for_each(parts.begin(), parts.end(), [&](auto& part) { sk_parts[part].push_back(sk_id); });
    });

    auto skeleton = sks_3d.add_objects();
    for (auto& part_ids : sk_parts) {
      auto& part = part_ids.first;
      auto& skeletons = part_ids.second;
      if (skeletons.size() < 2) continue;
      *skeleton->add_keypoints() = make_3d_part(part, skeletons);
    }
  }
  return sks_3d;
}

PointAnnotation SkeletonsGrouper::make_3d_part(arma::uword const& part, std::vector<unsigned int>& skeletons) {
  PointAnnotation sk_part;
  auto n_skeletons = skeletons.size();
  arma::mat A(3 * n_skeletons, 3 + n_skeletons, arma::fill::zeros);
  arma::vec b(3 * n_skeletons);
  auto indices = arma::regspace<arma::uvec>(0, 3 * 3 * n_skeletons - 1);
  A.elem(indices) = arma::repmat(-1.0 * arma::eye(3, 3), n_skeletons, 1);
  for (size_t k = 0; k < n_skeletons; ++k) {
    auto& sk_id = skeletons[k];
    auto& sk = this->data.by_id()[sk_id];
    auto m = sk->points.col(part);
    auto& calib = this->calibrations[sk->camera];
    arma::mat scale_intrinsic = sk->scale_intrinsic;
    arma::mat K = scale_intrinsic * arma_view(calib.mutable_intrinsic());
    arma::mat RT = get_extrinsic(calib, this->referencial);
    if (RT.is_empty()) continue;
    arma::mat R = RT.submat(0, 0, 2, 2);
    arma::mat t = RT.col(3).rows(0, 2);
    auto base_index = A.n_rows * (3 + k) + 3 * k;
    indices.clear();
    indices << base_index << base_index + 1 << base_index + 2;
    A.elem(indices) = (K * R).i() * m;  // W matrix (see article)
    indices.clear();
    base_index = 3 * k;
    indices << base_index << base_index + 1 << base_index + 2;
    b.elem(indices) = R.i() * t;
  }
  arma::vec X = arma::pinv(A) * b;
  sk_part.mutable_position()->set_x(X[0]);
  sk_part.mutable_position()->set_y(X[1]);
  sk_part.mutable_position()->set_z(X[2]);
  sk_part.set_id(this->part_index_map.by<col_index>().at(part));
  return sk_part;
}