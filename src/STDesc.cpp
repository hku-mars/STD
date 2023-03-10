#include "include/STDesc.h"
void down_sampling_voxel(pcl::PointCloud<pcl::PointXYZI> &pl_feat,
                         double voxel_size) {
  int intensity = rand() % 255;
  if (voxel_size < 0.01) {
    return;
  }
  std::unordered_map<VOXEL_LOC, M_POINT> voxel_map;
  uint plsize = pl_feat.size();

  for (uint i = 0; i < plsize; i++) {
    pcl::PointXYZI &p_c = pl_feat[i];
    float loc_xyz[3];
    for (int j = 0; j < 3; j++) {
      loc_xyz[j] = p_c.data[j] / voxel_size;
      if (loc_xyz[j] < 0) {
        loc_xyz[j] -= 1.0;
      }
    }

    VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
                       (int64_t)loc_xyz[2]);
    auto iter = voxel_map.find(position);
    if (iter != voxel_map.end()) {
      iter->second.xyz[0] += p_c.x;
      iter->second.xyz[1] += p_c.y;
      iter->second.xyz[2] += p_c.z;
      iter->second.intensity += p_c.intensity;
      iter->second.count++;
    } else {
      M_POINT anp;
      anp.xyz[0] = p_c.x;
      anp.xyz[1] = p_c.y;
      anp.xyz[2] = p_c.z;
      anp.intensity = p_c.intensity;
      anp.count = 1;
      voxel_map[position] = anp;
    }
  }
  plsize = voxel_map.size();
  pl_feat.clear();
  pl_feat.resize(plsize);

  uint i = 0;
  for (auto iter = voxel_map.begin(); iter != voxel_map.end(); ++iter) {
    pl_feat[i].x = iter->second.xyz[0] / iter->second.count;
    pl_feat[i].y = iter->second.xyz[1] / iter->second.count;
    pl_feat[i].z = iter->second.xyz[2] / iter->second.count;
    pl_feat[i].intensity = iter->second.intensity / iter->second.count;
    i++;
  }
}

void read_parameters(ros::NodeHandle &nh, ConfigSetting &config_setting) {

  // pre-preocess
  nh.param<double>("ds_size", config_setting.ds_size_, 0.5);
  nh.param<int>("maximum_corner_num", config_setting.maximum_corner_num_, 100);

  // key points
  nh.param<double>("plane_merge_normal_thre",
                   config_setting.plane_merge_normal_thre_, 0.1);
  nh.param<double>("plane_detection_thre", config_setting.plane_detection_thre_,
                   0.01);
  nh.param<double>("voxel_size", config_setting.voxel_size_, 2.0);
  nh.param<int>("voxel_init_num", config_setting.voxel_init_num_, 10);
  nh.param<double>("proj_image_resolution",
                   config_setting.proj_image_resolution_, 0.5);
  nh.param<double>("proj_dis_min", config_setting.proj_dis_min_, 0);
  nh.param<double>("proj_dis_max", config_setting.proj_dis_max_, 2);
  nh.param<double>("corner_thre", config_setting.corner_thre_, 10);

  // std descriptor
  nh.param<int>("descriptor_near_num", config_setting.descriptor_near_num_, 10);
  nh.param<double>("descriptor_min_len", config_setting.descriptor_min_len_, 2);
  nh.param<double>("descriptor_max_len", config_setting.descriptor_max_len_,
                   50);
  nh.param<double>("non_max_suppression_radius",
                   config_setting.non_max_suppression_radius_, 2.0);
  nh.param<double>("std_side_resolution", config_setting.std_side_resolution_,
                   0.2);

  // candidate search
  nh.param<int>("skip_near_num", config_setting.skip_near_num_, 50);
  nh.param<int>("candidate_num", config_setting.candidate_num_, 50);
  nh.param<int>("sub_frame_num", config_setting.sub_frame_num_, 10);
  nh.param<double>("rough_dis_threshold", config_setting.rough_dis_threshold_,
                   0.01);
  nh.param<double>("vertex_diff_threshold",
                   config_setting.vertex_diff_threshold_, 0.5);
  nh.param<double>("icp_threshold", config_setting.icp_threshold_, 0.5);
  nh.param<double>("normal_threshold", config_setting.normal_threshold_, 0.2);
  nh.param<double>("dis_threshold", config_setting.dis_threshold_, 0.5);

  std::cout << "Sucessfully load parameters:" << std::endl;
  std::cout << "----------------Main Parameters-------------------"
            << std::endl;
  std::cout << "voxel size:" << config_setting.voxel_size_ << std::endl;
  std::cout << "loop detection threshold: " << config_setting.icp_threshold_
            << std::endl;
  std::cout << "sub-frame number: " << config_setting.sub_frame_num_
            << std::endl;
  std::cout << "candidate number: " << config_setting.candidate_num_
            << std::endl;
  std::cout << "maximum corners size: " << config_setting.maximum_corner_num_
            << std::endl;
}

void load_pose_with_time(
    const std::string &pose_file,
    std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> &poses_vec,
    std::vector<double> &times_vec) {
  times_vec.clear();
  poses_vec.clear();
  std::ifstream fin(pose_file);
  std::string line;
  Eigen::Matrix<double, 1, 7> temp_matrix;
  while (getline(fin, line)) {
    std::istringstream sin(line);
    std::vector<std::string> Waypoints;
    std::string info;
    int number = 0;
    while (getline(sin, info, ' ')) {
      if (number == 0) {
        double time;
        std::stringstream data;
        data << info;
        data >> time;
        times_vec.push_back(time);
        number++;
      } else {
        double p;
        std::stringstream data;
        data << info;
        data >> p;
        temp_matrix[number - 1] = p;
        if (number == 7) {
          Eigen::Vector3d translation(temp_matrix[0], temp_matrix[1],
                                      temp_matrix[2]);
          Eigen::Quaterniond q(temp_matrix[6], temp_matrix[3], temp_matrix[4],
                               temp_matrix[5]);
          std::pair<Eigen::Vector3d, Eigen::Matrix3d> single_pose;
          single_pose.first = translation;
          single_pose.second = q.toRotationMatrix();
          poses_vec.push_back(single_pose);
        }
        number++;
      }
    }
  }
}

double time_inc(std::chrono::_V2::system_clock::time_point &t_end,
                std::chrono::_V2::system_clock::time_point &t_begin) {
  return std::chrono::duration_cast<std::chrono::duration<double>>(t_end -
                                                                   t_begin)
             .count() *
         1000;
}

pcl::PointXYZI vec2point(const Eigen::Vector3d &vec) {
  pcl::PointXYZI pi;
  pi.x = vec[0];
  pi.y = vec[1];
  pi.z = vec[2];
  return pi;
}
Eigen::Vector3d point2vec(const pcl::PointXYZI &pi) {
  return Eigen::Vector3d(pi.x, pi.y, pi.z);
}

bool attach_greater_sort(std::pair<double, int> a, std::pair<double, int> b) {
  return (a.first > b.first);
}

void publish_std_pairs(
    const std::vector<std::pair<STDesc, STDesc>> &match_std_pairs,
    const ros::Publisher &std_publisher) {
  visualization_msgs::MarkerArray ma_line;
  visualization_msgs::Marker m_line;
  m_line.type = visualization_msgs::Marker::LINE_LIST;
  m_line.action = visualization_msgs::Marker::ADD;
  m_line.ns = "lines";
  // Don't forget to set the alpha!
  m_line.scale.x = 0.25;
  m_line.pose.orientation.w = 1.0;
  m_line.header.frame_id = "camera_init";
  m_line.id = 0;
  int max_pub_cnt = 1;
  for (auto var : match_std_pairs) {
    if (max_pub_cnt > 100) {
      break;
    }
    max_pub_cnt++;
    m_line.color.a = 0.8;
    m_line.points.clear();
    m_line.color.r = 138.0 / 255;
    m_line.color.g = 226.0 / 255;
    m_line.color.b = 52.0 / 255;
    geometry_msgs::Point p;
    p.x = var.second.vertex_A_[0];
    p.y = var.second.vertex_A_[1];
    p.z = var.second.vertex_A_[2];
    Eigen::Vector3d t_p;
    t_p << p.x, p.y, p.z;
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);
    p.x = var.second.vertex_B_[0];
    p.y = var.second.vertex_B_[1];
    p.z = var.second.vertex_B_[2];
    t_p << p.x, p.y, p.z;
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);
    ma_line.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();
    p.x = var.second.vertex_C_[0];
    p.y = var.second.vertex_C_[1];
    p.z = var.second.vertex_C_[2];
    t_p << p.x, p.y, p.z;
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);
    p.x = var.second.vertex_B_[0];
    p.y = var.second.vertex_B_[1];
    p.z = var.second.vertex_B_[2];
    t_p << p.x, p.y, p.z;
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);
    ma_line.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();
    p.x = var.second.vertex_C_[0];
    p.y = var.second.vertex_C_[1];
    p.z = var.second.vertex_C_[2];
    t_p << p.x, p.y, p.z;
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);
    p.x = var.second.vertex_A_[0];
    p.y = var.second.vertex_A_[1];
    p.z = var.second.vertex_A_[2];
    t_p << p.x, p.y, p.z;
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);
    ma_line.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();
    // another
    m_line.points.clear();
    m_line.color.r = 1;
    m_line.color.g = 1;
    m_line.color.b = 1;
    p.x = var.first.vertex_A_[0];
    p.y = var.first.vertex_A_[1];
    p.z = var.first.vertex_A_[2];
    m_line.points.push_back(p);
    p.x = var.first.vertex_B_[0];
    p.y = var.first.vertex_B_[1];
    p.z = var.first.vertex_B_[2];
    m_line.points.push_back(p);
    ma_line.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();
    p.x = var.first.vertex_C_[0];
    p.y = var.first.vertex_C_[1];
    p.z = var.first.vertex_C_[2];
    m_line.points.push_back(p);
    p.x = var.first.vertex_B_[0];
    p.y = var.first.vertex_B_[1];
    p.z = var.first.vertex_B_[2];
    m_line.points.push_back(p);
    ma_line.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();
    p.x = var.first.vertex_C_[0];
    p.y = var.first.vertex_C_[1];
    p.z = var.first.vertex_C_[2];
    m_line.points.push_back(p);
    p.x = var.first.vertex_A_[0];
    p.y = var.first.vertex_A_[1];
    p.z = var.first.vertex_A_[2];
    m_line.points.push_back(p);
    ma_line.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();
  }
  for (int j = 0; j < 100 * 6; j++) {
    m_line.color.a = 0.00;
    ma_line.markers.push_back(m_line);
    m_line.id++;
  }
  std_publisher.publish(ma_line);
  m_line.id = 0;
  ma_line.markers.clear();
}

void STDescManager::GenerateSTDescs(
    pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
    std::vector<STDesc> &stds_vec) {

  // step1, voxelization and plane dection
  std::unordered_map<VOXEL_LOC, OctoTree *> voxel_map;
  init_voxel_map(input_cloud, voxel_map);
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr plane_cloud(
      new pcl::PointCloud<pcl::PointXYZINormal>);
  getPlane(voxel_map, plane_cloud);
  // std::cout << "[Description] planes size:" << plane_cloud->size() <<
  // std::endl;
  plane_cloud_vec_.push_back(plane_cloud);

  // step2, build connection for planes in the voxel map
  build_connection(voxel_map);

  // step3, extraction corner points
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr corner_points(
      new pcl::PointCloud<pcl::PointXYZINormal>);
  corner_extractor(voxel_map, input_cloud, corner_points);
  corner_cloud_vec_.push_back(corner_points);
  // std::cout << "[Description] corners size:" << corner_points->size()
  //           << std::endl;

  // step4, generate stable triangle descriptors
  stds_vec.clear();
  build_stdesc(corner_points, stds_vec);
  // std::cout << "[Description] stds size:" << stds_vec.size() << std::endl;

  // step5, clear memory
  for (auto iter = voxel_map.begin(); iter != voxel_map.end(); iter++) {
    delete (iter->second);
  }
  return;
}

void STDescManager::SearchLoop(
    const std::vector<STDesc> &stds_vec, std::pair<int, double> &loop_result,
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> &loop_transform,
    std::vector<std::pair<STDesc, STDesc>> &loop_std_pair) {

  if (stds_vec.size() == 0) {
    ROS_ERROR_STREAM("No STDescs!");
    loop_result = std::pair<int, double>(-1, 0);
    return;
  }
  // step1, select candidates, default number 50
  auto t1 = std::chrono::high_resolution_clock::now();
  std::vector<STDMatchList> candidate_matcher_vec;
  candidate_selector(stds_vec, candidate_matcher_vec);

  auto t2 = std::chrono::high_resolution_clock::now();
  // step2, select best candidates from rough candidates
  double best_score = 0;
  unsigned int best_candidate_id = -1;
  unsigned int triggle_candidate = -1;
  std::pair<Eigen::Vector3d, Eigen::Matrix3d> best_transform;
  std::vector<std::pair<STDesc, STDesc>> best_sucess_match_vec;
  for (size_t i = 0; i < candidate_matcher_vec.size(); i++) {
    double verify_score = -1;
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> relative_pose;
    std::vector<std::pair<STDesc, STDesc>> sucess_match_vec;
    candidate_verify(candidate_matcher_vec[i], verify_score, relative_pose,
                     sucess_match_vec);
    if (verify_score > best_score) {
      best_score = verify_score;
      best_candidate_id = candidate_matcher_vec[i].match_id_.second;
      best_transform = relative_pose;
      best_sucess_match_vec = sucess_match_vec;
      triggle_candidate = i;
    }
  }
  auto t3 = std::chrono::high_resolution_clock::now();

  // std::cout << "[Time] candidate selector: " << time_inc(t2, t1)
  //           << " ms, candidate verify: " << time_inc(t3, t2) << "ms"
  //           << std::endl;

  if (best_score > config_setting_.icp_threshold_) {
    loop_result = std::pair<int, double>(best_candidate_id, best_score);
    loop_transform = best_transform;
    loop_std_pair = best_sucess_match_vec;
    return;
  } else {
    loop_result = std::pair<int, double>(-1, 0);
    return;
  }
}

void STDescManager::AddSTDescs(const std::vector<STDesc> &stds_vec) {
  // update frame id
  current_frame_id_++;
  for (auto single_std : stds_vec) {
    // calculate the position of single std
    STDesc_LOC position;
    position.x = (int)(single_std.side_length_[0] + 0.5);
    position.y = (int)(single_std.side_length_[1] + 0.5);
    position.z = (int)(single_std.side_length_[2] + 0.5);
    position.a = (int)(single_std.angle_[0]);
    position.b = (int)(single_std.angle_[1]);
    position.c = (int)(single_std.angle_[2]);
    auto iter = data_base_.find(position);
    if (iter != data_base_.end()) {
      data_base_[position].push_back(single_std);
    } else {
      std::vector<STDesc> descriptor_vec;
      descriptor_vec.push_back(single_std);
      data_base_[position] = descriptor_vec;
    }
  }
  return;
}

void STDescManager::init_voxel_map(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
    std::unordered_map<VOXEL_LOC, OctoTree *> &voxel_map) {
  uint plsize = input_cloud->size();
  for (uint i = 0; i < plsize; i++) {
    Eigen::Vector3d p_c(input_cloud->points[i].x, input_cloud->points[i].y,
                        input_cloud->points[i].z);
    double loc_xyz[3];
    for (int j = 0; j < 3; j++) {
      loc_xyz[j] = p_c[j] / config_setting_.voxel_size_;
      if (loc_xyz[j] < 0) {
        loc_xyz[j] -= 1.0;
      }
    }
    VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
                       (int64_t)loc_xyz[2]);
    auto iter = voxel_map.find(position);
    if (iter != voxel_map.end()) {
      voxel_map[position]->voxel_points_.push_back(p_c);
    } else {
      OctoTree *octo_tree = new OctoTree(config_setting_);
      voxel_map[position] = octo_tree;
      voxel_map[position]->voxel_points_.push_back(p_c);
    }
  }
  std::vector<std::unordered_map<VOXEL_LOC, OctoTree *>::iterator> iter_list;
  std::vector<size_t> index;
  size_t i = 0;
  for (auto iter = voxel_map.begin(); iter != voxel_map.end(); ++iter) {
    index.push_back(i);
    i++;
    iter_list.push_back(iter);
  }
  // speed up initialization
  // #ifdef MP_EN
  //   omp_set_num_threads(MP_PROC_NUM);
  //   std::cout << "omp num:" << MP_PROC_NUM << std::endl;
  // #pragma omp parallel for
  // #endif
  for (int i = 0; i < index.size(); i++) {
    iter_list[i]->second->init_octo_tree();
  }
  // std::cout << "voxel num:" << index.size() << std::endl;
  // std::for_each(
  //     std::execution::par_unseq, index.begin(), index.end(),
  //     [&](const size_t &i) { iter_list[i]->second->init_octo_tree(); });
}

void STDescManager::build_connection(
    std::unordered_map<VOXEL_LOC, OctoTree *> &voxel_map) {
  for (auto iter = voxel_map.begin(); iter != voxel_map.end(); iter++) {
    if (iter->second->plane_ptr_->is_plane_) {
      OctoTree *current_octo = iter->second;
      for (int i = 0; i < 6; i++) {
        VOXEL_LOC neighbor = iter->first;
        if (i == 0) {
          neighbor.x = neighbor.x + 1;
        } else if (i == 1) {
          neighbor.y = neighbor.y + 1;
        } else if (i == 2) {
          neighbor.z = neighbor.z + 1;
        } else if (i == 3) {
          neighbor.x = neighbor.x - 1;
        } else if (i == 4) {
          neighbor.y = neighbor.y - 1;
        } else if (i == 5) {
          neighbor.z = neighbor.z - 1;
        }
        auto near = voxel_map.find(neighbor);
        if (near == voxel_map.end()) {
          current_octo->is_check_connect_[i] = true;
          current_octo->connect_[i] = false;
        } else {
          if (!current_octo->is_check_connect_[i]) {
            OctoTree *near_octo = near->second;
            current_octo->is_check_connect_[i] = true;
            int j;
            if (i >= 3) {
              j = i - 3;
            } else {
              j = i + 3;
            }
            near_octo->is_check_connect_[j] = true;
            if (near_octo->plane_ptr_->is_plane_) {
              // merge near octo
              Eigen::Vector3d normal_diff = current_octo->plane_ptr_->normal_ -
                                            near_octo->plane_ptr_->normal_;
              Eigen::Vector3d normal_add = current_octo->plane_ptr_->normal_ +
                                           near_octo->plane_ptr_->normal_;
              if (normal_diff.norm() <
                      config_setting_.plane_merge_normal_thre_ ||
                  normal_add.norm() <
                      config_setting_.plane_merge_normal_thre_) {
                current_octo->connect_[i] = true;
                near_octo->connect_[j] = true;
                current_octo->connect_tree_[i] = near_octo;
                near_octo->connect_tree_[j] = current_octo;
              } else {
                current_octo->connect_[i] = false;
                near_octo->connect_[j] = false;
              }
            } else {
              current_octo->connect_[i] = false;
              near_octo->connect_[j] = true;
              near_octo->connect_tree_[j] = current_octo;
            }
          }
        }
      }
    }
  }
}

void STDescManager::getPlane(
    const std::unordered_map<VOXEL_LOC, OctoTree *> &voxel_map,
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr &plane_cloud) {
  for (auto iter = voxel_map.begin(); iter != voxel_map.end(); iter++) {
    if (iter->second->plane_ptr_->is_plane_) {
      pcl::PointXYZINormal pi;
      pi.x = iter->second->plane_ptr_->center_[0];
      pi.y = iter->second->plane_ptr_->center_[1];
      pi.z = iter->second->plane_ptr_->center_[2];
      pi.normal_x = iter->second->plane_ptr_->normal_[0];
      pi.normal_y = iter->second->plane_ptr_->normal_[1];
      pi.normal_z = iter->second->plane_ptr_->normal_[2];
      plane_cloud->push_back(pi);
    }
  }
}

void STDescManager::corner_extractor(
    std::unordered_map<VOXEL_LOC, OctoTree *> &voxel_map,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr &corner_points) {

  pcl::PointCloud<pcl::PointXYZINormal>::Ptr prepare_corner_points(
      new pcl::PointCloud<pcl::PointXYZINormal>);

  // Avoid inconsistent voxel cutting caused by different view point
  std::vector<Eigen::Vector3i> voxel_round;
  for (int x = -1; x <= 1; x++) {
    for (int y = -1; y <= 1; y++) {
      for (int z = -1; z <= 1; z++) {
        Eigen::Vector3i voxel_inc(x, y, z);
        voxel_round.push_back(voxel_inc);
      }
    }
  }
  for (auto iter = voxel_map.begin(); iter != voxel_map.end(); iter++) {
    if (!iter->second->plane_ptr_->is_plane_) {
      VOXEL_LOC current_position = iter->first;
      OctoTree *current_octo = iter->second;
      int connect_index = -1;
      for (int i = 0; i < 6; i++) {
        if (current_octo->connect_[i]) {
          connect_index = i;
          OctoTree *connect_octo = current_octo->connect_tree_[connect_index];
          bool use = false;
          for (int j = 0; j < 6; j++) {
            if (connect_octo->is_check_connect_[j]) {
              if (connect_octo->connect_[j]) {
                use = true;
              }
            }
          }
          // if no plane near the voxel, skip
          if (use == false) {
            continue;
          }
          // only project voxels with points num > 10
          if (current_octo->voxel_points_.size() > 10) {
            Eigen::Vector3d projection_normal =
                current_octo->connect_tree_[connect_index]->plane_ptr_->normal_;
            Eigen::Vector3d projection_center =
                current_octo->connect_tree_[connect_index]->plane_ptr_->center_;
            std::vector<Eigen::Vector3d> proj_points;
            // proj the boundary voxel and nearby voxel onto adjacent plane
            for (auto voxel_inc : voxel_round) {
              VOXEL_LOC connect_project_position = current_position;
              connect_project_position.x += voxel_inc[0];
              connect_project_position.y += voxel_inc[1];
              connect_project_position.z += voxel_inc[2];
              auto iter_near = voxel_map.find(connect_project_position);
              if (iter_near != voxel_map.end()) {
                bool skip_flag = false;
                if (!voxel_map[connect_project_position]
                         ->plane_ptr_->is_plane_) {
                  if (voxel_map[connect_project_position]->is_project_) {
                    for (auto normal : voxel_map[connect_project_position]
                                           ->proj_normal_vec_) {
                      Eigen::Vector3d normal_diff = projection_normal - normal;
                      Eigen::Vector3d normal_add = projection_normal + normal;
                      // check if repeated project
                      if (normal_diff.norm() < 0.5 || normal_add.norm() < 0.5) {
                        skip_flag = true;
                      }
                    }
                  }
                  if (skip_flag) {
                    continue;
                  }
                  for (size_t j = 0; j < voxel_map[connect_project_position]
                                             ->voxel_points_.size();
                       j++) {
                    proj_points.push_back(
                        voxel_map[connect_project_position]->voxel_points_[j]);
                    voxel_map[connect_project_position]->is_project_ = true;
                    voxel_map[connect_project_position]
                        ->proj_normal_vec_.push_back(projection_normal);
                  }
                }
              }
            }
            // here do the 2D projection and corner extraction
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr sub_corner_points(
                new pcl::PointCloud<pcl::PointXYZINormal>);
            extract_corner(projection_center, projection_normal, proj_points,
                           sub_corner_points);
            for (auto pi : sub_corner_points->points) {
              prepare_corner_points->push_back(pi);
            }
          }
        }
      }
    }
  }
  non_maxi_suppression(prepare_corner_points);

  if (config_setting_.maximum_corner_num_ > prepare_corner_points->size()) {
    corner_points = prepare_corner_points;
  } else {
    std::vector<std::pair<double, int>> attach_vec;
    for (size_t i = 0; i < prepare_corner_points->size(); i++) {
      attach_vec.push_back(std::pair<double, int>(
          prepare_corner_points->points[i].intensity, i));
    }
    std::sort(attach_vec.begin(), attach_vec.end(), attach_greater_sort);
    for (size_t i = 0; i < config_setting_.maximum_corner_num_; i++) {
      corner_points->points.push_back(
          prepare_corner_points->points[attach_vec[i].second]);
    }
  }
}

void STDescManager::extract_corner(
    const Eigen::Vector3d &proj_center, const Eigen::Vector3d proj_normal,
    const std::vector<Eigen::Vector3d> proj_points,
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr &corner_points) {

  double resolution = config_setting_.proj_image_resolution_;
  double dis_threshold_min = config_setting_.proj_dis_min_;
  double dis_threshold_max = config_setting_.proj_dis_max_;
  double A = proj_normal[0];
  double B = proj_normal[1];
  double C = proj_normal[2];
  double D = -(A * proj_center[0] + B * proj_center[1] + C * proj_center[2]);
  Eigen::Vector3d x_axis(1, 1, 0);
  if (C != 0) {
    x_axis[2] = -(A + B) / C;
  } else if (B != 0) {
    x_axis[1] = -A / B;
  } else {
    x_axis[0] = 0;
    x_axis[1] = 1;
  }
  x_axis.normalize();
  Eigen::Vector3d y_axis = proj_normal.cross(x_axis);
  y_axis.normalize();
  double ax = x_axis[0];
  double bx = x_axis[1];
  double cx = x_axis[2];
  double dx =
      -(ax * proj_center[0] + bx * proj_center[1] + cx * proj_center[2]);
  double ay = y_axis[0];
  double by = y_axis[1];
  double cy = y_axis[2];
  double dy =
      -(ay * proj_center[0] + by * proj_center[1] + cy * proj_center[2]);
  std::vector<Eigen::Vector2d> point_list_2d;
  for (size_t i = 0; i < proj_points.size(); i++) {
    double x = proj_points[i][0];
    double y = proj_points[i][1];
    double z = proj_points[i][2];
    double dis = fabs(x * A + y * B + z * C + D);
    if (dis < dis_threshold_min || dis > dis_threshold_max) {
      continue;
    }
    Eigen::Vector3d cur_project;

    cur_project[0] = (-A * (B * y + C * z + D) + x * (B * B + C * C)) /
                     (A * A + B * B + C * C);
    cur_project[1] = (-B * (A * x + C * z + D) + y * (A * A + C * C)) /
                     (A * A + B * B + C * C);
    cur_project[2] = (-C * (A * x + B * y + D) + z * (A * A + B * B)) /
                     (A * A + B * B + C * C);
    pcl::PointXYZ p;
    p.x = cur_project[0];
    p.y = cur_project[1];
    p.z = cur_project[2];
    double project_x =
        cur_project[0] * ay + cur_project[1] * by + cur_project[2] * cy + dy;
    double project_y =
        cur_project[0] * ax + cur_project[1] * bx + cur_project[2] * cx + dx;
    Eigen::Vector2d p_2d(project_x, project_y);
    point_list_2d.push_back(p_2d);
  }
  double min_x = 10;
  double max_x = -10;
  double min_y = 10;
  double max_y = -10;
  if (point_list_2d.size() <= 5) {
    return;
  }
  for (auto pi : point_list_2d) {
    if (pi[0] < min_x) {
      min_x = pi[0];
    }
    if (pi[0] > max_x) {
      max_x = pi[0];
    }
    if (pi[1] < min_y) {
      min_y = pi[1];
    }
    if (pi[1] > max_y) {
      max_y = pi[1];
    }
  }
  // segment project cloud with a fixed resolution
  int segmen_base_num = 5;
  double segmen_len = segmen_base_num * resolution;
  int x_segment_num = (max_x - min_x) / segmen_len + 1;
  int y_segment_num = (max_y - min_y) / segmen_len + 1;
  int x_axis_len = (int)((max_x - min_x) / resolution + segmen_base_num);
  int y_axis_len = (int)((max_y - min_y) / resolution + segmen_base_num);
  std::vector<Eigen::Vector2d> img_container[x_axis_len][y_axis_len];
  double img_count_array[x_axis_len][y_axis_len] = {0};
  double gradient_array[x_axis_len][y_axis_len] = {0};
  double mean_x_array[x_axis_len][y_axis_len] = {0};
  double mean_y_array[x_axis_len][y_axis_len] = {0};
  for (int x = 0; x < x_axis_len; x++) {
    for (int y = 0; y < y_axis_len; y++) {
      img_count_array[x][y] = 0;
      mean_x_array[x][y] = 0;
      mean_y_array[x][y] = 0;
      gradient_array[x][y] = 0;
      std::vector<Eigen::Vector2d> single_container;
      img_container[x][y] = single_container;
    }
  }
  for (size_t i = 0; i < point_list_2d.size(); i++) {
    int x_index = (int)((point_list_2d[i][0] - min_x) / resolution);
    int y_index = (int)((point_list_2d[i][1] - min_y) / resolution);
    mean_x_array[x_index][y_index] += point_list_2d[i][0];
    mean_y_array[x_index][y_index] += point_list_2d[i][1];
    img_count_array[x_index][y_index]++;
    img_container[x_index][y_index].push_back(point_list_2d[i]);
  }
  // calc gradient
  for (int x = 0; x < x_axis_len; x++) {
    for (int y = 0; y < y_axis_len; y++) {
      double gradient = 0;
      int cnt = 0;
      int inc = 1;
      for (int x_inc = -inc; x_inc <= inc; x_inc++) {
        for (int y_inc = -inc; y_inc <= inc; y_inc++) {
          int xx = x + x_inc;
          int yy = y + y_inc;
          if (xx >= 0 && xx < x_axis_len && yy >= 0 && yy < y_axis_len) {
            if (xx != x || yy != y) {
              if (img_count_array[xx][yy] >= 0) {
                gradient += img_count_array[x][y] - img_count_array[xx][yy];
                cnt++;
              }
            }
          }
        }
      }
      if (cnt != 0) {
        gradient_array[x][y] = gradient * 1.0 / cnt;
      } else {
        gradient_array[x][y] = 0;
      }
    }
  }
  // extract corner by gradient
  std::vector<int> max_gradient_vec;
  std::vector<int> max_gradient_x_index_vec;
  std::vector<int> max_gradient_y_index_vec;
  for (int x_segment_index = 0; x_segment_index < x_segment_num;
       x_segment_index++) {
    for (int y_segment_index = 0; y_segment_index < y_segment_num;
         y_segment_index++) {
      double max_gradient = 0;
      int max_gradient_x_index = -10;
      int max_gradient_y_index = -10;
      for (int x_index = x_segment_index * segmen_base_num;
           x_index < (x_segment_index + 1) * segmen_base_num; x_index++) {
        for (int y_index = y_segment_index * segmen_base_num;
             y_index < (y_segment_index + 1) * segmen_base_num; y_index++) {
          if (img_count_array[x_index][y_index] > max_gradient) {
            max_gradient = img_count_array[x_index][y_index];
            max_gradient_x_index = x_index;
            max_gradient_y_index = y_index;
          }
        }
      }
      if (max_gradient >= config_setting_.corner_thre_) {
        max_gradient_vec.push_back(max_gradient);
        max_gradient_x_index_vec.push_back(max_gradient_x_index);
        max_gradient_y_index_vec.push_back(max_gradient_y_index);
      }
    }
  }
  // filter out line
  // calc line or not
  std::vector<Eigen::Vector2i> direction_list;
  Eigen::Vector2i d(0, 1);
  direction_list.push_back(d);
  d << 1, 0;
  direction_list.push_back(d);
  d << 1, 1;
  direction_list.push_back(d);
  d << 1, -1;
  direction_list.push_back(d);
  for (size_t i = 0; i < max_gradient_vec.size(); i++) {
    bool is_add = true;
    for (int j = 0; j < 4; j++) {
      Eigen::Vector2i p(max_gradient_x_index_vec[i],
                        max_gradient_y_index_vec[i]);
      Eigen::Vector2i p1 = p + direction_list[j];
      Eigen::Vector2i p2 = p - direction_list[j];
      int threshold = img_count_array[p[0]][p[1]] / 2;
      if (img_count_array[p1[0]][p1[1]] >= threshold &&
          img_count_array[p2[0]][p2[1]] >= threshold) {
        // is_add = false;
      } else {
        continue;
      }
    }
    if (is_add) {
      double px = mean_x_array[max_gradient_x_index_vec[i]]
                              [max_gradient_y_index_vec[i]] /
                  img_count_array[max_gradient_x_index_vec[i]]
                                 [max_gradient_y_index_vec[i]];
      double py = mean_y_array[max_gradient_x_index_vec[i]]
                              [max_gradient_y_index_vec[i]] /
                  img_count_array[max_gradient_x_index_vec[i]]
                                 [max_gradient_y_index_vec[i]];
      // reproject on 3D space
      Eigen::Vector3d coord = py * x_axis + px * y_axis + proj_center;
      pcl::PointXYZINormal pi;
      pi.x = coord[0];
      pi.y = coord[1];
      pi.z = coord[2];
      pi.intensity = max_gradient_vec[i];
      pi.normal_x = proj_normal[0];
      pi.normal_y = proj_normal[1];
      pi.normal_z = proj_normal[2];
      corner_points->points.push_back(pi);
    }
  }
  return;
}

void STDescManager::non_maxi_suppression(
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr &corner_points) {
  std::vector<bool> is_add_vec;
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr prepare_key_cloud(
      new pcl::PointCloud<pcl::PointXYZINormal>);
  pcl::KdTreeFLANN<pcl::PointXYZINormal> kd_tree;
  for (auto pi : corner_points->points) {
    prepare_key_cloud->push_back(pi);
    is_add_vec.push_back(true);
  }
  kd_tree.setInputCloud(prepare_key_cloud);
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  double radius = config_setting_.non_max_suppression_radius_;
  for (size_t i = 0; i < prepare_key_cloud->size(); i++) {
    pcl::PointXYZINormal searchPoint = prepare_key_cloud->points[i];
    if (kd_tree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch,
                             pointRadiusSquaredDistance) > 0) {
      Eigen::Vector3d pi(searchPoint.x, searchPoint.y, searchPoint.z);
      for (size_t j = 0; j < pointIdxRadiusSearch.size(); ++j) {
        Eigen::Vector3d pj(
            prepare_key_cloud->points[pointIdxRadiusSearch[j]].x,
            prepare_key_cloud->points[pointIdxRadiusSearch[j]].y,
            prepare_key_cloud->points[pointIdxRadiusSearch[j]].z);
        if (pointIdxRadiusSearch[j] == i) {
          continue;
        }
        if (prepare_key_cloud->points[i].intensity <=
            prepare_key_cloud->points[pointIdxRadiusSearch[j]].intensity) {
          is_add_vec[i] = false;
        }
      }
    }
  }
  corner_points->clear();
  for (size_t i = 0; i < is_add_vec.size(); i++) {
    if (is_add_vec[i]) {
      corner_points->points.push_back(prepare_key_cloud->points[i]);
    }
  }
  return;
}

void STDescManager::build_stdesc(
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &corner_points,
    std::vector<STDesc> &stds_vec) {
  stds_vec.clear();
  double scale = 1.0 / config_setting_.std_side_resolution_;
  int near_num = config_setting_.descriptor_near_num_;
  double max_dis_threshold = config_setting_.descriptor_max_len_;
  double min_dis_threshold = config_setting_.descriptor_min_len_;
  std::unordered_map<VOXEL_LOC, bool> feat_map;
  pcl::KdTreeFLANN<pcl::PointXYZINormal>::Ptr kd_tree(
      new pcl::KdTreeFLANN<pcl::PointXYZINormal>);
  kd_tree->setInputCloud(corner_points);
  std::vector<int> pointIdxNKNSearch(near_num);
  std::vector<float> pointNKNSquaredDistance(near_num);
  // Search N nearest corner points to form stds.
  for (size_t i = 0; i < corner_points->size(); i++) {
    pcl::PointXYZINormal searchPoint = corner_points->points[i];
    if (kd_tree->nearestKSearch(searchPoint, near_num, pointIdxNKNSearch,
                                pointNKNSquaredDistance) > 0) {
      for (int m = 1; m < near_num - 1; m++) {
        for (int n = m + 1; n < near_num; n++) {
          pcl::PointXYZINormal p1 = searchPoint;
          pcl::PointXYZINormal p2 = corner_points->points[pointIdxNKNSearch[m]];
          pcl::PointXYZINormal p3 = corner_points->points[pointIdxNKNSearch[n]];
          Eigen::Vector3d normal_inc1(p1.normal_x - p2.normal_x,
                                      p1.normal_y - p2.normal_y,
                                      p1.normal_z - p2.normal_z);
          Eigen::Vector3d normal_inc2(p3.normal_x - p2.normal_x,
                                      p3.normal_y - p2.normal_y,
                                      p3.normal_z - p2.normal_z);
          Eigen::Vector3d normal_add1(p1.normal_x + p2.normal_x,
                                      p1.normal_y + p2.normal_y,
                                      p1.normal_z + p2.normal_z);
          Eigen::Vector3d normal_add2(p3.normal_x + p2.normal_x,
                                      p3.normal_y + p2.normal_y,
                                      p3.normal_z + p2.normal_z);
          double a = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) +
                          pow(p1.z - p2.z, 2));
          double b = sqrt(pow(p1.x - p3.x, 2) + pow(p1.y - p3.y, 2) +
                          pow(p1.z - p3.z, 2));
          double c = sqrt(pow(p3.x - p2.x, 2) + pow(p3.y - p2.y, 2) +
                          pow(p3.z - p2.z, 2));
          if (a > max_dis_threshold || b > max_dis_threshold ||
              c > max_dis_threshold || a < min_dis_threshold ||
              b < min_dis_threshold || c < min_dis_threshold) {
            continue;
          }
          // re-range the vertex by the side length
          double temp;
          Eigen::Vector3d A, B, C;
          Eigen::Vector3i l1, l2, l3;
          Eigen::Vector3i l_temp;
          l1 << 1, 2, 0;
          l2 << 1, 0, 3;
          l3 << 0, 2, 3;
          if (a > b) {
            temp = a;
            a = b;
            b = temp;
            l_temp = l1;
            l1 = l2;
            l2 = l_temp;
          }
          if (b > c) {
            temp = b;
            b = c;
            c = temp;
            l_temp = l2;
            l2 = l3;
            l3 = l_temp;
          }
          if (a > b) {
            temp = a;
            a = b;
            b = temp;
            l_temp = l1;
            l1 = l2;
            l2 = l_temp;
          }
          // check augnmentation
          pcl::PointXYZ d_p;
          d_p.x = a * 1000;
          d_p.y = b * 1000;
          d_p.z = c * 1000;
          VOXEL_LOC position((int64_t)d_p.x, (int64_t)d_p.y, (int64_t)d_p.z);
          auto iter = feat_map.find(position);
          Eigen::Vector3d normal_1, normal_2, normal_3;
          if (iter == feat_map.end()) {
            Eigen::Vector3d vertex_attached;
            if (l1[0] == l2[0]) {
              A << p1.x, p1.y, p1.z;
              normal_1 << p1.normal_x, p1.normal_y, p1.normal_z;
              vertex_attached[0] = p1.intensity;
            } else if (l1[1] == l2[1]) {
              A << p2.x, p2.y, p2.z;
              normal_1 << p2.normal_x, p2.normal_y, p2.normal_z;
              vertex_attached[0] = p2.intensity;
            } else {
              A << p3.x, p3.y, p3.z;
              normal_1 << p3.normal_x, p3.normal_y, p3.normal_z;
              vertex_attached[0] = p3.intensity;
            }
            if (l1[0] == l3[0]) {
              B << p1.x, p1.y, p1.z;
              normal_2 << p1.normal_x, p1.normal_y, p1.normal_z;
              vertex_attached[1] = p1.intensity;
            } else if (l1[1] == l3[1]) {
              B << p2.x, p2.y, p2.z;
              normal_2 << p2.normal_x, p2.normal_y, p2.normal_z;
              vertex_attached[1] = p2.intensity;
            } else {
              B << p3.x, p3.y, p3.z;
              normal_2 << p3.normal_x, p3.normal_y, p3.normal_z;
              vertex_attached[1] = p3.intensity;
            }
            if (l2[0] == l3[0]) {
              C << p1.x, p1.y, p1.z;
              normal_3 << p1.normal_x, p1.normal_y, p1.normal_z;
              vertex_attached[2] = p1.intensity;
            } else if (l2[1] == l3[1]) {
              C << p2.x, p2.y, p2.z;
              normal_3 << p2.normal_x, p2.normal_y, p2.normal_z;
              vertex_attached[2] = p2.intensity;
            } else {
              C << p3.x, p3.y, p3.z;
              normal_3 << p3.normal_x, p3.normal_y, p3.normal_z;
              vertex_attached[2] = p3.intensity;
            }
            STDesc single_descriptor;
            single_descriptor.vertex_A_ = A;
            single_descriptor.vertex_B_ = B;
            single_descriptor.vertex_C_ = C;
            single_descriptor.center_ = (A + B + C) / 3;
            single_descriptor.vertex_attached_ = vertex_attached;
            single_descriptor.side_length_ << scale * a, scale * b, scale * c;
            single_descriptor.angle_[0] = fabs(5 * normal_1.dot(normal_2));
            single_descriptor.angle_[1] = fabs(5 * normal_1.dot(normal_3));
            single_descriptor.angle_[2] = fabs(5 * normal_3.dot(normal_2));
            // single_descriptor.angle << 0, 0, 0;
            single_descriptor.frame_id_ = current_frame_id_;
            Eigen::Matrix3d triangle_positon;
            feat_map[position] = true;
            stds_vec.push_back(single_descriptor);
          }
        }
      }
    }
  }
};

void STDescManager::candidate_selector(
    const std::vector<STDesc> &stds_vec,
    std::vector<STDMatchList> &candidate_matcher_vec) {
  double match_array[MAX_FRAME_N] = {0};
  std::vector<std::pair<STDesc, STDesc>> match_vec;
  std::vector<int> match_index_vec;
  std::vector<Eigen::Vector3i> voxel_round;
  for (int x = -1; x <= 1; x++) {
    for (int y = -1; y <= 1; y++) {
      for (int z = -1; z <= 1; z++) {
        Eigen::Vector3i voxel_inc(x, y, z);
        voxel_round.push_back(voxel_inc);
      }
    }
  }

  std::vector<bool> useful_match(stds_vec.size());
  std::vector<std::vector<size_t>> useful_match_index(stds_vec.size());
  std::vector<std::vector<STDesc_LOC>> useful_match_position(stds_vec.size());
  std::vector<size_t> index(stds_vec.size());
  for (size_t i = 0; i < index.size(); ++i) {
    index[i] = i;
    useful_match[i] = false;
  }
  // speed up matching
  int dis_match_cnt = 0;
  int final_match_cnt = 0;
#ifdef MP_EN
  omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
  for (size_t i = 0; i < stds_vec.size(); i++) {
    STDesc src_std = stds_vec[i];
    STDesc_LOC position;
    int best_index = 0;
    STDesc_LOC best_position;
    double dis_threshold =
        src_std.side_length_.norm() * config_setting_.rough_dis_threshold_;
    for (auto voxel_inc : voxel_round) {
      position.x = (int)(src_std.side_length_[0] + voxel_inc[0]);
      position.y = (int)(src_std.side_length_[1] + voxel_inc[1]);
      position.z = (int)(src_std.side_length_[2] + voxel_inc[2]);
      Eigen::Vector3d voxel_center((double)position.x + 0.5,
                                   (double)position.y + 0.5,
                                   (double)position.z + 0.5);
      if ((src_std.side_length_ - voxel_center).norm() < 1.5) {
        auto iter = data_base_.find(position);
        if (iter != data_base_.end()) {
          for (size_t j = 0; j < data_base_[position].size(); j++) {
            if ((src_std.frame_id_ - data_base_[position][j].frame_id_) >
                config_setting_.skip_near_num_) {
              double dis =
                  (src_std.side_length_ - data_base_[position][j].side_length_)
                      .norm();
              // rough filter with side lengths
              if (dis < dis_threshold) {
                dis_match_cnt++;
                // rough filter with vertex attached info
                double vertex_attach_diff =
                    2.0 *
                    (src_std.vertex_attached_ -
                     data_base_[position][j].vertex_attached_)
                        .norm() /
                    (src_std.vertex_attached_ +
                     data_base_[position][j].vertex_attached_)
                        .norm();
                // std::cout << "vertex diff:" << vertex_attach_diff <<
                // std::endl;
                if (vertex_attach_diff <
                    config_setting_.vertex_diff_threshold_) {
                  final_match_cnt++;
                  useful_match[i] = true;
                  useful_match_position[i].push_back(position);
                  useful_match_index[i].push_back(j);
                }
              }
            }
          }
        }
      }
    }
  }
  // std::cout << "dis match num:" << dis_match_cnt
  //           << ", final match num:" << final_match_cnt << std::endl;

  // record match index
  std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i>>
      index_recorder;
  for (size_t i = 0; i < useful_match.size(); i++) {
    if (useful_match[i]) {
      for (size_t j = 0; j < useful_match_index[i].size(); j++) {
        match_array[data_base_[useful_match_position[i][j]]
                              [useful_match_index[i][j]]
                                  .frame_id_] += 1;
        Eigen::Vector2i match_index(i, j);
        index_recorder.push_back(match_index);
        match_index_vec.push_back(
            data_base_[useful_match_position[i][j]][useful_match_index[i][j]]
                .frame_id_);
      }
    }
  }

  // select candidate according to the matching score
  for (int cnt = 0; cnt < config_setting_.candidate_num_; cnt++) {
    double max_vote = 1;
    int max_vote_index = -1;
    for (int i = 0; i < MAX_FRAME_N; i++) {
      if (match_array[i] > max_vote) {
        max_vote = match_array[i];
        max_vote_index = i;
      }
    }
    STDMatchList match_triangle_list;
    if (max_vote_index >= 0 && max_vote >= 5) {
      match_array[max_vote_index] = 0;
      match_triangle_list.match_id_.first = current_frame_id_;
      match_triangle_list.match_id_.second = max_vote_index;
      for (size_t i = 0; i < index_recorder.size(); i++) {
        if (match_index_vec[i] == max_vote_index) {
          std::pair<STDesc, STDesc> single_match_pair;
          single_match_pair.first = stds_vec[index_recorder[i][0]];
          single_match_pair.second =
              data_base_[useful_match_position[index_recorder[i][0]]
                                              [index_recorder[i][1]]]
                        [useful_match_index[index_recorder[i][0]]
                                           [index_recorder[i][1]]];
          match_triangle_list.match_list_.push_back(single_match_pair);
        }
      }
      candidate_matcher_vec.push_back(match_triangle_list);
    } else {
      break;
    }
  }
}

// Get the best candidate frame by geometry check
void STDescManager::candidate_verify(
    const STDMatchList &candidate_matcher, double &verify_score,
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> &relative_pose,
    std::vector<std::pair<STDesc, STDesc>> &sucess_match_vec) {
  sucess_match_vec.clear();
  int skip_len = (int)(candidate_matcher.match_list_.size() / 50) + 1;
  int use_size = candidate_matcher.match_list_.size() / skip_len;
  double dis_threshold = 3.0;
  std::vector<size_t> index(use_size);
  std::vector<int> vote_list(use_size);
  for (size_t i = 0; i < index.size(); i++) {
    index[i] = i;
  }
  std::mutex mylock;

#ifdef MP_EN
  omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
  for (size_t i = 0; i < use_size; i++) {
    auto single_pair = candidate_matcher.match_list_[i * skip_len];
    int vote = 0;
    Eigen::Matrix3d test_rot;
    Eigen::Vector3d test_t;
    triangle_solver(single_pair, test_t, test_rot);
    for (size_t j = 0; j < candidate_matcher.match_list_.size(); j++) {
      auto verify_pair = candidate_matcher.match_list_[j];
      Eigen::Vector3d A = verify_pair.first.vertex_A_;
      Eigen::Vector3d A_transform = test_rot * A + test_t;
      Eigen::Vector3d B = verify_pair.first.vertex_B_;
      Eigen::Vector3d B_transform = test_rot * B + test_t;
      Eigen::Vector3d C = verify_pair.first.vertex_C_;
      Eigen::Vector3d C_transform = test_rot * C + test_t;
      double dis_A = (A_transform - verify_pair.second.vertex_A_).norm();
      double dis_B = (B_transform - verify_pair.second.vertex_B_).norm();
      double dis_C = (C_transform - verify_pair.second.vertex_C_).norm();
      if (dis_A < dis_threshold && dis_B < dis_threshold &&
          dis_C < dis_threshold) {
        vote++;
      }
    }
    mylock.lock();
    vote_list[i] = vote;
    mylock.unlock();
  }
  int max_vote_index = 0;
  int max_vote = 0;
  for (size_t i = 0; i < vote_list.size(); i++) {
    if (max_vote < vote_list[i]) {
      max_vote_index = i;
      max_vote = vote_list[i];
    }
  }
  if (max_vote >= 4) {
    auto best_pair = candidate_matcher.match_list_[max_vote_index * skip_len];
    int vote = 0;
    Eigen::Matrix3d best_rot;
    Eigen::Vector3d best_t;
    triangle_solver(best_pair, best_t, best_rot);
    relative_pose.first = best_t;
    relative_pose.second = best_rot;
    for (size_t j = 0; j < candidate_matcher.match_list_.size(); j++) {
      auto verify_pair = candidate_matcher.match_list_[j];
      Eigen::Vector3d A = verify_pair.first.vertex_A_;
      Eigen::Vector3d A_transform = best_rot * A + best_t;
      Eigen::Vector3d B = verify_pair.first.vertex_B_;
      Eigen::Vector3d B_transform = best_rot * B + best_t;
      Eigen::Vector3d C = verify_pair.first.vertex_C_;
      Eigen::Vector3d C_transform = best_rot * C + best_t;
      double dis_A = (A_transform - verify_pair.second.vertex_A_).norm();
      double dis_B = (B_transform - verify_pair.second.vertex_B_).norm();
      double dis_C = (C_transform - verify_pair.second.vertex_C_).norm();
      if (dis_A < dis_threshold && dis_B < dis_threshold &&
          dis_C < dis_threshold) {
        sucess_match_vec.push_back(verify_pair);
      }
    }
    verify_score = plane_geometric_verify(
        plane_cloud_vec_.back(),
        plane_cloud_vec_[candidate_matcher.match_id_.second], relative_pose);
  } else {
    verify_score = -1;
  }
}

void STDescManager::triangle_solver(std::pair<STDesc, STDesc> &std_pair,
                                    Eigen::Vector3d &t, Eigen::Matrix3d &rot) {
  Eigen::Matrix3d src = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d ref = Eigen::Matrix3d::Zero();
  src.col(0) = std_pair.first.vertex_A_ - std_pair.first.center_;
  src.col(1) = std_pair.first.vertex_B_ - std_pair.first.center_;
  src.col(2) = std_pair.first.vertex_C_ - std_pair.first.center_;
  ref.col(0) = std_pair.second.vertex_A_ - std_pair.second.center_;
  ref.col(1) = std_pair.second.vertex_B_ - std_pair.second.center_;
  ref.col(2) = std_pair.second.vertex_C_ - std_pair.second.center_;
  Eigen::Matrix3d covariance = src * ref.transpose();
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(covariance, Eigen::ComputeThinU |
                                                        Eigen::ComputeThinV);
  Eigen::Matrix3d V = svd.matrixV();
  Eigen::Matrix3d U = svd.matrixU();
  rot = V * U.transpose();
  if (rot.determinant() < 0) {
    Eigen::Matrix3d K;
    K << 1, 0, 0, 0, 1, 0, 0, 0, -1;
    rot = V * K * U.transpose();
  }
  t = -rot * std_pair.first.center_ + std_pair.second.center_;
}

double STDescManager::plane_geometric_verify(
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &source_cloud,
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &target_cloud,
    const std::pair<Eigen::Vector3d, Eigen::Matrix3d> &transform) {
  Eigen::Vector3d t = transform.first;
  Eigen::Matrix3d rot = transform.second;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kd_tree(
      new pcl::KdTreeFLANN<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (size_t i = 0; i < target_cloud->size(); i++) {
    pcl::PointXYZ pi;
    pi.x = target_cloud->points[i].x;
    pi.y = target_cloud->points[i].y;
    pi.z = target_cloud->points[i].z;
    input_cloud->push_back(pi);
  }
  kd_tree->setInputCloud(input_cloud);
  std::vector<int> pointIdxNKNSearch(1);
  std::vector<float> pointNKNSquaredDistance(1);
  double useful_match = 0;
  double normal_threshold = config_setting_.normal_threshold_;
  double dis_threshold = config_setting_.dis_threshold_;
  for (size_t i = 0; i < source_cloud->size(); i++) {
    pcl::PointXYZINormal searchPoint = source_cloud->points[i];
    pcl::PointXYZ use_search_point;
    use_search_point.x = searchPoint.x;
    use_search_point.y = searchPoint.y;
    use_search_point.z = searchPoint.z;
    Eigen::Vector3d pi(searchPoint.x, searchPoint.y, searchPoint.z);
    pi = rot * pi + t;
    use_search_point.x = pi[0];
    use_search_point.y = pi[1];
    use_search_point.z = pi[2];
    Eigen::Vector3d ni(searchPoint.normal_x, searchPoint.normal_y,
                       searchPoint.normal_z);
    ni = rot * ni;
    int K = 3;
    if (kd_tree->nearestKSearch(use_search_point, K, pointIdxNKNSearch,
                                pointNKNSquaredDistance) > 0) {
      for (size_t j = 0; j < K; j++) {
        pcl::PointXYZINormal nearstPoint =
            target_cloud->points[pointIdxNKNSearch[j]];
        Eigen::Vector3d tpi(nearstPoint.x, nearstPoint.y, nearstPoint.z);
        Eigen::Vector3d tni(nearstPoint.normal_x, nearstPoint.normal_y,
                            nearstPoint.normal_z);
        Eigen::Vector3d normal_inc = ni - tni;
        Eigen::Vector3d normal_add = ni + tni;
        double point_to_plane = fabs(tni.transpose() * (pi - tpi));
        if ((normal_inc.norm() < normal_threshold ||
             normal_add.norm() < normal_threshold) &&
            point_to_plane < dis_threshold) {
          useful_match++;
          break;
        }
      }
    }
  }
  return useful_match / source_cloud->size();
}

void STDescManager::PlaneGeomrtricIcp(
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &source_cloud,
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &target_cloud,
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> &transform) {
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kd_tree(
      new pcl::KdTreeFLANN<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (size_t i = 0; i < target_cloud->size(); i++) {
    pcl::PointXYZ pi;
    pi.x = target_cloud->points[i].x;
    pi.y = target_cloud->points[i].y;
    pi.z = target_cloud->points[i].z;
    input_cloud->push_back(pi);
  }
  kd_tree->setInputCloud(input_cloud);
  ceres::Manifold *quaternion_manifold = new ceres::EigenQuaternionManifold;
  ceres::Problem problem;
  ceres::LossFunction *loss_function = nullptr;
  Eigen::Matrix3d rot = transform.second;
  Eigen::Quaterniond q(rot);
  Eigen::Vector3d t = transform.first;
  double para_q[4] = {q.x(), q.y(), q.z(), q.w()};
  double para_t[3] = {t(0), t(1), t(2)};
  problem.AddParameterBlock(para_q, 4, quaternion_manifold);
  problem.AddParameterBlock(para_t, 3);
  Eigen::Map<Eigen::Quaterniond> q_last_curr(para_q);
  Eigen::Map<Eigen::Vector3d> t_last_curr(para_t);
  std::vector<int> pointIdxNKNSearch(1);
  std::vector<float> pointNKNSquaredDistance(1);
  int useful_match = 0;
  for (size_t i = 0; i < source_cloud->size(); i++) {
    pcl::PointXYZINormal searchPoint = source_cloud->points[i];
    Eigen::Vector3d pi(searchPoint.x, searchPoint.y, searchPoint.z);
    pi = rot * pi + t;
    pcl::PointXYZ use_search_point;
    use_search_point.x = pi[0];
    use_search_point.y = pi[1];
    use_search_point.z = pi[2];
    Eigen::Vector3d ni(searchPoint.normal_x, searchPoint.normal_y,
                       searchPoint.normal_z);
    ni = rot * ni;
    if (kd_tree->nearestKSearch(use_search_point, 1, pointIdxNKNSearch,
                                pointNKNSquaredDistance) > 0) {
      pcl::PointXYZINormal nearstPoint =
          target_cloud->points[pointIdxNKNSearch[0]];
      Eigen::Vector3d tpi(nearstPoint.x, nearstPoint.y, nearstPoint.z);
      Eigen::Vector3d tni(nearstPoint.normal_x, nearstPoint.normal_y,
                          nearstPoint.normal_z);
      Eigen::Vector3d normal_inc = ni - tni;
      Eigen::Vector3d normal_add = ni + tni;
      double point_to_point_dis = (pi - tpi).norm();
      double point_to_plane = fabs(tni.transpose() * (pi - tpi));
      if ((normal_inc.norm() < config_setting_.normal_threshold_ ||
           normal_add.norm() < config_setting_.normal_threshold_) &&
          point_to_plane < config_setting_.dis_threshold_ &&
          point_to_point_dis < 3) {
        useful_match++;
        ceres::CostFunction *cost_function;
        Eigen::Vector3d curr_point(source_cloud->points[i].x,
                                   source_cloud->points[i].y,
                                   source_cloud->points[i].z);
        Eigen::Vector3d curr_normal(source_cloud->points[i].normal_x,
                                    source_cloud->points[i].normal_y,
                                    source_cloud->points[i].normal_z);

        cost_function = PlaneSolver::Create(curr_point, curr_normal, tpi, tni);
        problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
      }
    }
  }
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.max_num_iterations = 100;
  options.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  Eigen::Quaterniond q_opt(para_q[3], para_q[0], para_q[1], para_q[2]);
  rot = q_opt.toRotationMatrix();
  t << t_last_curr(0), t_last_curr(1), t_last_curr(2);
  transform.first = t;
  transform.second = rot;
  // std::cout << "useful match for icp:" << useful_match << std::endl;
}

void OctoTree::init_plane() {
  plane_ptr_->covariance_ = Eigen::Matrix3d::Zero();
  plane_ptr_->center_ = Eigen::Vector3d::Zero();
  plane_ptr_->normal_ = Eigen::Vector3d::Zero();
  plane_ptr_->points_size_ = voxel_points_.size();
  plane_ptr_->radius_ = 0;
  for (auto pi : voxel_points_) {
    plane_ptr_->covariance_ += pi * pi.transpose();
    plane_ptr_->center_ += pi;
  }
  plane_ptr_->center_ = plane_ptr_->center_ / plane_ptr_->points_size_;
  plane_ptr_->covariance_ =
      plane_ptr_->covariance_ / plane_ptr_->points_size_ -
      plane_ptr_->center_ * plane_ptr_->center_.transpose();
  Eigen::EigenSolver<Eigen::Matrix3d> es(plane_ptr_->covariance_);
  Eigen::Matrix3cd evecs = es.eigenvectors();
  Eigen::Vector3cd evals = es.eigenvalues();
  Eigen::Vector3d evalsReal;
  evalsReal = evals.real();
  Eigen::Matrix3d::Index evalsMin, evalsMax;
  evalsReal.rowwise().sum().minCoeff(&evalsMin);
  evalsReal.rowwise().sum().maxCoeff(&evalsMax);
  int evalsMid = 3 - evalsMin - evalsMax;
  if (evalsReal(evalsMin) < config_setting_.plane_detection_thre_) {
    plane_ptr_->normal_ << evecs.real()(0, evalsMin), evecs.real()(1, evalsMin),
        evecs.real()(2, evalsMin);
    plane_ptr_->min_eigen_value_ = evalsReal(evalsMin);
    plane_ptr_->radius_ = sqrt(evalsReal(evalsMax));
    plane_ptr_->is_plane_ = true;

    plane_ptr_->intercept_ = -(plane_ptr_->normal_(0) * plane_ptr_->center_(0) +
                               plane_ptr_->normal_(1) * plane_ptr_->center_(1) +
                               plane_ptr_->normal_(2) * plane_ptr_->center_(2));
    plane_ptr_->p_center_.x = plane_ptr_->center_(0);
    plane_ptr_->p_center_.y = plane_ptr_->center_(1);
    plane_ptr_->p_center_.z = plane_ptr_->center_(2);
    plane_ptr_->p_center_.normal_x = plane_ptr_->normal_(0);
    plane_ptr_->p_center_.normal_y = plane_ptr_->normal_(1);
    plane_ptr_->p_center_.normal_z = plane_ptr_->normal_(2);
  } else {
    plane_ptr_->is_plane_ = false;
  }
}

void OctoTree::init_octo_tree() {
  if (voxel_points_.size() > config_setting_.voxel_init_num_) {
    init_plane();
  }
}