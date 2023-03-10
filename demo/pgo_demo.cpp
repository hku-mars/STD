#include "include/STDesc.h"
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// Read KITTI data
std::vector<float> read_lidar_data(const std::string lidar_data_path) {
  std::ifstream lidar_data_file;
  lidar_data_file.open(lidar_data_path,
                       std::ifstream::in | std::ifstream::binary);
  if (!lidar_data_file) {
    std::cout << "Read End..." << std::endl;
    std::vector<float> nan_data;
    return nan_data;
  }
  lidar_data_file.seekg(0, std::ios::end);
  const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
  lidar_data_file.seekg(0, std::ios::beg);

  std::vector<float> lidar_data_buffer(num_elements);
  lidar_data_file.read(reinterpret_cast<char *>(&lidar_data_buffer[0]),
                       num_elements * sizeof(float));
  return lidar_data_buffer;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pgo_demo");
  ros::NodeHandle nh;
  std::string lidar_path = "";
  std::string pose_path = "";
  nh.param<std::string>("lidar_path", lidar_path, "");
  nh.param<std::string>("pose_path", pose_path, "");

  ConfigSetting config_setting;
  read_parameters(nh, config_setting);

  ros::Publisher pubOdomAftMapped =
      nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 10);
  ros::Publisher pubCureentCloud =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_current", 100);
  ros::Publisher pubCurrentCorner =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_key_points", 100);
  ros::Publisher pubMatchedCloud =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_matched", 100);
  ros::Publisher pubMatchedCorner =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_matched_key_points", 100);
  ros::Publisher pubSTD =
      nh.advertise<visualization_msgs::MarkerArray>("descriptor_line", 10);

  ros::Publisher pubCorrectCloud =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_correct", 10000);
  ros::Publisher pubOdomCorreted =
      nh.advertise<nav_msgs::Odometry>("/odom_corrected", 10);

  ros::Rate loop(500);
  ros::Rate slow_loop(100);
  std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> poses_vec;
  std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> key_poses_vec;

  std::vector<double> times_vec;
  load_pose_with_time(pose_path, poses_vec, times_vec);
  std::cout << "Sucessfully load pose with number: " << poses_vec.size()
            << std::endl;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_vec;

  STDescManager *std_manager = new STDescManager(config_setting);
  gtsam::Values initial;
  gtsam::NonlinearFactorGraph graph;
  gtsam::Vector Vector6(6);
  Vector6 << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6;
  gtsam::noiseModel::Diagonal::shared_ptr odometryNoise =
      gtsam::noiseModel::Diagonal::Variances(Vector6);
  gtsam::noiseModel::Base::shared_ptr robustLoopNoise;
  double loopNoiseScore = 0.1;
  gtsam::Vector robustNoiseVector6(
      6); // gtsam::Pose3 factor has 6 elements (6D)
  robustNoiseVector6 << loopNoiseScore, loopNoiseScore, loopNoiseScore,
      loopNoiseScore, loopNoiseScore, loopNoiseScore;
  robustLoopNoise = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Cauchy::Create(1),
      gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6));
  size_t cloudInd = 0;
  size_t keyCloudInd = 0;
  pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(
      new pcl::PointCloud<pcl::PointXYZI>());

  std::vector<double> descriptor_time;
  std::vector<double> querying_time;
  std::vector<double> update_time;
  int triggle_loop_num = 0;
  while (ros::ok()) {
    std::stringstream lidar_data_path;
    lidar_data_path << lidar_path << std::setfill('0') << std::setw(6)
                    << cloudInd << ".bin";
    std::vector<float> lidar_data = read_lidar_data(lidar_data_path.str());
    if (lidar_data.size() == 0) {
      break;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud(
        new pcl::PointCloud<pcl::PointXYZI>());
    Eigen::Vector3d translation = poses_vec[cloudInd].first;
    Eigen::Matrix3d rotation = poses_vec[cloudInd].second;
    for (std::size_t i = 0; i < lidar_data.size(); i += 4) {
      pcl::PointXYZI point;
      point.x = lidar_data[i];
      point.y = lidar_data[i + 1];
      point.z = lidar_data[i + 2];
      point.intensity = lidar_data[i + 3];
      Eigen::Vector3d pv = point2vec(point);
      pv = rotation * pv + translation;
      point = vec2point(pv);
      point.intensity = lidar_data[i + 3];
      current_cloud->push_back(point);
    }
    down_sampling_voxel(*current_cloud, config_setting.ds_size_);
    cloud_vec.push_back(current_cloud);
    for (auto pv : current_cloud->points) {
      temp_cloud->points.push_back(pv);
    }

    // check if keyframe
    if (cloudInd % config_setting.sub_frame_num_ == 0 && cloudInd != 0) {
      // use first frame's pose as key pose
      key_poses_vec.push_back(
          poses_vec[cloudInd - config_setting.sub_frame_num_]);
      std::cout << "Key Frame id:" << keyCloudInd
                << ", cloud size: " << temp_cloud->size() << std::endl;
      // step1. Descriptor Extraction
      auto t_descriptor_begin = std::chrono::high_resolution_clock::now();
      std::vector<STDesc> stds_vec;
      std_manager->GenerateSTDescs(temp_cloud, stds_vec);
      auto t_descriptor_end = std::chrono::high_resolution_clock::now();
      descriptor_time.push_back(time_inc(t_descriptor_end, t_descriptor_begin));
      // step2. Searching Loop
      auto t_query_begin = std::chrono::high_resolution_clock::now();
      std::pair<int, double> search_result(-1, 0);
      std::pair<Eigen::Vector3d, Eigen::Matrix3d> loop_transform;
      loop_transform.first << 0, 0, 0;
      loop_transform.second = Eigen::Matrix3d::Identity();
      std::vector<std::pair<STDesc, STDesc>> loop_std_pair;
      if (keyCloudInd > config_setting.skip_near_num_) {
        std_manager->SearchLoop(stds_vec, search_result, loop_transform,
                                loop_std_pair);
      }
      if (search_result.first > 0) {
        std::cout << "[Loop Detection] triggle loop: " << keyCloudInd << "--"
                  << search_result.first << ", score:" << search_result.second
                  << std::endl;
      }
      auto t_query_end = std::chrono::high_resolution_clock::now();
      querying_time.push_back(time_inc(t_query_end, t_query_begin));

      // step3. Add descriptors to the database
      auto t_map_update_begin = std::chrono::high_resolution_clock::now();
      std_manager->AddSTDescs(stds_vec);
      auto t_map_update_end = std::chrono::high_resolution_clock::now();
      update_time.push_back(time_inc(t_map_update_end, t_map_update_begin));
      std::cout << "[Time] descriptor extraction: "
                << time_inc(t_descriptor_end, t_descriptor_begin) << "ms, "
                << "query: " << time_inc(t_query_end, t_query_begin) << "ms, "
                << "update map:"
                << time_inc(t_map_update_end, t_map_update_begin) << "ms"
                << std::endl;
      std::cout << std::endl;

      pcl::PointCloud<pcl::PointXYZI> save_key_cloud;
      save_key_cloud = *temp_cloud;
      // down_sampling_voxel(save_key_cloud, 0.5);
      std_manager->key_cloud_vec_.push_back(save_key_cloud.makeShared());

      // publish
      sensor_msgs::PointCloud2 pub_cloud;
      pcl::toROSMsg(*temp_cloud, pub_cloud);
      pub_cloud.header.frame_id = "camera_init";
      pubCureentCloud.publish(pub_cloud);
      pcl::toROSMsg(*std_manager->corner_cloud_vec_.back(), pub_cloud);
      pub_cloud.header.frame_id = "camera_init";
      pubCurrentCorner.publish(pub_cloud);

      if (search_result.first > 0) {
        triggle_loop_num++;
        // add connection between near frame
        initial.insert(cloudInd,
                       gtsam::Pose3(gtsam::Rot3(poses_vec[cloudInd].second),
                                    gtsam::Point3(poses_vec[cloudInd].first)));
        // source i
        // target i-1
        Eigen::Vector3d t_ab = poses_vec[cloudInd - 1].first;
        Eigen::Matrix3d R_ab = poses_vec[cloudInd - 1].second;

        t_ab = R_ab.transpose() * (poses_vec[cloudInd].first - t_ab);
        R_ab = R_ab.transpose() * poses_vec[cloudInd].second;

        gtsam::Rot3 R_sam(R_ab);
        gtsam::Point3 t_sam(t_ab);

        gtsam::NonlinearFactor::shared_ptr near_factor(
            new gtsam::BetweenFactor<gtsam::Pose3>(cloudInd - 1, cloudInd,
                                                   gtsam::Pose3(R_sam, t_sam),
                                                   odometryNoise));
        graph.push_back(near_factor);

        int match_frame = search_result.first;
        // obtain optimal transform
        std_manager->PlaneGeomrtricIcp(
            std_manager->plane_cloud_vec_.back(),
            std_manager->plane_cloud_vec_[match_frame], loop_transform);

        /*
          add connection between loop frame.
          e.g. if src_key_frame_id 5 with sub frames 51~60 triggle loop with
                tar_key_frame_id 1 with sub frames 11~20, add connection between
          each sub frame, 51-11, 52-12,...,60-20.

        */
        int sub_frame_num = config_setting.sub_frame_num_;
        for (size_t j = 1; j <= sub_frame_num; j++) {
          int src_frame = cloudInd + j - sub_frame_num;
          Eigen::Matrix3d src_R =
              loop_transform.second * poses_vec[src_frame].second;
          Eigen::Vector3d src_t =
              loop_transform.second * poses_vec[src_frame].first +
              loop_transform.first;
          int tar_frame = match_frame * sub_frame_num + j;
          Eigen::Matrix3d tar_R = poses_vec[tar_frame].second;
          Eigen::Vector3d tar_t = poses_vec[tar_frame].first;

          gtsam::Point3 ttem(tar_R.transpose() * (src_t - tar_t));
          gtsam::Rot3 Rtem(tar_R.transpose() * src_R);
          gtsam::NonlinearFactor::shared_ptr loop_factor(
              new gtsam::BetweenFactor<gtsam::Pose3>(tar_frame, src_frame,
                                                     gtsam::Pose3(Rtem, ttem),
                                                     robustLoopNoise));
          graph.push_back(loop_factor);
        }

        pcl::PointCloud<pcl::PointXYZI> correct_cloud;
        pcl::toROSMsg(*std_manager->key_cloud_vec_[search_result.first],
                      pub_cloud);
        pub_cloud.header.frame_id = "camera_init";
        pubMatchedCloud.publish(pub_cloud);

        pcl::toROSMsg(*std_manager->corner_cloud_vec_[search_result.first],
                      pub_cloud);
        pub_cloud.header.frame_id = "camera_init";
        pubMatchedCorner.publish(pub_cloud);
        publish_std_pairs(loop_std_pair, pubSTD);

      } else {
        // add connection between near frame
        initial.insert(cloudInd,
                       gtsam::Pose3(gtsam::Rot3(poses_vec[cloudInd].second),
                                    gtsam::Point3(poses_vec[cloudInd].first)));
        Eigen::Vector3d t_ab = poses_vec[cloudInd - 1].first;
        Eigen::Matrix3d R_ab = poses_vec[cloudInd - 1].second;

        t_ab = R_ab.transpose() * (poses_vec[cloudInd].first - t_ab);
        R_ab = R_ab.transpose() * poses_vec[cloudInd].second;

        gtsam::Rot3 R_sam(R_ab);
        gtsam::Point3 t_sam(t_ab);

        gtsam::NonlinearFactor::shared_ptr near_factor(
            new gtsam::BetweenFactor<gtsam::Pose3>(cloudInd - 1, cloudInd,
                                                   gtsam::Pose3(R_sam, t_sam),
                                                   odometryNoise));
        graph.push_back(near_factor);
      }
      temp_cloud->clear();
      keyCloudInd++;
      loop.sleep();
    } else {
      if (cloudInd == 0) {
        initial.insert(0,
                       gtsam::Pose3(gtsam::Rot3(poses_vec[cloudInd].second),
                                    gtsam::Point3(poses_vec[cloudInd].first)));
        graph.add(gtsam::PriorFactor<gtsam::Pose3>(
            0,
            gtsam::Pose3(gtsam::Rot3(poses_vec[cloudInd].second),
                         gtsam::Point3(poses_vec[cloudInd].first)),
            odometryNoise));
      } else {
        // add connection between near frame
        initial.insert(cloudInd,
                       gtsam::Pose3(gtsam::Rot3(poses_vec[cloudInd].second),
                                    gtsam::Point3(poses_vec[cloudInd].first)));
        Eigen::Vector3d t_ab = poses_vec[cloudInd - 1].first;
        Eigen::Matrix3d R_ab = poses_vec[cloudInd - 1].second;

        t_ab = R_ab.transpose() * (poses_vec[cloudInd].first - t_ab);
        R_ab = R_ab.transpose() * poses_vec[cloudInd].second;

        gtsam::Rot3 R_sam(R_ab);
        gtsam::Point3 t_sam(t_ab);

        gtsam::NonlinearFactor::shared_ptr near_factor(
            new gtsam::BetweenFactor<gtsam::Pose3>(cloudInd - 1, cloudInd,
                                                   gtsam::Pose3(R_sam, t_sam),
                                                   odometryNoise));
        graph.push_back(near_factor);
      }
    }
    nav_msgs::Odometry odom;
    odom.header.frame_id = "camera_init";
    odom.pose.pose.position.x = translation[0];
    odom.pose.pose.position.y = translation[1];
    odom.pose.pose.position.z = translation[2];
    Eigen::Quaterniond q(rotation);
    odom.pose.pose.orientation.w = q.w();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    pubOdomAftMapped.publish(odom);
    loop.sleep();
    cloudInd++;
  }
  double mean_descriptor_time =
      std::accumulate(descriptor_time.begin(), descriptor_time.end(), 0) * 1.0 /
      descriptor_time.size();
  double mean_query_time =
      std::accumulate(querying_time.begin(), querying_time.end(), 0) * 1.0 /
      querying_time.size();
  double mean_update_time =
      std::accumulate(update_time.begin(), update_time.end(), 0) * 1.0 /
      update_time.size();
  std::cout << "Total key frame number:" << keyCloudInd
            << ", loop number:" << triggle_loop_num << std::endl;
  std::cout << "Time for descriptor extraction: " << mean_descriptor_time
            << "ms, query: " << mean_query_time
            << "ms, update: " << mean_update_time << "ms, total: "
            << mean_descriptor_time + mean_query_time + mean_update_time << "ms"
            << std::endl;

  auto t_pgo_begin = std::chrono::high_resolution_clock::now();
  gtsam::ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  gtsam::ISAM2 isam(parameters);
  isam.update(graph, initial);
  isam.update();
  auto t_pgo_end = std::chrono::high_resolution_clock::now();
  std::cout << "Solve pgo time:" << time_inc(t_pgo_end, t_pgo_begin) << " ms"
            << std::endl;

  // clear rviz
  pcl::PointCloud<pcl::PointXYZI> empty_cloud;
  sensor_msgs::PointCloud2 pub_cloud;
  pcl::toROSMsg(empty_cloud, pub_cloud);
  pub_cloud.header.frame_id = "camera_init";
  pubCureentCloud.publish(pub_cloud);
  loop.sleep();
  pubCurrentCorner.publish(pub_cloud);
  loop.sleep();
  pubMatchedCloud.publish(pub_cloud);
  loop.sleep();
  pubMatchedCorner.publish(pub_cloud);
  loop.sleep();
  std::vector<std::pair<STDesc, STDesc>> empty_std_pair;
  publish_std_pairs(empty_std_pair, pubSTD);
  slow_loop.sleep();

  gtsam::Values results = isam.calculateEstimate();

  for (size_t i = 0; i < results.size(); i++) {
    gtsam::Pose3 pose = results.at(i).cast<gtsam::Pose3>();
    pcl::PointCloud<pcl::PointXYZI> correct_cloud;

    Eigen::Vector3d opt_translation = pose.translation();
    Eigen::Quaterniond opt_q(pose.rotation().matrix());
    for (size_t j = 0; j < cloud_vec[i]->size(); j++) {
      pcl::PointXYZI pi = cloud_vec[i]->points[j];
      Eigen::Vector3d pv(pi.x, pi.y, pi.z);
      // back transform to get point cloud in body frame
      pv = poses_vec[i].second.transpose() * pv -
           poses_vec[i].second.transpose() * poses_vec[i].first;
      // transform with optimal poses
      pv = opt_q * pv + opt_translation;
      pi.x = pv[0];
      pi.y = pv[1];
      pi.z = pv[2];
      correct_cloud.push_back(pi);
    }
    // down_sampling_voxel(correct_cloud, 0.5);
    // publish corrected odom
    nav_msgs::Odometry odom;
    odom.header.frame_id = "camera_init";
    odom.pose.pose.position.x = opt_translation[0];
    odom.pose.pose.position.y = opt_translation[1];
    odom.pose.pose.position.z = opt_translation[2];
    odom.pose.pose.orientation.w = opt_q.w();
    odom.pose.pose.orientation.x = opt_q.x();
    odom.pose.pose.orientation.y = opt_q.y();
    odom.pose.pose.orientation.z = opt_q.z();
    pubOdomCorreted.publish(odom);
    slow_loop.sleep();
    sensor_msgs::PointCloud2 pub_cloud;
    pcl::toROSMsg(correct_cloud, pub_cloud);
    pub_cloud.header.frame_id = "camera_init";
    pubCorrectCloud.publish(pub_cloud);
    slow_loop.sleep();
  }

  return 0;
}