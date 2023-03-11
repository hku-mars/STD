#include "include/STDesc.h"
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>

// Read KITTI data
std::vector<float> read_lidar_data(const std::string lidar_data_path) {
  std::ifstream lidar_data_file;
  lidar_data_file.open(lidar_data_path,
                       std::ifstream::in | std::ifstream::binary);
  if (!lidar_data_file) {
    std::cout << "Read End..." << std::endl;
    std::vector<float> nan_data;
    return nan_data;
    // exit(-1);
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
  ros::init(argc, argv, "demo_kitti");
  ros::NodeHandle nh;
  std::string lidar_path = "";
  std::string pose_path = "";
  std::string config_path = "";
  nh.param<std::string>("lidar_path", lidar_path, "");
  nh.param<std::string>("pose_path", pose_path, "");

  ConfigSetting config_setting;
  read_parameters(nh, config_setting);

  ros::Publisher pubOdomAftMapped =
      nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 10);
  // ros::Publisher pubRegisterCloud =
  //     nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100);
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

  ros::Rate loop(500);
  ros::Rate slow_loop(10);

  std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> poses_vec;
  std::vector<double> times_vec;
  load_pose_with_time(pose_path, poses_vec, times_vec);
  std::cout << "Sucessfully load pose with number: " << poses_vec.size()
            << std::endl;

  STDescManager *std_manager = new STDescManager(config_setting);

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
      current_cloud->push_back(point);
    }
    down_sampling_voxel(*current_cloud, config_setting.ds_size_);
    for (auto pv : current_cloud->points) {
      temp_cloud->points.push_back(pv);
    }

    // check if keyframe
    if (cloudInd % config_setting.sub_frame_num_ == 0 && cloudInd != 0) {
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
        pcl::toROSMsg(*std_manager->key_cloud_vec_[search_result.first],
                      pub_cloud);
        pub_cloud.header.frame_id = "camera_init";
        pubMatchedCloud.publish(pub_cloud);
        slow_loop.sleep();
        pcl::toROSMsg(*std_manager->corner_cloud_vec_[search_result.first],
                      pub_cloud);
        pub_cloud.header.frame_id = "camera_init";
        pubMatchedCorner.publish(pub_cloud);
        publish_std_pairs(loop_std_pair, pubSTD);
        slow_loop.sleep();
        // getchar();
      }
      temp_cloud->clear();
      keyCloudInd++;
      loop.sleep();
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
  std::cout << "Mean time for descriptor extraction: " << mean_descriptor_time
            << "ms, query: " << mean_query_time
            << "ms, update: " << mean_update_time << "ms, total: "
            << mean_descriptor_time + mean_query_time + mean_update_time << "ms"
            << std::endl;
  return 0;
}