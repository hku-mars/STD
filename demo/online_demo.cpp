#include <Eigen/Geometry>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/common/transforms.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <queue>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <thread>

#include "include/STDesc.h"
#include "ros/init.h"

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloud;

std::mutex laser_mtx;
std::mutex odom_mtx;

std::queue<sensor_msgs::PointCloud2::ConstPtr> laser_buffer;
std::queue<nav_msgs::Odometry::ConstPtr> odom_buffer;

void laserCloudHandler(const sensor_msgs::PointCloud2::ConstPtr &msg) {
  std::unique_lock<std::mutex> lock(laser_mtx);

  laser_buffer.push(msg);
}

void OdomHandler(const nav_msgs::Odometry::ConstPtr &msg) {
  std::unique_lock<std::mutex> lock(odom_mtx);

  odom_buffer.push(msg);
}

bool syncPackages(PointCloud::Ptr &cloud, Eigen::Affine3d &pose) {
  if (laser_buffer.empty() || odom_buffer.empty())
    return false;

  auto laser_msg = laser_buffer.front();
  double laser_timestamp = laser_msg->header.stamp.toSec();

  auto odom_msg = odom_buffer.front();
  double odom_timestamp = odom_msg->header.stamp.toSec();

  // check if timestamps are matched
  if (abs(odom_timestamp - laser_timestamp) < 1e-3) {
    pcl::fromROSMsg(*laser_msg, *cloud);

    Eigen::Quaterniond r(
        odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);
    Eigen::Vector3d t(odom_msg->pose.pose.position.x,
                      odom_msg->pose.pose.position.y,
                      odom_msg->pose.pose.position.z);

    pose = Eigen::Affine3d::Identity();
    pose.translate(t);
    pose.rotate(r);

    std::unique_lock<std::mutex> l_lock(laser_mtx);
    std::unique_lock<std::mutex> o_lock(odom_mtx);

    laser_buffer.pop();
    odom_buffer.pop();

  } else if (odom_timestamp < laser_timestamp) {
    ROS_WARN("Current odometry is earlier than laser scan, discard one "
             "odometry data.");
    std::unique_lock<std::mutex> o_lock(odom_mtx);
    odom_buffer.pop();
    return false;
  } else {
    ROS_WARN(
        "Current laser scan is earlier than odometry, discard one laser scan.");
    std::unique_lock<std::mutex> l_lock(laser_mtx);
    laser_buffer.pop();
    return false;
  }

  return true;
}

void update_poses(const gtsam::Values &estimates,
                  std::vector<Eigen::Affine3d> &poses) {
  assert(estimates.size() == poses.size());

  poses.clear();

  for (int i = 0; i < estimates.size(); ++i) {
    auto est = estimates.at<gtsam::Pose3>(i);
    Eigen::Affine3d est_affine3d(est.matrix());
    poses.push_back(est_affine3d);
  }
}

void visualizeLoopClosure(
    const ros::Publisher &publisher,
    const std::vector<std::pair<int, int>> &loop_container,
    const std::vector<Eigen::Affine3d> &key_pose_vec) {
  if (loop_container.empty())
    return;

  visualization_msgs::MarkerArray markerArray;
  // 闭环顶点
  visualization_msgs::Marker markerNode;
  markerNode.header.frame_id = "camera_init"; // camera_init
  // markerNode.header.stamp = ros::Time().fromSec( keyframeTimes.back() );
  markerNode.action = visualization_msgs::Marker::ADD;
  markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
  markerNode.ns = "loop_nodes";
  markerNode.id = 0;
  markerNode.pose.orientation.w = 1;
  markerNode.scale.x = 0.3;
  markerNode.scale.y = 0.3;
  markerNode.scale.z = 0.3;
  markerNode.color.r = 0;
  markerNode.color.g = 0.8;
  markerNode.color.b = 1;
  markerNode.color.a = 1;
  // 闭环边
  visualization_msgs::Marker markerEdge;
  markerEdge.header.frame_id = "camera_init";
  // markerEdge.header.stamp = ros::Time().fromSec( keyframeTimes.back() );
  markerEdge.action = visualization_msgs::Marker::ADD;
  markerEdge.type = visualization_msgs::Marker::LINE_LIST;
  markerEdge.ns = "loop_edges";
  markerEdge.id = 1;
  markerEdge.pose.orientation.w = 1;
  markerEdge.scale.x = 0.1;
  markerEdge.color.r = 0.9;
  markerEdge.color.g = 0.9;
  markerEdge.color.b = 0;
  markerEdge.color.a = 1;

  // 遍历闭环
  for (auto it = loop_container.begin(); it != loop_container.end(); ++it) {
    int key_cur = it->first;
    int key_pre = it->second;
    geometry_msgs::Point p;
    p.x = key_pose_vec[key_cur].translation().x();
    p.y = key_pose_vec[key_cur].translation().y();
    p.z = key_pose_vec[key_cur].translation().z();
    markerNode.points.push_back(p);
    markerEdge.points.push_back(p);
    p.x = key_pose_vec[key_pre].translation().x();
    p.y = key_pose_vec[key_pre].translation().y();
    p.z = key_pose_vec[key_pre].translation().z();
    markerNode.points.push_back(p);
    markerEdge.points.push_back(p);
  }

  markerArray.markers.push_back(markerNode);
  markerArray.markers.push_back(markerEdge);
  publisher.publish(markerArray);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "online_demo");
  ros::NodeHandle nh;

  ConfigSetting config_setting;
  read_parameters(nh, config_setting);

  // ros::Publisher pubOdomAftMapped =
  // nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 10);
  ros::Publisher pubCurrentCloud =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_current", 100);
  ros::Publisher pubCurrentCorner =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_key_points", 100);
  ros::Publisher pubMatchedCloud =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_matched", 100);
  ros::Publisher pubMatchedCorner =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_matched_key_points", 100);
  ros::Publisher pubSTD =
      nh.advertise<visualization_msgs::MarkerArray>("descriptor_line", 10);

  ros::Publisher pubOriginCloud =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_origin", 10000);

  ros::Publisher pubCorrectCloud =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_correct", 10000);
  ros::Publisher pubCorrectPath =
      nh.advertise<nav_msgs::Path>("/correct_path", 100000);

  ros::Publisher pubOdomOrigin =
      nh.advertise<nav_msgs::Odometry>("/odom_origin", 10);
  ros::Publisher pubLoopConstraintEdge =
      nh.advertise<visualization_msgs::MarkerArray>("/loop_closure_constraints",
                                                    10);

  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(
      "/cloud_registered_body", 100, laserCloudHandler);
  ros::Subscriber subOdom =
      nh.subscribe<nav_msgs::Odometry>("/Odometry", 100, OdomHandler);

  STDescManager *std_manager = new STDescManager(config_setting);

  gtsam::Values initial;
  gtsam::NonlinearFactorGraph graph;

  // https://github.com/TixiaoShan/LIO-SAM/blob/6665aa0a4fcb5a9bb3af7d3923ae4a035b489d47/src/mapOptmization.cpp#L1385
  gtsam::noiseModel::Diagonal::shared_ptr odometryNoise =
      gtsam::noiseModel::Diagonal::Variances(
          (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());

  gtsam::noiseModel::Diagonal::shared_ptr priorNoise =
      gtsam::noiseModel::Diagonal::Variances(
          (gtsam::Vector(6) << 1e-2, 1e-2, M_PI * M_PI, 1e8, 1e8, 1e8)
              .finished()); // rad*rad, meter*meter

  double loopNoiseScore = 1e-1;
  gtsam::Vector robustNoiseVector6(
      6); // gtsam::Pose3 factor has 6 elements (6D)
  robustNoiseVector6 << loopNoiseScore, loopNoiseScore, loopNoiseScore,
      loopNoiseScore, loopNoiseScore, loopNoiseScore;
  gtsam::noiseModel::Base::shared_ptr robustLoopNoise =
      gtsam::noiseModel::Robust::Create(
          gtsam::noiseModel::mEstimator::Cauchy::Create(1),
          gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6));

  gtsam::ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  gtsam::ISAM2 isam(parameters);

  size_t cloudInd = 0;
  size_t keyCloudInd = 0;

  std::vector<PointCloud::Ptr> cloud_vec;
  std::vector<Eigen::Affine3d> pose_vec;
  std::vector<Eigen::Affine3d> origin_pose_vec;
  std::vector<Eigen::Affine3d> key_pose_vec;
  std::vector<std::pair<int, int>> loop_container;

  PointCloud::Ptr key_cloud(new PointCloud);

  bool has_loop_flag = false;
  gtsam::Values curr_estimate;

  Eigen::Affine3d last_pose;
  last_pose.setIdentity();
  while (ros::ok()) {
    ros::spinOnce();

    PointCloud::Ptr current_cloud_body(new PointCloud);
    PointCloud::Ptr current_cloud_world(new PointCloud);
    Eigen::Affine3d pose;
    if (syncPackages(current_cloud_body, pose)) {
      auto origin_estimate_affine3d = pose;
      pcl::transformPointCloud(*current_cloud_body, *current_cloud_world, pose);
      down_sampling_voxel(*current_cloud_world, config_setting.ds_size_);
      // down sample body cloud
      down_sampling_voxel(*current_cloud_body, 0.5);
      cloud_vec.push_back(current_cloud_body);
      pose_vec.push_back(pose);
      origin_pose_vec.push_back(pose);
      PointCloud origin_cloud;
      pcl::transformPointCloud(*current_cloud_body, origin_cloud,
                               origin_estimate_affine3d);
      sensor_msgs::PointCloud2 pub_cloud;
      pcl::toROSMsg(origin_cloud, pub_cloud);
      pub_cloud.header.frame_id = "camera_init";
      pubOriginCloud.publish(pub_cloud);

      Eigen::Quaterniond _r(origin_estimate_affine3d.rotation());
      nav_msgs::Odometry odom;
      odom.header.frame_id = "camera_init";
      odom.pose.pose.position.x = origin_estimate_affine3d.translation().x();
      odom.pose.pose.position.y = origin_estimate_affine3d.translation().y();
      odom.pose.pose.position.z = origin_estimate_affine3d.translation().z();
      odom.pose.pose.orientation.w = _r.w();
      odom.pose.pose.orientation.x = _r.x();
      odom.pose.pose.orientation.y = _r.y();
      odom.pose.pose.orientation.z = _r.z();
      pubOdomOrigin.publish(odom);

      *key_cloud += *current_cloud_world;
      initial.insert(cloudInd, gtsam::Pose3(pose.matrix()));

      if (!cloudInd) {
        graph.add(gtsam::PriorFactor<gtsam::Pose3>(
            0, gtsam::Pose3(pose.matrix()), odometryNoise));
      } else {
        auto prev_pose = gtsam::Pose3(origin_pose_vec[cloudInd - 1].matrix());
        auto curr_pose = gtsam::Pose3(pose.matrix());
        graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
            cloudInd - 1, cloudInd, prev_pose.between(curr_pose),
            odometryNoise));
      }

      // check if keyframe
      if (cloudInd % config_setting.sub_frame_num_ == 0 && cloudInd != 0) {
        ROS_INFO("key frame idx: [%d], key cloud size: [%d]", (int)keyCloudInd,
                 (int)key_cloud->size());
        // step1. Descriptor Extraction
        std::vector<STDesc> stds_vec;
        std_manager->GenerateSTDescs(key_cloud, stds_vec);

        // step2. Searching Loop
        std::pair<int, double> search_result(-1, 0);
        std::pair<Eigen::Vector3d, Eigen::Matrix3d> loop_transform;
        loop_transform.first << 0, 0, 0;
        loop_transform.second = Eigen::Matrix3d::Identity();
        std::vector<std::pair<STDesc, STDesc>> loop_std_pair;

        if (keyCloudInd > config_setting.skip_near_num_) {
          std_manager->SearchLoop(stds_vec, search_result, loop_transform,
                                  loop_std_pair);
        }

        // step3. Add descriptors to the database
        std_manager->AddSTDescs(stds_vec);

        // publish
        sensor_msgs::PointCloud2 pub_cloud;
        pcl::toROSMsg(*key_cloud, pub_cloud);
        pub_cloud.header.frame_id = "camera_init";
        pubCurrentCloud.publish(pub_cloud);
        pcl::toROSMsg(*std_manager->corner_cloud_vec_.back(), pub_cloud);
        pub_cloud.header.frame_id = "camera_init";
        pubCurrentCorner.publish(pub_cloud);

        std_manager->key_cloud_vec_.push_back(key_cloud->makeShared());

        if (search_result.first > 0) {
          std::cout << "[Loop Detection] triggle loop: " << keyCloudInd << "--"
                    << search_result.first << ", score:" << search_result.second
                    << std::endl;

          has_loop_flag = true;
          int match_frame = search_result.first;
          // obtain optimal transform
          std_manager->PlaneGeomrtricIcp(
              std_manager->plane_cloud_vec_.back(),
              std_manager->plane_cloud_vec_[match_frame], loop_transform);

          // std::cout << "delta transform:" << std::endl;
          // std::cout << "translation: " << loop_transform.first.transpose() <<
          // std::endl;

          // auto euler = loop_transform.second.eulerAngles(2, 1, 0) * 57.3;
          // std::cout << "rotation(ypr): " << euler[0] << ' ' << euler[1] << '
          // ' << euler[2]
          //           << std::endl;

          /*
            add connection between loop frame.
            e.g. if src_key_frame_id 5 with sub frames 51~60 triggle loop with
                  tar_key_frame_id 1 with sub frames 11~20, add connection
            between each sub frame, 51-11, 52-12,...,60-20.

          */
          int sub_frame_num = config_setting.sub_frame_num_;
          for (size_t j = 1; j <= sub_frame_num; j++) {
            int src_frame = cloudInd + j - sub_frame_num;

            auto delta_T = Eigen::Affine3d::Identity();
            delta_T.translate(loop_transform.first);
            delta_T.rotate(loop_transform.second);
            Eigen::Affine3d src_pose_refined = delta_T * pose_vec[src_frame];

            int tar_frame = match_frame * sub_frame_num + j;
            // old
            // Eigen::Affine3d tar_pose = pose_vec[tar_frame];
            Eigen::Affine3d tar_pose = origin_pose_vec[tar_frame];

            loop_container.push_back({tar_frame, src_frame});

            graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                tar_frame, src_frame,
                gtsam::Pose3(tar_pose.matrix())
                    .between(gtsam::Pose3(src_pose_refined.matrix())),
                robustLoopNoise));
          }

          pcl::toROSMsg(*std_manager->key_cloud_vec_[search_result.first],
                        pub_cloud);
          pub_cloud.header.frame_id = "camera_init";
          pubMatchedCloud.publish(pub_cloud);

          pcl::toROSMsg(*std_manager->corner_cloud_vec_[search_result.first],
                        pub_cloud);
          pub_cloud.header.frame_id = "camera_init";
          pubMatchedCorner.publish(pub_cloud);
          publish_std_pairs(loop_std_pair, pubSTD);
        }

        key_cloud->clear();
        ++keyCloudInd;
      }
      isam.update(graph, initial);
      isam.update();

      if (has_loop_flag) {
        isam.update();
        isam.update();
        isam.update();
        isam.update();
        isam.update();
      }

      graph.resize(0);
      initial.clear();

      curr_estimate = isam.calculateEstimate();
      update_poses(curr_estimate, pose_vec);

      auto latest_estimate_affine3d = pose_vec.back();

      if (has_loop_flag) {
        // publish correct cloud map
        PointCloud full_map;
        for (int i = 0; i < pose_vec.size(); ++i) {
          PointCloud correct_cloud;
          pcl::transformPointCloud(*cloud_vec[i], correct_cloud, pose_vec[i]);
          full_map += correct_cloud;
        }
        sensor_msgs::PointCloud2 pub_cloud;
        pcl::toROSMsg(full_map, pub_cloud);
        pub_cloud.header.frame_id = "camera_init";
        pubCorrectCloud.publish(pub_cloud);

        // publish corerct path
        nav_msgs::Path correct_path;
        for (int i = 0; i < pose_vec.size(); i += 1) {

          geometry_msgs::PoseStamped msg_pose;
          msg_pose.pose.position.x = pose_vec[i].translation()[0];
          msg_pose.pose.position.y = pose_vec[i].translation()[1];
          msg_pose.pose.position.z = pose_vec[i].translation()[2];
          Eigen::Quaterniond pose_q(pose_vec[i].rotation());
          msg_pose.header.frame_id = "camera_init";
          msg_pose.pose.orientation.x = pose_q.x();
          msg_pose.pose.orientation.y = pose_q.y();
          msg_pose.pose.orientation.z = pose_q.z();
          msg_pose.pose.orientation.w = pose_q.w();
          correct_path.poses.push_back(msg_pose);
        }
        correct_path.header.stamp = ros::Time::now();
        correct_path.header.frame_id = "camera_init";
        pubCorrectPath.publish(correct_path);
      }
      //   PointCloud correct_cloud;
      //   pcl::transformPointCloud(*current_cloud_body, correct_cloud,
      //                            latest_estimate_affine3d);
      //   sensor_msgs::PointCloud2 pub_cloud;
      //   pcl::toROSMsg(correct_cloud, pub_cloud);
      //   pub_cloud.header.frame_id = "camera_init";
      //   pubCorrectCloud.publish(pub_cloud);

      visualizeLoopClosure(pubLoopConstraintEdge, loop_container, pose_vec);

      has_loop_flag = false;
      ++cloudInd;
    }
  }

  // You can save full map with refined pose
  // assert(cloud_vec.size() == pose_vec.size());
  // PointCloud full_map;
  // for (int i = 0; i < pose_vec.size(); ++i) {
  //     PointCloud correct_cloud;
  //     pcl::transformPointCloud(*cloud_vec[i], correct_cloud, pose_vec[i]);
  //     full_map += correct_cloud;
  // }
  // down_sampling_voxel(full_map, 0.05);

  // std::cout << "saving map..." << std::endl;
  // pcl::io::savePCDFileBinary("/home/dustier/data/map.pcd", full_map);

  return 0;
}
