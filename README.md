# **STD: A Stable Triangle Descriptor for 3D place recognition**
# **1. Introduction**
**STD_detector** is a global descriptor for 3D place recognition. For a triangle, its shape is uniquely determined by the length of the sides or included angles. Moreover, the shape of triangles is completely invariant to rigid transformations. Based on this property, we first design an algorithm to efficiently extract local key points from the 3D point cloud and encode these key points into triangular descriptors. Then, place recognition is achieved by matching the side lengths (and some other information) of the descriptors between point clouds. The point correspondence obtained from the descriptor matching pair can be further used in geometric verification, which greatly improves the accuracy of place recognition.

<div align="center">
    <div align="center">
        <img src="https://github.com/ChongjianYUAN/STDesc_release/raw/master/pics/introduction.png" width = 75% >
    </div>
    <font color=#a0a0a0 size=2>A typical place recognition case with STD. These two frames of point clouds are collected by a small FOV LiDAR (Livox Avia) moving in opposite directions, resulting in a low point cloud overlap and drastic viewpoint change.</font>
</div>
  

## **1.1. Developers:**
The codes of this repo are contributed by:
[Chongjian Yuan (袁崇健)](https://github.com/ChongjianYUAN) and [Jiarong Lin (林家荣)](https://jiaronglin.com)


## **1.2. Related paper**
Our paper has been accepted to [**ICRA2023**](https://www.icra2023.org/), and our preprint version is now available on **arxiv**:  
[STD: Stable Triangle Descriptor for 3D place recognition](https://arxiv.org/abs/2209.12435)


## **1.3. Related video**
Our accompanying video is now available on **YouTube**.
<div align="center">
    <a href="https://youtu.be/O-9iXn1ME3g" target="_blank"><img src="https://github.com/ChongjianYUAN/STDesc_release/raw/master/pics/video_cover.png" width=60% /></a>
</div>

# **2. Prerequisites**

## **2.1 Ubuntu and [ROS](https://www.ros.org/)**
We tested our code on Ubuntu18.04 with ros melodic and Ubuntu20.04 with noetic. Additional ROS package is required:
```
sudo apt-get install ros-xxx-pcl-conversions
```

## **2.2 Eigen**
Following the official [Eigen installation](eigen.tuxfamily.org/index.php?title=Main_Page), or directly install Eigen by:
```
sudo apt-get install libeigen3-dev
```
## **2.3. ceres-solver (version>=2.1)**
Please kindly install ceres-solver by following the guide on [ceres Installation](http://ceres-solver.org/installation.html). Notice that the version of ceres-solver should higher than [ceres-solver 2.1.0](https://github.com/ceres-solver/ceres-solver/releases/tag/2.1.0)

## **2.4. GTSAM**
Following the official [GTSAM installation](https://gtsam.org/get_started/), or directly install GTSAM 4.x stable release by:
```
# Add PPA
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt update  # not necessary since Bionic
# Install:
sudo apt install libgtsam-dev libgtsam-unstable-dev
```
**!! IMPORTANT !!**: Please do not install the GTSAM of ***develop branch***, which are not compatible with our code! We are still figuring out this issue.


## **2.5 Prepare for the data**
Since this repo does not implement any method (i.e., LOAM, LIO, etc) for solving the pose for registering the LiDAR scan. So, you have to prepare two set of data for reproducing our results, include: **1) the LiDAR point cloud data. 2) the point cloud registration pose.**

### **2.5.1. Download our Example data**
Departure from the purpose of convenience, we provide two sets of data for your fast evaluation, which can be downloaded from [**OneDrive**](https://connecthkuhk-my.sharepoint.com/:f:/g/personal/ycj1_connect_hku_hk/EpIDIgeOD05HpZZouhP74IsBfHD9oDibPe1M0JWUyMnfew?e=3AXA9L) and [**BaiduNetDisk(百度网盘)**](https://pan.baidu.com/s/1eYqrmaD0kskyYco2l1n8MA?pwd=xnmr)


### **2.5.2. LiDAR Point cloud data**
- For the ***Kitti dataset*** (i.e., our Example-1), we read the raw scan data with suffix *".bin"*. These raw LiDAR scan data can be downloaded from the [Kitti Odometry benchmark website](https://www.cvlibs.net/datasets/kitti/eval_odometry.php).
- For the ***solid-state LiDAR dataset*** (i.e., our Example-2), we read the undistort scan data from the recorded *rosbag* files, whose bag file contains undistort LiDAR scan data in *rostopic: "/cloud_undistort"* 
### **2.5.3. Point cloud registration pose**
In the [poses file](https://connecthkuhk-my.sharepoint.com/:f:/g/personal/ycj1_connect_hku_hk/EgnGX4jC2zxDi-45YCfbioEBpPCfBVxa2LcrE-90oL4u_A?e=Lb4Yvv), the poses for LiDAR point cloud registration are given in the following data format:
```
Timestamp pos_x pos_y pos_z quat_x quat_y quat_z quat_w
```
where, ``Timestamp`` is the correspond sampling time stamp of a LiDAR scan, ``pose_{x,y,z}`` and ``quad_{x,y,z,w}`` are the translation and rotation (expressed used quaternion) of pose. 
# **3. Examples**
This reposity contains implementations of Stable Triangle Descriptor, as well as demos for place recognition and loop closure correction. For the **complete pipline of online LiDAR SLAM**, we will release this code along with the release of the **extended version**.

## **3.1. Example-1: place recognition with KITTI Odometry dataset**
<div align="center">
<img src="https://github.com/ChongjianYUAN/STDesc_release/raw/master/pics/demo1/demo1_right.gif"  width="48%" />
<img src="https://github.com/ChongjianYUAN/STDesc_release/raw/master/pics/demo1/demo1_left.gif"  width="48%" />
</div>

To run Example-1, you need to first download the [poses file](https://connecthkuhk-my.sharepoint.com/:f:/g/personal/ycj1_connect_hku_hk/EgnGX4jC2zxDi-45YCfbioEBpPCfBVxa2LcrE-90oL4u_A?e=Lb4Yvv) we provide.

Then, you should modify the **demo_kitti.launch** file
- Set the **lidar_path** to your local path
- Set the **pose_path** to your local path
```
cd $STD_ROS_DIR
source deve/setup.bash
roslaunch std_detector demo_kitti.launch
```
## **3.2. Example-2: place recognition with Livox LiDAR dataset**
<div align="center">
<img src="https://github.com/ChongjianYUAN/STDesc_release/raw/master/pics/demo2/demo2_left.gif"  width="48%" />
<img src="https://github.com/ChongjianYUAN/STDesc_release/raw/master/pics/demo2/demo2_right.gif"  width="48%" />
</div>

To run Example-2, you need to first download the [rosbag file and poses file](https://connecthkuhk-my.sharepoint.com/:f:/g/personal/ycj1_connect_hku_hk/EvP0ZWZXE-pFqdBYKG_I4egBd3QXMA578r1YgdeYZNq3vw?e=9sjmoB) we provide.
Then, you should modify the **demo_livox.launch** file
- Set the **bag_path** to your local path
- Set the **pose_path** to your local path
```
cd $STD_ROS_DIR
source deve/setup.bash
roslaunch std_detector demo_livox.launch
```
## **3.3. Example-3: loop closure correction on the KITTI Odometry dataset**

<div align="center">
    <div align="center">
        <img src="https://github.com/ChongjianYUAN/STDesc_release/raw/master/pics/demo3/demo3.jpg" width = 90% >
    </div>
    <font color=#a0a0a0 size=2>The point cloud map and trajectory before and after correction by STD.</font>
</div>

To run Example-3, you need to first download the [poses file](https://connecthkuhk-my.sharepoint.com/:f:/g/personal/ycj1_connect_hku_hk/EqG7JX15nKZNu1qK1FTUMNQB7HJ2wDz7IcBof6y9cXV4sg?e=bgGDjg) we provide or create your own pose file on the KITTI Odometry dataset with a LiDAR odom following the format: **timestamp x y z qx qy qz qw**
Then, you should modify the **demo_pgo.launch** file
- Set the **lidar_path** to your local path
- Set the **pose_path** to your local path
```
cd $STD_ROS_DIR
source deve/setup.bash
roslaunch std_detector demo_pgo.launch
```

## **3.4. Example-4: online loop closure correction with FAST-LIO2 integrated
To run Example-4, you need to install and configure [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) first. 
You can try the data `building_slower_motino_avia.bag` [here](https://drive.google.com/drive/folders/1EqNt6Bm_6Jf3beRf_RI3yrhiUCND09se)(provided by [zlwang7](https://github.com/zlwang7/S-FAST_LIO)), which is outdoor scan data with no loop closure other than the one between the starting point and the endpoint. Therefore, relying solely on the fast-lio algorithm results in obvious Z-axis drift, with STD loop detection and graph optimization, there will be a noticeable correction to the drift (although the result is not perfect).

```
# termianl 1: run FAST-LIO2
roslaunch fast_lio mapping_avia.launch

# terminal 2: run std online demo
roslaunch std_detector demo_online.launch

# terminal 3: play data
rosbag play building_slower_motion_avia.bag
```


# **Acknowledgments**
In the development of **STD_detector**, we stand on the shoulders of the following repositories:

- [Scan Context](https://github.com/irapkaist/scancontext): An Egocentric Spatial Descriptor for Place Recognition within {3D} Point Cloud Map
- [FAST-LIO](https://github.com/hku-mars/FAST_LIO): A computationally efficient and robust LiDAR-inertial odometry package.
- [VoxelMap](https://github.com/hku-mars/VoxelMap): An efficient and probabilistic adaptive(coarse-to-fine) voxel mapping method for 3D LiDAR.
- [R3LIVE](https://github.com/hku-mars/r2live): A Robust, Real-time, RGB-colored, LiDAR-Inertial-Visual tightly-coupled state Estimation and mapping package

# **Contact Us**
We are still working on improving the performance and reliability of our codes. For any technical issues, please contact us via email Chongjian Yuan < ycj1ATconnect.hku.hk >, Jiarong Lin < ziv.lin.ljrATgmail.com >.

For commercial use, please contact Dr. Fu Zhang < fuzhang@hku.hk >


# **License**
The source code of this package is released under [**GPLv2**](http://www.gnu.org/licenses/) license. We only allow it free for personal and academic usage. For commercial use, please contact us to negotiate a different license.

We are still working on improving the performance and reliability of our codes. For any technical issues, please contact contact us via email Chongjian Yuan < ycj1ATconnect.hku.hk >, Jiarong Lin < ziv.lin.ljrATgmail.com >.

If you use any code of this repo in your academic research, please cite **at least one** of our papers:
```
[1] Yuan, C., Lin, J., Zou, Z., Hong, X., & Zhang, F.. "STD: Stable Triangle Descriptor for 3D place recognition."
[2] Xu, W., Cai, Y., He, D., Lin, J., & Zhang, F. "Fast-lio2: Fast direct lidar-inertial odometry."
[3] Yuan, C., Xu, W., Liu, X., Hong, X., & Zhang, F. "Efficient and probabilistic adaptive voxel mapping for accurate online lidar odometry."
```
