/*
Copyright 2025 Andreas Loeffler

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include "lidar_mirror_fov_reshaper_runtime.hpp"

#include "lidar_mirror_fov_reshaper_transformation/lidar_mirror_fov_reshaper_transformation.hpp"

LidarMirrorFOVReshaperRuntime::LidarMirrorFOVReshaperRuntime()
: Node("lidar_mirror_fov_reshaper_runtime")
{
  // declare parameter
  this->declare_parameter<std::string>("laser_scanner.topic", "cloud");
  this->declare_parameter<std::string>("laser_scanner.frame", "cloud");
  this->declare_parameter<double>("laser_scanner.angle_min", -2.35619);
  this->declare_parameter<double>("laser_scanner.angle_max", 2.35619);
  this->declare_parameter<bool>("laser_scanner.use_poincloud_input", false);

  this->declare_parameter<bool>("visualization.pub_transformed_all_combined", true);
  this->declare_parameter<bool>("visualization.pub_front", false);
  this->declare_parameter<bool>("visualization.pub_transformed_left_mirror", false);
  this->declare_parameter<bool>("visualization.pub_transformed_right_mirror", false);
  this->declare_parameter<std::string>("visualization.pub_front_topic", "front");
  this->declare_parameter<std::string>("visualization.pub_left_mirror_topic", "left_mirror");
  this->declare_parameter<std::string>("visualization.pub_right_mirror_topic", "right_mirror");
  this->declare_parameter<std::string>("visualization.pub_all_combined_topic", "all_combined");
  this->declare_parameter<bool>("visualization.remove_unused_points", false);

  this->front_start_angle_ =
    lidarMirrorFOVReshaperTF::deg2rad(this->declare_parameter<double>("front.start_angle", -45.0));
  this->front_end_angle_ =
    lidarMirrorFOVReshaperTF::deg2rad(this->declare_parameter<double>("front.end_angle", -45.0));

  this->mirror_left_start_angle_ =
    lidarMirrorFOVReshaperTF::deg2rad(this->declare_parameter<double>("mirror_left.start_angle", 95.0));
  this->mirror_left_end_angle_ =
    lidarMirrorFOVReshaperTF::deg2rad(this->declare_parameter<double>("mirror_left.end_angle", 130.0));
  this->declare_parameter<std::vector<double>>("mirror_left.normal_vec", {0.0, 0.0, 0.0});
  this->declare_parameter<std::vector<double>>("mirror_left.support_vec", {0.0, 0.0, 0.0});

  this->mirror_right_start_angle_ = lidarMirrorFOVReshaperTF::deg2rad(
    this->declare_parameter<double>("mirror_right.start_angle", -130.0));
  this->mirror_right_end_angle_ =
    lidarMirrorFOVReshaperTF::deg2rad(this->declare_parameter<double>("mirror_right.end_angle", -95.0));
  this->declare_parameter<std::vector<double>>("mirror_right.normal_vec", {0.0, 0.0, 0.0});
  this->declare_parameter<std::vector<double>>("mirror_right.support_vec", {0.0, 0.0, 0.0});
  
  // get parameter
  this->get_parameter("laser_scanner.topic", this->laser_topic_);
  this->get_parameter("laser_scanner.frame", this->laser_frame_);
  this->get_parameter("laser_scanner.angle_min", this->laser_angle_min_);
  this->get_parameter("laser_scanner.angle_max", this->laser_angle_max_);

  this->get_parameter(
    "visualization.pub_transformed_all_combined", this->viz_pub_transformed_all_combined_);
  this->get_parameter("visualization.pub_front", this->viz_pub_front_);
  this->get_parameter(
    "visualization.pub_transformed_left_mirror", this->viz_pub_transformed_leftmirror_);
  this->get_parameter(
    "visualization.pub_transformed_right_mirror", this->viz_pub_transformed_rightmirror_);

  this->get_parameter("mirror_left.normal_vec", mirror_left_normal_vec_);
  this->get_parameter("mirror_left.support_vec", mirror_left_support_vec_);

  this->get_parameter("mirror_right.normal_vec", mirror_right_normal_vec_);
  this->get_parameter("mirror_right.support_vec", mirror_right_support_vec_);
  RCLCPP_INFO(
    this->get_logger(), "normal_vec: %f, %f, %f", this->mirror_right_normal_vec_[0],
    this->mirror_right_normal_vec_[1], this->mirror_right_normal_vec_[2]);

  // Subscriber
  if (this->get_parameter("laser_scanner.use_poincloud_input").as_bool()) {
    this->pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      laser_topic_, 10,
      std::bind(&LidarMirrorFOVReshaperRuntime::pointcloudCallback, this, std::placeholders::_1));
  } else {
    this->laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/sick_tim_5xx/scan", 10,
      std::bind(&LidarMirrorFOVReshaperRuntime::laserScanCallback, this, std::placeholders::_1));
  };

  // Tf utils
  this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  this->projector_ = laser_geometry::LaserProjection();

  // Publisher
  // lmfr = lidar_mirror_fov_reshaper_runtime
  this->pointcloud_front_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    this->get_parameter("visualization.pub_front_topic").as_string(), 10);

  this->pointcloud_left_transformed_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    this->get_parameter("visualization.pub_left_mirror_topic").as_string(), 10);

  this->pointcloud_right_transformed_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    this->get_parameter("visualization.pub_right_mirror_topic").as_string(), 10);

  this->pointcloud_transformed_all_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    this->get_parameter("visualization.pub_all_combined_topic").as_string(), 10);
};

void LidarMirrorFOVReshaperRuntime::splitPointcloud(
  const pcl::PointCloud<pcl::PointXYZI> & src_cloud,
  pcl::PointCloud<pcl::PointXYZI> * left_mirror_cloud,
  pcl::PointCloud<pcl::PointXYZI> * right_mirror_cloud,
  pcl::PointCloud<pcl::PointXYZI> * front_cloud, 
  pcl::PointIndices * indices_left_mirror,
  pcl::PointIndices * indices_right_mirror, pcl::PointIndices * indices_front,
  pcl::PointIndices * indices_zero_to_startrm, pcl::PointIndices * indices_endrm_to_startfront,
  pcl::PointIndices * indices_endfront_to_startlm, pcl::PointIndices * indices_endlm_to_end)
{
  indices_left_mirror->indices.clear();
  lidarMirrorFOVReshaperTF::getPointcloudIndices(
    src_cloud, this->mirror_left_start_angle_, this->mirror_left_end_angle_, indices_left_mirror);

  indices_right_mirror->indices.clear();
  lidarMirrorFOVReshaperTF::getPointcloudIndices(
    src_cloud, this->mirror_right_start_angle_, this->mirror_right_end_angle_,
    indices_right_mirror);

  indices_front->indices.clear();
  lidarMirrorFOVReshaperTF::getPointcloudIndices(
    src_cloud, this->front_start_angle_, this->front_end_angle_, indices_front);

  pcl::copyPointCloud(src_cloud, indices_left_mirror->indices, *left_mirror_cloud);
  pcl::copyPointCloud(src_cloud, indices_right_mirror->indices, *right_mirror_cloud);
  pcl::copyPointCloud(src_cloud, indices_front->indices, *front_cloud);

  // get indices for the transition between the clouds
  indices_zero_to_startrm->indices.clear();
  lidarMirrorFOVReshaperTF::getPointcloudIndices(
    src_cloud, this->laser_angle_min_, this->mirror_right_start_angle_, indices_zero_to_startrm);
  
  indices_endrm_to_startfront->indices.clear();
  lidarMirrorFOVReshaperTF::getPointcloudIndices(
    src_cloud, this->mirror_right_end_angle_, this->front_start_angle_,
    indices_endrm_to_startfront);

  indices_endfront_to_startlm->indices.clear();
  lidarMirrorFOVReshaperTF::getPointcloudIndices(
    src_cloud, this->front_end_angle_, this->mirror_left_start_angle_, indices_endfront_to_startlm);
  
  indices_endlm_to_end->indices.clear();
  lidarMirrorFOVReshaperTF::getPointcloudIndices(
    src_cloud, this->mirror_left_end_angle_, this->laser_angle_max_, indices_endlm_to_end);
};

void LidarMirrorFOVReshaperRuntime::pointcloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::PointCloud<pcl::PointXYZI> pcl_pointcloud_raw;
  pcl::fromROSMsg(*msg, pcl_pointcloud_raw);

  // split into 3 parts
  pcl::PointCloud<pcl::PointXYZI> * pcl_left_mirror_cloud = new pcl::PointCloud<pcl::PointXYZI>();
  pcl::PointCloud<pcl::PointXYZI> * pcl_right_mirror_cloud = new pcl::PointCloud<pcl::PointXYZI>();
  pcl::PointCloud<pcl::PointXYZI> * pcl_front_cloud = new pcl::PointCloud<pcl::PointXYZI>();
  pcl::PointCloud<pcl::PointXYZI> * all_cloud = new pcl::PointCloud<pcl::PointXYZI>();

  pcl::PointCloud<pcl::PointXYZI> * pcl_left_mirror_transformed =
    new pcl::PointCloud<pcl::PointXYZI>();
  pcl::PointCloud<pcl::PointXYZI> * pcl_right_mirror_transformed =
    new pcl::PointCloud<pcl::PointXYZI>();
  pcl::PointCloud<pcl::PointXYZI> * all_cloud_transformed = new pcl::PointCloud<pcl::PointXYZI>();

  pcl::PointCloud<pcl::PointXYZI> * pcl_endrm_to_startfront = new pcl::PointCloud<pcl::PointXYZI>();
  pcl::PointCloud<pcl::PointXYZI> * pcl_endfront_to_startlm = new pcl::PointCloud<pcl::PointXYZI>();
  pcl::PointCloud<pcl::PointXYZI> * pcl_endlm_to_end = new pcl::PointCloud<pcl::PointXYZI>();

  pcl::PointIndices * indices_left_mirror = new pcl::PointIndices();
  pcl::PointIndices * indices_right_mirror = new pcl::PointIndices();
  pcl::PointIndices * indices_front = new pcl::PointIndices();
  pcl::PointIndices * indices_zero_to_startrm = new pcl::PointIndices();
  pcl::PointIndices * indices_endrm_to_startfront = new pcl::PointIndices();
  pcl::PointIndices * indices_endfront_to_startlm = new pcl::PointIndices();
  pcl::PointIndices * indices_endlm_to_end = new pcl::PointIndices();

  this->splitPointcloud(
    pcl_pointcloud_raw, pcl_left_mirror_cloud, pcl_right_mirror_cloud, pcl_front_cloud,
    indices_left_mirror, indices_right_mirror, indices_front, indices_zero_to_startrm,
    indices_endrm_to_startfront, indices_endfront_to_startlm, indices_endlm_to_end);
  
      // ensure sum of all indices is equal to the size of the input pointcloud, those indices missed add to nan to be set indices
  int tgt_size = pcl_pointcloud_raw.size();
  int sum_indices = 0;
  sum_indices += indices_left_mirror->indices.size();
  sum_indices += indices_right_mirror->indices.size();
  sum_indices += indices_front->indices.size();
  sum_indices += indices_zero_to_startrm->indices.size();
  sum_indices += indices_endrm_to_startfront->indices.size();
  sum_indices += indices_endfront_to_startlm->indices.size();
  sum_indices += indices_endlm_to_end->indices.size();

  pcl::PointIndices * indices_missing = new pcl::PointIndices();
  if (sum_indices != tgt_size) {
    for (int i = 0; i < tgt_size; i++) {
      if (
        std::find(indices_left_mirror->indices.begin(), indices_left_mirror->indices.end(), i) ==
          indices_left_mirror->indices.end() &&
        std::find(indices_right_mirror->indices.begin(), indices_right_mirror->indices.end(), i) ==
          indices_right_mirror->indices.end() &&
        std::find(indices_front->indices.begin(), indices_front->indices.end(), i) ==
          indices_front->indices.end() &&
        std::find(
          indices_zero_to_startrm->indices.begin(), indices_zero_to_startrm->indices.end(), i) ==
          indices_zero_to_startrm->indices.end() &&
        std::find(
          indices_endrm_to_startfront->indices.begin(), indices_endrm_to_startfront->indices.end(),
          i) == indices_endrm_to_startfront->indices.end() &&
        std::find(
          indices_endfront_to_startlm->indices.begin(), indices_endfront_to_startlm->indices.end(),
          i) == indices_endfront_to_startlm->indices.end() &&
        std::find(indices_endlm_to_end->indices.begin(), indices_endlm_to_end->indices.end(), i) ==
          indices_endlm_to_end->indices.end()) {
        indices_missing->indices.push_back(i);
      }
    };
  }

  pcl::copyPointCloud(pcl_pointcloud_raw, *indices_endrm_to_startfront, *pcl_endrm_to_startfront);
  pcl::copyPointCloud(pcl_pointcloud_raw, *indices_endfront_to_startlm, *pcl_endfront_to_startlm);
  pcl::copyPointCloud(pcl_pointcloud_raw, *indices_endlm_to_end, *pcl_endlm_to_end);

  lidarMirrorFOVReshaperTF::transformCloud(
    *pcl_front_cloud, *pcl_left_mirror_cloud, *pcl_right_mirror_cloud,
    this->mirror_left_normal_vec_, this->mirror_left_support_vec_, this->mirror_right_normal_vec_,
    this->mirror_right_support_vec_, all_cloud_transformed, pcl_left_mirror_transformed,
    pcl_right_mirror_transformed);

  pcl_left_mirror_transformed->width = pcl_left_mirror_transformed->size();
  pcl_left_mirror_transformed->height = 1;
  pcl_left_mirror_transformed->is_dense = true;

  pcl::copyPointCloud(pcl_pointcloud_raw, *indices_zero_to_startrm, *all_cloud);
  all_cloud->insert(
    all_cloud->end(), pcl_right_mirror_transformed->begin(), pcl_right_mirror_transformed->end());
  all_cloud->insert(
    all_cloud->end(), pcl_endrm_to_startfront->begin(), pcl_endrm_to_startfront->end());
  all_cloud->insert(all_cloud->end(), pcl_front_cloud->begin(), pcl_front_cloud->end());
  all_cloud->insert(
    all_cloud->end(), pcl_endfront_to_startlm->begin(), pcl_endfront_to_startlm->end());
  all_cloud->insert(
    all_cloud->end(), pcl_left_mirror_transformed->begin(), pcl_left_mirror_transformed->end());
  all_cloud->insert(all_cloud->end(), pcl_endlm_to_end->begin(), pcl_endlm_to_end->end());

  if (this->get_parameter("visualization.remove_unused_points").as_bool()) {
    this->setNaNAtIndices(all_cloud, indices_zero_to_startrm);
    this->setNaNAtIndices(all_cloud, indices_endrm_to_startfront);
    this->setNaNAtIndices(all_cloud, indices_endfront_to_startlm);
    this->setNaNAtIndices(all_cloud, indices_endlm_to_end);

    this->setNaNAtIndices(all_cloud, indices_missing);
  }

  if (this->viz_pub_transformed_all_combined_)
    publishPointCloud(*all_cloud, this->pointcloud_transformed_all_pub_, msg->header);

  if (this->viz_pub_front_) {
    publishPointCloud(*pcl_front_cloud, this->pointcloud_front_pub_, msg->header);
  }

  if (this->viz_pub_transformed_leftmirror_)
    publishPointCloud(
      *pcl_left_mirror_transformed, this->pointcloud_left_transformed_pub_, msg->header);

  if (this->viz_pub_transformed_rightmirror_)
    publishPointCloud(
      *pcl_right_mirror_transformed, this->pointcloud_right_transformed_pub_, msg->header);

  delete pcl_left_mirror_cloud;
  delete pcl_right_mirror_cloud;
  delete pcl_front_cloud;
  delete all_cloud;

  delete pcl_left_mirror_transformed;
  delete pcl_right_mirror_transformed;
  delete all_cloud_transformed;
  delete pcl_endrm_to_startfront;
  delete pcl_endfront_to_startlm;
  delete pcl_endlm_to_end;

  delete indices_left_mirror;
  delete indices_right_mirror;
  delete indices_front;
  delete indices_zero_to_startrm;
  delete indices_endrm_to_startfront;
  delete indices_endfront_to_startlm;
  delete indices_endlm_to_end;

  delete indices_missing;
};

void LidarMirrorFOVReshaperRuntime::setNaNAtIndices(
  pcl::PointCloud<pcl::PointXYZI> * cloud, pcl::PointIndices * indices)
{
  for (const int index : indices->indices) {
    cloud->points[index].x = std::numeric_limits<float>::quiet_NaN();
    cloud->points[index].y = std::numeric_limits<float>::quiet_NaN();
    cloud->points[index].z = std::numeric_limits<float>::quiet_NaN();
    cloud->points[index].intensity = std::numeric_limits<float>::quiet_NaN();
  }
}

void LidarMirrorFOVReshaperRuntime::laserScanCallback(const sensor_msgs::msg::LaserScan & msg)
{
  sensor_msgs::msg::PointCloud2 cloud_msg;
  this->laserScanToPCL(msg, cloud_msg);

  this->pointcloudCallback(std::make_shared<sensor_msgs::msg::PointCloud2>(cloud_msg));
};

void LidarMirrorFOVReshaperRuntime::laserScanToPCL(
  const sensor_msgs::msg::LaserScan & scan_msg, sensor_msgs::msg::PointCloud2 & cloud_msg)
{
  projector_.projectLaser(scan_msg, cloud_msg);
};

void LidarMirrorFOVReshaperRuntime::publishPointCloud(
  const pcl::PointCloud<pcl::PointXYZI> & cloud,
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher,
  const std_msgs::msg::Header & header)
{
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header = header;
  cloud_msg.is_dense = true;

  publisher->publish(cloud_msg);
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarMirrorFOVReshaperRuntime>());
  rclcpp::shutdown();

  return 0;
}