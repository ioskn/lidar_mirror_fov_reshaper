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

#include "lidar_mirror_fov_reshaper_calibration.hpp"

LidarMirrorFOVReshaperCalib::LidarMirrorFOVReshaperCalib()
: Node("lidar_mirror_fov_reshaper_calibration")
{
  this->laser_scanner_topic_ = this->declare_parameter<std::string>("laser_scanner.topic", "cloud");
  this->laser_scanner_frame_id_ =
    this->declare_parameter<std::string>("laser_scanner.laser_frame", "cloud");
  this->laser_scanner_angle_min_ =
    this->deg2rad(this->declare_parameter<int>("laser_scanner.angle_min", -135));
  this->laser_scanner_angle_max_ =
    this->deg2rad(this->declare_parameter<int>("laser_scanner.angle_max", 135));
  this->declare_parameter<bool>("laser_scanner.use_pointcloud_input", false);

  this->viz_transformed_cloud_all =
    this->declare_parameter<bool>("visualization.pub_transformed_all_combined", false);
  this->viz_transformed_cloud_rm =
    this->declare_parameter<bool>("visualization.pub_transformed_right_mirror", false);
  this->viz_transformed_cloud_lm =
    this->declare_parameter<bool>("visualization.pub_transformed_left_mirror", false);
  this->viz_transformed_cloud_front =
    this->declare_parameter<bool>("visualization.pub_front", false);
  this->viz_normal_vectors = this->declare_parameter<bool>("visualization.normal_vectors", false);
  this->viz_optimized_planes_all =
    this->declare_parameter<bool>("visualization.optimized_planes_all", false);
  this->viz_optimized_plane_rm =
    this->declare_parameter<bool>("visualization.optimized_plane_rm", false);
  this->viz_optimized_plane_lm =
    this->declare_parameter<bool>("visualization.optimized_plane_lm", false);
  this->viz_optimized_plane_opt_plane =
    this->declare_parameter<bool>("visualization.optimized_plane_opt_plane", false);
  this->viz_optimization_plane_box_ =
    this->declare_parameter<bool>("visualization.optimization_plane_box", false);

  this->filter_method = this->declare_parameter<int>("optimization.filter_method", 0);
  this->filter_dist_threshold =
    this->declare_parameter<double>("optimization.distance_threshold", 0.05);
  this->interpolation_window =
    this->declare_parameter<int>("optimization.interpolation_window", -1);
  this->averaging_n_clouds = this->declare_parameter<int>("optimization.averaging_n_clouds", -1);
  this->intensity_threshold_percentage_ =
    this->declare_parameter<double>("intensity_threshold_percentage", 0.5);
  this->optimization_buffer_size_ = this->declare_parameter<int>("optimization.buffer_size", 10);
  this->optimization_epsabs_ = this->declare_parameter<double>("optimization.epsabs", 1e-6);
  this->optimization_stepsize_ = this->declare_parameter<double>("optimization.stepsize", 1e-3);
  this->optimization_iter_max_ = this->declare_parameter<int>("optimization.iter_max", 100);
  this->optimization_adaptive_stepsize_ =
    this->declare_parameter<bool>("optimization.adaptive_stepsize", false);
  this->optimization_opt_mirror_orientation_ =
    this->declare_parameter<bool>("optimization.opt_mirror_orientation", true);
  this->optimization_opt_mirror_support_vec_ =
    this->declare_parameter<bool>("optimization.opt_mirror_support_vec", false);
  this->write_optimized_params_ =
    this->declare_parameter<bool>("optimization.write_optimized_params", false);
  this->write_optimization_history_ =
    this->declare_parameter<bool>("optimization.write_optimization_history", false);
  this->optimization_history_file_ = this->declare_parameter<std::string>(
    "optimization.optimization_history_file", "optimization_history.csv");
  this->optimized_params_meta_file_ = this->declare_parameter<std::string>(
    "optimization.optimized_params_meta_file", "open_see_ground_calib_methods.csv");
  this->optimized_params_file_ = this->declare_parameter<std::string>(
    "optimization.optimized_params_file", "open_see_ground_calib_results.csv");
  this->declare_parameter<std::string>("optimization.optimized_params_out_dir", "");

  this->optimization_verbose_ = this->declare_parameter<int>("optimization.verbose", 0);
  this->optimization_evaluation_no_batches_ =
    this->declare_parameter<int>("optimization.evaluation_no_batches", 2);

  this->opt_mirror_orientation_all =
    this->declare_parameter<bool>("optimization.optimization_mirror_orientation.opt_all", false);
  this->opt_mirror_orientation_mirror_svs = this->declare_parameter<bool>(
    "optimization.optimization_mirror_orientation.opt_mirror_svs", false);
  this->opt_mirror_orientation_mirror_nvs = this->declare_parameter<bool>(
    "optimization.optimization_mirror_orientation.opt_mirror_nvs", true);
  this->opt_mirror_orientation_plane_sv = this->declare_parameter<bool>(
    "optimization.optimization_mirror_orientation.opt_plane_sv", true);
  this->apply_opt_osg_settings = this->declare_parameter<bool>(
    "optimization.optimization_mirror_orientation.opt_osg_settings", false);

  this->opt_mirror_orientation_rm_sv_x = this->declare_parameter<bool>(
    "optimization.optimization_mirror_orientation.right_mirror.support_vec.x", true);
  this->opt_mirror_orientation_rm_sv_y = this->declare_parameter<bool>(
    "optimization.optimization_mirror_orientation.right_mirror.support_vec.y", true);
  this->opt_mirror_orientation_rm_sv_z = this->declare_parameter<bool>(
    "optimization.optimization_mirror_orientation.right_mirror.support_vec.z", true);
  this->opt_mirror_orientation_rm_nv_x = this->declare_parameter<bool>(
    "optimization.optimization_mirror_orientation.right_mirror.normal_vec.x", true);
  this->opt_mirror_orientation_rm_nv_y = this->declare_parameter<bool>(
    "optimization.optimization_mirror_orientation.right_mirror.normal_vec.y", true);
  this->opt_mirror_orientation_rm_nv_z = this->declare_parameter<bool>(
    "optimization.optimization_mirror_orientation.right_mirror.normal_vec.z", true);

  this->opt_mirror_orientation_lm_sv_x = this->declare_parameter<bool>(
    "optimization.optimization_mirror_orientation.left_mirror.support_vec.x", true);
  this->opt_mirror_orientation_lm_sv_y = this->declare_parameter<bool>(
    "optimization.optimization_mirror_orientation.left_mirror.support_vec.y", true);
  this->opt_mirror_orientation_lm_sv_z = this->declare_parameter<bool>(
    "optimization.optimization_mirror_orientation.left_mirror.support_vec.z", true);
  this->opt_mirror_orientation_lm_nv_x = this->declare_parameter<bool>(
    "optimization.optimization_mirror_orientation.left_mirror.normal_vec.x", true);
  this->opt_mirror_orientation_lm_nv_y = this->declare_parameter<bool>(
    "optimization.optimization_mirror_orientation.left_mirror.normal_vec.y", true);
  this->opt_mirror_orientation_lm_nv_z = this->declare_parameter<bool>(
    "optimization.optimization_mirror_orientation.left_mirror.normal_vec.z", true);

  this->opt_mirror_orientation_plane_sv_x = this->declare_parameter<bool>(
    "optimization.optimization_mirror_orientation.calibration_plane.support_vec.x", true);
  this->opt_mirror_orientation_plane_sv_y = this->declare_parameter<bool>(
    "optimization.optimization_mirror_orientation.calibration_plane.support_vec.y", true);
  this->opt_mirror_orientation_plane_sv_z = this->declare_parameter<bool>(
    "optimization.optimization_mirror_orientation.calibration_plane.support_vec.z", true);

  this->opt_mirror_orientation_plane_nv_x = this->declare_parameter<bool>(
    "optimization.optimization_mirror_orientation.calibration_plane.normal_vec.x", true);
  this->opt_mirror_orientation_plane_nv_y = this->declare_parameter<bool>(
    "optimization.optimization_mirror_orientation.calibration_plane.normal_vec.y", true);
  this->opt_mirror_orientation_plane_nv_z = this->declare_parameter<bool>(
    "optimization.optimization_mirror_orientation.calibration_plane.normal_vec.z", true);

  this->plane_support_vec_ =
    this->declare_parameter<std::vector<double>>("calibration_plane.support_vec", {0.0, 0.0, 0.0});
  this->plane_helper_p1_ =
    this->declare_parameter<std::vector<double>>("calibration_plane.helper_p1", {0.0, 0.0, 0.0});
  this->plane_helper_p2_ =
    this->declare_parameter<std::vector<double>>("calibration_plane.helper_p2", {0.0, 0.0, 0.0});
  this->plane_helper_p3_ =
    this->declare_parameter<std::vector<double>>("calibration_plane.helper_p3", {0.0, 0.0, 0.0});

  this->front_start_angle_ =
    this->deg2rad(this->declare_parameter<double>("mirror_front.start_angle", 0.0));
  this->front_end_angle_ =
    this->deg2rad(this->declare_parameter<double>("mirror_front.end_angle", 0.0));

  this->mirror_safety_bufferzone_size_lm =
    this->declare_parameter<int>("mirror_left.mirror_safety_bufferzone_size", 2);
  this->auto_define_lm_start_angle_ =
    this->declare_parameter<bool>("mirror_left.auto_define_start_angle", false);
  this->auto_define_lm_end_angle_ =
    this->declare_parameter<bool>("mirror_left.auto_define_end_angle", false);
  this->auto_define_lm_angle_mode_ =
    this->declare_parameter<int>("mirror_left.auto_define_angle_mode", 0);
  this->mirror_left_start_angle_ =
    this->deg2rad(this->declare_parameter<int>("mirror_left.start_angle", 90));
  this->mirror_left_end_angle_ =
    this->deg2rad(this->declare_parameter<int>("mirror_left.end_angle", 135));
  this->mirror_left_helper_p1_ =
    this->declare_parameter<std::vector<double>>("mirror_left.helper_p1", {0.0, 0.0, -1.0});
  this->mirror_left_helper_p2_ =
    this->declare_parameter<std::vector<double>>("mirror_left.helper_p2", {0.0, 0.0, -1.0});
  this->mirror_left_helper_p3_ =
    this->declare_parameter<std::vector<double>>("mirror_left.helper_p3", {0.0, 0.0, -1.0});
  this->mirror_left_support_vec_ =
    this->declare_parameter<std::vector<double>>("mirror_left.support_vec", {0.0, 0.0, 0.0});
  this->mirror_left_normal_vec_ =
    this->declare_parameter<std::vector<double>>("mirror_left.normal_vec", {0.6, -0.6, -0.4});

  this->mirror_safety_bufferzone_size_rm =
    this->declare_parameter<int>("mirror_right.mirror_safety_bufferzone_size", 2);
  this->auto_define_rm_start_angle_ =
    this->declare_parameter<bool>("mirror_right.auto_define_start_angle", false);
  this->auto_define_rm_end_angle_ =
    this->declare_parameter<bool>("mirror_right.auto_define_end_angle", false);
  this->auto_define_rm_angle_mode_ =
    this->declare_parameter<int>("mirror_right.auto_define_angle_mode", 0);
  this->mirror_right_start_angle_ =
    this->deg2rad(this->declare_parameter<int>("mirror_right.start_angle", -135));
  this->mirror_right_end_angle_ =
    this->deg2rad(this->declare_parameter<int>("mirror_right.end_angle", 135));
  this->mirror_right_helper_p1_ =
    this->declare_parameter<std::vector<double>>("mirror_right.helper_p1", {0.0, 0.0, -1.0});
  this->mirror_right_helper_p2_ =
    this->declare_parameter<std::vector<double>>("mirror_right.helper_p2", {0.0, 0.0, -1.0});
  this->mirror_right_helper_p3_ =
    this->declare_parameter<std::vector<double>>("mirror_right.helper_p3", {0.0, 0.0, -1.0});
  this->mirror_right_support_vec_ =
    this->declare_parameter<std::vector<double>>("mirror_right.support_vec", {0.0, 0.0, 0.0});
  this->mirror_right_normal_vec_ =
    this->declare_parameter<std::vector<double>>("mirror_right.normal_vec", {0.6, 0.6, -0.4});

  // subscriber
  if (!this->get_parameter("laser_scanner.use_pointcloud_input").as_bool()) {
    this->input_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      this->laser_scanner_topic_, 10,
      std::bind(&LidarMirrorFOVReshaperCalib::scanCallback, this, std::placeholders::_1));
  } else {
    this->input_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      this->laser_scanner_topic_, 10,
      std::bind(&LidarMirrorFOVReshaperCalib::pointcloudCallback, this, std::placeholders::_1));
  }

  // viz
  this->transformed_cloud_all_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("transformed_cloud_all", 10);
  this->transformed_cloud_lm_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("transformed_cloud_lm", 10);
  this->transformed_cloud_rm_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("transformed_cloud_rm", 10);

  this->normal_vectors_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("normal_vectors", 10);

  this->optimized_planes_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("optimized_planes", 10);
  this->optimized_plane_lm_pub_ =
    this->create_publisher<visualization_msgs::msg::Marker>("optimized_plane_lm", 10);
  this->optimized_plane_rm_pub_ =
    this->create_publisher<visualization_msgs::msg::Marker>("optimized_plane_rm", 10);
  this->optimized_plane_opt_plane_pub_ =
    this->create_publisher<visualization_msgs::msg::Marker>("optimized_plane_opt_plane", 10);

  this->cube_marker_pub_ =
    this->create_publisher<visualization_msgs::msg::Marker>("cube_marker", 10);

  // optimization
  lidarMirrorFOVReshaperTF::calcMirrorPlaneNormal(
    this->plane_helper_p1_, this->plane_helper_p2_, this->plane_helper_p3_,
    this->plane_normal_vec_);

  this->ros_ns = std::string("lidar_mirror_fov_reshaper_calibration");

  this->num_opt_algorithm = NLOPT_LN_NELDERMEAD;

  lidarMirrorFOVReshaperTF::calcMirrorPlaneNormal(
    this->mirror_left_helper_p1_, this->mirror_left_helper_p2_, this->mirror_left_helper_p3_,
    this->mirror_left_normal_vec_);

  lidarMirrorFOVReshaperTF::calcMirrorPlaneNormal(
    this->mirror_right_helper_p1_, this->mirror_right_helper_p2_, this->mirror_right_helper_p3_,
    this->mirror_right_normal_vec_);

  if (this->apply_opt_osg_settings) this->initOpenSeeGroundCalibOrientationParams();

  if (this->intensity_threshold_percentage_ > 1.0 || this->intensity_threshold_percentage_ < 0.0) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Intensity threshold percentage must be between 0 and 1!\nResetting to Default (0.5)");
    this->intensity_threshold_percentage_ = 0.5;
  };

  if (this->averaging_n_clouds > 0) {
    int scans_per_batch =
      this->optimization_buffer_size_ / this->optimization_evaluation_no_batches_;
    if ((scans_per_batch % averaging_n_clouds) != 0) {
      this->optimization_buffer_size_ =
        this->optimization_evaluation_no_batches_ * averaging_n_clouds;
      RCLCPP_WARN(
        this->get_logger(),
        "Number of scans per batch mod number of scans to average != 0. Adjusting buffer size to "
        "%d",
        this->optimization_buffer_size_);
    }
  }
};

LidarMirrorFOVReshaperCalib::~LidarMirrorFOVReshaperCalib(){};

void LidarMirrorFOVReshaperCalib::initOpenSeeGroundCalibOrientationParams()
{
  this->opt_mirror_orientation_rm_sv_x = false;
  this->opt_mirror_orientation_rm_sv_y = false;
  this->opt_mirror_orientation_rm_sv_z = false;

  this->opt_mirror_orientation_rm_nv_x = true;
  this->opt_mirror_orientation_rm_nv_y = false;
  this->opt_mirror_orientation_rm_nv_z = true;

  this->opt_mirror_orientation_lm_sv_x = false;
  this->opt_mirror_orientation_lm_sv_y = false;
  this->opt_mirror_orientation_lm_sv_z = false;

  this->opt_mirror_orientation_lm_nv_x = true;
  this->opt_mirror_orientation_lm_nv_y = false;
  this->opt_mirror_orientation_lm_nv_z = true;

  this->opt_mirror_orientation_plane_sv_x = true;
  this->opt_mirror_orientation_plane_sv_y = true;
  this->opt_mirror_orientation_plane_sv_z = true;

  this->opt_mirror_orientation_plane_nv_x = false;
  this->opt_mirror_orientation_plane_nv_y = true;
  this->opt_mirror_orientation_plane_nv_z = true;
};

void LidarMirrorFOVReshaperCalib::pointcloudCallbackAveraging(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // save raw data for stddev calculation
  pcl::PointCloud<pcl::PointXYZI> pcl_msg_left_mirror;
  pcl::PointCloud<pcl::PointXYZI> pcl_msg_right_mirror;
  pcl::PointCloud<pcl::PointXYZI> pcl_msg_front;
  pcl::PointCloud<pcl::PointXYZI> pcl_msg;
  pcl::fromROSMsg(*msg, pcl_msg);
  pcl::PointIndices * indices_left_mirror = new pcl::PointIndices;
  lidarMirrorFOVReshaperTF::getPointcloudIndices(
    pcl_msg, this->mirror_left_start_angle_, this->mirror_left_end_angle_, indices_left_mirror);

  pcl::PointIndices * indices_right_mirror = new pcl::PointIndices;
  lidarMirrorFOVReshaperTF::getPointcloudIndices(
    pcl_msg, this->mirror_right_start_angle_, this->mirror_right_end_angle_, indices_right_mirror);

  pcl::PointIndices * indices_front = new pcl::PointIndices;
  indices_front->indices.clear();
  lidarMirrorFOVReshaperTF::getPointcloudIndices(
    pcl_msg, this->front_start_angle_, this->front_end_angle_, indices_front);

  pcl::copyPointCloud(pcl_msg, *indices_left_mirror, pcl_msg_left_mirror);
  pcl::copyPointCloud(pcl_msg, *indices_right_mirror, pcl_msg_right_mirror);
  pcl::copyPointCloud(pcl_msg, *indices_front, pcl_msg_front);

  this->addPointCloudToBuffer(this->raw_pointcloud_buffer_left_mirror_, pcl_msg_left_mirror);
  this->addPointCloudToBuffer(this->raw_pointcloud_buffer_right_mirror_, pcl_msg_right_mirror);
  this->addPointCloudToBuffer(this->raw_pointcloud_buffer_front_, pcl_msg_front);

  delete indices_left_mirror;
  delete indices_right_mirror;
  delete indices_front;
  // ----------------------

  // check size of raw buffers, if == batch_size, do same as normal and clear buffer
  // if < batch_size, add to buffer and return
  if (this->raw_pointcloud_buffer_.size() < this->averaging_n_clouds) {
    pcl::PointCloud<pcl::PointXYZI> pcl_msg;
    pcl::fromROSMsg(*msg, pcl_msg);
    this->raw_pointcloud_buffer_.push_back(pcl_msg);
    RCLCPP_INFO(
      this->get_logger(), "Buffering pointclouds... Currently %ld",
      this->raw_pointcloud_buffer_.size());
    RCLCPP_INFO(this->get_logger(), "# of Batches: %ld", this->pointcloud_buffer_front_.size());
  }

  if (this->raw_pointcloud_buffer_.size() >= this->averaging_n_clouds) {
    pcl::PointCloud<pcl::PointXYZI> pcl_msg;
    pcl::fromROSMsg(*msg, pcl_msg);

    // create avg pointcloud
    pcl::PointCloud<pcl::PointXYZI> pcl_msg_avg;
    pcl_msg_avg = this->raw_pointcloud_buffer_[0];
    for (size_t i = 1; i < this->raw_pointcloud_buffer_.size(); i++) {
      for (size_t j = 0; j < pcl_msg_avg.size(); j++) {
        pcl_msg_avg[j].x += this->raw_pointcloud_buffer_[i][j].x;
        pcl_msg_avg[j].y += this->raw_pointcloud_buffer_[i][j].y;
        pcl_msg_avg[j].z += this->raw_pointcloud_buffer_[i][j].z;
        pcl_msg_avg[j].intensity += this->raw_pointcloud_buffer_[i][j].intensity;
      };
    }

    for (size_t j = 0; j < pcl_msg_avg.size(); j++) {
      pcl_msg_avg[j].x /= this->averaging_n_clouds;
      pcl_msg_avg[j].y /= this->averaging_n_clouds;
      pcl_msg_avg[j].z /= this->averaging_n_clouds;
      pcl_msg_avg[j].intensity /= this->averaging_n_clouds;
    }

    if (this->auto_define_lm_start_angle_)
      this->identifyMirrorPoints(pcl_msg_avg, 0, this->auto_define_lm_angle_mode_);

    if (this->auto_define_rm_start_angle_)
      this->identifyMirrorPoints(pcl_msg_avg, 1, this->auto_define_rm_angle_mode_);

    this->splitPointclouds(pcl_msg_avg);

    this->raw_pointcloud_buffer_.clear();
  } else {
    return;
  }

  if (
    this->pointcloud_buffer_front_.size() >=
    (unsigned int)this->optimization_evaluation_no_batches_) {
    this->optimize();
  }
  return;
};

void LidarMirrorFOVReshaperCalib::pointcloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (averaging_n_clouds > 0) {
    this->pointcloudCallbackAveraging(msg);
    return;
  }
  if (
    ((this->optimization_buffer_size_ >= 0) &&
     (this->pointcloud_buffer_front_.size() < (unsigned int)this->optimization_buffer_size_)) &&
    (!this->tmp_avg_flag)) {
    RCLCPP_INFO(
      this->get_logger(), "Buffering pointclouds... Currently %ld",
      this->pointcloud_buffer_front_.size());

    pcl::PointCloud<pcl::PointXYZI> pcl_msg;
    pcl::fromROSMsg(*msg, pcl_msg);

    if (this->auto_define_lm_start_angle_)
      this->identifyMirrorPoints(pcl_msg, 0, this->auto_define_lm_angle_mode_);

    if (this->auto_define_rm_start_angle_)
      this->identifyMirrorPoints(pcl_msg, 1, this->auto_define_rm_angle_mode_);

    this->splitPointclouds(pcl_msg);
    if (this->averaging_n_clouds > 0) {
      this->averagingResourceBuffers();
    };
  } else if (this->opt_flag_) {
    this->optimize();
    rclcpp::shutdown();
  } else
    return;
};

void LidarMirrorFOVReshaperCalib::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // @todo Test this
  if (this->laser_scans.size() < this->optimization_buffer_size_) {
    RCLCPP_INFO(this->get_logger(), "Buffering laser scans... Currently %ld", laser_scans.size());
    laser_scans.push_back(*msg);

    sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg =
      std::make_shared<sensor_msgs::msg::PointCloud2>();
    sensor_msgs::PointCloud2Modifier modifier(*cloud_msg);
    modifier.setPointCloud2Fields(
      4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
      sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(msg->ranges.size());
    sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_intensity(*cloud_msg, "intensity");

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      const float range = msg->ranges[i];
      const float angle = msg->angle_min + i * msg->angle_increment;
      *iter_x = range * std::cos(angle);
      *iter_y = range * std::sin(angle);
      *iter_z = 0.0;
      *iter_intensity = msg->intensities[i];
      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_intensity;
    }
    this->pointcloudCallback(cloud_msg);
  } else {
    this->optimize();
    rclcpp::shutdown();
  }
};

void LidarMirrorFOVReshaperCalib::identifyMirrorPoints(
  const pcl::PointCloud<pcl::PointXYZI> & src_cloud, int mirror /* = 0*/, int mode /*=0*/)
{
  pcl::PointIndices * min_max_indices = new pcl::PointIndices;

  if (mode == 0) {
    if (mirror == 0)
      this->identifyMirrorPointsSlope(
        src_cloud, this->mirror_left_start_angle_, this->mirror_left_end_angle_);
    else if (mirror == 1)
      this->identifyMirrorPointsSlope(
        src_cloud, this->mirror_right_start_angle_, this->mirror_right_end_angle_);
    else
      RCLCPP_ERROR(this->get_logger(), "Invalid mirror for identifyMirrorPoints!");
  } else if (mode == 1) {
    RCLCPP_INFO(this->get_logger(), "Identifying mirror points via mean distance...");
    if (mirror == 0) {
      this->identifyMirrorPointsMeanDist(
        src_cloud, this->mirror_left_start_angle_, this->mirror_left_end_angle_, min_max_indices);
      RCLCPP_INFO(
        this->get_logger(), "Min idx: %d, Max idx: %d", min_max_indices->indices[0],
        min_max_indices->indices[1]);
    } else if (mirror == 1) {
      this->identifyMirrorPointsMeanDist(
        src_cloud, this->mirror_right_start_angle_, this->mirror_right_end_angle_, min_max_indices);
      RCLCPP_INFO(
        this->get_logger(), "Min idx: %d, Max idx: %d", min_max_indices->indices[0],
        min_max_indices->indices[1]);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Invalid mirror for identifyMirrorPoints!");
    };
  } else {
    RCLCPP_ERROR(this->get_logger(), "Invalid mode for identifyMirrorPoints!");
  };
  delete min_max_indices;

  return;
};

void LidarMirrorFOVReshaperCalib::identifyMirrorPointsMeanDist(
  const pcl::PointCloud<pcl::PointXYZI> & src_cloud, double inital_guess_min,
  double inital_guess_max, pcl::PointIndices * min_max_indices, bool apply_sigma /* = false*/)
{
  double avg_range = 0.0;
  double sigma_range = 0.0;
  std::vector<double> ranges;

  pcl::PointIndices * inital_guess_indices = new pcl::PointIndices;
  lidarMirrorFOVReshaperTF::getPointcloudIndices(
    src_cloud, inital_guess_min, inital_guess_max, inital_guess_indices);

  pcl::PointCloud<pcl::PointXYZI> src_cloud_cropped;
  pcl::copyPointCloud(src_cloud, *inital_guess_indices, src_cloud_cropped);
  delete inital_guess_indices;

  for (pcl::PointXYZI & point : src_cloud_cropped) {
    double tmp_dist = std::sqrt(std::pow(point.x, 2) + std::pow(point.y, 2));
    avg_range += tmp_dist;
    ranges.push_back(tmp_dist);
  };

  avg_range /= src_cloud.size();

  for (const double & range : ranges) {
    sigma_range += std::pow(range - avg_range, 2);
  };
  sigma_range = std::sqrt(sigma_range / src_cloud.size());

  std::vector<int> idx_mirror_points;
  for (size_t i = 0; i < ranges.size(); i++) {
    if (
      (std::abs(ranges[i]) < avg_range + sigma_range) &&
      (std::abs(ranges[i]) > avg_range - sigma_range)) {
      idx_mirror_points.push_back(i);
    };
  };

  int idx_new_min = idx_mirror_points.front();
  int idx_new_max = idx_mirror_points.back();

  int inital_guess_min_idx = static_cast<int>(inital_guess_min - this->laser_scanner_angle_min_);
  int inital_guess_max_idx = static_cast<int>(inital_guess_max - this->laser_scanner_angle_min_);

  int diff_ig_min = inital_guess_min_idx - inital_guess_min;
  int diff_ig_max = inital_guess_max_idx - inital_guess_max;

  min_max_indices->indices.push_back(inital_guess_min_idx + diff_ig_min);
  min_max_indices->indices.push_back(inital_guess_max_idx + diff_ig_max);
};

void LidarMirrorFOVReshaperCalib::identifyMirrorPointsSlope(
  const pcl::PointCloud<pcl::PointXYZI> & src_cloud, double inital_guess_min,
  double inital_guess_max, int sliding_window_size /* = 0*/)
{
  std::vector<double> ranges, slopes;

  pcl::PointIndices * inital_guess_indices = new pcl::PointIndices;
  lidarMirrorFOVReshaperTF::getPointcloudIndices(
    src_cloud, inital_guess_min, inital_guess_max, inital_guess_indices);

  pcl::PointCloud<pcl::PointXYZI> src_cloud_cropped;
  pcl::copyPointCloud(src_cloud, *inital_guess_indices, src_cloud_cropped);
  delete inital_guess_indices;

  for (size_t i = 0; i < src_cloud_cropped.size(); i++) {
    ranges.push_back(src_cloud_cropped[i].x);
  }

  std::vector<int> idx_mirror_points;
  for (size_t i = 1 + sliding_window_size; i < ranges.size() - sliding_window_size; i++) {
    double slope = (ranges[i + sliding_window_size] - ranges[i - sliding_window_size]);
    if (sliding_window_size > 0) slope /= sliding_window_size * 2;
    slopes.push_back(slope);
  }

  std::sort(slopes.begin(), slopes.end());
  double slope_threshold = slopes[static_cast<int>(slopes.size() * 0.95)];
  RCLCPP_INFO(
    this->get_logger(), "Slope threshold (95th percentile) determined: %f", slope_threshold);

  for (size_t i = 1; i < slopes.size(); i++) {
    double slope_delta = std::abs(slopes[i]) - std::abs(slopes[i - 1]);
    if (slope_delta > std::abs(slope_threshold)) {
      RCLCPP_INFO(this->get_logger(), "Jump at idx:%ld", i);
    }
  }

  this->mirror_right_start_angle_ = 0.0;
  this->mirror_right_end_angle_ = 0.0;
  this->mirror_left_start_angle_ = 0.0;
  this->mirror_left_end_angle_ = 0.0;
};

void LidarMirrorFOVReshaperCalib::splitPointclouds(
  const pcl::PointCloud<pcl::PointXYZI> & src_cloud)
{
  // split into 3 parts
  pcl::PointCloud<pcl::PointXYZI> pcl_msg_left_mirror;
  pcl::PointCloud<pcl::PointXYZI> pcl_msg_right_mirror;
  pcl::PointCloud<pcl::PointXYZI> pcl_msg_front;

  pcl::PointIndices * indices_left_mirror = new pcl::PointIndices;
  lidarMirrorFOVReshaperTF::getPointcloudIndices(
    src_cloud, this->mirror_left_start_angle_, this->mirror_left_end_angle_, indices_left_mirror);

  pcl::PointIndices * indices_right_mirror = new pcl::PointIndices;
  lidarMirrorFOVReshaperTF::getPointcloudIndices(
    src_cloud, this->mirror_right_start_angle_, this->mirror_right_end_angle_,
    indices_right_mirror);

  pcl::PointIndices * indices_front = new pcl::PointIndices;
  indices_front->indices.clear();
  lidarMirrorFOVReshaperTF::getPointcloudIndices(
    src_cloud, this->front_start_angle_, this->front_end_angle_, indices_front);

  pcl::copyPointCloud(src_cloud, *indices_left_mirror, pcl_msg_left_mirror);
  pcl::copyPointCloud(src_cloud, *indices_right_mirror, pcl_msg_right_mirror);
  pcl::copyPointCloud(src_cloud, *indices_front, pcl_msg_front);

  this->addPointCloudToBuffer(this->pointcloud_buffer_left_mirror_, pcl_msg_left_mirror);
  this->addPointCloudToBuffer(this->pointcloud_buffer_right_mirror_, pcl_msg_right_mirror);
  this->addPointCloudToBuffer(this->pointcloud_buffer_front_, pcl_msg_front);

  this->addPointCloudToBuffer(this->pointcloud_buffer_, src_cloud);

  delete indices_left_mirror;
  delete indices_right_mirror;
  delete indices_front;
};

void LidarMirrorFOVReshaperCalib::addPointCloudToBuffer(
  std::vector<pcl::PointCloud<pcl::PointXYZI>> & pc_buffer,
  const pcl::PointCloud<pcl::PointXYZI> & pc_cmp)
{
  if (pc_buffer.size() == 0) {
    pc_buffer.push_back(pc_cmp);
    return;
  } else {
    if (pc_buffer.back().size() != pc_cmp.size()) {
      RCLCPP_ERROR(
        this->get_logger(), "Pointcloud buffer size mismatch!\nBuffer: %ld, New: %ld",
        pc_buffer.back().size(), pc_cmp.size());
      rclcpp::shutdown();
    } else
      pc_buffer.push_back(pc_cmp);
  }
};
double pointToLineDistance(
  const pcl::PointXYZI & point, const Eigen::Vector3d & linePoint,
  const Eigen::Vector3d & lineDirection)
{
  Eigen::Vector3d p(point.x, point.y, point.z);
  Eigen::Vector3d diff = p - linePoint;
  Eigen::Vector3d crossProduct = diff.cross(lineDirection);
  return crossProduct.norm() / lineDirection.norm();
};

// Function to fit a line to the point cloud and identify outliers
std::vector<int> identifyOutliers(const pcl::PointCloud<pcl::PointXYZI> & cloud, double threshold)
{
  size_t n = cloud.size();
  if (n < 2) {
    std::cerr << "Not enough points to fit a line." << std::endl;
    return {};
  }

  Eigen::Vector3d mean(0, 0, 0);
  for (const auto & point : cloud) {
    mean += Eigen::Vector3d(point.x, point.y, point.z);
  }
  mean /= n;

  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const auto & point : cloud) {
    Eigen::Vector3d diff = Eigen::Vector3d(point.x, point.y, point.z) - mean;
    covariance += diff * diff.transpose();
  }
  covariance /= n;

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(covariance);
  Eigen::Vector3d lineDirection = eig.eigenvectors().col(2);

  std::vector<pcl::PointXYZI> outliers;
  std::vector<int> outlier_indices;
  for (int i = 0; i < n; ++i) {
    double distance = pointToLineDistance(cloud.at(i), mean, lineDirection);
    if (distance > threshold) {
      outliers.push_back(cloud.at(i));
      outlier_indices.push_back(i);
    };
  }
  return outlier_indices;
};

void LidarMirrorFOVReshaperCalib::getHighestIntensityIdx(
  std::vector<pcl::PointCloud<pcl::PointXYZI>> & pointcloud_buffer, std::vector<int> & hI_idx,
  bool mirror_left /* = 0*/)
{
  for (size_t cloud_index = 0; cloud_index < pointcloud_buffer.size(); ++cloud_index) {
    std::vector<int> indices_highest_intensity;
    int idx_highest_intensity = -1;
    double max_intensity = 0.0;
    double intensity_threshold = 0.0;

    for (size_t i = 0; i < pointcloud_buffer[cloud_index].size(); ++i) {
      if (pointcloud_buffer[cloud_index][i].intensity >= max_intensity) {
        max_intensity = pointcloud_buffer[cloud_index][i].intensity;
      }
    }
    intensity_threshold = max_intensity;
    intensity_threshold *= this->intensity_threshold_percentage_;

    for (size_t i = 0; i < pointcloud_buffer[cloud_index].size(); ++i) {
      if (pointcloud_buffer[cloud_index].at(i).intensity >= intensity_threshold) {
        indices_highest_intensity.push_back(
          i);  // because of the order of execution, already sorted
      };
    }

    if (indices_highest_intensity.size() > 1) {
      if ((indices_highest_intensity.size() % 2) == 0) {
        idx_highest_intensity =
          indices_highest_intensity[(indices_highest_intensity.size() / 2) - 1];
      } else {
        idx_highest_intensity =
          indices_highest_intensity[std::round(indices_highest_intensity.size() / 2) - 1];
      }
    } else if (indices_highest_intensity.size() == 1) {
      idx_highest_intensity = indices_highest_intensity[0];
    } else {
      RCLCPP_ERROR(this->get_logger(), "No points found with intensity above threshold!");
      return;
    };

    if (
      mirror_left &&
      idx_highest_intensity >= static_cast<int>(pointcloud_buffer[cloud_index].size()) -
                                 this->mirror_safety_bufferzone_size_lm) {
      if (this->mirror_safety_bufferzone_size_lm != 0) {
        RCLCPP_WARN(
          this->get_logger(),
          "Highest intensity point is too close to the right mirror edge! "
          "Index: %d, Intensity: %f\nintensity threshold was set to %f\n Setting index to %d",
          idx_highest_intensity, max_intensity, intensity_threshold,
          (static_cast<int>(pointcloud_buffer[cloud_index].size()) -
           this->mirror_safety_bufferzone_size_lm - 1));
        idx_highest_intensity = static_cast<int>(pointcloud_buffer[cloud_index].size()) -
                                this->mirror_safety_bufferzone_size_lm - 1;
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "Highest intensity point is too close to the right mirror edge! "
          "Index: %d, Intensity: %f\nintensity threshold was set to %f\n Setting index to %d",
          idx_highest_intensity, max_intensity, intensity_threshold,
          static_cast<int>(pointcloud_buffer[cloud_index].size()) -
            this->mirror_safety_bufferzone_size_lm);
      };
    } else if (!mirror_left && idx_highest_intensity <= this->mirror_safety_bufferzone_size_rm) {
      if (this->mirror_safety_bufferzone_size_rm != 0) {
        RCLCPP_WARN(
          this->get_logger(),
          "Highest intensity point is too close to the left mirror edge! "
          "Index: %d, Intensity: %f\nintensity threshold was set to %f\n Setting index to %d",
          idx_highest_intensity, max_intensity, intensity_threshold,
          this->mirror_safety_bufferzone_size_rm + 1);
        idx_highest_intensity = this->mirror_safety_bufferzone_size_rm + 1;
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "Highest intensity point is too close to the left mirror edge! "
          "Index: %d, Intensity: %f\nintensity threshold was set to %f\n Setting index to %d",
          idx_highest_intensity, max_intensity, intensity_threshold,
          this->mirror_safety_bufferzone_size_rm);
      }
    }

    if (this->interpolation_window > 0) {
      pcl::PointXYZI highest_intensity_interpp, interpolated_point_minus_1,
        interpolated_point_plus_1, interpolated_point_minus_2, interpolated_point_plus_2;
      try {
        if (mirror_left) {
          if (
            idx_highest_intensity + this->interpolation_window <
            pointcloud_buffer[cloud_index].size())  // all indices used for interpolation are valid
          {
            highest_intensity_interpp = this->interpolate(
              pointcloud_buffer[cloud_index].at(idx_highest_intensity - this->interpolation_window),
              pointcloud_buffer[cloud_index].at(idx_highest_intensity + this->interpolation_window),
              0.5, max_intensity);

            pointcloud_buffer[cloud_index].at(idx_highest_intensity) = highest_intensity_interpp;

            interpolated_point_minus_1 = this->interpolate(
              pointcloud_buffer[cloud_index].at(idx_highest_intensity - this->interpolation_window),
              pointcloud_buffer[cloud_index].at(idx_highest_intensity), 0.25, max_intensity);

            pointcloud_buffer[cloud_index].at(idx_highest_intensity - 1) =
              interpolated_point_minus_1;

            interpolated_point_plus_1 = this->interpolate(
              pointcloud_buffer[cloud_index].at(idx_highest_intensity),
              pointcloud_buffer[cloud_index].at(idx_highest_intensity + this->interpolation_window),
              0.25, max_intensity);

            pointcloud_buffer[cloud_index].at(idx_highest_intensity + 1) =
              interpolated_point_plus_1;

            interpolated_point_minus_2 = this->interpolate(
              pointcloud_buffer[cloud_index].at(idx_highest_intensity - this->interpolation_window),
              pointcloud_buffer[cloud_index].at(idx_highest_intensity), 0.75, max_intensity);

            pointcloud_buffer[cloud_index].at(idx_highest_intensity - 2) =
              interpolated_point_minus_2;

            interpolated_point_plus_2 = this->interpolate(
              pointcloud_buffer[cloud_index].at(idx_highest_intensity),
              pointcloud_buffer[cloud_index].at(idx_highest_intensity + this->interpolation_window),
              0.75, max_intensity);

            pointcloud_buffer[cloud_index].at(idx_highest_intensity + 2) =
              interpolated_point_plus_2;
          } else {
            highest_intensity_interpp = this->interpolate(
              pointcloud_buffer[cloud_index].at(idx_highest_intensity - 1),
              pointcloud_buffer[cloud_index].at(idx_highest_intensity), 0.5, max_intensity);
          }
        } else {  // rm

          if (
            idx_highest_intensity - this->interpolation_window >=
            0)  // all indices used for interpolation are valid
          {
            highest_intensity_interpp = this->interpolate(
              pointcloud_buffer[cloud_index].at(idx_highest_intensity - this->interpolation_window),
              pointcloud_buffer[cloud_index].at(idx_highest_intensity + this->interpolation_window),
              0.5, max_intensity);

            pointcloud_buffer[cloud_index].at(idx_highest_intensity) = highest_intensity_interpp;

            interpolated_point_minus_1 = this->interpolate(
              pointcloud_buffer[cloud_index].at(idx_highest_intensity - this->interpolation_window),
              pointcloud_buffer[cloud_index].at(idx_highest_intensity), 0.25, max_intensity);

            pointcloud_buffer[cloud_index].at(idx_highest_intensity - 1) =
              interpolated_point_minus_1;

            interpolated_point_plus_1 = this->interpolate(
              pointcloud_buffer[cloud_index].at(idx_highest_intensity),
              pointcloud_buffer[cloud_index].at(idx_highest_intensity + this->interpolation_window),
              0.25, max_intensity);

            pointcloud_buffer[cloud_index].at(idx_highest_intensity + 1) =
              interpolated_point_plus_1;
          }
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Interpolation Error: %s", e.what());
        RCLCPP_ERROR(
          this->get_logger(), "Size of pointcloud: %ld", pointcloud_buffer[cloud_index].size());
        RCLCPP_ERROR(this->get_logger(), "Left mirror: %d", mirror_left);
        RCLCPP_ERROR(this->get_logger(), "Index: %d", idx_highest_intensity);
      }
    } else if (
      this->interpolation_window == 0 && idx_highest_intensity > 0 &&
      idx_highest_intensity < pointcloud_buffer[cloud_index].size() - 1) {
      // interpolate between highest intensity point-1 and highest intensity point+1
      pcl::PointXYZI interpolated_point;
      interpolated_point = this->interpolate(
        pointcloud_buffer[cloud_index].at(idx_highest_intensity - 1),
        pointcloud_buffer[cloud_index].at(idx_highest_intensity + 1), 0.5, max_intensity);

      pointcloud_buffer[cloud_index].at(idx_highest_intensity) = interpolated_point;
    }

    // set intensity for all other points to 0
    for (size_t i = 0; i < pointcloud_buffer[cloud_index].size(); i++) {
      if (i != idx_highest_intensity) {
        pointcloud_buffer[cloud_index].at(i).intensity = 0.0;
      }
    };

    hI_idx.push_back(idx_highest_intensity);
  }

  RCLCPP_INFO(
    this->get_logger(), "Avg idx high intensity: %ld (mirror left: %d)",
    std::accumulate(hI_idx.begin(), hI_idx.end(), 0) / hI_idx.size(), mirror_left);
};

pcl::PointXYZI LidarMirrorFOVReshaperCalib::interpolate(
  const pcl::PointXYZI & p1, const pcl::PointXYZI & p2, float t, double intensity)
{
  pcl::PointXYZI p;
  p.x = p1.x + t * (p2.x - p1.x);
  p.y = p1.y + t * (p2.y - p1.y);
  p.z = p1.z + t * (p2.z - p1.z);
  p.intensity = intensity;
  return p;
}

void LidarMirrorFOVReshaperCalib::transformPointclouds()
{
  // print normal and support vectors
  RCLCPP_INFO(
    this->get_logger(), "Left mirror normal vector: %f, %f, %f", this->mirror_left_normal_vec_[0],
    this->mirror_left_normal_vec_[1], this->mirror_left_normal_vec_[2]);

  RCLCPP_INFO(
    this->get_logger(), "Left mirror support vector: %f, %f, %f", this->mirror_left_support_vec_[0],
    this->mirror_left_support_vec_[1], this->mirror_left_support_vec_[2]);

  RCLCPP_INFO(
    this->get_logger(), "Right mirror normal vector: %f, %f, %f", this->mirror_right_normal_vec_[0],
    this->mirror_right_normal_vec_[1], this->mirror_right_normal_vec_[2]);

  RCLCPP_INFO(
    this->get_logger(), "Right mirror support vector: %f, %f, %f",
    this->mirror_right_support_vec_[0], this->mirror_right_support_vec_[1],
    this->mirror_right_support_vec_[2]);

  for (size_t i = 0; i < pointcloud_buffer_left_mirror_.size(); i++) {
    pcl::PointCloud<pcl::PointXYZI> * transformed_cloud_combined =
      new pcl::PointCloud<pcl::PointXYZI>;
    pcl::PointCloud<pcl::PointXYZI> * transformed_cloud_left_mirror =
      new pcl::PointCloud<pcl::PointXYZI>;
    pcl::PointCloud<pcl::PointXYZI> * transformed_cloud_right_mirror =
      new pcl::PointCloud<pcl::PointXYZI>;

    lidarMirrorFOVReshaperTF::transformCloud(
      this->pointcloud_buffer_front_[i], this->pointcloud_buffer_left_mirror_[i],
      this->pointcloud_buffer_right_mirror_[i], this->mirror_left_normal_vec_,
      this->mirror_left_support_vec_, this->mirror_right_normal_vec_,
      this->mirror_right_support_vec_, transformed_cloud_combined, transformed_cloud_left_mirror,
      transformed_cloud_right_mirror);

    if (
      this->viz_transformed_cloud_all || this->viz_transformed_cloud_lm ||
      this->viz_transformed_cloud_rm) {
      this->visualizeTransformedPointclouds(
        *transformed_cloud_combined, *transformed_cloud_right_mirror,
        *transformed_cloud_left_mirror);
    };

    this->transformed_pointcloud_buffer_.push_back(*transformed_cloud_combined);
    this->transformed_pointcloud_buffer_left_mirror_.push_back(*transformed_cloud_left_mirror);
    this->transformed_pointcloud_buffer_right_mirror_.push_back(*transformed_cloud_right_mirror);

    delete transformed_cloud_combined;
    delete transformed_cloud_left_mirror;
    delete transformed_cloud_right_mirror;
  };
};

void LidarMirrorFOVReshaperCalib::averagingScan(){
  // @todo implement
};

void LidarMirrorFOVReshaperCalib::averagingResourceBuffers()
{
  pcl::PointCloud<pcl::PointXYZI> averaged_cloud_left_mirror, averaged_cloud_right_mirror,
    averaged_cloud_front, averaged_cloud;

  averaged_cloud_left_mirror.resize(
    this->pointcloud_buffer_left_mirror_.at(0).size(), pcl::PointXYZI(0, 0, 0, 0));
  averaged_cloud_right_mirror.resize(
    this->pointcloud_buffer_right_mirror_.at(0).size(), pcl::PointXYZI(0, 0, 0, 0));
  averaged_cloud_front.resize(
    this->pointcloud_buffer_front_.at(0).size(), pcl::PointXYZI(0, 0, 0, 0));
  averaged_cloud.resize(this->pointcloud_buffer_.at(0).size(), pcl::PointXYZI(0, 0, 0, 0));

  if (this->pointcloud_buffer_left_mirror_.size() == this->averaging_n_clouds) {
    RCLCPP_INFO(
      this->get_logger(), "Averaging pointclouds... Currently %ld",
      this->pointcloud_buffer_left_mirror_.size());
    RCLCPP_INFO(this->get_logger(), "averaging_n_clouds: %d", this->averaging_n_clouds);
    // averaged_cloud_left_mirror.insert(
    //   averaged_cloud_left_mirror.end(), this->pointcloud_buffer_left_mirror_[0].begin(),
    //   this->pointcloud_buffer_left_mirror_[0].end());
    // averaged_cloud_right_mirror.insert(
    //   averaged_cloud_right_mirror.end(), this->pointcloud_buffer_right_mirror_[0].begin(),
    //   this->pointcloud_buffer_right_mirror_[0].end());
    // averaged_cloud_front.insert(
    //   averaged_cloud_front.end(), this->pointcloud_buffer_front_[0].begin(),
    //   this->pointcloud_buffer_front_[0].end());
    // averaged_cloud.insert(
    //   averaged_cloud.end(), this->pointcloud_buffer_[0].begin(), this->pointcloud_buffer_[0].end());
    if (
      this->pointcloud_buffer_left_mirror_.size() != this->pointcloud_buffer_right_mirror_.size() ||
      this->pointcloud_buffer_left_mirror_.size() != this->pointcloud_buffer_front_.size() ||
      this->pointcloud_buffer_left_mirror_.size() != this->pointcloud_buffer_.size()) {
      RCLCPP_ERROR(this->get_logger(), "Pointcloud buffer sizes do not match! Exiting...");
      RCLCPP_ERROR(
        this->get_logger(), "Left mirror: %ld", this->pointcloud_buffer_left_mirror_.size());
      RCLCPP_ERROR(
        this->get_logger(), "Right mirror: %ld", this->pointcloud_buffer_right_mirror_.size());
      RCLCPP_ERROR(this->get_logger(), "Front: %ld", this->pointcloud_buffer_front_.size());
      RCLCPP_ERROR(this->get_logger(), "All: %ld", this->pointcloud_buffer_.size());
      rclcpp::shutdown();
    };

    // ürint size of all averaged_cloud_left_mirror and all other buffers
    RCLCPP_INFO(
      this->get_logger(), "Size of averaged_cloud_left_mirror: %ld",
      averaged_cloud_left_mirror.size());
    RCLCPP_INFO(
      this->get_logger(), "Size of averaged_cloud_right_mirror: %ld",
      averaged_cloud_right_mirror.size());
    RCLCPP_INFO(
      this->get_logger(), "Size of averaged_cloud_front: %ld", averaged_cloud_front.size());
    RCLCPP_INFO(this->get_logger(), "Size of averaged_cloud: %ld", averaged_cloud.size());
    RCLCPP_INFO(
      this->get_logger(), "Size of pointcloud_buffer_front_: %ld",
      this->pointcloud_buffer_front_.at(0).size());

    try {
      for (size_t i = 0; i < this->pointcloud_buffer_left_mirror_.size(); i++) {
        // Sum the points in left mirror point cloud buffer
        for (size_t j = 0; j < this->pointcloud_buffer_left_mirror_.at(i).size(); j++) {
          averaged_cloud_left_mirror.at(j).x += this->pointcloud_buffer_left_mirror_[i].at(j).x;
          averaged_cloud_left_mirror.at(j).y += this->pointcloud_buffer_left_mirror_[i].at(j).y;
          averaged_cloud_left_mirror.at(j).z += this->pointcloud_buffer_left_mirror_[i].at(j).z;
          averaged_cloud_left_mirror.at(j).intensity +=
            this->pointcloud_buffer_left_mirror_[i].at(j).intensity;
        }

        // Sum the points in right mirror point cloud buffer
        for (size_t j = 0; j < this->pointcloud_buffer_right_mirror_.at(i).size(); j++) {
          averaged_cloud_right_mirror.at(j).x += this->pointcloud_buffer_right_mirror_[i].at(j).x;
          averaged_cloud_right_mirror.at(j).y += this->pointcloud_buffer_right_mirror_[i].at(j).y;
          averaged_cloud_right_mirror.at(j).z += this->pointcloud_buffer_right_mirror_[i].at(j).z;
          averaged_cloud_right_mirror.at(j).intensity +=
            this->pointcloud_buffer_right_mirror_[i].at(j).intensity;
        }

        for (size_t j = 0; j < this->pointcloud_buffer_front_.at(i).size(); j++) {
          if (j >= averaged_cloud_front.size()) {
            RCLCPP_ERROR(this->get_logger(), "Index out of bounds: %ld", j);
            RCLCPP_ERROR(
              this->get_logger(), "Size of averaged_cloud_front: %ld", averaged_cloud_front.size());
            RCLCPP_ERROR(
              this->get_logger(), "Size of pointcloud_buffer_front_: %ld",
              this->pointcloud_buffer_front_.at(i).size());
            rclcpp::shutdown();
          }
          averaged_cloud_front.at(j).x += this->pointcloud_buffer_front_[i].at(j).x;
          averaged_cloud_front.at(j).y += this->pointcloud_buffer_front_[i].at(j).y;
          averaged_cloud_front.at(j).z += this->pointcloud_buffer_front_[i].at(j).z;
          averaged_cloud_front.at(j).intensity += this->pointcloud_buffer_front_[i].at(j).intensity;
        }

        // Sum the points in the general point cloud buffer
        for (size_t j = 0; j < this->pointcloud_buffer_.at(i).size(); j++) {
          averaged_cloud.at(j).x += this->pointcloud_buffer_[i].at(j).x;
          averaged_cloud.at(j).y += this->pointcloud_buffer_[i].at(j).y;
          averaged_cloud.at(j).z += this->pointcloud_buffer_[i].at(j).z;
          averaged_cloud.at(j).intensity += this->pointcloud_buffer_[i].at(j).intensity;
        }
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Error in averaging scans: %s", e.what());
      rclcpp::shutdown();
    }

    // Average the points by dividing by the number of point clouds
    size_t buffer_size = this->pointcloud_buffer_left_mirror_.size();
    for (size_t j = 0; j < averaged_cloud_left_mirror.size(); j++) {
      averaged_cloud_left_mirror.at(j).x /= buffer_size;
      averaged_cloud_left_mirror.at(j).y /= buffer_size;
      averaged_cloud_left_mirror.at(j).z /= buffer_size;
      averaged_cloud_left_mirror.at(j).intensity /= buffer_size;
    }

    for (size_t j = 0; j < averaged_cloud_right_mirror.size(); j++) {
      averaged_cloud_right_mirror.at(j).x /= buffer_size;
      averaged_cloud_right_mirror.at(j).y /= buffer_size;
      averaged_cloud_right_mirror.at(j).z /= buffer_size;
      averaged_cloud_right_mirror.at(j).intensity /= buffer_size;
    }

    for (size_t j = 0; j < averaged_cloud_front.size(); j++) {
      averaged_cloud_front.at(j).x /= buffer_size;
      averaged_cloud_front.at(j).y /= buffer_size;
      averaged_cloud_front.at(j).z /= buffer_size;
      averaged_cloud_front.at(j).intensity /= buffer_size;
    }

    for (size_t j = 0; j < averaged_cloud.size(); j++) {
      averaged_cloud.at(j).x /= buffer_size;
      averaged_cloud.at(j).y /= buffer_size;
      averaged_cloud.at(j).z /= buffer_size;
      averaged_cloud.at(j).intensity /= buffer_size;
    }
  } else if ((this->pointcloud_buffer_left_mirror_.size() % this->averaging_n_clouds + 1) == 0) {
    // take the last n scans and average them
    size_t starting_idx = this->pointcloud_buffer_left_mirror_.size() % this->averaging_n_clouds;
    averaged_cloud_left_mirror.insert(
      averaged_cloud_left_mirror.end(), this->pointcloud_buffer_left_mirror_[starting_idx].begin(),
      this->pointcloud_buffer_left_mirror_[starting_idx].end());
    averaged_cloud_right_mirror.insert(
      averaged_cloud_right_mirror.end(),
      this->pointcloud_buffer_right_mirror_[starting_idx].begin(),
      this->pointcloud_buffer_right_mirror_[starting_idx].end());
    averaged_cloud_front.insert(
      averaged_cloud_front.end(), this->pointcloud_buffer_front_[starting_idx].begin(),
      this->pointcloud_buffer_front_[starting_idx].end());
    averaged_cloud.insert(
      averaged_cloud.end(), this->pointcloud_buffer_[starting_idx].begin(),
      this->pointcloud_buffer_[starting_idx].end());

    for (size_t i = starting_idx; i < this->pointcloud_buffer_left_mirror_.size(); i++) {
      for (size_t j = 0; j < this->pointcloud_buffer_left_mirror_.at(i).size(); j++) {
        averaged_cloud_left_mirror.at(j).x += this->pointcloud_buffer_left_mirror_[i].at(j).x;
        averaged_cloud_left_mirror.at(j).x /= 2;
        averaged_cloud_left_mirror.at(j).y += this->pointcloud_buffer_left_mirror_[i].at(j).y;
        averaged_cloud_left_mirror.at(j).y /= 2;
        averaged_cloud_left_mirror.at(j).z += this->pointcloud_buffer_left_mirror_[i].at(j).z;
        averaged_cloud_left_mirror.at(j).z /= 2;
      };

      for (size_t j = 0; j < this->pointcloud_buffer_right_mirror_.at(i).size(); j++) {
        averaged_cloud_right_mirror.at(j).x += this->pointcloud_buffer_right_mirror_[i].at(j).x;
        averaged_cloud_right_mirror.at(j).x /= 2;
        averaged_cloud_right_mirror.at(j).y += this->pointcloud_buffer_right_mirror_[i].at(j).y;
        averaged_cloud_right_mirror.at(j).y /= 2;
        averaged_cloud_right_mirror.at(j).z += this->pointcloud_buffer_right_mirror_[i].at(j).z;
        averaged_cloud_right_mirror.at(j).z /= 2;
      };

      for (size_t j = 0; j < this->pointcloud_buffer_front_.at(i).size(); j++) {
        averaged_cloud_front.at(j).x += this->pointcloud_buffer_front_[i].at(j).x;
        averaged_cloud_front.at(j).x /= 2;
        averaged_cloud_front.at(j).y += this->pointcloud_buffer_front_[i].at(j).y;
        averaged_cloud_front.at(j).y /= 2;
        averaged_cloud_front.at(j).z += this->pointcloud_buffer_front_[i].at(j).z;
        averaged_cloud_front.at(j).z /= 2;
      };

      for (size_t j = 0; j < this->pointcloud_buffer_.at(i).size(); j++) {
        averaged_cloud.at(j).x += this->pointcloud_buffer_[i].at(j).x;
        // averaged_cloud.at(j).x /= 2;
        averaged_cloud.at(j).y += this->pointcloud_buffer_[i].at(j).y;
        // averaged_cloud.at(j).y /= 2;
        averaged_cloud.at(j).z += this->pointcloud_buffer_[i].at(j).z;
        // averaged_cloud.at(j).z /= 2;
      };
    };
  } else {
    // this->pointcloud_buffer_left_mirror_.clear();
    // this->pointcloud_buffer_right_mirror_.clear();
    // this->pointcloud_buffer_front_.clear();
    // this->pointcloud_buffer_.clear();
    return;
  }
  for (size_t i = 0; i < this->averaging_n_clouds; i++) {
    this->pointcloud_buffer_left_mirror_.pop_back();
    this->pointcloud_buffer_right_mirror_.pop_back();
    this->pointcloud_buffer_front_.pop_back();
    this->pointcloud_buffer_.pop_back();
  }
  this->tmp_avg_flag = true;

  this->pointcloud_buffer_left_mirror_.push_back(averaged_cloud_left_mirror);
  this->pointcloud_buffer_right_mirror_.push_back(averaged_cloud_right_mirror);
  this->pointcloud_buffer_front_.push_back(averaged_cloud_front);
  this->pointcloud_buffer_.push_back(averaged_cloud);
};

void LidarMirrorFOVReshaperCalib::getTargetPoint(
  std::vector<pcl::PointCloud<pcl::PointXYZI>> & pointcloud_buffer, std::vector<int> & hI_idx,
  bool mode /* = 0 */, bool mirror_left /* = 0 */)
{
  if (pointcloud_buffer.size() == 0) {
    RCLCPP_ERROR(this->get_logger(), "Pointcloud buffer is empty!");
    return;
  };

  if (!mode) {
    this->getHighestIntensityIdx(pointcloud_buffer, hI_idx, mirror_left);
    return;
  };

  for (size_t cloud_index = 0; cloud_index < pointcloud_buffer.size(); ++cloud_index) {
    std::vector<int> outliers =
      identifyOutliers(pointcloud_buffer[cloud_index], this->filter_dist_threshold);
    std::vector<pcl::PointXYZI> outliers_points(outliers.size(), pcl::PointXYZI(0, 0, 0, 0));
    for (size_t i = 0; i < outliers.size(); i++) {
      outliers_points.at(i) = (pointcloud_buffer[cloud_index].at(outliers[i]));
    };
    std::vector<double> outlier_dists(outliers.size(), 0.0);
    for (size_t i = 0; i < outliers.size(); i++) {
      outlier_dists[i] = std::sqrt(
        std::pow(outliers_points[i].x, 2) + std::pow(outliers_points[i].y, 2) +
        std::pow(outliers_points[i].z, 2));
    };
    int idx_highest_intensity;

    if (outliers.size() > 1) {
      idx_highest_intensity = std::distance(
        outlier_dists.begin(), std::max_element(outlier_dists.begin(), outlier_dists.end()));

    } else if (outliers.size() == 0) {
      RCLCPP_ERROR(
        this->get_logger(),
        "No outliers found!. Possible reason: filter_distance_threshold too high, currently set to "
        "%f",
        this->filter_dist_threshold);
      return;
    } else {
      idx_highest_intensity = outliers[0];
    }

    if (idx_highest_intensity >= pointcloud_buffer.at(cloud_index).size()) {
      RCLCPP_ERROR(
        this->get_logger(), "Index of highest intensity point is out of bounds! Index: %d",
        idx_highest_intensity);
      return;
    }
    hI_idx.push_back(idx_highest_intensity);

    double max_intensity = DBL_MAX / 2;
    pcl::PointXYZI interpolated_point, interpolated_point_m1, interpolated_point_p1;
    if (
      idx_highest_intensity >= 2 &&
      idx_highest_intensity < pointcloud_buffer[cloud_index].size() - 2) {
      interpolated_point = this->interpolate(
        pointcloud_buffer[cloud_index].at(idx_highest_intensity - 2),
        pointcloud_buffer[cloud_index].at(idx_highest_intensity + 2), 0.5, max_intensity);

      pointcloud_buffer[cloud_index].at(idx_highest_intensity) = interpolated_point;
      interpolated_point_m1 = this->interpolate(
        pointcloud_buffer[cloud_index].at(idx_highest_intensity - 2),
        pointcloud_buffer[cloud_index].at(idx_highest_intensity), 0.5, max_intensity);
      pointcloud_buffer[cloud_index].at(idx_highest_intensity - 1) = interpolated_point_m1;

      interpolated_point_p1 = this->interpolate(
        pointcloud_buffer[cloud_index].at(idx_highest_intensity),
        pointcloud_buffer[cloud_index].at(idx_highest_intensity + 2), 0.5, max_intensity);
      pointcloud_buffer[cloud_index].at(idx_highest_intensity + 1) = interpolated_point_p1;
    } else if (
      idx_highest_intensity >= 1 &&
      idx_highest_intensity < pointcloud_buffer[cloud_index].size() - 1) {
      interpolated_point = this->interpolate(
        pointcloud_buffer[cloud_index].at(idx_highest_intensity - 1),
        pointcloud_buffer[cloud_index].at(idx_highest_intensity + 1), 0.5, max_intensity);

      pointcloud_buffer[cloud_index].at(idx_highest_intensity) = interpolated_point;
    } else {
      RCLCPP_WARN(
        this->get_logger(), "Highest intensity point is too close to the edge! Index: %d",
        idx_highest_intensity);
    }
  };
};

void LidarMirrorFOVReshaperCalib::optimize()
{
  this->getTargetPoint(
    this->pointcloud_buffer_left_mirror_, this->indices_high_intensity_lm_, 0, 1);
  this->getTargetPoint(
    this->pointcloud_buffer_right_mirror_, this->indices_high_intensity_rm_, 0, 0);

  this->transformPointclouds();
  RCLCPP_INFO(
    this->get_logger(), "Size per buffer: %ld", this->pointcloud_buffer_left_mirror_.size());

  if (!this->indices_high_intensity_lm_.size() || !this->indices_high_intensity_rm_.size()) {
    RCLCPP_ERROR(
      this->get_logger(), "No high intensity points found!\nOptimization not possible! Exiting...");
    return;
  }

  this->opt_flag_ = 0;

  if (this->optimization_opt_mirror_orientation_ && !this->optimization_opt_mirror_support_vec_) {
    RCLCPP_INFO(this->get_logger(), "Optimizing mirror orientation...");
    bool status_opt_mirror_orientation = this->optimizeMirrorOrientations();
    if (!status_opt_mirror_orientation)
      RCLCPP_ERROR(this->get_logger(), "Optimization of mirror orientation failed!");
  } else if (
    !this->optimization_opt_mirror_orientation_ && this->optimization_opt_mirror_support_vec_) {
    RCLCPP_INFO(this->get_logger(), "Optimizing mirror support vectors...");
    bool status_opt_mirror_support_vec = this->optimizeMirrorSupportVectors();
    if (!status_opt_mirror_support_vec)
      RCLCPP_ERROR(this->get_logger(), "Optimization of mirror support vector failed!");
  } else if (
    this->optimization_opt_mirror_orientation_ && this->optimization_opt_mirror_support_vec_) {
    RCLCPP_INFO(this->get_logger(), "Optimizing mirror orientation and support vectors...");
    bool status_opt_mirror_orientation = this->optimizeMirrorOrientations();
    bool status_opt_mirror_support_vec = this->optimizeMirrorSupportVectors();

    if (!status_opt_mirror_orientation || !status_opt_mirror_support_vec)
      RCLCPP_ERROR(
        this->get_logger(),
        "Optimization of mirror orientation and support vector "
        "failed!\n"
        "Status of mirror orientation optimization:\n%d\n"
        "Status of mirror support vector optimization:\n%d\n",
        status_opt_mirror_orientation, status_opt_mirror_support_vec);
  } else {
    RCLCPP_ERROR(
      this->get_logger(),
      "No optimization method selected!\n"
      "Check your parameters!");
  };
  RCLCPP_INFO(this->get_logger(), "Optimization finished!");
  RCLCPP_INFO(
    this->get_logger(),
    "Optimization Results Angles:\nPitch left mirror: %f\nPitch right mirror: %f\nYaw left mirror: "
    "%f\nYaw right mirror: %f",
    this->rad2deg(this->optimization_results_.pitch_left_mirror),
    this->rad2deg(this->optimization_results_.pitch_right_mirror),
    this->rad2deg(this->optimization_results_.yaw_left_mirror),
    this->rad2deg(this->optimization_results_.yaw_right_mirror));

  size_t optimization_id = std::hash<std::string>{}(
    std::to_string(std::chrono::system_clock::now().time_since_epoch().count()));

  if (this->write_optimized_params_) this->writeOptimizationResults();
  if (this->write_optimization_history_)
    this->exportOptimizationHistory(
      this->optimization_results_.history,
      this->get_parameter("optimization.optimization_history_file").as_string(),
      std::to_string(optimization_id));

  rclcpp::shutdown();
};

void LidarMirrorFOVReshaperCalib::writeOptimizationResultsMetadata(
  std::ofstream & meta_file, std::string & method_name)
{
  std::vector<bool> opt_flags_rm_sv = {
    this->opt_mirror_orientation_rm_sv_x, this->opt_mirror_orientation_rm_sv_y,
    this->opt_mirror_orientation_rm_sv_z};
  std::vector<bool> opt_flags_rm_nv = {
    this->opt_mirror_orientation_rm_nv_x, this->opt_mirror_orientation_rm_nv_y,
    this->opt_mirror_orientation_rm_nv_z};

  std::vector<bool> opt_flags_lm_sv = {
    this->opt_mirror_orientation_lm_sv_x, this->opt_mirror_orientation_lm_sv_y,
    this->opt_mirror_orientation_lm_sv_z};

  std::vector<bool> opt_flags_lm_nv = {
    this->opt_mirror_orientation_lm_nv_x, this->opt_mirror_orientation_lm_nv_y,
    this->opt_mirror_orientation_lm_nv_z};

  std::vector<bool> opt_flags_plane_sv = {
    this->opt_mirror_orientation_plane_sv_x, this->opt_mirror_orientation_plane_sv_y,
    this->opt_mirror_orientation_plane_sv_z};

  std::vector<bool> opt_flags_plane_nv = {
    this->opt_mirror_orientation_plane_nv_x, this->opt_mirror_orientation_plane_nv_y,
    this->opt_mirror_orientation_plane_nv_z};

  meta_file << "\n";
  meta_file << method_name << ",";
  meta_file << this->get_parameter("setting").as_double() << ",";
  meta_file << this->optimization_buffer_size_ << ",";
  meta_file << std::fixed << std::setprecision(5) << this->optimization_epsabs_ << ",";
  meta_file << this->roundTo4Decimals(this->optimization_stepsize_) << ",";
  meta_file << this->optimization_iter_max_ << ",";
  meta_file << this->numOptAlgorithm2String(this->num_opt_algorithm).c_str() << ",";
  meta_file << this->vectorToString(opt_flags_lm_sv) << ",";
  meta_file << this->vectorToString(opt_flags_lm_nv) << ",";
  meta_file << this->vectorToString(opt_flags_rm_sv) << ",";
  meta_file << this->vectorToString(opt_flags_rm_nv) << ",";
  meta_file << this->vectorToString(opt_flags_plane_sv) << ",";
  meta_file << this->vectorToString(opt_flags_plane_nv) << ",";
  meta_file << this->vectorToString(this->plane_helper_p1_) << ",";
  meta_file << this->vectorToString(this->plane_helper_p2_) << ",";
  meta_file << this->vectorToString(this->plane_helper_p3_) << ",";
  meta_file << this->vectorToString(this->plane_support_vec_) << ",";
  meta_file << this->rad2deg(this->mirror_left_start_angle_) << ",";
  meta_file << this->rad2deg(this->mirror_left_end_angle_) << ",";
  meta_file << this->auto_define_lm_angle_mode_ << ",";
  meta_file << this->rad2deg(this->mirror_right_start_angle_) << ",";
  meta_file << this->rad2deg(this->mirror_right_end_angle_) << ",";
  meta_file << this->auto_define_rm_angle_mode_ << ",";
  meta_file << this->rad2deg(this->front_start_angle_) << ",";
  meta_file << this->rad2deg(this->front_end_angle_) << ",";
  meta_file << this->vectorToString(this->mirror_left_helper_p1_) << ",";
  meta_file << this->vectorToString(this->mirror_left_helper_p2_) << ",";
  meta_file << this->vectorToString(this->mirror_left_helper_p3_) << ",";
  meta_file << this->vectorToString(this->mirror_left_support_vec_) << ",";
  meta_file << this->vectorToString(this->mirror_right_helper_p1_) << ",";
  meta_file << this->vectorToString(this->mirror_right_helper_p2_) << ",";
  meta_file << this->vectorToString(this->mirror_right_helper_p3_) << ",";
  meta_file << this->vectorToString(this->mirror_right_support_vec_);

  meta_file.close();
};

void LidarMirrorFOVReshaperCalib::writeOptimizationResultsStats(
  std::ofstream & results_stats_file, std::string & method_name)
{
  results_stats_file << "\n";
  results_stats_file << method_name << ",";
  results_stats_file << this->roundTo4Decimals(this->optimization_results_.rms_all) << ",";
  results_stats_file << this->roundTo4Decimals(this->optimization_results_.rms_lm_reflectionpoint)
                     << ",";
  results_stats_file << this->roundTo4Decimals(this->optimization_results_.rms_rm_reflectionpoint)
                     << ",";
  results_stats_file << this->roundTo4Decimals(this->optimization_results_.rms_non_rp_all) << ",";
  results_stats_file << this->roundTo4Decimals(this->optimization_results_.rms_plane_lm) << ",";
  results_stats_file << this->roundTo4Decimals(this->optimization_results_.rms_plane_rm) << ",";
  results_stats_file << this->roundTo4Decimals(this->optimization_results_.rms_front) << ",";
  results_stats_file << this->rad2deg(this->optimization_results_.roll_left_mirror) << ",";
  results_stats_file << this->rad2deg(this->optimization_results_.roll_right_mirror) << ",";
  results_stats_file << this->rad2deg(this->optimization_results_.pitch_left_mirror) << ",";
  results_stats_file << this->rad2deg(this->optimization_results_.pitch_right_mirror) << ",";
  results_stats_file << this->rad2deg(this->optimization_results_.yaw_left_mirror) << ",";
  results_stats_file << this->rad2deg(this->optimization_results_.yaw_right_mirror) << ",";
  results_stats_file << this->roundTo4Decimals(
                          this->optimization_results_.stddev_rms_minibatches_all)
                     << ",";
  results_stats_file << this->roundTo4Decimals(
                          this->optimization_results_.stddev_rms_minibatches_non_rp_all)
                     << ",";
  results_stats_file << this->roundTo4Decimals(
                          this->optimization_results_.stddev_rms_minibatches_lm_rp)
                     << ",";
  results_stats_file << this->roundTo4Decimals(
                          this->optimization_results_.stddev_rms_minibatches_rm_rp)
                     << ",";
  results_stats_file << this->roundTo4Decimals(
                          this->optimization_results_.stddev_rms_minibatches_lm_all)
                     << ",";
  results_stats_file << this->roundTo4Decimals(
                          this->optimization_results_.stddev_rms_minibatches_rm_all)
                     << ",";
  results_stats_file << this->rad2deg(this->optimization_results_.stddev_roll_lm) << ",";
  results_stats_file << this->rad2deg(this->optimization_results_.stddev_roll_rm) << ",";
  results_stats_file << this->rad2deg(this->optimization_results_.stddev_pitch_lm) << ",";
  results_stats_file << this->rad2deg(this->optimization_results_.stddev_pitch_rm) << ",";
  results_stats_file << this->rad2deg(this->optimization_results_.stddev_yaw_lm) << ",";
  results_stats_file << this->rad2deg(this->optimization_results_.stddev_yaw_rm) << ",";
  results_stats_file << this->roundTo4Decimals(this->optimization_results_.stddev_angle_zero)
                     << ",";
  results_stats_file << this->roundTo4Decimals(this->optimization_results_.stddev_dist_rm) << ",";
  results_stats_file << this->roundTo4Decimals(this->optimization_results_.stddev_dist_lm) << ",";
  results_stats_file << this->optimization_results_.no_batches << ",";
  results_stats_file << this->optimization_results_.batch_size << ",";
  results_stats_file << this->vectorToString(this->mirror_left_normal_vec_) << ",";
  results_stats_file << this->vectorToString(this->mirror_right_normal_vec_);

  results_stats_file.close();
};

void LidarMirrorFOVReshaperCalib::writeOptimizationResults()
{
  std::string meta_filepath =
    ament_index_cpp::get_package_share_directory("lidar_mirror_fov_reshaper_calibration") +
    "/results/" + this->optimized_params_meta_file_;
  std::string results_stats_filepath =
    ament_index_cpp::get_package_share_directory("lidar_mirror_fov_reshaper_calibration") +
    "/results/" + this->optimized_params_file_;

  std::ofstream meta_file, results_stats_file;
  meta_file.open(meta_filepath, std::ios::app);

  int curr_entry_idx = this->getCSVrowcount(meta_filepath);
  std::string method_name = "method_" + std::to_string(curr_entry_idx);

  this->writeOptimizationResultsMetadata(meta_file, method_name);
  RCLCPP_INFO(this->get_logger(), "Meta file written to: %s", meta_filepath.c_str());

  results_stats_file.open(results_stats_filepath, std::ios::app);
  this->writeOptimizationResultsStats(results_stats_file, method_name);
  RCLCPP_INFO(this->get_logger(), "Results stats file written to: %s", meta_filepath.c_str());

  std::string meta_output_dir =
    ament_index_cpp::get_package_share_directory("lidar_mirror_fov_reshaper_calibration");
  meta_output_dir += "/results/";
  struct stat meta_buffer;
  if (stat(meta_output_dir.c_str(), &meta_buffer) != 0) {
    RCLCPP_INFO(this->get_logger(), "Creating results directory: %s", meta_output_dir.c_str());
    std::string cmd = "mkdir -p " + meta_output_dir;
    system(cmd.c_str());
  };

  std::string perm_meta_filepath = meta_output_dir + this->optimized_params_meta_file_;
  std::string perm_results_filepath =
    this->get_parameter("optimization.optimized_params_out_dir").as_string() +
    this->optimized_params_file_;

  // check if file exists
  struct stat buffer;
  if (stat(perm_results_filepath.c_str(), &buffer) != 0) {
    RCLCPP_INFO(
      this->get_logger(), "Creating perm results file: %s", perm_results_filepath.c_str());
    std::ofstream perm_results_file;
    perm_results_file.open(perm_results_filepath, std::ios::app);
    perm_results_file
      << "method_name,rms_all,rms_lm_reflectionpoint,rms_rm_reflectionpoint,rms_non_rp_all,"
         "roll_left_mirror,roll_right_mirror,pitch_left_mirror,pitch_right_mirror,"
         "yaw_left_mirror,yaw_right_mirror,stddev_rms_minibatches_all,stddev_rms_minibatches_"
         "front,"
         "stddev_rms_minibatches_lm_rp,stddev_rms_minibatches_rm_rp,stddev_angle_zero,no_batches,"
         "batch_"
         "size",
      "normal_vec_lm", "normal_vec_rm";
    perm_results_file.close();
  };
  if (stat(perm_meta_filepath.c_str(), &buffer) != 0) {
    RCLCPP_INFO(this->get_logger(), "Creating perm meta file: %s", perm_meta_filepath.c_str());
    std::ofstream perm_meta_file;
    perm_meta_file.open(perm_meta_filepath, std::ios::app);
    perm_meta_file
      << "method_name,setting,buffer_size,epsabs,stepsize,iter_max,optimization_method,"
         "opt_flags_lm_sv,opt_flags_lm_nv,"
         "opt_flags_rm_sv,opt_flags_rm_nv,"
         "opt_flags_plane_sv,opt_flags_plane_nv,"
         "plane_helper_p1,plane_helper_p2,"
         "plane_helper_p3,plane_support_vec,"
         "mirror_left_start_angle,mirror_left_end_angle,"
         "define_lm_angle_mode,"
         "mirror_right_start_angle,mirror_right_end_angle,"
         "define_rm_angle_mode,"
         "front_start_angle,front_end_angle,"
         "mirror_left_helper_p1,mirror_left_helper_p2,mirror_left_helper_p3,mirror_left_support_"
         "vec,mirror_right_helper_p1,mirror_right_helper_p2,mirror_right_helper_p3,"
         "mirror_right_support_vec";
    perm_meta_file.close();
  };

  std::ofstream perm_meta_file, perm_results_stats_file;
  perm_meta_file.open(perm_meta_filepath, std::ios::app);
  this->writeOptimizationResultsMetadata(perm_meta_file, method_name);
  RCLCPP_INFO(this->get_logger(), "Perm Meta file written to: %s", perm_meta_filepath.c_str());

  perm_results_stats_file.open(perm_results_filepath, std::ios::app);
  this->writeOptimizationResultsStats(perm_results_stats_file, method_name);
  RCLCPP_INFO(
    this->get_logger(), "Perm Results stats file written to: %s", perm_results_filepath.c_str());

  return;
};

template <typename T>
std::string LidarMirrorFOVReshaperCalib::vectorToString(std::vector<T> & vec)
{
  std::stringstream str_stream;
  str_stream << "[";
  for (size_t i = 0; i < vec.size(); i++) {
    str_stream << vec[i];
    if (i != vec.size() - 1) str_stream << ";";
  };
  str_stream << "]";
  return str_stream.str();
};

int LidarMirrorFOVReshaperCalib::getCSVrowcount(std::string & filename)
{
  int row_count = 0;
  std::string line;
  std::ifstream file(filename);
  while (std::getline(file, line)) row_count++;

  file.close();

  return row_count;
};

void LidarMirrorFOVReshaperCalib::initConstraints(std::vector<bool> * opt_constraints)
{
  if (this->opt_mirror_orientation_all) {
    std::fill(opt_constraints->begin(), opt_constraints->end(), true);
    return;
  }

  opt_constraints->at(0) = this->opt_mirror_orientation_plane_sv_x;
  opt_constraints->at(1) = this->opt_mirror_orientation_plane_sv_y;
  opt_constraints->at(2) = this->opt_mirror_orientation_plane_sv_z;
  opt_constraints->at(3) = this->opt_mirror_orientation_plane_nv_x;
  opt_constraints->at(4) = this->opt_mirror_orientation_plane_nv_y;
  opt_constraints->at(5) = this->opt_mirror_orientation_plane_nv_z;
  opt_constraints->at(6) = this->opt_mirror_orientation_rm_sv_x;
  opt_constraints->at(7) = this->opt_mirror_orientation_rm_sv_y;
  opt_constraints->at(8) = this->opt_mirror_orientation_rm_sv_z;
  opt_constraints->at(9) = this->opt_mirror_orientation_rm_nv_x;
  opt_constraints->at(10) = this->opt_mirror_orientation_rm_nv_y;
  opt_constraints->at(11) = this->opt_mirror_orientation_rm_nv_z;
  opt_constraints->at(12) = this->opt_mirror_orientation_lm_sv_x;
  opt_constraints->at(13) = this->opt_mirror_orientation_lm_sv_y;
  opt_constraints->at(14) = this->opt_mirror_orientation_lm_sv_z;
  opt_constraints->at(15) = this->opt_mirror_orientation_lm_nv_x;
  opt_constraints->at(16) = this->opt_mirror_orientation_lm_nv_y;
  opt_constraints->at(17) = this->opt_mirror_orientation_lm_nv_z;

  if (this->opt_mirror_orientation_mirror_svs) {
    std::fill(opt_constraints->begin() + 6, opt_constraints->begin() + 9, true);    // right mirror
    std::fill(opt_constraints->begin() + 12, opt_constraints->begin() + 15, true);  // left mirror
  }
  // else
  // {
  //   std::fill(opt_constraints->begin() + 6, opt_constraints->begin() + 9, false);   // right mirror
  //   std::fill(opt_constraints->begin() + 12, opt_constraints->begin() + 15, false); // left mirror
  // }

  if (this->opt_mirror_orientation_mirror_nvs) {
    std::fill(opt_constraints->begin() + 9, opt_constraints->begin() + 12, true);   // right mirror
    std::fill(opt_constraints->begin() + 15, opt_constraints->begin() + 18, true);  // left mirror
  }
  // else
  // {
  //   std::fill(opt_constraints->begin() + 9, opt_constraints->begin() + 12, false);  // right mirror
  //   std::fill(opt_constraints->begin() + 15, opt_constraints->begin() + 18, false); // left mirror
  // }

  if (this->opt_mirror_orientation_plane_sv) {
    std::fill(opt_constraints->begin(), opt_constraints->begin() + 3, true);  // plane
  }
  // else
  // {
  //   std::fill(opt_constraints->begin(), opt_constraints->begin() + 3, false); // plane
  // }
};

void LidarMirrorFOVReshaperCalib::printOptimizationResults(
  bool print_normal_vecs /*= false*/, bool print_support_vecs /*= false*/)
{
  if (print_normal_vecs) {
    RCLCPP_INFO(
      this->get_logger(), "Right Mirror normal vector: [%f, %f, %f]",
      this->mirror_right_normal_vec_[0], this->mirror_right_normal_vec_[1],
      this->mirror_right_normal_vec_[2]);
    RCLCPP_INFO(
      this->get_logger(), "Left Mirror normal vector: [%f, %f, %f]",
      this->mirror_left_normal_vec_[0], this->mirror_left_normal_vec_[1],
      this->mirror_left_normal_vec_[2]);
    RCLCPP_INFO(
      this->get_logger(), "Calibration plane normal vector: [%f, %f, %f]",
      this->plane_normal_vec_[0], this->plane_normal_vec_[1], this->plane_normal_vec_[2]);
  }
  if (print_support_vecs) {
    RCLCPP_INFO(
      this->get_logger(), "Right Mirror support vector: [%f, %f, %f]",
      this->mirror_right_support_vec_[0], this->mirror_right_support_vec_[1],
      this->mirror_right_support_vec_[2]);
    RCLCPP_INFO(
      this->get_logger(), "Left Mirror support vector: [%f, %f, %f]",
      this->mirror_left_support_vec_[0], this->mirror_left_support_vec_[1],
      this->mirror_left_support_vec_[2]);
    RCLCPP_INFO(
      this->get_logger(), "Calibration Plane support vector: [%f, %f, %f]",
      this->plane_support_vec_[0], this->plane_support_vec_[1], this->plane_support_vec_[2]);
  }
};

void LidarMirrorFOVReshaperCalib::optimizeVerbose(
  optimization_params & opt_params, std::vector<double> & plane_support_vec,
  std::vector<double> & plane_normal_vec, std::vector<double> & mirror_right_support_vec,
  std::vector<double> & mirror_right_normal_vec, std::vector<double> & mirror_left_support_vec,
  std::vector<double> & mirror_left_normal_vec, int no_opt_params /* = 18 */,
  double epsabs /* = 1e-5 */, double stepsize /* = 1e-3 */, size_t iter_max /* =1000 */,
  int verbose /* = false */, int no_batches /* = -1 */, int batch_size /* = -1 */)
{
  std::vector<optimization_params> opt_params_batches;
  std::vector<double> rms_all_batches, rms_non_rp_batches, rms_lm_rp_batches, rms_rm_rp_batches,
    rms_rm_all_batches, rms_lm_all_batches, rms_front_all_batches, roll_lm_batches,
    pitch_lm_batches, yaw_lm_batches, roll_rm_batches, pitch_rm_batches, yaw_rm_batches;
  std::vector<double> plane_normal_vec_raw = plane_normal_vec;
  std::vector<double> mirror_right_normal_vec_raw = mirror_right_normal_vec;
  std::vector<double> mirror_left_normal_vec_raw = mirror_left_normal_vec;
  std::vector<double> plane_support_vec_raw = plane_support_vec;
  std::vector<double> mirror_right_support_vec_raw = mirror_right_support_vec;
  std::vector<double> mirror_left_support_vec_raw = mirror_left_support_vec;

  if (no_batches > 1) {
    batch_size = opt_params.src_cloud_vec.size() / no_batches;

    for (int i = 0; i < no_batches; i++) {
      optimization_params opt_params_tmp;
      opt_params_tmp.src_cloud_vec = std::vector<pcl::PointCloud<pcl::PointXYZI>>(
        opt_params.src_cloud_vec.begin() + i * batch_size,
        opt_params.src_cloud_vec.begin() + (i + 1) * batch_size);
      opt_params_tmp.left_mirrored_pointclouds = std::vector<pcl::PointCloud<pcl::PointXYZI>>(
        opt_params.left_mirrored_pointclouds.begin() + i * batch_size,
        opt_params.left_mirrored_pointclouds.begin() + (i + 1) * batch_size);
      opt_params_tmp.right_mirrored_pointclouds = std::vector<pcl::PointCloud<pcl::PointXYZI>>(
        opt_params.right_mirrored_pointclouds.begin() + i * batch_size,
        opt_params.right_mirrored_pointclouds.begin() + (i + 1) * batch_size);
      optimization_result * opt_result = new optimization_result();
      opt_params_tmp.opt_result = opt_result;
      opt_params_tmp.opt_result->verbose_output = verbose;
      opt_params_tmp.opt_constraints = opt_params.opt_constraints;

      opt_params_tmp.left_mirror_high_intensity_indices = std::vector<int>(
        opt_params.left_mirror_high_intensity_indices.begin() + i * batch_size,
        opt_params.left_mirror_high_intensity_indices.begin() + (i + 1) * batch_size);

      opt_params_tmp.right_mirror_high_intensity_indices = std::vector<int>(
        opt_params.right_mirror_high_intensity_indices.begin() + i * batch_size,
        opt_params.right_mirror_high_intensity_indices.begin() + (i + 1) * batch_size);

      RCLCPP_INFO(
        rclcpp::get_logger("OSG_TF"), "Optimizing batch %d, size: %ld", i,
        opt_params_tmp.src_cloud_vec.size());

      this->optimizeNonVerbose(
        opt_params_tmp, plane_support_vec_raw, plane_normal_vec_raw, mirror_right_support_vec_raw,
        mirror_right_normal_vec_raw, mirror_left_support_vec_raw, mirror_left_normal_vec_raw,
        no_opt_params, epsabs, stepsize, iter_max, false);

      rms_all_batches.push_back(opt_params_tmp.opt_result->rms_all);
      rms_non_rp_batches.push_back(opt_params_tmp.opt_result->rms_non_rp_all);
      rms_rm_all_batches.push_back(opt_params_tmp.opt_result->rms_plane_rm);
      rms_lm_all_batches.push_back(opt_params_tmp.opt_result->rms_plane_lm);
      rms_front_all_batches.push_back(opt_params_tmp.opt_result->rms_front);

      rms_rm_rp_batches.push_back(opt_params_tmp.opt_result->rms_rm_reflectionpoint);
      rms_lm_rp_batches.push_back(opt_params_tmp.opt_result->rms_lm_reflectionpoint);

      roll_lm_batches.push_back(opt_params_tmp.opt_result->roll_left_mirror);
      pitch_lm_batches.push_back(opt_params_tmp.opt_result->pitch_left_mirror);
      yaw_lm_batches.push_back(opt_params_tmp.opt_result->yaw_left_mirror);

      roll_rm_batches.push_back(opt_params_tmp.opt_result->roll_right_mirror);
      pitch_rm_batches.push_back(opt_params_tmp.opt_result->pitch_right_mirror);
      yaw_rm_batches.push_back(opt_params_tmp.opt_result->yaw_right_mirror);

      RCLCPP_INFO(this->get_logger(), "rms_all: %f", opt_params_tmp.opt_result->rms_all);
      opt_params_batches.push_back(opt_params_tmp);

      delete opt_result;
    };
  } else
    batch_size = opt_params.src_cloud_vec.size();

  RCLCPP_INFO(
    rclcpp::get_logger("OSG_TF"), "Batch optimization done, now optimizing on all data\n");

  // run optimization on all data
  optimization_params opt_params_all;
  opt_params_all.src_cloud_vec = opt_params.src_cloud_vec;
  opt_params_all.left_mirrored_pointclouds = opt_params.left_mirrored_pointclouds;
  opt_params_all.right_mirrored_pointclouds = opt_params.right_mirrored_pointclouds;
  opt_params_all.left_mirror_high_intensity_indices = opt_params.left_mirror_high_intensity_indices;
  opt_params_all.right_mirror_high_intensity_indices =
    opt_params.right_mirror_high_intensity_indices;
  optimization_result * opt_result_all = new optimization_result();
  opt_params_all.opt_result = opt_result_all;
  opt_params_all.opt_result->verbose_output = verbose;
  opt_params_all.opt_constraints = opt_params.opt_constraints;

  this->optimizeNonVerbose(
    opt_params_all, plane_support_vec, plane_normal_vec, mirror_right_support_vec,
    mirror_right_normal_vec, mirror_left_support_vec, mirror_left_normal_vec, no_opt_params, epsabs,
    stepsize, iter_max, true);

  double rms_all_batches_stddev = lidarMirrorFOVReshaperTF::getStdDev(rms_all_batches);
  double rms_non_rp_all_batches_stddev = lidarMirrorFOVReshaperTF::getStdDev(rms_non_rp_batches);
  double rms_rm_rp_batches_stddev = lidarMirrorFOVReshaperTF::getStdDev(rms_rm_rp_batches);
  double rms_lm_rp_batches_stddev = lidarMirrorFOVReshaperTF::getStdDev(rms_lm_rp_batches);
  double rms_lm_all_batches_stddev = lidarMirrorFOVReshaperTF::getStdDev(rms_lm_all_batches);
  double rms_rm_all_batches_stddev = lidarMirrorFOVReshaperTF::getStdDev(rms_rm_all_batches);

  double stddev_lm_roll = lidarMirrorFOVReshaperTF::getStdDev(roll_lm_batches);
  double stddev_lm_pitch = lidarMirrorFOVReshaperTF::getStdDev(pitch_lm_batches);
  double stddev_lm_yaw = lidarMirrorFOVReshaperTF::getStdDev(yaw_lm_batches);

  double stddev_rm_roll = lidarMirrorFOVReshaperTF::getStdDev(roll_rm_batches);
  double stddev_rm_pitch = lidarMirrorFOVReshaperTF::getStdDev(pitch_rm_batches);
  double stddev_rm_yaw = lidarMirrorFOVReshaperTF::getStdDev(yaw_rm_batches);

  RCLCPP_INFO(
    rclcpp::get_logger("OSG_TF"),
    "Optimization summary:\n"
    "stddev of rms_all: %f\n",
    rms_all_batches_stddev);

  RCLCPP_INFO(
    rclcpp::get_logger("OSG_TF"), " YAWS ALL SIZE: %d\n",
    opt_params_all.opt_result->history.yaw_left_mirror.size());

  opt_params.opt_result->stddev_rms_minibatches_all = rms_all_batches_stddev;
  opt_params.opt_result->stddev_rms_minibatches_non_rp_all = rms_non_rp_all_batches_stddev;
  opt_params.opt_result->stddev_rms_minibatches_rm_rp = rms_rm_rp_batches_stddev;
  opt_params.opt_result->stddev_rms_minibatches_lm_rp = rms_lm_rp_batches_stddev;
  opt_params.opt_result->stddev_rms_minibatches_lm_all = rms_lm_all_batches_stddev;
  opt_params.opt_result->stddev_rms_minibatches_rm_all = rms_rm_all_batches_stddev;
  opt_params.opt_result->batch_size = batch_size;
  opt_params.opt_result->rms_all = opt_params_all.opt_result->rms_all;
  opt_params.opt_result->rms_non_rp_all = opt_params_all.opt_result->rms_non_rp_all;
  opt_params.opt_result->rms_rm_reflectionpoint = opt_params_all.opt_result->rms_rm_reflectionpoint;
  opt_params.opt_result->rms_lm_reflectionpoint = opt_params_all.opt_result->rms_lm_reflectionpoint;
  opt_params.opt_result->rms_plane_rm = opt_params_all.opt_result->rms_plane_rm;
  opt_params.opt_result->rms_plane_lm = opt_params_all.opt_result->rms_plane_lm;
  opt_params.opt_result->rms_front = opt_params_all.opt_result->rms_front;
  opt_params.opt_result->optimization_status = opt_params_all.opt_result->optimization_status;
  opt_params.opt_result->stddev_roll_lm = stddev_lm_roll;
  opt_params.opt_result->stddev_pitch_lm = stddev_lm_pitch;
  opt_params.opt_result->stddev_yaw_lm = stddev_lm_yaw;
  opt_params.opt_result->stddev_roll_rm = stddev_rm_roll;
  opt_params.opt_result->stddev_pitch_rm = stddev_rm_pitch;
  opt_params.opt_result->stddev_yaw_rm = stddev_rm_yaw;
  opt_params.opt_result->history = opt_params_all.opt_result->history;

  delete opt_result_all;

  return;
};

std::string LidarMirrorFOVReshaperCalib::numOptAlgorithm2String(nlopt_algorithm algo)
{
  static const std::map<nlopt_algorithm, std::string> algo_map = {
    {NLOPT_LN_NELDERMEAD, "NLOPT_LN_NELDERMEAD"}, {NLOPT_LN_BOBYQA, "NLOPT_BOBYQA"},
    // ...
  };

  auto it = algo_map.find(algo);
  return it != algo_map.end() ? it->second : "UNKNOWN";
}

void LidarMirrorFOVReshaperCalib::optimizeNonVerbose(
  optimization_params & opt_params, std::vector<double> & plane_support_vec,
  std::vector<double> & plane_normal_vec, std::vector<double> & mirror_right_support_vec,
  std::vector<double> & mirror_right_normal_vec, std::vector<double> & mirror_left_support_vec,
  std::vector<double> & mirror_left_normal_vec, const int & no_opt_params, const double & epsabs,
  const double & stepsize, const size_t & iter_max, bool plot_error /*= false*/)
{
  nlopt_opt opt = nlopt_create(this->num_opt_algorithm, no_opt_params);

  nlopt_set_min_objective(opt, LidarMirrorFOVReshaperCalib::errFnMirrorFix, &opt_params);

  nlopt_set_maxeval(opt, iter_max);
  nlopt_set_ftol_abs(opt, epsabs);

  std::vector<double> x(no_opt_params);
  x[0] = plane_support_vec[0];
  x[1] = plane_support_vec[1];
  x[2] = plane_support_vec[2];
  x[3] = plane_normal_vec[0];
  x[4] = plane_normal_vec[1];
  x[5] = plane_normal_vec[2];
  x[6] = mirror_right_support_vec[0];
  x[7] = mirror_right_support_vec[1];
  x[8] = mirror_right_support_vec[2];
  x[9] = mirror_right_normal_vec[0];
  x[10] = mirror_right_normal_vec[1];
  x[11] = mirror_right_normal_vec[2];
  x[12] = mirror_left_support_vec[0];
  x[13] = mirror_left_support_vec[1];
  x[14] = mirror_left_support_vec[2];
  x[15] = mirror_left_normal_vec[0];
  x[16] = mirror_left_normal_vec[1];
  x[17] = mirror_left_normal_vec[2];

  std::vector<bool> * opt_constraints = opt_params.opt_constraints;
  if (opt_constraints != NULL) {
    if (opt_constraints->size() != no_opt_params) {
      RCLCPP_ERROR(
        rclcpp::get_logger("OSG_TF"),
        "Error: input constraints vector is not of size 18, size of input constraints vector: %ld",
        opt_constraints->size());
      return;
    };

    for (int i = 0; i < no_opt_params; i++) {
      if (!opt_constraints->at(i)) {
        nlopt_set_lower_bound(opt, i, x[i]);
        nlopt_set_upper_bound(opt, i, x[i]);
        RCLCPP_INFO(
          rclcpp::get_logger("OSG_TF"), "Constraint applied on parameter %d, value: %f", i, x[i]);
      }
    }
  };

  // set calib plane to be 90deg
  // nlopt_set_lower_bound(opt, 4, plane_normal_vec[1]);
  // nlopt_set_upper_bound(opt, 4, plane_normal_vec[1]);

  // nlopt_set_lower_bound(opt, 5, plane_normal_vec[2]);
  // nlopt_set_upper_bound(opt, 5, plane_normal_vec[2]);

  double minf;
  int no_iterations = 0;
  auto start = std::chrono::high_resolution_clock::now();

  nlopt_result res = nlopt_optimize(opt, x.data(), &minf);
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  no_iterations = nlopt_get_numevals(opt);
  nlopt_destroy(opt);

  plane_support_vec = {x[0], x[1], x[2]};
  plane_normal_vec = {x[3], x[4], x[5]};
  mirror_right_support_vec = {x[6], x[7], x[8]};
  mirror_right_normal_vec = {x[9], x[10], x[11]};
  mirror_left_support_vec = {x[12], x[13], x[14]};
  mirror_left_normal_vec = {x[15], x[16], x[17]};

  std::string opt_status_str;
  try {
    opt_status_str = nlopt_result_to_string((nlopt_result)res);
  } catch (const std::out_of_range & oor) {
    RCLCPP_ERROR(
      rclcpp::get_logger("OSG_TF"),
      "Decoding optimization status failed.\nRaw encoded status code: %d", res);
  };

  RCLCPP_INFO(
    rclcpp::get_logger("OSG_TF"),
    "\nOptimization took %ld (ms)\n"
    "Optimization iterations: %d\n"
    "Optimization result(mm): %f\n"
    "Optimization status: %s\n",
    duration.count() / 1000, no_iterations, (minf * 1000), opt_status_str.c_str());

  RCLCPP_INFO(this->get_logger(), "Optimization done\n");

  opt_params.opt_result->optimization_status = res;

  return;
};

double LidarMirrorFOVReshaperCalib::errFnMirrorFix(
  unsigned n, const double * x, double * grad, void * function_params)
{
  // Unwrap values from the array x
  const size_t vector_size = 3;
  std::vector<double> plane_support_vec(x, x + vector_size);
  std::vector<double> plane_normal_vec(x + vector_size, x + (vector_size * 2));
  std::vector<double> mirror_right_support_vec(x + (vector_size * 2), x + (vector_size * 3));
  std::vector<double> mirror_right_normal_vec(x + (vector_size * 3), x + (vector_size * 4));
  std::vector<double> mirror_left_support_vec(x + (vector_size * 4), x + (vector_size * 5));
  std::vector<double> mirror_left_normal_vec(x + (vector_size * 5), x + (vector_size * 6));

  optimization_params opt_params = (*(optimization_params *)function_params);
  std::vector<pcl::PointCloud<pcl::PointXYZI>> src_cloud_vec_front = opt_params.src_cloud_vec;

  std::vector<pcl::PointCloud<pcl::PointXYZI>> left_mirrored_pointclouds =
    opt_params.left_mirrored_pointclouds;
  std::vector<pcl::PointCloud<pcl::PointXYZI>> right_mirrored_pointclouds =
    opt_params.right_mirrored_pointclouds;

  std::vector<pcl::PointCloud<pcl::PointXYZI>> transformed_cloud;
  std::vector<pcl::PointCloud<pcl::PointXYZI>> transformed_cloud_left_mirror;
  std::vector<pcl::PointCloud<pcl::PointXYZI>>
    transformed_cloud_right_mirror;  // bag of right_mirror pointclouds

  // Process each source cloud
  for (size_t i = 0; i < src_cloud_vec_front.size(); i++) {
    pcl::PointCloud<pcl::PointXYZI> tmp_cloud;
    pcl::PointCloud<pcl::PointXYZI> tmp_cloud_left_mirror;
    pcl::PointCloud<pcl::PointXYZI> tmp_cloud_right_mirror;

    lidarMirrorFOVReshaperTF::transformCloud(
      src_cloud_vec_front[i], left_mirrored_pointclouds[i], right_mirrored_pointclouds[i],
      mirror_left_normal_vec, mirror_left_support_vec, mirror_right_normal_vec,
      mirror_right_support_vec, &tmp_cloud, &tmp_cloud_left_mirror, &tmp_cloud_right_mirror);

    transformed_cloud.push_back(tmp_cloud);
    transformed_cloud_left_mirror.push_back(tmp_cloud_left_mirror);
    transformed_cloud_right_mirror.push_back(tmp_cloud_right_mirror);
  }
  std::vector<pcl::PointXYZI> right_mirror_refl_points;
  std::vector<pcl::PointXYZI> left_mirror_refl_points;

  lidarMirrorFOVReshaperTF::getMirrorReflectionPoints(
    transformed_cloud_left_mirror, opt_params.left_mirror_high_intensity_indices,
    left_mirror_refl_points);

  lidarMirrorFOVReshaperTF::getMirrorReflectionPoints(
    transformed_cloud_right_mirror, opt_params.right_mirror_high_intensity_indices,
    right_mirror_refl_points);

  double rms_non_rp_sum =
    0;  // point - plane distance (all points (front + mirrors)); plane = calib plane
  double rms_rm_reflectionpoint =
    0;  // rms vec-vec rms of euclidean distance between points mirrored by
        // right mirror which are known to hit the calib reflector piece
  double rms_lm_reflectionpoint = 0;  // same as above but for left mirror

  double rms_mirror_right_all = 0;  // all points mirrored by right mirror onto the calib plane
  double rms_mirror_left_all = 0;   // all points mirrored by left mirror onto the calib plane
  double rms_front = 0;             // all rays from front to calib plane

  std::vector<double> dist_values_unsquared_rmidx5, dist_values_unsquared_lmidx5;

  // iter over all lm and rm points and calculate pointplane dist of idx 5
  for (size_t i = 0; i < transformed_cloud_left_mirror.size(); i++) {
    std::vector<double> point_plane_distances;

    lidarMirrorFOVReshaperTF::calcPointPlaneDist(
      transformed_cloud_left_mirror[i], plane_support_vec, plane_normal_vec,
      &point_plane_distances);

    dist_values_unsquared_lmidx5.push_back(std::abs(point_plane_distances[0]));
  }
  for (size_t i = 0; i < transformed_cloud_right_mirror.size(); i++) {
    std::vector<double> point_plane_distances;

    lidarMirrorFOVReshaperTF::calcPointPlaneDist(
      transformed_cloud_right_mirror[i], plane_support_vec, plane_normal_vec,
      &point_plane_distances);
    dist_values_unsquared_rmidx5.push_back(std::abs(point_plane_distances[0]));
  }

  // remove points from each cloud where intensity > 50% max
  std::vector<pcl::PointCloud<pcl::PointXYZI>> transformed_cloud_left_mirror_filtered;
  std::vector<std::vector<int>> left_cloud_valid_indices;
  for (size_t i = 0; i < transformed_cloud_left_mirror.size(); i++) {
    double max_intensity = 0.0;
    for (size_t j = 0; j < transformed_cloud_left_mirror[i].size(); j++) {
      if (transformed_cloud_left_mirror[i].at(j).intensity >= max_intensity) {
        max_intensity = transformed_cloud_left_mirror[i].at(j).intensity;
      }
    };
    pcl::PointCloud<pcl::PointXYZI> tmp_cloud;
    std::vector<int> tmp_indices;
    for (size_t j = 0; j < transformed_cloud_left_mirror[i].size(); j++) {
      if (transformed_cloud_left_mirror[i].at(j).intensity < 0.5 * max_intensity) {
        tmp_cloud.push_back(transformed_cloud_left_mirror[i].at(j));
        tmp_indices.push_back(j);
      }
    };

    left_cloud_valid_indices.push_back(tmp_indices);
    transformed_cloud_left_mirror_filtered.push_back(tmp_cloud);
  };

  std::vector<pcl::PointCloud<pcl::PointXYZI>> transformed_cloud_right_mirror_filtered;
  std::vector<std::vector<int>> right_cloud_valid_indices;
  for (size_t i = 0; i < transformed_cloud_right_mirror.size(); i++) {
    double max_intensity = 0.0;
    for (size_t j = 0; j < transformed_cloud_right_mirror[i].size(); j++) {
      if (transformed_cloud_right_mirror[i].at(j).intensity >= max_intensity) {
        max_intensity = transformed_cloud_right_mirror[i].at(j).intensity;
      }
    };
    pcl::PointCloud<pcl::PointXYZI> tmp_cloud;
    std::vector<int> tmp_indices;
    for (size_t j = 0; j < transformed_cloud_right_mirror[i].size(); j++) {
      if (transformed_cloud_right_mirror[i].at(j).intensity < 0.6 * max_intensity) {
        tmp_cloud.push_back(transformed_cloud_right_mirror[i].at(j));
        tmp_indices.push_back(j);
      }
    };

    right_cloud_valid_indices.push_back(tmp_indices);
    transformed_cloud_right_mirror_filtered.push_back(tmp_cloud);
  };

  std::vector<int> dsize_lm;
  for (size_t i = 0; i < transformed_cloud_left_mirror.size(); i++) {
    int dsize =
      transformed_cloud_left_mirror[i].size() - transformed_cloud_left_mirror_filtered[i].size();
    dsize_lm.push_back(dsize);
  };
  std::vector<int> dsize_rm;
  for (size_t i = 0; i < transformed_cloud_right_mirror.size(); i++) {
    int dsize =
      transformed_cloud_right_mirror[i].size() - transformed_cloud_right_mirror_filtered[i].size();
    dsize_rm.push_back(dsize);
  };
  int avg_dsize_lm = std::accumulate(dsize_lm.begin(), dsize_lm.end(), 0);
  avg_dsize_lm /= dsize_lm.size();
  int avg_dsize_rm = std::accumulate(dsize_rm.begin(), dsize_rm.end(), 0);
  avg_dsize_rm /= dsize_rm.size();

  // remove invalid points each corresponding cloud
  for (size_t i = 0; i < transformed_cloud.size(); i++) {
    pcl::PointCloud<pcl::PointXYZI> tmp_cloud;
    for (size_t j = 0; j < transformed_cloud[i].size(); j++) {
      if (
        std::find(left_cloud_valid_indices[i].begin(), left_cloud_valid_indices[i].end(), j) ==
          left_cloud_valid_indices[i].end() &&
        std::find(right_cloud_valid_indices[i].begin(), right_cloud_valid_indices[i].end(), j) ==
          right_cloud_valid_indices[i].end()) {
        tmp_cloud.push_back(transformed_cloud[i].at(j));
      }
    }
    transformed_cloud[i] = tmp_cloud;
  }

  // all points known to hit the calib plane (front + mirrors)
  for (auto & src_cloud : transformed_cloud) {
    std::vector<double> point_plane_distances;

    lidarMirrorFOVReshaperTF::calcPointPlaneDist(
      src_cloud, plane_support_vec, plane_normal_vec, &point_plane_distances);

    rms_non_rp_sum += std::inner_product(
      point_plane_distances.begin(), point_plane_distances.end(), point_plane_distances.begin(),
      0.0);
  }

  // all points mirrored by right mirror onto the reflector
  for (auto & point : right_mirror_refl_points) {
    std::vector<double> point_vec = {point.x, point.y, point.z};
    std::vector<double> point_vec_minus_plane_support_vec(3);

    std::transform(
      point_vec.begin(), point_vec.end(), plane_support_vec.begin(),
      point_vec_minus_plane_support_vec.begin(), std::minus<double>());

    rms_rm_reflectionpoint += std::inner_product(
      point_vec_minus_plane_support_vec.begin(), point_vec_minus_plane_support_vec.end(),
      point_vec_minus_plane_support_vec.begin(), 0.0);
  }

  for (auto & point : left_mirror_refl_points) {
    std::vector<double> point_vec = {point.x, point.y, point.z};
    std::vector<double> point_vec_minus_plane_support_vec(3);

    std::transform(
      point_vec.begin(), point_vec.end(), plane_support_vec.begin(),
      point_vec_minus_plane_support_vec.begin(), std::minus<double>());

    rms_lm_reflectionpoint += std::inner_product(
      point_vec_minus_plane_support_vec.begin(), point_vec_minus_plane_support_vec.end(),
      point_vec_minus_plane_support_vec.begin(), 0.0);
  }

  for (
    auto & cloud :
    transformed_cloud_left_mirror_filtered) {  // all rays from left mirror to calib plane w/o high intensity points
    std::vector<double> point_plane_distances;

    lidarMirrorFOVReshaperTF::calcPointPlaneDist(
      cloud, plane_support_vec, plane_normal_vec, &point_plane_distances);

    rms_mirror_left_all += std::inner_product(
      point_plane_distances.begin(), point_plane_distances.end(), point_plane_distances.begin(),
      0.0);
  }

  for (auto & cloud : transformed_cloud_right_mirror_filtered) {
    // for (auto & cloud : transformed_cloud_right_mirror) {
    std::vector<double> point_plane_distances;

    lidarMirrorFOVReshaperTF::calcPointPlaneDist(
      cloud, plane_support_vec, plane_normal_vec, &point_plane_distances);

    rms_mirror_right_all += std::inner_product(
      point_plane_distances.begin(), point_plane_distances.end(), point_plane_distances.begin(),
      0.0);
  }

  for (auto & cloud : src_cloud_vec_front) {
    std::vector<double> point_plane_distances;

    lidarMirrorFOVReshaperTF::calcPointPlaneDist(
      cloud, plane_support_vec, plane_normal_vec, &point_plane_distances);

    rms_front += std::inner_product(
      point_plane_distances.begin(), point_plane_distances.end(), point_plane_distances.begin(),
      0.0);
  }

  int num_points_right = 0;
  int num_points_left = 0;
  int num_points_front = 0;

  for (auto & cloud : src_cloud_vec_front) {
    num_points_front += cloud.size();
  }

  for (auto & cloud : transformed_cloud_right_mirror_filtered) {
    num_points_right += cloud.size();
  }

  for (auto & cloud : transformed_cloud_left_mirror_filtered) {
    num_points_left += cloud.size();
  }

  int num_points_all = num_points_front + num_points_right + num_points_left;

  rms_front /= num_points_front;             // all rays from front to calib plane
  rms_mirror_right_all /= num_points_right;  // all rays from right mirror to calib plane
  rms_mirror_left_all /= num_points_left;    // all rays from left mirror to calib plane
  rms_non_rp_sum /= num_points_all;          // all points to calib plane (front + mirrors)
  //   rms_plane_sum = (rms_plane_sum / (transformed_cloud.size() * transformed_cloud[0].points.size()));
  rms_rm_reflectionpoint /=
    right_mirror_refl_points.size();  // all rays known to hit reflector from right mirror
  rms_lm_reflectionpoint /=
    left_mirror_refl_points.size();  // all rays known to hit reflector from left mirror

  double rms_all = std::sqrt(rms_front + rms_mirror_right_all + rms_mirror_left_all);

  rms_non_rp_sum = std::sqrt(rms_non_rp_sum);
  rms_rm_reflectionpoint = std::sqrt(rms_rm_reflectionpoint);
  rms_lm_reflectionpoint = std::sqrt(rms_lm_reflectionpoint);

  rms_mirror_right_all = std::sqrt(rms_mirror_right_all);
  rms_mirror_left_all = std::sqrt(rms_mirror_left_all);
  rms_front = std::sqrt(rms_front);

  rms_all += rms_lm_reflectionpoint + rms_rm_reflectionpoint;

  if (opt_params.opt_result->verbose_output == 1 || opt_params.opt_result->verbose_output == 2) {
    opt_params.opt_result->rms_all = rms_all;
    opt_params.opt_result->rms_non_rp_all = rms_non_rp_sum;
    opt_params.opt_result->rms_rm_reflectionpoint = rms_rm_reflectionpoint;
    opt_params.opt_result->rms_lm_reflectionpoint = rms_lm_reflectionpoint;
    opt_params.opt_result->rms_plane_lm = rms_mirror_left_all;
    opt_params.opt_result->rms_plane_rm = rms_mirror_right_all;
    opt_params.opt_result->rms_front = rms_front;

    std::vector<double> tmp_rpy_lm = lidarMirrorFOVReshaperTF::calcRPY(mirror_left_normal_vec);
    std::vector<double> tmp_rpy_rm = lidarMirrorFOVReshaperTF::calcRPY(mirror_right_normal_vec);

    opt_params.opt_result->roll_left_mirror = tmp_rpy_lm[0];
    opt_params.opt_result->pitch_left_mirror = tmp_rpy_lm[1];
    opt_params.opt_result->yaw_left_mirror = tmp_rpy_lm[2];

    opt_params.opt_result->roll_right_mirror = tmp_rpy_rm[0];
    opt_params.opt_result->pitch_right_mirror = tmp_rpy_rm[1];
    opt_params.opt_result->yaw_right_mirror = tmp_rpy_rm[2];

    if (opt_params.opt_result->verbose_output == 2) {
      RCLCPP_INFO(
        rclcpp::get_logger("OSG_TF"),
        "rms_all: %f(m),"
        "rms_non_rp_sum: %f(m),"
        "rms_rm_reflectionpoints: %f(m),"
        "rms_lm_reflectionpoints: %f(m),"
        "rms_mirror_right_all: %f(m),"
        "rms_mirror_left_all: %f(m)",
        "rms_front: %f(m)", rms_all, rms_non_rp_sum, rms_rm_reflectionpoint, rms_lm_reflectionpoint,
        rms_mirror_right_all, rms_mirror_left_all, rms_front);
    }
  } else if (opt_params.opt_result->verbose_output == 3) {
    opt_params.opt_result->rms_all = rms_all;
    opt_params.opt_result->rms_non_rp_all = rms_non_rp_sum;
    opt_params.opt_result->rms_rm_reflectionpoint = rms_rm_reflectionpoint;
    opt_params.opt_result->rms_lm_reflectionpoint = rms_lm_reflectionpoint;
    opt_params.opt_result->rms_plane_lm = rms_mirror_left_all;
    opt_params.opt_result->rms_plane_rm = rms_mirror_right_all;
    opt_params.opt_result->rms_front = rms_front;

    std::vector<double> tmp_rpy_lm = lidarMirrorFOVReshaperTF::calcRPY(mirror_left_normal_vec);
    std::vector<double> tmp_rpy_rm = lidarMirrorFOVReshaperTF::calcRPY(mirror_right_normal_vec);

    opt_params.opt_result->roll_left_mirror = tmp_rpy_lm[0];
    opt_params.opt_result->pitch_left_mirror = tmp_rpy_lm[1];
    opt_params.opt_result->yaw_left_mirror = tmp_rpy_lm[2];

    opt_params.opt_result->roll_right_mirror = tmp_rpy_rm[0];
    opt_params.opt_result->pitch_right_mirror = tmp_rpy_rm[1];
    opt_params.opt_result->yaw_right_mirror = tmp_rpy_rm[2];

    opt_params.opt_result->history.err_all.push_back(rms_all);
    opt_params.opt_result->history.err_front.push_back(rms_front);
    opt_params.opt_result->history.err_lm_non_rp.push_back(rms_mirror_left_all);
    opt_params.opt_result->history.err_rm_non_rp.push_back(rms_mirror_right_all);
    opt_params.opt_result->history.err_reflection_point_lm.push_back(rms_lm_reflectionpoint);
    opt_params.opt_result->history.err_reflection_point_rm.push_back(rms_rm_reflectionpoint);
    opt_params.opt_result->history.left_mirror_nv.push_back(mirror_left_normal_vec);
    opt_params.opt_result->history.right_mirror_nv.push_back(mirror_right_normal_vec);
    opt_params.opt_result->history.pitch_left_mirror.push_back(tmp_rpy_lm[1]);
    opt_params.opt_result->history.pitch_right_mirror.push_back(tmp_rpy_rm[1]);
    opt_params.opt_result->history.yaw_left_mirror.push_back(tmp_rpy_lm[2]);
    opt_params.opt_result->history.yaw_right_mirror.push_back(tmp_rpy_rm[2]);
  };

  return rms_all;
};

void LidarMirrorFOVReshaperCalib::initOptimization(
  optimization_params & opt_params, std::vector<double> & plane_support_vec,
  std::vector<double> & plane_normal_vec, std::vector<double> & mirror_right_support_vec,
  std::vector<double> & mirror_right_normal_vec, std::vector<double> & mirror_left_support_vec,
  std::vector<double> & mirror_left_normal_vec, int no_opt_params /* = 18 */,
  double epsabs /* = 1e-5 */, double stepsize /* = 1e-3 */, size_t iter_max /* =1000 */,
  int verbose /* = 0 */, int no_batches /* = -1 */, int batch_size /* = -1 */)
{
  if (verbose)
    this->optimizeVerbose(
      opt_params, plane_support_vec, plane_normal_vec, mirror_right_support_vec,
      mirror_right_normal_vec, mirror_left_support_vec, mirror_left_normal_vec, no_opt_params,
      epsabs, stepsize, iter_max, verbose, no_batches, batch_size);
  else
    this->optimizeNonVerbose(
      opt_params, plane_support_vec, plane_normal_vec, mirror_right_support_vec,
      mirror_right_normal_vec, mirror_left_support_vec, mirror_left_normal_vec, no_opt_params,
      epsabs, stepsize, iter_max, false);

  RCLCPP_INFO(
    rclcpp::get_logger("OSG_TF"),
    "\nrms_all (mm): %f\n"
    "rms_non_rp_all (mm): %f\n"
    "rms_rm_reflectionpoint (mm): %f\n"
    "rms_lm_reflectionpoint (mm): %f\n"
    "rms_plane_rm (mm): %f\n"
    "rms_plane_lm (mm): %f\n",
    "rms_front (mm): %f\n", this->roundTo4Decimals(opt_params.opt_result->rms_all),
    this->roundTo4Decimals(opt_params.opt_result->rms_non_rp_all),
    this->roundTo4Decimals(opt_params.opt_result->rms_rm_reflectionpoint),
    this->roundTo4Decimals(opt_params.opt_result->rms_lm_reflectionpoint),
    this->roundTo4Decimals(opt_params.opt_result->rms_plane_rm),
    this->roundTo4Decimals(opt_params.opt_result->rms_plane_lm),
    this->roundTo4Decimals(opt_params.opt_result->rms_front));

  return;
};

bool LidarMirrorFOVReshaperCalib::optimizeMirrorOrientations()
{
  // calib_plane_sv, calib_plane_nv, lm_sv, lm_nv, rm_sv, rm_nv -> 6 elements * each of dim 3 = 18
  // ONLY update this dim if the # of optimization variables changes!
  int dim_of_optimization = 18;
  optimization_result * opt_result = new optimization_result;

  opt_result->verbose_output = this->optimization_verbose_;
  opt_result->no_batches = this->optimization_evaluation_no_batches_;
  opt_result->history = optimization_history();

  std::vector<bool> * opt_constraints = new std::vector<bool>(dim_of_optimization);
  std::fill(opt_constraints->begin(), opt_constraints->end(), false);

  this->initConstraints(opt_constraints);

  optimization_params opt_params = {
    this->pointcloud_buffer_front_,
    this->pointcloud_buffer_right_mirror_,
    this->pointcloud_buffer_left_mirror_,
    this->indices_high_intensity_lm_,
    this->indices_high_intensity_rm_,
    opt_constraints,
    opt_result};

  this->initOptimization(
    opt_params, this->plane_support_vec_, this->plane_normal_vec_, this->mirror_right_support_vec_,
    this->mirror_right_normal_vec_, this->mirror_left_support_vec_, this->mirror_left_normal_vec_,
    dim_of_optimization, this->optimization_epsabs_, this->optimization_stepsize_,
    this->optimization_iter_max_, this->optimization_verbose_,
    this->optimization_evaluation_no_batches_, -1);

  int opt_status = opt_params.opt_result->optimization_status;

  this->printOptimizationResults(true, true);

  // cleanup
  delete opt_constraints;

  // Visualize normal vectors
  if (this->viz_normal_vectors) this->visualizeNormalVectors(*opt_result);
  this->optimization_results_ = *opt_result;
  delete opt_result;
  // Transform raw pointclouds with optimized parameters
  pcl::PointCloud<pcl::PointXYZI> transformed_cloud_combined, transformed_cloud_rm,
    transformed_cloud_lm;
  lidarMirrorFOVReshaperTF::transformCloud(
    this->pointcloud_buffer_front_[0], this->pointcloud_buffer_left_mirror_[0],
    this->pointcloud_buffer_right_mirror_[0], this->mirror_left_normal_vec_,
    this->mirror_left_support_vec_, this->mirror_right_normal_vec_, this->mirror_right_support_vec_,
    &transformed_cloud_combined, &transformed_cloud_lm, &transformed_cloud_rm);

  // Visualize optimized planes (mirrors and optimization/calibration plane)
  if (
    this->viz_optimized_planes_all || this->viz_optimized_plane_lm ||
    this->viz_optimized_plane_rm) {
    visualization_msgs::msg::MarkerArray::SharedPtr optimized_planes_viz(
      new visualization_msgs::msg::MarkerArray);
    this->visualizeOptimizedPlanes(optimized_planes_viz);
  }

  // Visualize transformed pointclouds
  if (
    this->viz_transformed_cloud_all || this->viz_transformed_cloud_lm ||
    this->viz_transformed_cloud_rm)
    this->visualizeTransformedPointclouds(
      transformed_cloud_combined, transformed_cloud_rm, transformed_cloud_lm);

  if (this->viz_optimization_plane_box_) {
    RCLCPP_INFO(this->get_logger(), "Visualizing optimization plane box...");
    visualization_msgs::msg::Marker::SharedPtr cube_marker =
      std::make_shared<visualization_msgs::msg::Marker>();

    std::vector<double> plane_normal_vec_rpy =
      lidarMirrorFOVReshaperTF::calcRPY(this->plane_normal_vec_);
    tf2::Quaternion plane_normal_vec_quat;
    plane_normal_vec_quat.setRPY(
      plane_normal_vec_rpy[0], plane_normal_vec_rpy[1], plane_normal_vec_rpy[2]);
    std::vector<double> position_marker = {
      this->plane_support_vec_[0], this->plane_support_vec_[1], this->plane_support_vec_[2]};
    std::vector<double> rgba_marker = {0.5, 0.0, 0.3, 0.7};
    std::vector<double> dimensions_calib_plane = {0.04, 1.25, 4.0};
    this->initPlaneMarker(
      position_marker, plane_normal_vec_quat, 99, dimensions_calib_plane, cube_marker,
      this->laser_scanner_frame_id_.c_str(), rgba_marker);

    this->cube_marker_pub_->publish(*cube_marker);
  };

  return opt_status > 0 ? true : false;
};

void LidarMirrorFOVReshaperCalib::exportOptimizationHistory(
  const optimization_history & opt_hist, const std::string & filename,
  const std::string & optimization_method)
{
  RCLCPP_INFO(this->get_logger(), "Exporting optimization history...");
  std::string file_path =
    ament_index_cpp::get_package_share_directory("lidar_mirror_fov_reshaper_calibration") +
    "/results/" + filename;

  std::ofstream history_file;
  bool file_exists = std::ifstream(file_path).good();
  history_file.open(file_path, std::ios::app);

  if (!file_exists) {
    RCLCPP_INFO(this->get_logger(), "Creating history file: %s", file_path.c_str());
    history_file << "optimization_method,pitch_left_mirror [deg],yaw_left_mirror "
                    "[deg],pitch_right_mirror [deg],yaw_right_mirror [deg],err_"
                    "front [mm],err_lm_non_"
                    "rp [mm],err_rm_non_rp [mm],err_reflection_point_lm "
                    "[mm],err_reflection_point_rm [mm],err_all [mm]\n";
  }

  if (this->getCSVrowcount(file_path) > 2) history_file << "\n";

  for (size_t i = 0; i < opt_hist.pitch_left_mirror.size(); i++) {
    history_file << optimization_method;
    history_file << ",";
    history_file << opt_hist.pitch_left_mirror[i];
    history_file << ",";
    history_file << opt_hist.yaw_left_mirror[i];
    history_file << ",";
    history_file << opt_hist.pitch_right_mirror[i];
    history_file << ",";
    history_file << opt_hist.yaw_right_mirror[i];
    history_file << ",";
    history_file << opt_hist.err_front[i];
    history_file << ",";
    history_file << opt_hist.err_lm_non_rp[i];
    history_file << ",";
    history_file << opt_hist.err_rm_non_rp[i];
    history_file << ",";
    history_file << opt_hist.err_reflection_point_lm[i];
    history_file << ",";
    history_file << opt_hist.err_reflection_point_rm[i];
    history_file << ",";
    history_file << opt_hist.err_all[i];
    history_file << "\n";
  };
  history_file.close();
};

void LidarMirrorFOVReshaperCalib::visualizeNormalVectors(optimization_result & opt_result)
{
  visualization_msgs::msg::MarkerArray::SharedPtr normal_vector_viz(
    new visualization_msgs::msg::MarkerArray);

  // calc orientation of normal vectors in order to visualize them
  std::vector<double> mirror_right_normal_vec_rpy =
    lidarMirrorFOVReshaperTF::calcRPY(this->mirror_right_normal_vec_);

  std::vector<double> mirror_left_normal_vec_rpy =
    lidarMirrorFOVReshaperTF::calcRPY(this->mirror_left_normal_vec_);

  std::vector<double> plane_normal_vec_rpy =
    lidarMirrorFOVReshaperTF::calcRPY(this->plane_normal_vec_);

  opt_result.roll_left_mirror = mirror_left_normal_vec_rpy[0];
  opt_result.pitch_left_mirror = mirror_left_normal_vec_rpy[1];
  opt_result.yaw_left_mirror = mirror_left_normal_vec_rpy[2];

  opt_result.roll_right_mirror = mirror_right_normal_vec_rpy[0];
  opt_result.pitch_right_mirror = mirror_right_normal_vec_rpy[1];
  opt_result.yaw_right_mirror = mirror_right_normal_vec_rpy[2];

  visualization_msgs::msg::Marker::SharedPtr marker_normal_vec_r =
    std::make_shared<visualization_msgs::msg::Marker>();
  std::vector<double> marker_normal_vec_r_rgba = {0.0, 0.0, 1.0, 1.0};

  int viz_plane_id = 0;

  this->initVectorVisualizer(
    marker_normal_vec_r, this->laser_scanner_frame_id_.c_str(), this->ros_ns.c_str(), viz_plane_id,
    marker_normal_vec_r_rgba);

  geometry_msgs::msg::Point p;

  p.x = this->mirror_right_support_vec_[0];
  p.y = this->mirror_right_support_vec_[1];
  p.z = this->mirror_right_support_vec_[2];
  marker_normal_vec_r->points.push_back(p);
  p.x = this->mirror_right_support_vec_[0] + this->mirror_right_normal_vec_[0];
  p.y = this->mirror_right_support_vec_[1] + this->mirror_right_normal_vec_[1];
  p.z = this->mirror_right_support_vec_[2] + this->mirror_right_normal_vec_[2];
  marker_normal_vec_r->points.push_back(p);

  normal_vector_viz->markers.push_back(*marker_normal_vec_r);

  visualization_msgs::msg::Marker::SharedPtr marker_normal_vec_l =
    std::make_shared<visualization_msgs::msg::Marker>();
  std::vector<double> marker_normal_vec_l_rgba = {0.0, 1.0, 0.0, 1.0};

  viz_plane_id++;
  this->initVectorVisualizer(
    marker_normal_vec_l, this->laser_scanner_frame_id_.c_str(), this->ros_ns.c_str(), viz_plane_id,
    marker_normal_vec_l_rgba);

  p.x = this->mirror_left_support_vec_[0];
  p.y = this->mirror_left_support_vec_[1];
  p.z = this->mirror_left_support_vec_[2];
  marker_normal_vec_l->points.push_back(p);
  p.x = this->mirror_left_support_vec_[0] + this->mirror_left_normal_vec_[0];
  p.y = this->mirror_left_support_vec_[1] + this->mirror_left_normal_vec_[1];
  p.z = this->mirror_left_support_vec_[2] + this->mirror_left_normal_vec_[2];
  marker_normal_vec_l->points.push_back(p);
  normal_vector_viz->markers.push_back(*marker_normal_vec_l);

  visualization_msgs::msg::Marker::SharedPtr marker_normal_vec_plane =
    std::make_shared<visualization_msgs::msg::Marker>();
  std::vector<double> marker_normal_vec_plane_rgba = {1.0, 0.0, 0.0, 1.0};

  viz_plane_id++;
  this->initVectorVisualizer(
    marker_normal_vec_plane, this->laser_scanner_frame_id_.c_str(), this->ros_ns.c_str(),
    viz_plane_id, marker_normal_vec_plane_rgba);
  p.x = this->plane_support_vec_[0];
  p.y = this->plane_support_vec_[1];
  p.z = this->plane_support_vec_[2];
  marker_normal_vec_plane->points.push_back(p);
  p.x = this->plane_support_vec_[0] + this->plane_normal_vec_.at(0);
  p.y = this->plane_support_vec_[1] + this->plane_normal_vec_.at(1);
  p.z = this->plane_support_vec_[2] + this->plane_normal_vec_.at(2);
  marker_normal_vec_plane->points.push_back(p);
  normal_vector_viz->markers.push_back(*marker_normal_vec_plane);

  if (this->viz_normal_vectors) this->normal_vectors_pub_->publish(*normal_vector_viz);
};

void LidarMirrorFOVReshaperCalib::visualizeTransformedPointclouds(
  pcl::PointCloud<pcl::PointXYZI> & transformed_cloud_all,
  pcl::PointCloud<pcl::PointXYZI> & transformed_cloud_rm,
  pcl::PointCloud<pcl::PointXYZI> & transformed_cloud_lm)
{
  int covered_area_lm = std::abs(this->rad2deg(this->mirror_left_end_angle_)) -
                        std::abs(this->rad2deg(this->mirror_left_start_angle_));
  int covered_area_rm = std::abs(this->rad2deg(this->mirror_right_end_angle_)) -
                        std::abs(this->rad2deg(this->mirror_right_start_angle_));

  int d_lm = transformed_cloud_lm.points.size() - covered_area_lm;
  int d_rm = transformed_cloud_rm.points.size() - covered_area_rm;

  if (d_lm < 0 || d_rm < 0) {
    RCLCPP_ERROR(
      this->get_logger(), "Transformed cloud size mismatch! Differences LM: %d, RM: %d", d_lm,
      d_rm);
    return;
  }

  if (this->viz_transformed_cloud_all) {
    sensor_msgs::msg::PointCloud2 transformed_cloud_combined_msg;
    pcl::toROSMsg(transformed_cloud_all, transformed_cloud_combined_msg);
    transformed_cloud_combined_msg.header.frame_id = this->laser_scanner_frame_id_;
    transformed_cloud_combined_msg.header.stamp = this->get_clock()->now();
    transformed_cloud_combined_msg.height = 1;
    transformed_cloud_combined_msg.width = transformed_cloud_all.points.size();
    transformed_cloud_combined_msg.is_dense = true;
    transformed_cloud_combined_msg.is_bigendian = false;
    this->transformed_cloud_all_pub_->publish(transformed_cloud_combined_msg);
  }

  if (this->viz_transformed_cloud_lm) {
    sensor_msgs::msg::PointCloud2 transformed_cloud_lm_msg;
    pcl::toROSMsg(transformed_cloud_lm, transformed_cloud_lm_msg);
    transformed_cloud_lm_msg.header.frame_id = this->laser_scanner_frame_id_;
    transformed_cloud_lm_msg.header.stamp = this->get_clock()->now();
    transformed_cloud_lm_msg.height = 1;
    transformed_cloud_lm_msg.width = transformed_cloud_lm.points.size();
    transformed_cloud_lm_msg.is_dense = true;
    transformed_cloud_lm_msg.is_bigendian = false;

    this->transformed_cloud_lm_pub_->publish(transformed_cloud_lm_msg);
  }

  if (this->viz_transformed_cloud_rm) {
    sensor_msgs::msg::PointCloud2 transformed_cloud_rm_msg;
    pcl::toROSMsg(transformed_cloud_rm, transformed_cloud_rm_msg);
    transformed_cloud_rm_msg.header.frame_id = this->laser_scanner_frame_id_;
    transformed_cloud_rm_msg.header.stamp = this->get_clock()->now();
    transformed_cloud_rm_msg.height = 1;
    transformed_cloud_rm_msg.width = transformed_cloud_rm.points.size();
    transformed_cloud_rm_msg.is_dense = true;
    transformed_cloud_rm_msg.is_bigendian = false;
    this->transformed_cloud_rm_pub_->publish(transformed_cloud_rm_msg);
  }
};

void LidarMirrorFOVReshaperCalib::visualizeOptimizedPlanes(
  visualization_msgs::msg::MarkerArray::SharedPtr optimized_planes_viz)
{
  visualization_msgs::msg::Marker::SharedPtr marker_opt_plane_mesh =
    std::make_shared<visualization_msgs::msg::Marker>();
  visualization_msgs::msg::Marker::SharedPtr marker_opt_lm_plane_mesh =
    std::make_shared<visualization_msgs::msg::Marker>();
  visualization_msgs::msg::Marker::SharedPtr marker_opt_rm_plane_mesh =
    std::make_shared<visualization_msgs::msg::Marker>();
  std::vector<double> opt_plane_rgba = {1.0, 0.0, 0.0, 0.5};
  std::vector<double> rm_rgba = {0.0, 0.0, 1.0, 0.5};
  std::vector<double> lm_rgba = {0.0, 1.0, 0.0, 0.5};

  tf2::Quaternion plane_normal_vec_quat;
  std::vector<double> plane_normal_vec_rpy =
    lidarMirrorFOVReshaperTF::calcRPY(this->plane_normal_vec_);
  plane_normal_vec_quat.setRPY(
    plane_normal_vec_rpy[0], plane_normal_vec_rpy[1], plane_normal_vec_rpy[2]);

  tf2::Quaternion mirror_right_normal_vec_quat;
  mirror_right_normal_vec_quat.setRPY(
    this->optimization_results_.roll_right_mirror, this->optimization_results_.pitch_right_mirror,
    this->optimization_results_.yaw_right_mirror);

  tf2::Quaternion mirror_left_normal_vec_quat;
  mirror_left_normal_vec_quat.setRPY(
    this->optimization_results_.roll_left_mirror, this->optimization_results_.pitch_left_mirror,
    this->optimization_results_.yaw_left_mirror);

  // draw planes
  std::vector<double> dimensions_mirrors = {0.005, 0.06, 0.04};  // [m]
  std::vector<double> dimensions_calib_plane = {0.0025, 0.75, 4.0};
  this->initPlaneMarker(
    this->plane_support_vec_, plane_normal_vec_quat, 0, dimensions_calib_plane,
    marker_opt_plane_mesh, this->laser_scanner_frame_id_.c_str(), opt_plane_rgba);
  optimized_planes_viz->markers.push_back(*marker_opt_plane_mesh);

  this->initPlaneMarker(
    this->mirror_left_support_vec_, mirror_left_normal_vec_quat, 1, dimensions_mirrors,
    marker_opt_lm_plane_mesh, this->laser_scanner_frame_id_.c_str(), lm_rgba);
  optimized_planes_viz->markers.push_back(*marker_opt_lm_plane_mesh);

  this->initPlaneMarker(
    this->mirror_right_support_vec_, mirror_right_normal_vec_quat, 2, dimensions_mirrors,
    marker_opt_rm_plane_mesh, this->laser_scanner_frame_id_.c_str(), rm_rgba);
  optimized_planes_viz->markers.push_back(*marker_opt_rm_plane_mesh);

  if (this->viz_optimized_planes_all)
    this->optimized_planes_pub_->publish(*optimized_planes_viz);
  else {
    if (this->viz_optimized_plane_opt_plane)
      this->optimized_plane_opt_plane_pub_->publish(optimized_planes_viz->markers[0]);
    if (this->viz_optimized_plane_lm)
      this->optimized_plane_lm_pub_->publish(optimized_planes_viz->markers[1]);
    if (this->viz_optimized_plane_rm)
      this->optimized_plane_rm_pub_->publish(optimized_planes_viz->markers[2]);
  }
};

void LidarMirrorFOVReshaperCalib::initPlaneMarker(
  std::vector<double> & position, tf2::Quaternion & orienation, int id,
  std::vector<double> dimensions, visualization_msgs::msg::Marker::SharedPtr marker,
  const char * src_frame_id, std::vector<double> & rgba)
{
  marker->header.frame_id = src_frame_id;
  marker->header.stamp = rclcpp::Time(0);
  marker->id = id;
  marker->type = visualization_msgs::msg::Marker::CUBE;
  marker->action = visualization_msgs::msg::Marker::ADD;
  marker->pose.position.x = position[0];
  marker->pose.position.y = position[1];
  marker->pose.position.z = position[2];
  marker->pose.orientation.x = orienation.x();
  marker->pose.orientation.y = orienation.y();
  marker->pose.orientation.z = orienation.z();
  marker->pose.orientation.w = orienation.w();
  marker->scale.x = dimensions[0];
  marker->scale.y = dimensions[1];
  marker->scale.z = dimensions[2];
  marker->color.r = rgba[0];
  marker->color.g = rgba[1];
  marker->color.b = rgba[2];
  marker->color.a = rgba[3];
};

double LidarMirrorFOVReshaperCalib::roundTo4Decimals(double value)
{
  double precision = 1e-5;
  double rounded_val = std::round(value / precision) * precision;
  return (rounded_val * 1000);  // convert to mm
};

void LidarMirrorFOVReshaperCalib::initVectorVisualizer(
  visualization_msgs::msg::Marker::SharedPtr marker, const char * src_frame_id, const char * ns,
  int id, std::vector<double> & rgba)
{
  marker->header.frame_id = src_frame_id;
  marker->header.stamp = rclcpp::Time(0);
  marker->ns = ns;
  marker->id = id;
  marker->type = visualization_msgs::msg::Marker::ARROW;
  marker->action = visualization_msgs::msg::Marker::ADD;
  marker->scale.x = 0.01;
  marker->scale.y = 0.01;
  marker->scale.z = 0.01;
  marker->color.r = rgba[0];
  marker->color.g = rgba[1];
  marker->color.b = rgba[2];
  marker->color.a = rgba[3];
};

std::vector<double> crossProduct(std::vector<double> & a, std::vector<double> & b)
{
  if (a.size() != 3 || b.size() != 3) {
    RCLCPP_ERROR(
      rclcpp::get_logger("OSG_TF"), "Input vectors must be of dim 3\nAre of dim: a: %ld, b: %ld",
      a.size(), b.size());
    return {};
  }

  std::vector<double> c;
  c.push_back(a[1] * b[2] - a[2] * b[1]);
  c.push_back(a[2] * b[0] - a[0] * b[2]);
  c.push_back(a[0] * b[1] - a[1] * b[0]);
  return c;
};

double LidarMirrorFOVReshaperCalib::rad2deg(const double & rad) { return ((rad * 180) / M_PI); };

double LidarMirrorFOVReshaperCalib::deg2rad(const double & deg) { return ((deg * M_PI) / 180); };

bool LidarMirrorFOVReshaperCalib::optimizeMirrorSupportVectors(
  int tgt_angle_lm /*=-1*/, int tgt_angle_rm /*=-1*/)
{
  int tgt_idx_lm = lidarMirrorFOVReshaperTF::angle2ArrIdx(
    tgt_angle_lm, this->laser_scanner_angle_min_, this->laser_scanner_angle_max_,
    this->mirror_left_start_angle_, this->mirror_left_end_angle_);
  int tgt_idx_rm = lidarMirrorFOVReshaperTF::angle2ArrIdx(
    tgt_angle_rm, this->laser_scanner_angle_min_, this->laser_scanner_angle_max_,
    this->mirror_right_start_angle_, this->mirror_right_end_angle_);

  if (tgt_idx_lm < 0) tgt_idx_lm = this->pointcloud_buffer_left_mirror_.at(0).points.size() / 2;

  if (tgt_idx_rm < 0) tgt_idx_rm = this->pointcloud_buffer_right_mirror_.at(0).points.size() / 2;

  std::vector<double> avg_lm_sv = {0.0, 0.0, 0.0};
  std::vector<double> avg_rm_sv = {0.0, 0.0, 0.0};

  for (size_t i = 0; i < this->pointcloud_buffer_left_mirror_.size(); i++) {
    avg_lm_sv[0] += this->pointcloud_buffer_left_mirror_.at(i).points[tgt_idx_lm].x;
    avg_lm_sv[1] += this->pointcloud_buffer_left_mirror_.at(i).points[tgt_idx_lm].y;
    avg_lm_sv[2] += this->pointcloud_buffer_left_mirror_.at(i).points[tgt_idx_lm].z;

    avg_rm_sv[0] += this->pointcloud_buffer_right_mirror_.at(i).points[tgt_idx_rm].x;
    avg_rm_sv[1] += this->pointcloud_buffer_right_mirror_.at(i).points[tgt_idx_rm].y;
    avg_rm_sv[2] += this->pointcloud_buffer_right_mirror_.at(i).points[tgt_idx_rm].z;
  };

  avg_lm_sv[0] /= this->pointcloud_buffer_left_mirror_.size();
  avg_lm_sv[1] /= this->pointcloud_buffer_left_mirror_.size();
  avg_lm_sv[2] /= this->pointcloud_buffer_left_mirror_.size();

  avg_rm_sv[0] /= this->pointcloud_buffer_right_mirror_.size();
  avg_rm_sv[1] /= this->pointcloud_buffer_right_mirror_.size();
  avg_rm_sv[2] /= this->pointcloud_buffer_right_mirror_.size();

  RCLCPP_INFO(
    this->get_logger(), "Avg. left mirror support vector: [%f, %f, %f]", avg_lm_sv[0], avg_lm_sv[1],
    avg_lm_sv[2]);
  RCLCPP_INFO(
    this->get_logger(), "Avg. right mirror support vector: [%f, %f, %f]", avg_rm_sv[0],
    avg_rm_sv[1], avg_rm_sv[2]);
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarMirrorFOVReshaperCalib>());
  rclcpp::shutdown();
  return 0;
};