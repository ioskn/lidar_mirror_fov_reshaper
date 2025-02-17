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

#ifndef LIDAR_MIRROR_FOV_RESHAPER_CALIB_HPP
#define LIDAR_MIRROR_FOV_RESHAPER_CALIB_HPP

#define _USE_MATH_DEFINES

#include <float.h>
#include <nlopt.h>
#include <pcl/PointIndices.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sys/stat.h>
#include <tf2/LinearMath/Quaternion.h>

#include <Eigen/Dense>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <iomanip>  // std::fixed; std::setprecision
#include <iostream>
#include <map>
#include <numeric>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "lidar_mirror_fov_reshaper_transformation/lidar_mirror_fov_reshaper_transformation.hpp"

using namespace std::chrono;

class LidarMirrorFOVReshaperCalib : public rclcpp::Node
{
public:
  LidarMirrorFOVReshaperCalib();
  ~LidarMirrorFOVReshaperCalib();

private:
  // -----------------------------
  struct optimization_history
  {
    std::vector<std::vector<double>> left_mirror_nv;
    std::vector<std::vector<double>> right_mirror_nv;
    std::vector<double> pitch_left_mirror;
    std::vector<double> pitch_right_mirror;
    std::vector<double> yaw_left_mirror;
    std::vector<double> yaw_right_mirror;
    std::vector<double> err_reflection_point_lm;
    std::vector<double> err_reflection_point_rm;
    std::vector<double> err_lm_non_rp;
    std::vector<double> err_rm_non_rp;
    std::vector<double> err_front;
    std::vector<double> err_all;
  } typedef optimization_history;

  struct optimization_result
  {
    int verbose_output;
    int no_batches;
    int batch_size;
    int optimization_status;
    double rms_all;
    double rms_front;
    double rms_lm_reflectionpoint;
    double rms_rm_reflectionpoint;
    double rms_non_rp_all;
    double rms_plane_lm;
    double rms_plane_rm;
    std::vector<double> dist_angle_0;
    double roll_left_mirror;
    double roll_right_mirror;
    double pitch_left_mirror;
    double pitch_right_mirror;
    double yaw_left_mirror;
    double yaw_right_mirror;
    double stddev_rms_minibatches_lm_rp;
    double stddev_rms_minibatches_rm_rp;
    double stddev_rms_minibatches_lm_all;
    double stddev_rms_minibatches_rm_all;
    double stddev_rms_minibatches_non_rp_all;
    double stddev_rms_minibatches_all;
    std::vector<double> minibatches_roll_left_mirror;
    std::vector<double> minibatches_roll_right_mirror;
    std::vector<double> minibatches_pitch_left_mirror;
    std::vector<double> minibatches_pitch_right_mirror;
    std::vector<double> minibatches_yaw_left_mirror;
    std::vector<double> minibatches_yaw_right_mirror;
    double stddev_roll_lm;
    double stddev_roll_rm;
    double stddev_pitch_lm;
    double stddev_pitch_rm;
    double stddev_yaw_lm;
    double stddev_yaw_rm;
    double stddev_angle_zero;
    double stddev_dist_lm;
    double stddev_dist_rm;
    optimization_history history;
  } typedef optimization_result;

  /**
   * @brief Struct containing all parameters needed for the optimization
   *
   * @param[in] src_cloud_vec vector containing pointcloud
   * @param[in] right_mirrored_pointclouds vector containing pointclouds mirrored by right mirror
   * @param[in] left_mirrored_pointclouds vector containing pointclouds mirrored by left mirror
   * @param[in] left_mirror_high_intensity_indices vector containing indices of points with high intensity per left mirror cloud
   * @param[in] right_mirror_high_intensity_indices vector containing indices of points with high intensity per right mirror cloud
   * @param[in] opt_constraints pointer to vector containing constraints for optimization
   * @param[out] opt_result pointer to optimization result, used for evaluation
   *
   */
  struct optimization_params
  {
    std::vector<pcl::PointCloud<pcl::PointXYZI>> src_cloud_vec;
    std::vector<pcl::PointCloud<pcl::PointXYZI>> right_mirrored_pointclouds;
    std::vector<pcl::PointCloud<pcl::PointXYZI>> left_mirrored_pointclouds;
    std::vector<int> left_mirror_high_intensity_indices;
    std::vector<int> right_mirror_high_intensity_indices;
    std::vector<bool> * opt_constraints;
    optimization_result * opt_result;
  } typedef optimization_params;

  /**
   * @brief 
   * 
   * @param n 
   * @param x 
   * @param grad 
   * @param function_params 
   * @return double 
   */
  static double errFnMirrorFix(unsigned n, const double * x, double * grad, void * function_params);

  void optimizeNonVerbose(
    optimization_params & opt_params, std::vector<double> & plane_support_vec,
    std::vector<double> & plane_normal_vec, std::vector<double> & mirror_right_support_vec,
    std::vector<double> & mirror_right_normal_vec, std::vector<double> & mirror_left_support_vec,
    std::vector<double> & mirror_left_normal_vec, const int & no_opt_params, const double & epsabs,
    const double & stepsize, const size_t & iter_max, bool plot_error /*= false*/);

  /**
   * @brief Optimizes a given function using the GSL library
   *
   * @param[in] src_cloud vec of source pointclouds
   * @param[in] plane_support_vec support vector of mirror plane
   * @param[in] plane_normal_vec normal vector of mirror plane
   * @param[in] mirror_right_support_vec support vector of right mirror
   * @param[in] mirror_right_normal_vec normal vector of right mirror
   * @param[in] mirror_left_support_vec support vector of left mirror
   * @param[in] mirror_left_normal_vec normal vector of left mirror
   * @param[in] errFn pointer to function to be optimized
   * @param[in] no_opt_params number of parameters to be optimized
   *                          within the errFn
   * @param[in] epsabs absolute tolerance against which to the
   *                  norm of the gradient is compared to. Default: 1e-5
   * @param[in] stepsize stepsize of the optimization algorithm,
   *                  applied onto all input arguments. Default: 1e-3
   * @param[in] iter_max maximum number of iterations. Default: 1e3
   * @param[in] verbose calculate and return additional calibration results
   * @param[in] no_batches number of batches to be used in optimization result calculation, requires verbose to be true. Non positive indicates non-batch-evaluation. Default: 1
   * @param[in] batch_size size of minibatches to be used in optimization result calculation, requires verbose to be true. Non positive indicates non-batch-evaluation. Default: -1
   * @return void
   */
  void initOptimization(
    optimization_params & opt_params, std::vector<double> & plane_support_vec,
    std::vector<double> & plane_normal_vec, std::vector<double> & mirror_right_support_vec,
    std::vector<double> & mirror_right_normal_vec, std::vector<double> & mirror_left_support_vec,
    std::vector<double> & mirror_left_normal_vec, int no_opt_params /* = 18 */,
    double epsabs /* = 1e-5 */, double stepsize /* = 1e-3 */, size_t iter_max /* =1000 */,
    int verbose /* = 0 */, int no_batches /* = -1 */, int batch_size /* = -1 */);

  /**
   * @brief 
   * 
   * @param opt_params
   * @param plane_support_vec
   * @param plane_normal_vec
   * @param mirror_right_support_vec
   * @param mirror_right_normal_vec
   * @param mirror_left_support_vec
   * @param mirror_left_normal_vec
   * @param no_opt_params
   * @param epsabs
   * @param stepsize
   * @param iter_max
   * @param verbose
   * @param no_batches
   * @param batch_size
   */
  void optimizeVerbose(
    optimization_params & opt_params, std::vector<double> & plane_support_vec,
    std::vector<double> & plane_normal_vec, std::vector<double> & mirror_right_support_vec,
    std::vector<double> & mirror_right_normal_vec, std::vector<double> & mirror_left_support_vec,
    std::vector<double> & mirror_left_normal_vec, int no_opt_params /* = 18 */,
    double epsabs /* = 1e-5 */, double stepsize /* = 1e-3 */, size_t iter_max /* =1000 */,
    int verbose /* = false */, int no_batches /* = -1 */, int batch_size /* = -1 */);

  /**
   * @brief Callback function for incoming pointcloud messages
   *
   * @param[in] msg Incoming Pointcloud message
   */
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  /**
   * @brief Callback function for incoming laser scan messages
   *
   * @param[in] msg Incoming Laser Scan message
   */
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  /**
   * @brief Init Optimizationprocess. Utilizes the OSG_TF library functionality
   */
  void optimize();

  /**
   * @brief Optimize the orientation of the mirrors
   *          (Normal vectors of mirror planes)
   */
  bool optimizeMirrorOrientations();

  /** 
   * @brief Optimize the support vectors of the mirrors
   *          x-y planar distance between lidar and mirror
   */
  bool optimizeMirrorSupportVectors(int tgt_angle_lm = -1, int tgt_angle_rm = -1);

  /**
   * @brief Write the optimization results to a std out
   */
  void writeOptimizationResults();

  /**
   * @brief Split the incoming pointcloud into 3 parts. Only splitting is performed,
   *        no transformation is being applied as shown in the paper, transformation
   *        is done in the transformPointclouds() function.
   *
   *        The 3 parts are:
   *         1. Front part
   *         2. Left mirror part
   *         3. Right mirror part
   *
   * @param[in] src_cloud Pointcloud to be split
   */
  void splitPointclouds(const pcl::PointCloud<pcl::PointXYZI> & src_cloud);

  /**
   * @brief Transform each of the 3 pointclouds seperately, based on the
   *        formulas given in the paper.
   *
   */
  void transformPointclouds();

  /**
   * @brief Get the Highest Intensity Idx object
   *
   * @param pointcloud_buffer Vector of pointclouds
   * @param hI_idx Indices of the highest intensity points
   * @param mirror_left 0: left mirror, 1: right mirror (default: 0)
   */
  void getHighestIntensityIdx(
    std::vector<pcl::PointCloud<pcl::PointXYZI>> & pointcloud_buffer, std::vector<int> & hI_idx,
    bool mirror_left = 0);

  /**
   * @brief initialize a marker object based on core parameters
   *
   * @param[in] position position of marker in relation to src_frame_id
   * @param[in] orienation quaternion orientation of marker in relation to src_frame_id
   * @param[in] id marker id
   * @param[in] filepath path to mesh file
   * @param[out] marker marker object to be initialized
   * @param[in] src_frame_id frame id of marker
   * @param[in] rgba rgba color coded vector
   *
   */
  void initPlaneMarker(
    std::vector<double> & position, tf2::Quaternion & orienation, int id,
    std::vector<double> dimensions, visualization_msgs::msg::Marker::SharedPtr marker,
    const char * src_frame_id, std::vector<double> & rgba);

  /**
   * @brief initialize a marker object based on core parameters
   *
   * @param marker[in/out] marker object to be initialized
   * @param src_frame_id[in] frame id of marker
   * @param ns[in] namespace of marker
   * @param id[in] marker id
   * @param position[in] position of marker in relation to src_frame_id
   * @param orientation[in] orientation of marker in relation to src_frame_id
   * @param rgba[in] rgba color coded vector
   */
  void initVectorVisualizer(
    visualization_msgs::msg::Marker::SharedPtr marker, const char * src_frame_id, const char * ns,
    int id, std::vector<double> & rgba);

  /**
   * @brief Calculate the current row index of a given csv file
   *
   * @param[in] filename File to calculate the row index for
   * @return rowcount of the given csv file
   */
  int getCSVrowcount(std::string & filename);

  /**
   * @brief Transform a given vector to a string
   *
   * @param[in] vec Input vector
   * @return str_vector String representation of the input vector
   */
  template <typename T>
  std::string vectorToString(std::vector<T> & vec);

  /**
   * @brief Write the optimization results metadata to a csv file
   *
   * @param[in] filename File to write the results metadata to
   * @param[in] method_name Name of the optimization method used
   */
  void writeOptimizationResultsMetadata(std::ofstream & meta_file, std::string & method_name);

  /**
   * @brief  Write the optimization results to a csv file
   *
   * @param[in] results_stats_file File to write the results to
   * @param[in] method_name Name of the optimization method used
   */
  void writeOptimizationResultsStats(std::ofstream & results_stats_file, std::string & method_name);

  /**
   * @brief
   *
   * @param[out] opt_constraints Vector of optimization constraints
   */
  void initConstraints(std::vector<bool> * opt_constraints);

  /**
   * @brief Convert radians to degrees
   * 
   * @param[in] rad angle in radians 
   * @return double 
   */
  double rad2deg(const double & rad);

  /**
   * @brief Convert degrees to radians
   * 
   * @param[in] deg angle in degrees 
   * @return (double) angle in radians
   */
  double deg2rad(const double & deg);

  /**
   * @brief Identify the mirror points based on the given distance
   * 
   * @param[in] src_cloud Pointcloud to identify the mirror points
   * @param[in] mirror 0: left mirror, 1: right mirror
   * @param[in] mode 
   */
  void identifyMirrorPoints(
    const pcl::PointCloud<pcl::PointXYZI> & src_cloud, int mirror = 0, int mode = 0);

  /**
   * @brief 
   * 
   * @param[in] src_cloud 
   * @param[in] inital_guess_min 
   * @param[in] inital_guess_max
   * @param[out] min_max_indices 
   * @param[in] sliding_window_size Size of the sliding window used to eval the slope in the given window range. Defaults to 0. > 0 means using the elememnts +- sliding_window_size
   */
  void identifyMirrorPointsSlope(
    const pcl::PointCloud<pcl::PointXYZI> & src_cloud, double inital_guess_min,
    double inital_guess_max, int sliding_window_size = 1);

  /**
   * @brief 
   * 
   * @param src_cloud 
   * @param inital_guess_min 
   * @param inital_guess_max 
   * @param min_max_indices 
   * @param apply_sigma 
   */
  void identifyMirrorPointsMeanDist(
    const pcl::PointCloud<pcl::PointXYZI> & src_cloud, double inital_guess_min,
    double inital_guess_max, pcl::PointIndices * min_max_indices, bool apply_sigma = false);

  /**
   * @brief Initialize the optimization parameters according to the used parameters during during the lidar_mirror_reshaper calibration
   */
  void initOpenSeeGroundCalibOrientationParams();

  /**
   * @brief 
   * 
   * @param print_normal_vecs 
   * @param print_support_vecs 
   */
  void printOptimizationResults(bool print_normal_vecs = false, bool print_support_vecs = false);

  /**
   * @brief 
   * 
   * @param optimized_planes_viz 
   */
  void visualizeOptimizedPlanes(
    visualization_msgs::msg::MarkerArray::SharedPtr optimized_planes_viz);

  /**
   * @brief 
   * 
   * @param opt_result 
   */
  void visualizeNormalVectors(optimization_result & opt_result);

  /**
   * @brief 
   * 
   * @param transformed_cloud_all 
   * @param transformed_cloud_rm 
   * @param transformed_cloud_lm 
   */
  void visualizeTransformedPointclouds(
    pcl::PointCloud<pcl::PointXYZI> & transformed_cloud_all,
    pcl::PointCloud<pcl::PointXYZI> & transformed_cloud_rm,
    pcl::PointCloud<pcl::PointXYZI> & transformed_cloud_lm);

  /**
   * @brief 
   * 
   * @param p1 
   * @param p2 
   * @param t 
   * @param intensity 
   * @return pcl::PointXYZI 
   */
  pcl::PointXYZI interpolate(
    const pcl::PointXYZI & p1, const pcl::PointXYZI & p2, float t, double intensity);

  /**
   * @brief 
   * 
   */
  void averagingResourceBuffers();

  /**
   * @brief 
   * 
   */
  void averagingScan();

  /**
   * @brief 
   * 
   * @param algo 
   * @return std::string 
   */
  std::string numOptAlgorithm2String(nlopt_algorithm algo);

  /**
 * @brief Add pointcloud to a given buffer, ensures size compatibility of cloud to be added and buffer. Terminates if size is not compatible
 * 
 * @param pc_buffer 
 * @param pc_cmp Cloud to be added to the buffer
 */
  void addPointCloudToBuffer(
    std::vector<pcl::PointCloud<pcl::PointXYZI>> & pc_buffer,
    const pcl::PointCloud<pcl::PointXYZI> & pc_cmp);

  /**
   * @brief 
   * 
   * @param msg 
   */
  void pointcloudCallbackAveraging(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  /**
   * @brief 
   * 
   * @param opt_hist 
   * @param filename 
   * @param optimization_method 
   */
  void exportOptimizationHistory(
    const optimization_history & opt_hist, const std::string & filename,
    const std::string & optimization_method);

  /**
   * @brief extract/identify target points from the given pointcloud buffer
   * 
   * @param pointcloud_buffer buffer containing the pointclouds
   * @param hI_idx indices of the highest intensity points
   * @param mode 0: intensity based, 1: distance based
   * @param mirror_left 0: left mirror, 1: right mirror
   */
  void getTargetPoint(
    std::vector<pcl::PointCloud<pcl::PointXYZI>> & pointcloud_buffer, std::vector<int> & hI_idx,
    bool mode = 0, bool mirror_left = 0);

  /**
   * @brief 
   * 
   * @param value 
   * @return double 
   */
  double roundTo4Decimals(double value);

  // Subscriber
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr input_cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr input_scan_sub_;

  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_cloud_all_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_cloud_rm_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_cloud_lm_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr normal_vectors_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr optimized_plane_lm_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr optimized_plane_rm_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr optimized_plane_opt_plane_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cube_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr optimized_planes_pub_;

  // parameters
  int filter_method;
  double filter_dist_threshold;
  bool smoothingInputData;
  std::string laser_scanner_topic_;
  std::string laser_scanner_frame_id_;
  double laser_scanner_angle_min_;
  double laser_scanner_angle_max_;

  // vizualization parameters
  bool viz_transformed_cloud_all;
  bool viz_transformed_cloud_rm;
  bool viz_transformed_cloud_lm;
  bool viz_transformed_cloud_front;
  bool viz_normal_vectors;
  bool viz_optimized_planes_all;
  bool viz_optimized_plane_rm;
  bool viz_optimized_plane_lm;
  bool viz_optimized_plane_opt_plane;
  bool viz_optimization_plane_box_;

  int optimization_buffer_size_;
  bool optimization_opt_mirror_support_vec_;
  bool optimization_opt_mirror_orientation_;

  double optimization_epsabs_;
  double optimization_stepsize_;
  int optimization_iter_max_;
  bool optimization_adaptive_stepsize_;

  bool write_optimized_params_;
  bool write_optimization_history_;
  std::string optimization_history_file_;
  std::string optimized_params_meta_file_;
  std::string optimized_params_file_;
  int optimization_verbose_;
  int optimization_evaluation_no_batches_;

  int front_start_angle_;
  int front_end_angle_;

  int mirror_safety_bufferzone_size_lm;
  bool auto_define_lm_start_angle_;
  bool auto_define_lm_end_angle_;
  int auto_define_lm_angle_mode_;
  int mirror_left_start_angle_;
  int mirror_left_end_angle_;

  std::vector<double> mirror_left_helper_p1_;
  std::vector<double> mirror_left_helper_p2_;
  std::vector<double> mirror_left_helper_p3_;
  std::vector<double> mirror_left_support_vec_;

  int mirror_safety_bufferzone_size_rm;
  bool auto_define_rm_start_angle_;
  bool auto_define_rm_end_angle_;
  int auto_define_rm_angle_mode_;
  int mirror_right_start_angle_;
  int mirror_right_end_angle_;

  std::vector<double> mirror_right_helper_p1_;
  std::vector<double> mirror_right_helper_p2_;
  std::vector<double> mirror_right_helper_p3_;
  std::vector<double> mirror_right_support_vec_;

  std::vector<double> mirror_right_normal_vec_;
  std::vector<double> mirror_left_normal_vec_;
  std::vector<double> plane_normal_vec_;

  std::vector<double> plane_support_vec_;
  std::vector<double> plane_helper_p1_;
  std::vector<double> plane_helper_p2_;
  std::vector<double> plane_helper_p3_;

  // short-hand optimization flags
  bool opt_mirror_orientation_all;
  bool opt_mirror_orientation_mirror_svs;
  bool opt_mirror_orientation_mirror_nvs;
  bool opt_mirror_orientation_plane_sv;
  bool apply_opt_osg_settings;

  // mirror orientation optimization flags
  bool opt_mirror_orientation_rm_sv_x;
  bool opt_mirror_orientation_rm_sv_y;
  bool opt_mirror_orientation_rm_sv_z;

  bool opt_mirror_orientation_rm_nv_x;
  bool opt_mirror_orientation_rm_nv_y;
  bool opt_mirror_orientation_rm_nv_z;

  bool opt_mirror_orientation_lm_sv_x;
  bool opt_mirror_orientation_lm_sv_y;
  bool opt_mirror_orientation_lm_sv_z;

  bool opt_mirror_orientation_lm_nv_x;
  bool opt_mirror_orientation_lm_nv_y;
  bool opt_mirror_orientation_lm_nv_z;

  // calibration plane optimization flags
  bool opt_mirror_orientation_plane_sv_x;
  bool opt_mirror_orientation_plane_sv_y;
  bool opt_mirror_orientation_plane_sv_z;

  bool opt_mirror_orientation_plane_nv_x;
  bool opt_mirror_orientation_plane_nv_y;
  bool opt_mirror_orientation_plane_nv_z;

  // class members
  std::vector<pcl::PointCloud<pcl::PointXYZI>> pointcloud_buffer_;
  std::vector<pcl::PointCloud<pcl::PointXYZI>> pointcloud_buffer_front_;
  std::vector<pcl::PointCloud<pcl::PointXYZI>> pointcloud_buffer_left_mirror_;
  std::vector<pcl::PointCloud<pcl::PointXYZI>> pointcloud_buffer_right_mirror_;

  std::vector<pcl::PointCloud<pcl::PointXYZI>> transformed_pointcloud_buffer_;
  std::vector<pcl::PointCloud<pcl::PointXYZI>> transformed_pointcloud_buffer_front_;
  std::vector<pcl::PointCloud<pcl::PointXYZI>> transformed_pointcloud_buffer_left_mirror_;
  std::vector<pcl::PointCloud<pcl::PointXYZI>> transformed_pointcloud_buffer_right_mirror_;

  // temporary @todo fix this
  std::vector<pcl::PointCloud<pcl::PointXYZI>> raw_pointcloud_buffer_;
  std::vector<pcl::PointCloud<pcl::PointXYZI>> raw_pointcloud_buffer_front_;
  std::vector<pcl::PointCloud<pcl::PointXYZI>> raw_pointcloud_buffer_left_mirror_;
  std::vector<pcl::PointCloud<pcl::PointXYZI>> raw_pointcloud_buffer_right_mirror_;
  // -----------------------------

  double intensity_threshold_percentage_;
  int interpolation_window;
  int averaging_n_clouds;

  bool opt_flag_ = 1;
  optimization_result optimization_results_;
  std::vector<int> indices_high_intensity_lm_;
  std::vector<int> indices_high_intensity_rm_;
  std::vector<sensor_msgs::msg::LaserScan> laser_scans;

  bool tmp_avg_flag = false;

  std::string ros_ns;
  nlopt_algorithm num_opt_algorithm;
};

#endif  // LIDAR_MIRROR_FOV_RESHAPER_CALIB_HPP