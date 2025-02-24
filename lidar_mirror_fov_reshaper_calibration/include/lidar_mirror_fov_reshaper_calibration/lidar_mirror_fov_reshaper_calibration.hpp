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
#include <laser_geometry/laser_geometry.hpp>
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
 * @brief Calculates the error function for the lidar mirror field of view (FOV) reshaping calibration.
 * 
 * This function evaluates the Root Mean Square (RMS) error based on various points (front, right mirror, left mirror) and their distances to a calibration plane. It also calculates reflection point errors and updates the calibration results with detailed RMS values.
 *
 * The optimization algorithm uses this error function to adjust mirror parameters for an optimal calibration.
 * 
 * @param n The number of parameters in the optimization (length of `x`).
 * @param x Array of optimization parameters (support vectors and normal vectors of mirrors and plane).
 * @param grad Array for storing the gradients of the error function (if required by the optimizer).
 * @param function_params A pointer to the function parameters, including the lidar point clouds and calibration results.
 * 
 * @return double The computed RMS error for the calibration, summing up various error components including front, left mirror, and right mirror distances.
 */
  static double errFnMirrorFix(unsigned n, const double * x, double * grad, void * function_params);

  /**
   * @brief @todo
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
   */
  void optimizeNonVerbose(
    optimization_params & opt_params, std::vector<double> & plane_support_vec,
    std::vector<double> & plane_normal_vec, std::vector<double> & mirror_right_support_vec,
    std::vector<double> & mirror_right_normal_vec, std::vector<double> & mirror_left_support_vec,
    std::vector<double> & mirror_left_normal_vec, const int & no_opt_params, const double & epsabs,
    const double & stepsize, const size_t & iter_max);

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
 * @brief Perform optimization with verbose output, possibly in batches.
 *
 * This function performs the optimization process with verbose logging and optional batching of data.
 * It splits the data into smaller batches and optimizes each batch independently before performing
 * the final optimization on all data. The function outputs various optimization results such as RMS
 * values, angle deviations, and the optimization status.
 * 
 * @param[in,out] opt_params The optimization parameters containing point clouds and constraints.
 * @param[in,out] plane_support_vec The support vector of the plane.
 * @param[in,out] plane_normal_vec The normal vector of the plane.
 * @param[in,out] mirror_right_support_vec The support vector of the right mirror.
 * @param[in,out] mirror_right_normal_vec The normal vector of the right mirror.
 * @param[in,out] mirror_left_support_vec The support vector of the left mirror.
 * @param[in,out] mirror_left_normal_vec The normal vector of the left mirror.
 * @param[in] no_opt_params Number of optimization parameters (default is 18).
 * @param[in] epsabs The absolute tolerance for optimization (default is 1e-5).
 * @param[in] stepsize The step size for the optimization algorithm (default is 1e-3).
 * @param[in] iter_max The maximum number of iterations for the optimization (default is 1000).
 * @param[in] verbose Flag to enable or disable verbose logging (default is false).
 * @param[in] no_batches The number of batches to split the data into (default is -1, indicating no batching).
 * @param[in] batch_size The size of each batch (default is -1, indicating auto calculation based on data size).
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
   * @brief Init Optimizationprocess. Utilizes the LMFR_TF library functionality
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
   * 
   * @param[in] tgt_angle_lm Target angle for left mirror
   * @param[in] tgt_angle_rm Target angle for right mirror
   * @return bool
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
   * @param id[in] marker id
   * @param position[in] position of marker in relation to src_frame_id
   * @param orientation[in] orientation of marker in relation to src_frame_id
   * @param rgba[in] rgba color coded vector
   * @param scale[in] scale of the marker 
   */
  void initVectorVisualizer(
    visualization_msgs::msg::Marker::SharedPtr marker, const char * src_frame_id, int id,
    std::vector<double> & rgba, double scale = 0.001);

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
 * @brief Initializes the optimization constraints for mirror orientation calibration.
 * 
 * This function sets up the optimization constraints based on the current configuration of mirror orientation and plane parameters. It populates the `opt_constraints` vector with boolean values indicating which parameters are subject to optimization. The constraints can be applied to the plane, right mirror, and left mirror based on the object's member variables.
 * 
 * If the optimization for mirror orientations is enabled, corresponding values in the `opt_constraints` vector will be set to `true`. Otherwise, they remain unchanged.
 * 
 * @param[out] opt_constraints A vector of boolean values representing the optimization constraints for the calibration. 
 *                              The size of the vector should be at least 18, corresponding to the different parameters for the plane and the two mirrors.
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
 * @brief Visualizes the optimized planes and mirrors using ROS markers.
 * 
 * This function visualizes the optimized calibration plane and the left and right mirrors as 3D markers in a ROS environment. The planes and mirrors are represented using their support vectors and the corresponding normal vectors. The visualization markers are created and added to the provided `MarkerArray` for display. Depending on the configuration, the markers are either published together or separately for the calibration plane, left mirror, and right mirror.
 * 
 * @param[in] optimized_planes_viz Shared pointer to the `MarkerArray` that holds the 3D markers for the optimized planes and mirrors.
 */
  void visualizeOptimizedPlanes(
    visualization_msgs::msg::MarkerArray::SharedPtr optimized_planes_viz);

  /**
 * @brief Visualizes the normal vectors for the left mirror, right mirror, and calibration plane.
 * 
 * This function visualizes the normal vectors for the left and right mirrors as well as the calibration plane in a ROS environment. The normal vectors are visualized as 3D arrows in the point cloud coordinate system. The roll, pitch, and yaw angles of the normal vectors are computed and stored in the provided `opt_result` object for later use. The normal vectors are displayed using markers and can be published based on the configuration.
 * 
 * @param[in] opt_result A reference to the `optimization_result` object where the roll, pitch, and yaw values for the left and right mirror normal vectors are stored.
 */
  void visualizeNormalVectors(optimization_result & opt_result);

  /**
 * @brief Visualizes and publishes transformed point clouds for the combined, left mirror, and right mirror data.
 * 
 * This function publishes the transformed point clouds for the entire set, as well as for the left and right mirrors, as ROS `PointCloud2` messages. It checks the sizes of the transformed point clouds to ensure they match the expected dimensions based on the coverage area of the mirrors. If the point cloud sizes are inconsistent, an error message is logged. The point clouds are then converted to the ROS format and published to their respective topics.
 * 
 * @param[in] transformed_cloud_all The transformed point cloud representing the entire dataset, including the left and right mirrors.
 * @param[in] transformed_cloud_rm The transformed point cloud for the right mirror.
 * @param[in] transformed_cloud_lm The transformed point cloud for the left mirror.
 */
  void visualizeTransformedPointclouds(
    pcl::PointCloud<pcl::PointXYZI> & transformed_cloud_all,
    pcl::PointCloud<pcl::PointXYZI> & transformed_cloud_rm,
    pcl::PointCloud<pcl::PointXYZI> & transformed_cloud_lm);

  /**
 * @brief Interpolates between two points and assigns a given intensity to the resulting point.
 * 
 * This function performs linear interpolation between two `pcl::PointXYZI` points, `p1` and `p2`, based on the interpolation parameter `t` (where `t` ranges from 0 to 1). The resulting interpolated point is calculated by linearly interpolating the x, y, and z coordinates of the two input points. The intensity of the resulting point is set to the given `intensity` value.
 * 
 * @param[in] p1 The first point, used as the starting point for interpolation.
 * @param[in] p2 The second point, used as the ending point for interpolation.
 * @param[in] t The interpolation factor, where 0 results in `p1` and 1 results in `p2`.
 * @param[in] intensity The intensity value to assign to the interpolated point.
 * 
 * @return A new `pcl::PointXYZI` point that is the result of the interpolation, with the specified intensity.
 */
  pcl::PointXYZI interpolate(
    const pcl::PointXYZI & p1, const pcl::PointXYZI & p2, float t, double intensity);

  /**
 * @brief Averages point clouds from various buffers to create a single averaged point cloud.
 * 
 * This function performs averaging of point clouds stored in different buffers: left mirror, right mirror, front, and general buffers. It checks if the buffers have accumulated the required number of point clouds (defined by `averaging_n_clouds`). If the buffer size matches, the function sums the points from all the clouds and then divides by the total number of clouds to compute the average. If the buffer size does not match the expected size, the function will log an error and shut down. After averaging, the function pushes the averaged point clouds back into the respective buffers.
 * 
 * The function also ensures that the point clouds in the buffers are aligned and of equal size before performing averaging, and if the buffer sizes do not match, the program shuts down to avoid incorrect processing.
 * 
 * @return void
 */
  void averagingResourceBuffers();

  /**
 * @brief Converts a given `nlopt_algorithm` to a string representation.
 * 
 * This function takes an optimization algorithm identifier of type `nlopt_algorithm` and returns its corresponding string representation. The mapping between the algorithm and its string name is defined in a static `std::map`. If the provided algorithm is found in the map, the corresponding string is returned. Otherwise, it returns "UNKNOWN".
 * 
 * @param algo The optimization algorithm to be converted to a string.
 * @return A string representing the optimization algorithm.
 */
  std::string numOptAlgorithm2String(nlopt_algorithm algo);

  /**
 * @brief Add pointcloud to a given buffer, ensures size compatibility of cloud to be added and buffer.
 * 
 * This function adds a point cloud (`pc_cmp`) to a given buffer (`pc_buffer`). If the buffer is empty, the cloud is simply pushed into the buffer. If the buffer is not empty, it checks the size of the last point cloud in the buffer and compares it with the size of the cloud to be added. If the sizes do not match, an error message is logged and the program is terminated. If the sizes match, the cloud is added to the buffer.
 * 
 * @param pc_buffer The buffer to which the point cloud is to be added. This is a vector of point clouds.
 * @param pc_cmp The point cloud that is to be added to the buffer.
 */
  void addPointCloudToBuffer(
    std::vector<pcl::PointCloud<pcl::PointXYZI>> & pc_buffer,
    const pcl::PointCloud<pcl::PointXYZI> & pc_cmp);

  /**
 * @brief Callback for processing incoming point clouds with averaging functionality.
 * 
 * This function processes incoming point clouds by first separating the points based on their angles into different regions: left mirror, right mirror, and front. These separated point clouds are then added to their respective buffers for further processing. If the buffer has fewer point clouds than the defined batch size (`averaging_n_clouds`), the point cloud is simply added to the buffer. Once the buffer reaches the batch size, the function averages the stored point clouds, splits them into different parts (e.g., mirror and front points), and clears the buffer for the next batch.
 * 
 * It also performs optimization if a certain number of batches have been processed.
 * 
 * @param msg The incoming point cloud message (type `sensor_msgs::msg::PointCloud2`) containing raw point cloud data.
 */
  void pointcloudCallbackAveraging(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  /**
 * @brief Exports the optimization history to a CSV file.
 * 
 * This function writes the provided optimization history data to a CSV file for later analysis. The file is stored in the `results` directory of the package. If the file already exists, the function appends the data to it. The file includes various optimization metrics such as pitch and yaw of the left and right mirrors, error values related to front, left mirror, right mirror, and reflection points.
 * 
 * The exported data helps in tracking the optimization process over time and can be used for further analysis or visualization.
 * 
 * @param opt_hist The optimization history object containing the data to be exported.
 * @param filename The name of the file to export the data to.
 * @param optimization_method The method used for optimization (used as a descriptor in the file).
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
   * @brief Round a given double value to n decimals places
   * 
   * @param[in] value unrounded double value
   * @param[in] n number of decimal places
   * @return double rounded value
   */
  double roundToNDecimals(double value, int n = 4);

  /**
 * @brief Identifies outliers in a point cloud based on the distance from a fitted line.
 * 
 * This function fits a line to the point cloud using principal component analysis (PCA). It computes the line's direction and identifies points that are outliers based on their perpendicular distance to the fitted line. Any point that exceeds the specified distance threshold from the line is considered an outlier.
 * 
 * The function returns the indices of the points that are identified as outliers. These outliers can be used for further processing, such as filtering out noise or analyzing unusual patterns in the data.
 * 
 * @param cloud The input point cloud to be analyzed.
 * @param threshold The maximum distance from the line for a point to be considered an outlier. Points with a distance greater than this threshold will be classified as outliers.
 * @return A vector of indices of the outlier points in the cloud.
 */
  std::vector<int> identifyOutliers(
    const pcl::PointCloud<pcl::PointXYZI> & cloud, double threshold);

  /**
 * @brief Calculates the perpendicular distance from a point to a line in 3D space.
 * 
 * This function computes the perpendicular (shortest) distance from a point in 3D space to a line. The line is defined by a point on the line and a direction vector. The distance is calculated using the cross product of the vector from the point to the line and the line's direction vector. The norm of this cross product, divided by the norm of the line's direction vector, gives the distance.
 * 
 * @param point The point for which the distance to the line is being calculated.
 * @param linePoint A point on the line.
 * @param lineDirection The direction vector of the line.
 * @return The perpendicular distance from the point to the line.
 */
  double pointToLineDistance(
    const pcl::PointXYZI & point, const Eigen::Vector3d & linePoint,
    const Eigen::Vector3d & lineDirection);

  /**
 * @brief Calculates the cross product of two 3D vectors.
 * 
 * This function computes the cross product of two 3D vectors. The cross product results in a new vector that is perpendicular to both input vectors. The function assumes that both input vectors have a size of 3. If the input vectors do not have a size of 3, an error message will be logged, and an empty vector will be returned.
 * 
 * @param a The first vector in the cross product calculation.
 * @param b The second vector in the cross product calculation.
 * @return A vector containing the cross product of the two input vectors.
 *         If the input vectors are not 3D, an empty vector is returned.
 */
  std::vector<double> crossProduct(std::vector<double> & a, std::vector<double> & b);

  int applyMirrorBoundary(bool mirror_left, int idx_highest_intensity, size_t cloud_size);

  void applyInterpolation(
    bool mirror_left, int idx_highest_intensity, pcl::PointCloud<pcl::PointXYZI> & cloud,
    double max_intensity);

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
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr calib_plane_box_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr optimized_planes_pub_;

  // parameters
  int filter_method_;
  double filter_dist_threshold_;
  std::string laser_scanner_topic_;
  std::string laser_scanner_frame_id_;
  double laser_scanner_angle_min_;
  double laser_scanner_angle_max_;

  // vizualization parameters
  bool viz_transformed_cloud_all_;
  bool viz_transformed_cloud_rm_;
  bool viz_transformed_cloud_lm_;
  bool viz_transformed_cloud_front_;
  bool viz_normal_vectors_;
  bool viz_optimized_planes_all_;
  bool viz_optimized_plane_rm_;
  bool viz_optimized_plane_lm_;
  bool viz_optimized_plane_opt_plane_;
  bool viz_optimization_plane_box_;
  std::vector<double> viz_mirror_planes_dim_;
  std::vector<double> viz_calib_plane_dim_;

  int optimization_buffer_size_;
  bool optimization_opt_mirror_support_vec_;
  bool optimization_opt_mirror_orientation_;

  double optimization_epsabs_;
  double optimization_stepsize_;
  int optimization_iter_max_;

  bool write_optimized_params_;
  bool write_optimization_history_;
  std::string optimization_history_file_;
  std::string optimized_params_meta_file_;
  std::string optimized_params_file_;
  int optimization_verbose_;
  int optimization_evaluation_no_batches_;

  double front_start_angle_;
  double front_end_angle_;

  int mirror_safety_bufferzone_size_lm_;
  int auto_define_lm_angle_mode_;
  double mirror_left_start_angle_;
  double mirror_left_end_angle_;

  std::vector<double> mirror_left_helper_p1_;
  std::vector<double> mirror_left_helper_p2_;
  std::vector<double> mirror_left_helper_p3_;
  std::vector<double> mirror_left_support_vec_;

  int mirror_safety_bufferzone_size_rm_;
  int auto_define_rm_angle_mode_;
  double mirror_right_start_angle_;
  double mirror_right_end_angle_;

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
  bool opt_mirror_orientation_all_;
  bool opt_mirror_orientation_mirror_svs_;
  bool opt_mirror_orientation_mirror_nvs_;
  bool opt_mirror_orientation_plane_sv_;
  bool apply_opt_osg_settings_;

  // mirror orientation optimization flags
  bool opt_mirror_orientation_rm_sv_x_;
  bool opt_mirror_orientation_rm_sv_y_;
  bool opt_mirror_orientation_rm_sv_z_;

  bool opt_mirror_orientation_rm_nv_x_;
  bool opt_mirror_orientation_rm_nv_y_;
  bool opt_mirror_orientation_rm_nv_z_;
  bool rm_auto_calc_normal_vec_;

  bool opt_mirror_orientation_lm_sv_x;
  bool opt_mirror_orientation_lm_sv_y;
  bool opt_mirror_orientation_lm_sv_z;

  bool opt_mirror_orientation_lm_nv_x_;
  bool opt_mirror_orientation_lm_nv_y_;
  bool opt_mirror_orientation_lm_nv_z_;
  bool lm_auto_calc_normal_vec_;

  // calibration plane optimization flags
  bool opt_mirror_orientation_plane_sv_x_;
  bool opt_mirror_orientation_plane_sv_y_;
  bool opt_mirror_orientation_plane_sv_z_;

  bool opt_mirror_orientation_plane_nv_x_;
  bool opt_mirror_orientation_plane_nv_y_;
  bool opt_mirror_orientation_plane_nv_z_;

  // class members
  std::vector<pcl::PointCloud<pcl::PointXYZI>> pointcloud_buffer_;
  std::vector<pcl::PointCloud<pcl::PointXYZI>> pointcloud_buffer_front_;
  std::vector<pcl::PointCloud<pcl::PointXYZI>> pointcloud_buffer_left_mirror_;
  std::vector<pcl::PointCloud<pcl::PointXYZI>> pointcloud_buffer_right_mirror_;

  std::vector<pcl::PointCloud<pcl::PointXYZI>> transformed_pointcloud_buffer_;
  std::vector<pcl::PointCloud<pcl::PointXYZI>> transformed_pointcloud_buffer_front_;
  std::vector<pcl::PointCloud<pcl::PointXYZI>> transformed_pointcloud_buffer_left_mirror_;
  std::vector<pcl::PointCloud<pcl::PointXYZI>> transformed_pointcloud_buffer_right_mirror_;

  laser_geometry::LaserProjection projector_;

  std::vector<pcl::PointCloud<pcl::PointXYZI>> raw_pointcloud_buffer_;
  std::vector<pcl::PointCloud<pcl::PointXYZI>> raw_pointcloud_buffer_front_;
  std::vector<pcl::PointCloud<pcl::PointXYZI>> raw_pointcloud_buffer_left_mirror_;
  std::vector<pcl::PointCloud<pcl::PointXYZI>> raw_pointcloud_buffer_right_mirror_;

  double intensity_threshold_percentage_;
  int interpolation_window;
  int averaging_n_clouds;

  bool opt_flag_ = 1;
  optimization_result optimization_results_;
  std::vector<int> indices_high_intensity_lm_;
  std::vector<int> indices_high_intensity_rm_;
  std::vector<sensor_msgs::msg::LaserScan> laser_scans_;

  bool tmp_avg_flag = false;

  nlopt_algorithm num_opt_algorithm_;
};

#endif  // LIDAR_MIRROR_FOV_RESHAPER_CALIB_HPP