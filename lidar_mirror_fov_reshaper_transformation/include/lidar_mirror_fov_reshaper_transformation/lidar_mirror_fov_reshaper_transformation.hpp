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

#ifndef LIDAR_MIRROR_FOV_RESHAPER_TF_HPP
#define LIDAR_MIRROR_FOV_RESHAPER_TF_HPP

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <numeric>
#include <type_traits>
#include <vector>

#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace lidarMirrorFOVReshaperTF
{

/**
   * @brief Calculates the reflection point/point of intersection of lidar ray and mirror plane
   *        part of formula 1) (Only available in the documentation):
   *
   * \begin{equation}
   * I = \frac{n \cdot w}{n \cdot d} \cdot d \label{eq:eq1}
   * \end{equation}
   *
   * decomposed into:
   * \f{eqnarray*}{
   * tmp1 &=& n \cdot w \label{eq:eq11} \\
   * tmp2 &=& n \cdot d \label{eq:eq12} \\
   * tmp3 &=& tmp1/tmp2 \label{eq:eq13} \\
   * I &=& tmp3 \cdot d  \label{eq:eq14} \\
   * \f}
   * where:
   *
   * \f$n = normal\_vec\f$
   *
   * \f$w = support\_vec\f$
   *
   * \f$d = distance\_vec\_raw\f$
   *
   * @param[in] normal_vec normal of mirror plane
   * @param[in] support_vec support vector of mirror plane
   * @param[in] distance_vec_raw distance vector, raw lidar distanced measured
   * @param[out] reflection_point point of intersection of lidar ray and mirror plane
   */
std::vector<double> getReflectionPoint(
  const std::vector<double> & normal_vec, const std::vector<double> & support_vec,
  const std::vector<double> & distance_vec_raw);

/**
   * @brief Extract and normalize points from within given source pointcloud. Part of formula 2) (Only available in the documentation):
   *
   * \begin{equation}
   * r = \frac{d}{\left | d \right |}-2\left ( \frac{d}{\left | d \right |} \cdot \frac{n}{\left | n \right |} \right ) \cdot \frac{n}{\left | n \right |} \label{eq:eq2}
   * \end{equation}
   * where:
   *
   * \f$d = distance\_vec\_raw\f$
   *
   * \f$n = normal\_vec\f$
   *
   * @param[in] src_cloud source point cloud, cropped to target mirror area
   * @param[out] normalized_points vector containing normalized points
   */
void normalizePoints(
  const pcl::PointCloud<pcl::PointXYZI> & src_cloud,
  std::vector<std::vector<double>> * normalized_points);

/**
 * @brief Transform indice values to angles. Assumes that indice spacing equals 1deg angle spacing.
 * 
 * @param[in] idx incdice to be translated
 * @param[in] size @todo
 * @return double angle corresponding to the input indice 
 */
double idxToAngle(int idx, int size);

/**
   * @brief Get the Normalized Vector object
   *
   * @param[in] vec vector containing non-normalized data
   * @param[out] normalized_vec vector containing normalized data
   */
std::vector<double> getNormalizedVector(const std::vector<double> & vec_src);

/**
   * @brief Get the Unit Reflection Vector object
   *
   * @param[in] normalized_distance_vec @todo
   * @param[in] normalized_mirror_normal_vec normalized normal vector representing the mirrors orientation
   * @param[out] unit_reflection_vec @todo
   */
void getUnitReflectionVector(
  const std::vector<double> & normalized_distance_vec,
  const std::vector<double> & normalized_mirror_normal_vec,
  std::vector<double> & unit_reflection_vec);

/**
   * @brief Get the Unit Reflection Vectors object
   *
   * @param[in] normalized_distance_vectors vector containing normalized distance vectors
   * @param[in] normalized_points vector containing normalized points
   * @param[out] unit_reflection_vectors vector containing unit reflection vectors
   */
void getUnitReflectionVectors(
  const std::vector<std::vector<double>> & normalized_distance_vectors,
  const std::vector<double> & normalized_mirror_normal_vec,
  std::vector<std::vector<double>> * unit_reflection_vectors);

/**
   * @brief Get the Magnitude point objects from within given source pointcloud
   *
   * @param[in] src_cloud source point cloud
   * @param[out] point_magnitude vector containing magnitude of each point
   */
void getPointMagnitude(
  const pcl::PointCloud<pcl::PointXYZI> & src_cloud, std::vector<double> * point_magnitude);

/**
   * @brief Get the Magnitude point objects from within given source vector
   *
   * @param[in] src_vec source vector, containing points stored as vector objects
   * @param[out] point_magnitude vector containing magnitude of each point/vector
   */
void getPointMagnitude(
  const std::vector<std::vector<double>> & src_vec, std::vector<double> * point_magnitude);

/**
   * @brief Get the Magnitude of a vector object of arbitrary type. Using the following equation:
   *
   * \begin{equation}
   * \left \| v \right \|=\sqrt{{v_1}^2 + {v_2}^2 + ... + {v_n}^2}
   * \end{equation}
   *
   * @param[in] src_vec source vector
   * @return magnitude of vector
   */
double getVectorMagnitude(const std::vector<double> & src_vec);

/**
   * @brief @todo
   *
   * @param[in] reflection_point @todo
   * @param[in] reflection_vector @todo
   * @param[out] transformed_vector @todo
   */
void getTransformedVector(
  const std::vector<double> & reflection_point, const std::vector<double> & reflection_vector,
  std::vector<double> & transformed_vector);

/**
   * @brief Get the Transformed Vectors object
   *
   * @param[in] reflection_points @todo
   * @param[in] reflection_points_to_tf_data_vectors @todo
   * @param[out] transformed_distance_vectors @todo
   */
void getTransformedVectors(
  const std::vector<std::vector<double>> & reflection_points,
  const std::vector<std::vector<double>> & reflection_points_to_tf_data_vectors,
  std::vector<std::vector<double>> * transformed_distance_vectors);

/**
   * @brief
   *
   * @param[in] unit_reflection_vec @todo
   * @param[in] mag_reflection_vec @todo
   * @param[in] mag_distance_vec @todo
   * @param[out] reflection_point_to_tf_data_point @todo
   */
std::vector<double> _getVectorToTransformedDataPoint(
  const std::vector<double> & unit_reflection_vec, double mag_reflection_vec,
  double mag_distance_vec);

/**
   * @brief Get the Vectors To Transformed Data Points object
   *
   * @param[in] unit_reflection_vectors @todo
   * @param[in] magnitudes_reflection_points @todo
   * @param[in] magnitudes_distance_vectors @todo
   * @param[out] reflection_points_to_tf_data_points @todo
   */
void getVectorsToTransformedDataPoints(
  const std::vector<std::vector<double>> & unit_reflection_vectors,
  const std::vector<double> & magnitudes_reflection_points,
  const std::vector<double> & magnitudes_distance_vectors,
  std::vector<std::vector<double>> * reflection_points_to_tf_data_points);

/**
   * @brief @todo
   *
   * @param[in] distance_vectors @todo
   * @param[out] target_cloud @todo
   */
void transformVectorIntoPointCloud(
  const std::vector<std::vector<double>> & distance_vectors,
  pcl::PointCloud<pcl::PointXYZI> * target_cloud);

/**
   * @brief Calculates the reflection points on the mirror plane of each
   *          point within the given source pointcloud
   *
   * @param[in] src_cloud source point cloud
   * @param[in] normal_vec normal of mirror plane.
   *                          Corresponds the targeted mirror of src_cloud.
   *                          Origin == point on mirror plane.
   * @param[in] support_vec support vector of mirror plane.
   *                          Corresponds the targeted mirror of src_cloud.
   *                          Origin == point onto
   * @param[out] reflection_points vector containing reflection points,
   *              per point in source point cloud.
   *              Therefore reflection_points.size == src_cloud->points.size
   */
void calcReflectionPoints(
  const pcl::PointCloud<pcl::PointXYZI> & src_cloud, const std::vector<double> & normal_vec,
  const std::vector<double> & support_vec, std::vector<std::vector<double>> * reflection_points);

/**
   * @brief Calculates the mirror plane of the given three points
   *
   * @param[in] helper_p1 first point
   * @param[in] helper_p2 second point
   * @param[in] helper_p3 third point
   *
   * @param[out] normal_vec normal vector to of plane thats being calculated
   *                         using the given three helper points
   */
void calcMirrorPlaneNormal(
  const std::vector<double> & helper_p1, const std::vector<double> & helper_p2,
  const std::vector<double> & helper_p3, std::vector<double> & normal_vec);

/**
   * @brief Calculates the distance from a cartesian point to a plane
   *         defined by a normal vector and a support vector
   *
   * @param[in] src_cloud pointcloud containing points to be used in calculation
   * @param[in] plane_support_vec support vector of plane
   * @param[in] plane_normal_vec normal vector of plane
   * @param[out] point_plane_dist calculated distance from point to plane
   *
   */
void calcPointPlaneDist(
  const pcl::PointCloud<pcl::PointXYZI> & src_cloud, const std::vector<double> & plane_support_vec,
  const std::vector<double> & plane_normal_vec, std::vector<double> * point_plane_distances);

/**
   * @brief Transform raw pointcloud data by
   *          applying the formulas shown in the paper
   *
   * @param[in] src_cloud (raw) source pointcloud-data
   * @param[in] src_cloud_left_mirror (raw) source pointcloud-data
   *                                  containing data mirrored by left mirror
   * @param[in] src_cloud_right_mirror (raw) source pointcloud-data
   *                                  containing data mirrored by right mirror
   * @param[in] mirror_left_normal_vec normal vector of left mirror
   * @param[in] mirror_left_support_vec support vector of left mirror
   * @param[in] mirror_right_normal_vec normal vector of right mirror
   * @param[in] mirror_right_support_vec support vector of right mirror
   * @param[out] transformed_cloud transformed/mirrored pointcloud-data
   * @param[out] transformed_cloud_left_mirror transformed/mirrored pointcloud-data
   * @param[out] transformed_cloud_right_mirror transformed/mirrored pointcloud-data
   */
void transformCloud(
  const pcl::PointCloud<pcl::PointXYZI> & src_cloud_front,
  const pcl::PointCloud<pcl::PointXYZI> & src_cloud_left_mirror,
  const pcl::PointCloud<pcl::PointXYZI> & src_cloud_right_mirror,
  const std::vector<double> & mirror_left_normal_vec,
  const std::vector<double> & mirror_left_support_vec,
  const std::vector<double> & mirror_right_normal_vec,
  const std::vector<double> & mirror_right_support_vec,
  pcl::PointCloud<pcl::PointXYZI> * transformed_cloud,
  pcl::PointCloud<pcl::PointXYZI> * transformed_cloud_left_mirror,
  pcl::PointCloud<pcl::PointXYZI> * transformed_cloud_right_mirror);

/**
   * @brief Get the Mirror Center Points object
   *
   * @param[in] pointcloud_buffer @todo
   * @param[in] reflection_angle @todo
   * @param[out] reflection_points @todo
   */
void getMirrorReflectionPoints(
  const std::vector<pcl::PointCloud<pcl::PointXYZI>> & pointcloud_buffer,
  std::vector<int> & indices, std::vector<pcl::PointXYZI> & reflection_points);

/**
   * @brief Calculate the standard deviation of a given vector
   *
   * @param[in] vec input vector
   * @return standard deviation of given vector
   */
double getStdDev(const std::vector<double> & vec);

/**
   * @brief Get the Pointcloud Indices object
   *
   * @param[in] pcl_msg      @todo
   * @param[in] start_angle @todo
   * @param[in] end_angle @todo
   * @param[out] indices @todo
   */
void getPointcloudIndices(
  const pcl::PointCloud<pcl::PointXYZI> & pcl_msg, int start_angle, int end_angle,
  pcl::PointIndices * indices);

/**
   * @brief Calculate the RPY angles based on the given normal vector
   *
   * @param normal_vec @todo
   * @return std::vector<double>
   */
std::vector<double> calcRPY(std::vector<double> normal_vec);

/**
   * @brief Convert degrees to radians
   * 
   * @param[in] deg angle in degrees 
   * @return (double) angle in radians
   */
double deg2rad(const double & deg);

/**
 * @brief Translate angular data into its corresponding indice. Used to translate between angular spaced data into array indexed data
 * 
 * @param[in] tgt_angle angle to be translated
 * @param[in] angle_min [deg] lower bound interval of the angle data interval
 * @param[in] angle_max [deg] upper bound interval of the angle data interval
 * @param[in] tgt_area_angle_min @todo
 * @param[in] tgt_area_angle_max @todo
 * @param[in] angle_increment [deg] angle increment of the tgt_angle data interval
 * @return int target indice corresponding to the input angle
 */
int angle2ArrIdx(
  double tgt_angle, double angle_min, double angle_max, double tgt_area_angle_min,
  double tgt_area_angle_max, double angle_increment = 1);

};  // namespace lidarMirrorFOVReshaperTF

#endif  // LIDAR_MIRROR_FOV_RESHAPER_TF_HPP