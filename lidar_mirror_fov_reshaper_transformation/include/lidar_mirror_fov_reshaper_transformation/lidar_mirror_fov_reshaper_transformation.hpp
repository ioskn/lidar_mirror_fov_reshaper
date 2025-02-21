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
 * @param[in] idx indice to be translated into its corresponding angle
 * @param[in] size number of scans in the reference FOV
 * @return double angle corresponding to the input indice. [deg]
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
 * @brief Compute the unit reflection vector based on an incident direction and mirror normal.
 *
 * This function calculates the unit reflection vector using the reflection formula:
 *
 * \f[
 * r = d - 2 (d \cdot n) n
 * \f]
 *
 * where:
 * - \f$d\f$ is the normalized incident direction vector.
 * - \f$n\f$ is the normalized mirror normal vector.
 * - \f$r\f$ is the resulting reflection vector.
 *
 * @param[in] normalized_distance_vec Normalized vector representing the incident direction.
 * @param[in] normalized_mirror_normal_vec Normalized normal vector representing the mirror's orientation.
 * @param[out] unit_reflection_vec Computed unit reflection vector.
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
 * @brief Compute the transformed vector by applying the reflection vector to a given point.
 *
 * This function calculates the transformed vector by adding the reflection vector 
 * to the reflection point:
 *
 * \f[
 * t = p + r
 * \f]
 *
 * where:
 * - \f$p\f$ is the reflection point.
 * - \f$r\f$ is the reflection vector.
 * - \f$t\f$ is the resulting transformed vector.
 *
 * @param[in] reflection_point The base point from which the transformation starts.
 * @param[in] reflection_vector The reflection vector applied to the reflection point.
 * @param[out] transformed_vector The computed transformed vector.
 */
void getTransformedVector(
  const std::vector<double> & reflection_point, const std::vector<double> & reflection_vector,
  std::vector<double> & transformed_vector);

/**
 * @brief Compute transformed distance vectors from reflection points and transformation vectors.
 *
 * This function calculates the transformed distance vectors by applying the transformation 
 * vectors to their corresponding reflection points. The transformation is performed as:
 *
 * \f[
 * t_i = p_i + v_i
 * \f]
 *
 * where:
 * - \f$p_i\f$ is the \f$i^{th}\f$ reflection point.
 * - \f$v_i\f$ is the corresponding transformation vector.
 * - \f$t_i\f$ is the resulting transformed vector.
 *
 * If a reflection point has zero magnitude, the transformed vector is set to \f$\{0,0,0\}\f$.
 *
 * @param[in] reflection_points A vector of reflection points.
 * @param[in] reflection_points_to_tf_data_vectors A vector of transformation vectors to be applied to the reflection points.
 * @param[out] transformed_distance_vectors A pointer to the vector that will store the computed transformed vectors.
 */
void getTransformedVectors(
  const std::vector<std::vector<double>> & reflection_points,
  const std::vector<std::vector<double>> & reflection_points_to_tf_data_vectors,
  std::vector<std::vector<double>> * transformed_distance_vectors);

/**
 * @brief Compute the vector from a reflection point to a transformed data point.
 *
 * This function calculates the vector leading to a transformed data point by scaling 
 * the unit reflection vector based on the difference in magnitudes between the 
 * distance vector and the reflection vector:
 *
 * \f[
 * v = (m_d - m_r) \cdot u
 * \f]
 *
 * where:
 * - \f$u\f$ is the unit reflection vector.
 * - \f$m_r\f$ is the magnitude of the reflection vector.
 * - \f$m_d\f$ is the magnitude of the distance vector.
 * - \f$v\f$ is the resulting vector leading to the transformed data point.
 *
 * If \f$m_d = m_r\f$, an error is logged, and \f$(m_d - m_r)\f$ is set to 1 to prevent division by zero.
 *
 * @param[in] unit_reflection_vec A unit vector representing the reflection direction.
 * @param[in] mag_reflection_vec The magnitude of the reflection vector.
 * @param[in] mag_distance_vec The magnitude of the distance vector.
 * @return std::vector<double> The computed vector leading to the transformed data point.
 */
std::vector<double> _getVectorToTransformedDataPoint(
  const std::vector<double> & unit_reflection_vec, double mag_reflection_vec,
  double mag_distance_vec);


/**
 * @brief Compute vectors leading to transformed data points from reflection points.
 *
 * This function calculates a set of vectors that map reflection points to transformed 
 * data points using the difference in magnitudes between distance and reflection vectors. 
 * The transformation follows the formula:
 *
 * \f[
 * v_i = (m_{d_i} - m_{r_i}) \cdot u_i
 * \f]
 *
 * where:
 * - \f$u_i\f$ is the \f$i^{th}\f$ unit reflection vector.
 * - \f$m_{r_i}\f$ is the magnitude of the \f$i^{th}\f$ reflection point.
 * - \f$m_{d_i}\f$ is the magnitude of the \f$i^{th}\f$ distance vector.
 * - \f$v_i\f$ is the resulting transformed vector.
 *
 * If \f$m_{d_i} = 0\f$, the function assigns a zero vector \f$\{0,0,0\}\f$ to avoid errors.
 * If input vector sizes do not match, an error is logged.
 *
 * @param[in] unit_reflection_vectors A vector of unit reflection vectors.
 * @param[in] magnitudes_reflection_points A vector of magnitudes for the reflection points.
 * @param[in] magnitudes_distance_vectors A vector of magnitudes for the distance vectors.
 * @param[out] reflection_points_to_tf_data_points A pointer to the output vector storing computed transformed vectors.
 */
void getVectorsToTransformedDataPoints(
  const std::vector<std::vector<double>> & unit_reflection_vectors,
  const std::vector<double> & magnitudes_reflection_points,
  const std::vector<double> & magnitudes_distance_vectors,
  std::vector<std::vector<double>> * reflection_points_to_tf_data_points);

/**
 * @brief Convert a set of distance vectors into a point cloud representation.
 *
 * This function transforms a collection of 3D distance vectors into a 
 * PCL point cloud by mapping each vector \f$(x, y, z)\f$ into a `pcl::PointXYZI` object.
 *
 * @param[in] distance_vectors A vector of 3D distance vectors, where each element is \f$(x, y, z)\f$.
 * @param[out] target_cloud A pointer to the output PCL point cloud containing the transformed points.
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
 * @brief Extract mirror reflection points from a buffered point cloud sequence.
 *
 * This function retrieves specific reflection points from a sequence of 
 * point cloud frames, using the provided indices to extract the relevant 
 * points from each frame.
 *
 * If the sizes of `pointcloud_buffer` and `indices` do not match, an error is logged.
 *
 * @param[in] pointcloud_buffer A vector of point clouds representing buffered frames.
 * @param[in] indices A vector of indices specifying which points to extract from each frame.
 * @param[out] reflection_points A vector to store the extracted reflection points.
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
 * @brief Extract indices of points within a specified angular range from a point cloud.
 *
 * This function filters the points in a point cloud based on their angular position 
 * and extracts the indices of the points that lie within the specified angular range. 
 * The angular range is defined by `start_angle` and `end_angle`. 
 * The function uses the `idxToAngle` method to convert point indices to angles.
 *
 * @param[in] pcl_msg A point cloud message (`pcl::PointCloud<pcl::PointXYZI>`) containing the point cloud data.
 * @param[in] start_angle The starting angle (in radians) for the range of interest.
 * @param[in] end_angle The ending angle (in radians) for the range of interest.
 * @param[out] indices A pointer to a `pcl::PointIndices` object where the filtered point indices will be stored.
 */
void getPointcloudIndices(
  const pcl::PointCloud<pcl::PointXYZI> & pcl_msg, double start_angle, double end_angle,
  pcl::PointIndices * indices);


/**
 * @brief Calculate the Roll, Pitch, and Yaw (RPY) angles from the given normal vector.
 *
 * This function computes the Roll, Pitch, and Yaw angles based on the provided 3D normal vector. 
 * The normal vector is first normalized, then the Pitch and Yaw angles are calculated using standard 
 * trigonometric relations. Roll is set to zero, as it is not relevant for the computation in this case.
 *
 * @param normal_vec A 3D normal vector from which the RPY angles will be calculated.
 * @return A vector containing the RPY angles: [roll, pitch, yaw], where:
 * - roll is set to 0 (as it is not calculated).
 * - pitch is computed from the normal vector's magnitude in the x-y plane and z-component.
 * - yaw is the angle of the projection of the normal vector onto the x-y plane.
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
 * @brief Translate an angular value into its corresponding array index.
 *
 * This function maps an angle in a given angular range to its corresponding 
 * index in an array of angular data. The translation accounts for the angular 
 * interval and increment, as well as an optional target area angle range. 
 * This is useful for converting angularly spaced data into array-indexed data.
 *
 * @param[in] tgt_angle The target angle (in degrees) to be translated into an array index.
 * @param[in] angle_min The lower bound of the angle data interval (in degrees).
 * @param[in] angle_max The upper bound of the angle data interval (in degrees).
 * @param[in] tgt_area_angle_min The minimum angle (in degrees) for the target area to be mapped.
 * @param[in] tgt_area_angle_max The maximum angle (in degrees) for the target area to be mapped.
 * @param[in] angle_increment The angular increment (in degrees) between consecutive data points (default: 1 degree).
 * @return int The array index corresponding to the input angle.
 */
int angle2ArrIdx(
  double tgt_angle, double angle_min, double angle_max, double tgt_area_angle_min,
  double tgt_area_angle_max, double angle_increment = 1);


};  // namespace lidarMirrorFOVReshaperTF

#endif  // LIDAR_MIRROR_FOV_RESHAPER_TF_HPP