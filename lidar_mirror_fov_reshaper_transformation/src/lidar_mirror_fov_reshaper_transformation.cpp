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

#include "lidar_mirror_fov_reshaper_transformation.hpp"

double lidarMirrorFOVReshaperTF::deg2rad(const double & deg) { return ((deg * M_PI) / 180); };

std::vector<double> lidarMirrorFOVReshaperTF::getReflectionPoint(
  const std::vector<double> & normal_vec, const std::vector<double> & support_vec,
  const std::vector<double> & distance_vec_raw)
{
  std::vector<double> reflection_point(normal_vec.size());

  // n x w
  double normal_support_dp =
    std::inner_product(normal_vec.begin(), normal_vec.end(), support_vec.begin(), 0.0);

  // n x d
  double normal_distance_dp =
    std::inner_product(normal_vec.begin(), normal_vec.end(), distance_vec_raw.begin(), 0.0);

  if (normal_distance_dp == 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("LMFR_TF"),
      "Normal vector and support vector are orthogonal, bad estimate! Calculation Terminated! "
      "(Cross Product of normal vector and support vector is %f\n normal_vec: "
      "[%f,%f,%f]\nsupport_vec:[%f,%f,%f])",
      normal_distance_dp, normal_vec[0], normal_vec[1], normal_vec[2], support_vec[0],
      support_vec[1], support_vec[2]);
    return std::vector<double>{0, 0, 0};
  }

  // dp_ratio = n * w / n * d
  double dp_ratio = normal_support_dp / normal_distance_dp;

  for (auto & coord : distance_vec_raw) {
    reflection_point.push_back(coord * dp_ratio);
  };

  return reflection_point;
};

std::vector<double> lidarMirrorFOVReshaperTF::getNormalizedVector(
  const std::vector<double> & vec_src)
{
  std::vector<double> normalized_vec;
  double vector_mag =
    std::sqrt(std::inner_product(vec_src.begin(), vec_src.end(), vec_src.begin(), 0.0));

  for (auto & i : vec_src) {
    normalized_vec.push_back(i / vector_mag);
  };

  return normalized_vec;
};

void lidarMirrorFOVReshaperTF::normalizePoints(
  const pcl::PointCloud<pcl::PointXYZI> & src_cloud,
  std::vector<std::vector<double>> * normalized_points)
{
  for (auto & point : src_cloud.points) {
    std::vector<double> distance_vec_raw = {point.x, point.y, point.z};
    std::vector<double> normalized_distance_vec =
      lidarMirrorFOVReshaperTF::getNormalizedVector(distance_vec_raw);
    normalized_points->push_back(normalized_distance_vec);
  };
};

void lidarMirrorFOVReshaperTF::getUnitReflectionVector(
  const std::vector<double> & normalized_distance_vec,
  const std::vector<double> & normalized_mirror_normal_vec,
  std::vector<double> & unit_reflection_vec)
{
  // normalized_distance_vec * normalized_mirror_normal_vec
  double dist_vec_dot_normal_vec = std::inner_product(
    normalized_distance_vec.begin(), normalized_distance_vec.end(),
    normalized_mirror_normal_vec.begin(), 0.0);

  // 2 * (normalized_distance_vec * normalized_mirror_normal_vec)
  double scaled_dist_vec_dot_normal_vec = 2 * dist_vec_dot_normal_vec;

  // 2 * (normalized_distance_vec * normalized_mirror_normal_vec) * normalized_mirror_normal_vec
  std::vector<double> factor_vec;
  for (size_t i = 0; i < normalized_mirror_normal_vec.size(); i++) {
    // factor_vec.push_back(scaled_dist_vec_dot_normal_vec * normalized_mirror_normal_vec[i]);
    factor_vec.push_back(scaled_dist_vec_dot_normal_vec * normalized_mirror_normal_vec[i]);
  }
  // normalized_distance_vec - 2 * (normalized_distance_vec * normalized_mirror_normal_vec) * normalized_mirror_normal_vec
  for (size_t i = 0; i < normalized_distance_vec.size(); i++) {
    unit_reflection_vec.push_back(normalized_distance_vec[i] - factor_vec[i]);
  }
};

void lidarMirrorFOVReshaperTF::getUnitReflectionVectors(
  const std::vector<std::vector<double>> & normalized_distance_vectors,
  const std::vector<double> & normalized_mirror_normal_vec,
  std::vector<std::vector<double>> * unit_reflection_vectors)
{
  for (auto & normalized_distance_vec : normalized_distance_vectors) {
    std::vector<double> reflection_vec;
    lidarMirrorFOVReshaperTF::getUnitReflectionVector(
      normalized_distance_vec, normalized_mirror_normal_vec, reflection_vec);

    unit_reflection_vectors->push_back(reflection_vec);
  }
};

void lidarMirrorFOVReshaperTF::getPointMagnitude(
  const pcl::PointCloud<pcl::PointXYZI> & src_cloud, std::vector<double> * point_magnitude)
{
  for (auto & point : src_cloud.points) {
    std::vector<double> distance_vec_tmp = {point.x, point.y, point.z};
    double mag_dist_vec = lidarMirrorFOVReshaperTF::getVectorMagnitude(distance_vec_tmp);
    point_magnitude->push_back(mag_dist_vec);
  };
};

void lidarMirrorFOVReshaperTF::getPointMagnitude(
  const std::vector<std::vector<double>> & src_vec, std::vector<double> * point_magnitude)
{
  for (auto & point_vec : src_vec) {
    if (std::accumulate(point_vec.begin(), point_vec.end(), 0.0) == 0) {
      point_magnitude->push_back(0);
      continue;
    };
    double mag_dist_vec = lidarMirrorFOVReshaperTF::getVectorMagnitude(point_vec);
    point_magnitude->push_back(mag_dist_vec);
  };
};

double lidarMirrorFOVReshaperTF::getVectorMagnitude(const std::vector<double> & src_vec)
{
  return std::sqrt(std::inner_product(src_vec.begin(), src_vec.end(), src_vec.begin(), 0.0));
};

void lidarMirrorFOVReshaperTF::getTransformedVector(
  const std::vector<double> & reflection_point, const std::vector<double> & reflection_vector,
  std::vector<double> & transformed_vector)
{
  for (size_t i = 0; i < reflection_point.size(); i++) {
    transformed_vector.push_back(reflection_point[i] + reflection_vector[i]);
  }
};

void lidarMirrorFOVReshaperTF::getTransformedVectors(
  const std::vector<std::vector<double>> & reflection_points,                     // I
  const std::vector<std::vector<double>> & reflection_points_to_tf_data_vectors,  // v
  std::vector<std::vector<double>> * transformed_distance_vectors)                // d'
{
  if (reflection_points.size() != reflection_points_to_tf_data_vectors.size()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("LMFR_TF"),
      "Error: input vectors are not of same size, size of reflection_points: %ld, size of "
      "reflection_points_to_tf_data_vectors: %ld",
      reflection_points.size(), reflection_points_to_tf_data_vectors.size());
    return;
  };

  for (size_t i = 0; i < reflection_points.size(); i++) {
    if (lidarMirrorFOVReshaperTF::getVectorMagnitude(reflection_points[i]) == 0) {
      transformed_distance_vectors->push_back({0, 0, 0});
      continue;
    }
    std::vector<double> transformed_vector_tmp;
    lidarMirrorFOVReshaperTF::getTransformedVector(
      reflection_points[i], reflection_points_to_tf_data_vectors[i], transformed_vector_tmp);

    transformed_distance_vectors->push_back(transformed_vector_tmp);
  };
};

std::vector<double> lidarMirrorFOVReshaperTF::getVectorToTransformedDataPoint(
  const std::vector<double> & unit_reflection_vec, double mag_reflection_vec,
  double mag_distance_vec)
{
  std::vector<double> reflection_point_to_tf_data_point;

  double diff_mag = mag_distance_vec - mag_reflection_vec;
  for (size_t i = 0; i < unit_reflection_vec.size(); i++) {  // r
    if (diff_mag == 0) {
      RCLCPP_ERROR(
        rclcpp::get_logger("LMFR_TF"),
        "Error: magnitude of reflection vector and distance vector are equal, bad estimate! "
        "Optimization Terminated!");
      diff_mag = 1;
    }
    double res = diff_mag * unit_reflection_vec[i];
    reflection_point_to_tf_data_point.push_back(res);
  }
  return reflection_point_to_tf_data_point;
};

void lidarMirrorFOVReshaperTF::getVectorsToTransformedDataPoints(
  const std::vector<std::vector<double>> & unit_reflection_vectors,
  const std::vector<double> & magnitudes_reflection_points,
  const std::vector<double> & magnitudes_distance_vectors,
  std::vector<std::vector<double>> * reflection_points_to_tf_data_points)

{
  if (
    unit_reflection_vectors.size() != magnitudes_reflection_points.size() ||
    unit_reflection_vectors.size() != magnitudes_distance_vectors.size()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("LMFR_TF"),
      "Error: input vectors are not of same size\n \
        size of unit_reflection_vectors: %ld\n \
        size of magnitudes_reflection_points: %ld\n \
        size of magnitudes_distance_vectors: %ld\n",
      unit_reflection_vectors.size(), magnitudes_reflection_points.size(),
      magnitudes_distance_vectors.size());
    return;
  };

  for (size_t i = 0; i < unit_reflection_vectors.size(); i++) {
    if (magnitudes_distance_vectors[i] == 0) {
      reflection_points_to_tf_data_points->push_back({0, 0, 0});
      continue;
    }
    std::vector<double> reflection_point_to_tf_data_point =
      lidarMirrorFOVReshaperTF::getVectorToTransformedDataPoint(
        unit_reflection_vectors[i], magnitudes_reflection_points[i],
        magnitudes_distance_vectors[i]);
    reflection_points_to_tf_data_points->push_back(reflection_point_to_tf_data_point);
  };
};

void lidarMirrorFOVReshaperTF::transformVectorIntoPointCloud(
  const std::vector<std::vector<double>> & distance_vectors,
  pcl::PointCloud<pcl::PointXYZI> * target_cloud)
{
  for (auto & point_vec : distance_vectors) {
    pcl::PointXYZI point;
    point.x = point_vec[0];
    point.y = point_vec[1];
    point.z = point_vec[2];
    target_cloud->push_back(point);
  };
};

void lidarMirrorFOVReshaperTF::calcReflectionPoints(
  const pcl::PointCloud<pcl::PointXYZI> & src_cloud, const std::vector<double> & normal_vec,
  const std::vector<double> & support_vec, std::vector<std::vector<double>> * reflection_points)
{
  for (auto & point : src_cloud.points) {
    std::vector<double> distance_vec_raw{point.x, point.y, point.z};
    if (lidarMirrorFOVReshaperTF::getVectorMagnitude(distance_vec_raw) == 0.0) {
      RCLCPP_WARN(
        rclcpp::get_logger("LMFR_TF"),
        "Raw-Input Point is equal to origin, no reflection point calculated!\nPotential fix: "
        "Re-define your mirror-FOV(s)");
      reflection_points->push_back(distance_vec_raw);
      continue;
    };

    std::vector<double> reflection_point =
      lidarMirrorFOVReshaperTF::getReflectionPoint(normal_vec, support_vec, distance_vec_raw);
    reflection_points->push_back(reflection_point);
  };
};

void lidarMirrorFOVReshaperTF::calcMirrorPlaneNormal(
  const std::vector<double> & helper_p1, const std::vector<double> & helper_p2,
  const std::vector<double> & helper_p3, std::vector<double> & normal_vec)
{
  if (helper_p1 == helper_p2 && helper_p2 == helper_p3) {
    RCLCPP_ERROR(
      rclcpp::get_logger("LMFR_TF"),
      "Error: All Helper Points are the same, bad estimate! Calculation of normal vector "
      "Terminated!");
    RCLCPP_ERROR(
      rclcpp::get_logger("LMFR_TF"),
      "[helper_p1]: [%f, %f, %f]\n[helper_p2]: [%f, %f, %f]\n[helper_p3]: [%f, %f, %f]",
      helper_p1[0], helper_p1[1], helper_p1[2], helper_p2[0], helper_p2[1], helper_p2[2],
      helper_p3[0], helper_p3[1], helper_p3[2]);
    normal_vec = {NAN, NAN, NAN};
    return;
  }

  normal_vec.clear();

  normal_vec.resize(3, 0.0);

  std::vector<double> u_vec = {
    helper_p2[0] - helper_p1[0], helper_p2[1] - helper_p1[1], helper_p2[2] - helper_p1[2]};

  std::vector<double> v_vec = {
    helper_p3[0] - helper_p1[0], helper_p3[1] - helper_p1[1], helper_p3[2] - helper_p1[2]};

  normal_vec[0] = u_vec[1] * v_vec[2] - u_vec[2] * v_vec[1];
  normal_vec[1] = u_vec[2] * v_vec[0] - u_vec[0] * v_vec[2];
  normal_vec[2] = u_vec[0] * v_vec[1] - u_vec[1] * v_vec[0];

  // Check if collinear
  double cross_magnitude = std::sqrt(
    normal_vec[0] * normal_vec[0] + normal_vec[1] * normal_vec[1] + normal_vec[2] * normal_vec[2]);

  if (cross_magnitude < 1e-9)  // Manually set threshold
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("LMFR_TF"),
      "Error: Points are collinear, bad estimate! Calculation Terminated!");
    normal_vec = {NAN, NAN, NAN};
    return;
  }

  double normal_vec_mag = std::sqrt(
    normal_vec[0] * normal_vec[0] + normal_vec[1] * normal_vec[1] + normal_vec[2] * normal_vec[2]);

  normal_vec[0] /= normal_vec_mag;
  normal_vec[1] /= normal_vec_mag;
  normal_vec[2] /= normal_vec_mag;

  // ensure normal vec is in either quadrant 1 or 2 (x-y-plane)
  // valid orientation representations could also be in quadrant 3 or 4 (x-y-plane),
  // just a preference, since the original setting includes mirrors mirroring towards
  // the 1st and 2nd quadrant of the x-y-plane
  if (normal_vec[0] < 0) {
    normal_vec[0] *= -1;
    normal_vec[1] *= -1;
    normal_vec[2] *= -1;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("LMFR_TF"), "Calculated normal vector: (%f, %f, %f)", normal_vec[0],
    normal_vec[1], normal_vec[2]);
};

void lidarMirrorFOVReshaperTF::calcPointPlaneDist(
  const pcl::PointCloud<pcl::PointXYZI> & src_cloud, const std::vector<double> & plane_support_vec,
  const std::vector<double> & plane_normal_vec, std::vector<double> * point_plane_distances)
{
  std::vector<double> plane_normal_vec_normalized =
    lidarMirrorFOVReshaperTF::getNormalizedVector(plane_normal_vec);

  for (const pcl::PointXYZI & point : src_cloud.points) {
    std::vector<double> point_vec = {point.x, point.y, point.z};
    std::vector<double> point_vec_minus_plane_support_vec(3);

    std::transform(
      point_vec.begin(), point_vec.end(), plane_support_vec.begin(),
      point_vec_minus_plane_support_vec.begin(), std::minus<double>());

    double point_plane_dist = std::abs(std::inner_product(
      plane_normal_vec_normalized.begin(), plane_normal_vec_normalized.end(),
      point_vec_minus_plane_support_vec.begin(), 0.0));
    // double point_plane_dist = std::abs(std::inner_product(
    // point_vec_minus_plane_support_vec.begin(), point_vec_minus_plane_support_vec.end(),
    // plane_normal_vec_normalized->begin(), 0.0));

    point_plane_distances->push_back(point_plane_dist);
  }
};

void lidarMirrorFOVReshaperTF::unwrapVector(
  const std::vector<double> & src_vec, std::vector<double> & plane_support_vec,
  std::vector<double> & plane_normal_vec, std::vector<double> & mirror_right_support_vec,
  std::vector<double> & mirror_right_normal_vec, std::vector<double> & mirror_left_support_vec,
  std::vector<double> & mirror_left_normal_vec)
{
  if (src_vec.size() != 18) {
    RCLCPP_ERROR(
      rclcpp::get_logger("LMFR_TF"),
      "Error: input vector is not of size 18, size of input vector: %ld", src_vec.size());
    return;
  };

  plane_support_vec = {src_vec[0], src_vec[1], src_vec[2]};
  plane_normal_vec = {src_vec[3], src_vec[4], src_vec[5]};
  mirror_right_support_vec = {src_vec[6], src_vec[7], src_vec[8]};
  mirror_right_normal_vec = {src_vec[9], src_vec[10], src_vec[11]};
  mirror_left_support_vec = {src_vec[12], src_vec[13], src_vec[14]};
  mirror_left_normal_vec = {src_vec[15], src_vec[16], src_vec[17]};
};

void lidarMirrorFOVReshaperTF::getMirrorReflectionPoints(
  const std::vector<pcl::PointCloud<pcl::PointXYZI>> & pointcloud_buffer,
  std::vector<int> & indices, std::vector<pcl::PointXYZI> & reflection_points)
{
  if (pointcloud_buffer.size() != indices.size()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("LMFR_TF"),
      "Error: input vectors are not of same size, size of pointcloud_buffer: %ld, size of "
      "indices: %ld",
      pointcloud_buffer.size(), indices.size());
    return;
  };

  for (size_t i = 0; i < pointcloud_buffer.size(); i++) {
    reflection_points.push_back(pointcloud_buffer[i].at(indices[i]));
  }
};

double lidarMirrorFOVReshaperTF::getStdDev(const std::vector<double> & vec)
{
  double mean = std::accumulate(vec.begin(), vec.end(), 0.0) / vec.size();
  double variance = 0.0;
  for (auto & dist : vec) {
    variance += std::pow(dist - mean, 2);
  };
  variance /= (vec.size());
  double stdev = std::sqrt(variance);

  return stdev;
};

void lidarMirrorFOVReshaperTF::transformCloud(
  const pcl::PointCloud<pcl::PointXYZI> & src_cloud_front,
  const pcl::PointCloud<pcl::PointXYZI> & src_cloud_left_mirror,
  const pcl::PointCloud<pcl::PointXYZI> & src_cloud_right_mirror,
  const std::vector<double> & mirror_left_normal_vec,
  const std::vector<double> & mirror_left_support_vec,
  const std::vector<double> & mirror_right_normal_vec,
  const std::vector<double> & mirror_right_support_vec,
  pcl::PointCloud<pcl::PointXYZI> * transformed_cloud,
  pcl::PointCloud<pcl::PointXYZI> * transformed_cloud_left_mirror,
  pcl::PointCloud<pcl::PointXYZI> * transformed_cloud_right_mirror)
{
  std::vector<double> intensities, intensities_left_mirror, intensities_right_mirror;
  intensities.reserve(
    src_cloud_front.size() + src_cloud_left_mirror.size() + src_cloud_right_mirror.size());
  intensities_left_mirror.reserve(src_cloud_left_mirror.size());
  intensities_right_mirror.reserve(src_cloud_right_mirror.size());

  for (const auto & point : src_cloud_right_mirror.points)
    intensities_right_mirror.push_back(point.intensity);

  intensities.insert(
    intensities.end(), intensities_right_mirror.begin(), intensities_right_mirror.end());

  for (const auto & point : src_cloud_front.points) intensities.push_back(point.intensity);

  for (const auto & point : src_cloud_left_mirror.points)
    intensities_left_mirror.push_back(point.intensity);

  intensities.insert(
    intensities.end(), intensities_left_mirror.begin(), intensities_left_mirror.end());

  // eq. 1) (see README - Publications)
  std::vector<std::vector<double>> * reflection_points_left_mirror =
    new std::vector<std::vector<double>>;
  std::vector<std::vector<double>> * reflection_points_right_mirror =
    new std::vector<std::vector<double>>;

  lidarMirrorFOVReshaperTF::calcReflectionPoints(
    src_cloud_left_mirror, mirror_left_normal_vec, mirror_left_support_vec,
    reflection_points_left_mirror);

  lidarMirrorFOVReshaperTF::calcReflectionPoints(
    src_cloud_right_mirror, mirror_right_normal_vec, mirror_right_support_vec,
    reflection_points_right_mirror);

  // eq. 2) (see README - Publications)
  // calc unit vector r of reflection ray
  // each vector contains n 3-dim vectors (x, y, z)
  std::vector<std::vector<double>> * normalized_distance_vectors_left_mirror =
    new std::vector<std::vector<double>>;
  std::vector<std::vector<double>> * normalized_distance_vectors_right_mirror =
    new std::vector<std::vector<double>>;

  lidarMirrorFOVReshaperTF::normalizePoints(
    src_cloud_left_mirror, normalized_distance_vectors_left_mirror);
  lidarMirrorFOVReshaperTF::normalizePoints(
    src_cloud_right_mirror, normalized_distance_vectors_right_mirror);

  // sanity check
  if (
    normalized_distance_vectors_left_mirror->size() != src_cloud_left_mirror.size() ||
    normalized_distance_vectors_right_mirror->size() != src_cloud_right_mirror.size()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("LMFR_TF"),
      "Error: normalized distance vectors size does not match pointcloud size\n"
      "normalized_distance_vectors_left_mirror size: %ld, src_cloud_left_mirror size: %ld\n"
      "normalized_distance_vectors_right_mirror size: %ld, src_cloud_right_mirror size: %ld",
      normalized_distance_vectors_left_mirror->size(), src_cloud_left_mirror.size(),
      normalized_distance_vectors_right_mirror->size(), src_cloud_right_mirror.size());
    return;
  };

  // get normalized vector of mirror normal vector
  std::vector<double> normalized_mirror_left_normal_vec =
    lidarMirrorFOVReshaperTF::getNormalizedVector(mirror_left_normal_vec);
  std::vector<double> normalized_mirror_right_normal_vec =
    lidarMirrorFOVReshaperTF::getNormalizedVector(mirror_right_normal_vec);

  // r = normalized_distance_vec - 2 * (normalized_distance_vec * normalized_mirror_normal_vec) * normalized_mirror_normal_vec
  std::vector<std::vector<double>> * unit_reflection_vectors_left_mirror =
    new std::vector<std::vector<double>>;
  std::vector<std::vector<double>> * unit_reflection_vectors_right_mirror =
    new std::vector<std::vector<double>>;

  lidarMirrorFOVReshaperTF::getUnitReflectionVectors(
    *normalized_distance_vectors_left_mirror, normalized_mirror_left_normal_vec,
    unit_reflection_vectors_left_mirror);

  lidarMirrorFOVReshaperTF::getUnitReflectionVectors(
    *normalized_distance_vectors_right_mirror, normalized_mirror_right_normal_vec,
    unit_reflection_vectors_right_mirror);

  // calc magnitude of distance vector (each point in pointcloud) and reflection point
  std::vector<double> * mag_distance_vectors_left_mirror = new std::vector<double>;
  std::vector<double> * mag_distance_vectors_right_mirror = new std::vector<double>;

  lidarMirrorFOVReshaperTF::getPointMagnitude(
    src_cloud_left_mirror, mag_distance_vectors_left_mirror);
  lidarMirrorFOVReshaperTF::getPointMagnitude(
    src_cloud_right_mirror, mag_distance_vectors_right_mirror);

  std::vector<double> * mag_reflection_points_left_mirror = new std::vector<double>;  // I
  std::vector<double> * mag_reflection_points_right_mirror = new std::vector<double>;

  lidarMirrorFOVReshaperTF::getPointMagnitude(
    *reflection_points_left_mirror, mag_reflection_points_left_mirror);
  lidarMirrorFOVReshaperTF::getPointMagnitude(
    *reflection_points_right_mirror, mag_reflection_points_right_mirror);

  // eq. 3) (see README - Publications)
  // calc vector v of reflection point to transformed data point
  std::vector<std::vector<double>> * reflection_point_to_tf_data_vectors_left_mirror =
    new std::vector<std::vector<double>>;
  std::vector<std::vector<double>> * reflection_point_to_tf_data_vectors_right_mirror =
    new std::vector<std::vector<double>>;

  lidarMirrorFOVReshaperTF::getVectorsToTransformedDataPoints(
    *unit_reflection_vectors_left_mirror, *mag_reflection_points_left_mirror,
    *mag_distance_vectors_left_mirror, reflection_point_to_tf_data_vectors_left_mirror);

  lidarMirrorFOVReshaperTF::getVectorsToTransformedDataPoints(
    *unit_reflection_vectors_right_mirror, *mag_reflection_points_right_mirror,
    *mag_distance_vectors_right_mirror, reflection_point_to_tf_data_vectors_right_mirror);

  // eq. 4) (see README - Publications)
  // calc vector d' (transformed vector d)
  // d' = reflection_point + reflection_vector
  std::vector<std::vector<double>> * transformed_distance_vectors_left_mirror =
    new std::vector<std::vector<double>>;
  std::vector<std::vector<double>> * transformed_distance_vectors_right_mirror =
    new std::vector<std::vector<double>>;

  lidarMirrorFOVReshaperTF::getTransformedVectors(
    *reflection_points_left_mirror, *reflection_point_to_tf_data_vectors_left_mirror,
    transformed_distance_vectors_left_mirror);

  lidarMirrorFOVReshaperTF::getTransformedVectors(
    *reflection_points_right_mirror, *reflection_point_to_tf_data_vectors_right_mirror,
    transformed_distance_vectors_right_mirror);

  // transform distance vectors back into pointcloud
  lidarMirrorFOVReshaperTF::transformVectorIntoPointCloud(
    *transformed_distance_vectors_left_mirror, transformed_cloud_left_mirror);
  lidarMirrorFOVReshaperTF::transformVectorIntoPointCloud(
    *transformed_distance_vectors_right_mirror, transformed_cloud_right_mirror);

  transformed_cloud->insert(
    transformed_cloud->begin(), transformed_cloud_right_mirror->begin(),
    transformed_cloud_right_mirror->end());
  transformed_cloud->insert(
    transformed_cloud->end(), src_cloud_front.begin(), src_cloud_front.end());
  transformed_cloud->insert(
    transformed_cloud->end(), transformed_cloud_left_mirror->begin(),
    transformed_cloud_left_mirror->end());

  for (size_t i = 0; i < intensities.size(); i++) {
    transformed_cloud->points[i].intensity = intensities[i];
  };

  for (size_t i = 0; i < transformed_cloud_left_mirror->size(); i++) {
    transformed_cloud_left_mirror->points[i].intensity = intensities_left_mirror[i];
  };

  for (size_t i = 0; i < transformed_cloud_right_mirror->size(); i++) {
    transformed_cloud_right_mirror->points[i].intensity = intensities_right_mirror[i];
  };

  // cleanup
  delete reflection_points_left_mirror;
  delete reflection_points_right_mirror;
  delete normalized_distance_vectors_left_mirror;
  delete normalized_distance_vectors_right_mirror;
  delete unit_reflection_vectors_left_mirror;
  delete unit_reflection_vectors_right_mirror;
  delete mag_distance_vectors_left_mirror;
  delete mag_distance_vectors_right_mirror;
  delete mag_reflection_points_left_mirror;
  delete mag_reflection_points_right_mirror;
  delete reflection_point_to_tf_data_vectors_left_mirror;
  delete reflection_point_to_tf_data_vectors_right_mirror;
  delete transformed_distance_vectors_left_mirror;
  delete transformed_distance_vectors_right_mirror;
};

void lidarMirrorFOVReshaperTF::getPointcloudIndices(
  const pcl::PointCloud<pcl::PointXYZI> & pcl_msg, double start_angle, double end_angle,
  pcl::PointIndices * indices)
{
  indices->header = pcl_msg.header;

  for (size_t i = 0; i < pcl_msg.points.size(); i++) {
    double angle = lidarMirrorFOVReshaperTF::idxToAngle(i, pcl_msg.points.size());
    angle = lidarMirrorFOVReshaperTF::deg2rad(angle);
    if (angle >= start_angle && angle < end_angle) indices->indices.push_back(i);
  };
};

double lidarMirrorFOVReshaperTF::idxToAngle(int idx, int size) { return idx - size / 2; };

std::vector<double> lidarMirrorFOVReshaperTF::calcRPY(std::vector<double> normal_vec)
{
  normal_vec = lidarMirrorFOVReshaperTF::getNormalizedVector(normal_vec);

  double roll = 0.0;  // we dont care about roll
  double pitch = atan2(sqrt(pow(normal_vec[0], 2) + pow(normal_vec[1], 2)), normal_vec[2]);
  pitch -= M_PI / 2;

  double yaw = atan2(normal_vec[1], normal_vec[0]);

  std::vector<double> rpy = {roll, pitch, yaw};

  return rpy;
};

int lidarMirrorFOVReshaperTF::angle2ArrIdx(
  double tgt_angle, double angle_min, double angle_max, double tgt_area_angle_min,
  double tgt_area_angle_max, double angle_increment /*= 1 */)
{
  if (angle_increment != 1) {
    RCLCPP_ERROR(
      rclcpp::get_logger("LMFR_TF"),
      "Error: angle_increment is not 1, this function is only implemented for angle_increment = 1");
    return -1;
  };

  if (tgt_angle < 0) tgt_angle -= angle_min;
  tgt_angle += tgt_area_angle_min;

  if (tgt_angle > tgt_area_angle_max || tgt_angle > angle_max) {
    RCLCPP_ERROR(
      rclcpp::get_logger("LMFR_TF"),
      "Error: target angle is out of bounds, tgt_angle: %f, tgt_area_angle_min: %f, "
      "tgt_area_angle_max: %f, angle_min: %f, angle_max: %f",
      tgt_angle, tgt_area_angle_min, tgt_area_angle_max, angle_min, angle_max);
    // return -1;
  };

  return tgt_angle;
};