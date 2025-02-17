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

#ifndef LIDARMIRRORFOVRESHAPERRUNTIME_HPP
#define LIDARMIRRORFOVRESHAPERRUNTIME_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <laser_geometry/laser_geometry.hpp>
#include <limits>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vector>

class LidarMirrorFOVReshaperRuntime : public rclcpp::Node
{
public:
  LidarMirrorFOVReshaperRuntime();
  ~LidarMirrorFOVReshaperRuntime() = default;

private:
  /**
     * @brief Callback function for incoming pointcloud messages
     *
     * @param[in] msg Incoming Pointcloud message
     */
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  /**
     * @brief Split the incoming combined raw pointcloud into three separate pointclouds, one per mirror and one for the unmirrored 
     * front area. Splitting is based on prior defined FOV angles.
     *
     * @param[in] src_cloud raw pointcloud, composed of the left mirror, right mirror and front area. W/o any transformations applied.
     * @param[out] left_mirror_cloud croppped FOV corresponding to the left mirror area
     * @param[out] right_mirror_cloud cropped FOV corresponding to the right mirror area
     * @param[out] front_cloud cropped FOV corresponding to the unmirrored front area
     * @param[out] all_cloud 
     * 
     */
  void splitPointcloud(
    const pcl::PointCloud<pcl::PointXYZI> & src_cloud,
    pcl::PointCloud<pcl::PointXYZI> * left_mirror_cloud,
    pcl::PointCloud<pcl::PointXYZI> * right_mirror_cloud,
    pcl::PointCloud<pcl::PointXYZI> * front_cloud, pcl::PointIndices * indices_left_mirror,
    pcl::PointIndices * indices_right_mirror, pcl::PointIndices * indices_front,
    pcl::PointIndices * indices_zero_to_startrm, pcl::PointIndices * indices_endrm_to_startfront,
    pcl::PointIndices * indices_endfront_to_startlm, pcl::PointIndices * indices_endlm_to_end);

  /**
     * @brief publish arbitrary pcl based pointcloud as PointCloud2 message
     *
     * @param[in] cloud pointcloud to be published
     * @param[in] publisher publisher object
     * @param[in] header header of the pointcloud to be published
     */
  void publishPointCloud(
    const pcl::PointCloud<pcl::PointXYZI> & cloud,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher,
    const std_msgs::msg::Header & header);

  /**
   * @brief laserscan callback function
   * 
   * @param msg incoming laserscan message
   */
  void laserScanCallback(const sensor_msgs::msg::LaserScan & msg);

  /**
   * @brief Transform laser scan to pointcloud
   * 
   * @param[in] scan_msg laser scan to be transformed
   * @param[out] cloud_msg target pointcloud 
   */
  void laserScanToPCL(
    const sensor_msgs::msg::LaserScan & scan_msg, sensor_msgs::msg::PointCloud2 & cloud_msg);

  /**
   * @brief Set pointcloud points to NaN At Indices specified by the indices object
   * 
   * @param[in out] cloud pointcloud to be modified
   * @param[in] indices indices object specifying the indices to be set to NaN
   */
  void setNaNAtIndices(pcl::PointCloud<pcl::PointXYZI> * cloud, pcl::PointIndices * indices);

  // Subscriber
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_transformed_all_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_left_transformed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_right_transformed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_transformed_all_pub_2d;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_front_pub_;

  // Tf utils
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  laser_geometry::LaserProjection projector_;

  // config parameters
  std::string laser_topic_;
  std::string laser_frame_;
  double laser_angle_min_;
  double laser_angle_max_;

  bool viz_pub_transformed_all_combined;
  bool viz_pub_front;
  bool viz_pub_transformed_leftmirror;
  bool viz_pub_transformed_rightmirror;

  double front_start_angle_;
  double front_end_angle_;

  double mirror_left_dead_zone_;
  double mirror_left_start_angle_;
  double mirror_left_end_angle_;
  std::vector<double> mirror_left_normal_vec_;
  std::vector<double> mirror_left_support_vec_;

  double mirror_right_dead_zone_;
  double mirror_right_start_angle_;
  double mirror_right_end_angle_;
  std::vector<double> mirror_right_normal_vec_;
  std::vector<double> mirror_right_support_vec_;
};
#endif  // LIDARMIRRORFOVRESHAPERRUNTIME_HPP
