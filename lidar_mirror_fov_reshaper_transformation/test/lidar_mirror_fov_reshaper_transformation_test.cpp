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

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

/*
 * Ground-Truth values calculated via wolfram-alpha
 */
class lidarMirrorFOVReshaperTFTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("lidar_mirror_fov_reshaper_tf_test");

    callback_received_ = false;
    sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "test_topic", 10, [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        callback_received_ = true;
        received_msg_ = *msg;
      });
    pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("test_topic", 10);
  };

  void TearDown() override { rclcpp::shutdown(); }

  void publishPointCloud(const sensor_msgs::msg::PointCloud2 & cloud) { pub_->publish(cloud); };

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  sensor_msgs::msg::PointCloud2 received_msg_;
  bool callback_received_;
};

TEST_F(lidarMirrorFOVReshaperTFTest, TestPointCloudCallback)
{
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl_cloud.width = 2;
  pcl_cloud.height = 1;
  pcl_cloud.points.resize(pcl_cloud.width * pcl_cloud.height);

  for (size_t i = 0; i < pcl_cloud.points.size(); ++i) {
    pcl_cloud.points[i].x = static_cast<float>(i);
    pcl_cloud.points[i].y = static_cast<float>(i * 2);
    pcl_cloud.points[i].z = static_cast<float>(i * 3);
  }

  sensor_msgs::msg::PointCloud2 ros_cloud;
  pcl::toROSMsg(pcl_cloud, ros_cloud);
  ros_cloud.header.frame_id = "test_frame";
  ros_cloud.header.stamp = rclcpp::Clock().now();

  publishPointCloud(ros_cloud);
  rclcpp::spin_some(node_);

  ASSERT_TRUE(callback_received_);
  ASSERT_EQ(received_msg_.header.frame_id, "test_frame");
  ASSERT_EQ(received_msg_.width, 2);
  ASSERT_EQ(received_msg_.height, 1);
};

TEST_F(lidarMirrorFOVReshaperTFTest, testGetIndicesFromAngles) {}

TEST_F(lidarMirrorFOVReshaperTFTest, testGetNormalizedVector_PositiveValues)
{
  std::vector<double> src_vec = {3.0, 4.0};
  std::vector<double> expected_vec = {0.6, 0.8};

  std::vector<double> res = lidarMirrorFOVReshaperTF::getNormalizedVector(src_vec);

  ASSERT_EQ(res.size(), expected_vec.size());
  for (size_t i = 0; i < res.size(); ++i) {
    ASSERT_NEAR(res[i], expected_vec[i], 1E-5);
  }
}

TEST_F(lidarMirrorFOVReshaperTFTest, testGetNormalizedVector_NegativeValues)
{
  std::vector<double> src_vec = {-3.0, -4.0};
  std::vector<double> expected_vec = {-0.6, -0.8};

  std::vector<double> res = lidarMirrorFOVReshaperTF::getNormalizedVector(src_vec);

  ASSERT_EQ(res.size(), expected_vec.size());
  for (size_t i = 0; i < res.size(); ++i) {
    ASSERT_NEAR(res[i], expected_vec[i], 1E-5);
  }
}

TEST_F(lidarMirrorFOVReshaperTFTest, testGetNormalizedVector_MixedValues)
{
  std::vector<double> src_vec = {1.0, -2.0, 2.0};
  std::vector<double> expected_vec = {1.0 / 3.0, -2.0 / 3.0, 2.0 / 3.0};

  std::vector<double> res = lidarMirrorFOVReshaperTF::getNormalizedVector(src_vec);

  ASSERT_EQ(res.size(), expected_vec.size());
  for (size_t i = 0; i < res.size(); ++i) {
    ASSERT_NEAR(res[i], expected_vec[i], 1E-5);
  }
};

TEST_F(lidarMirrorFOVReshaperTFTest, testGetNormalizedVector_ZeroVector)
{
  std::vector<double> src_vec = {0.0, 0.0, 0.0};
  std::vector<double> res = lidarMirrorFOVReshaperTF::getNormalizedVector(src_vec);

  ASSERT_EQ(res.size(), src_vec.size());
  for (double val : res) {
    ASSERT_TRUE(std::isnan(val) || val == 0.0);
  }
};

TEST_F(lidarMirrorFOVReshaperTFTest, testGetNormalizedVector_SingleElement)
{
  std::vector<double> src_vec = {10.0};
  std::vector<double> expected_vec = {1.0};

  std::vector<double> res = lidarMirrorFOVReshaperTF::getNormalizedVector(src_vec);

  ASSERT_EQ(res.size(), expected_vec.size());
  ASSERT_NEAR(res[0], expected_vec[0], 1E-5);
};

TEST_F(lidarMirrorFOVReshaperTFTest, testGetNormalizedVector_EmptyVector)
{
  std::vector<double> src_vec = {};
  std::vector<double> res = lidarMirrorFOVReshaperTF::getNormalizedVector(src_vec);

  ASSERT_TRUE(res.empty());
};

TEST_F(lidarMirrorFOVReshaperTFTest, testNormalizePoints_PositiveValues)
{
  pcl::PointCloud<pcl::PointXYZI> src_cloud;
  src_cloud.points.push_back(pcl::PointXYZI{3.0, 4.0, 0.0, 0.0});  // mag = 5
  src_cloud.points.push_back(pcl::PointXYZI{1.0, 2.0, 2.0, 0.0});  // mag = 3

  std::vector<std::vector<double>> normalized_points;
  lidarMirrorFOVReshaperTF::normalizePoints(src_cloud, &normalized_points);

  ASSERT_EQ(normalized_points.size(), src_cloud.size());

  std::vector<std::vector<double>> expected = {{0.6, 0.8, 0.0}, {1.0 / 3.0, 2.0 / 3.0, 2.0 / 3.0}};

  for (size_t i = 0; i < normalized_points.size(); ++i) {
    for (size_t j = 0; j < 3; ++j) {
      ASSERT_NEAR(normalized_points[i][j], expected[i][j], 1E-5);
    }
  }
};

TEST_F(lidarMirrorFOVReshaperTFTest, testNormalizePoints_NegativeValues)
{
  pcl::PointCloud<pcl::PointXYZI> src_cloud;
  src_cloud.points.push_back(pcl::PointXYZI{-3.0, -4.0, 0.0, 0.0});   // mag = 5
  src_cloud.points.push_back(pcl::PointXYZI{-1.0, -2.0, -2.0, 0.0});  // mag = 3

  std::vector<std::vector<double>> normalized_points;
  lidarMirrorFOVReshaperTF::normalizePoints(src_cloud, &normalized_points);

  ASSERT_EQ(normalized_points.size(), src_cloud.size());

  std::vector<std::vector<double>> expected = {
    {-0.6, -0.8, 0.0}, {-1.0 / 3.0, -2.0 / 3.0, -2.0 / 3.0}};

  for (size_t i = 0; i < normalized_points.size(); ++i) {
    for (size_t j = 0; j < 3; ++j) {
      ASSERT_NEAR(normalized_points[i][j], expected[i][j], 1E-5);
    }
  }
};

TEST_F(lidarMirrorFOVReshaperTFTest, testNormalizePoints_ZeroPoint)
{
  pcl::PointCloud<pcl::PointXYZI> src_cloud;
  src_cloud.points.push_back(pcl::PointXYZI{0.0, 0.0, 0.0, 0.0});  // mag = 0

  std::vector<std::vector<double>> normalized_points;
  lidarMirrorFOVReshaperTF::normalizePoints(src_cloud, &normalized_points);

  ASSERT_EQ(normalized_points.size(), src_cloud.size());

  for (size_t i = 0; i < 3; ++i) {
    ASSERT_TRUE(std::isnan(normalized_points[0][i]) || normalized_points[0][i] == 0.0);
  }
};

TEST_F(lidarMirrorFOVReshaperTFTest, testNormalizePoints_SinglePoint)
{
  pcl::PointCloud<pcl::PointXYZI> src_cloud;
  src_cloud.points.push_back(pcl::PointXYZI{5.0, 0.0, 0.0, 0.0});  // mag = 5

  std::vector<std::vector<double>> normalized_points;
  lidarMirrorFOVReshaperTF::normalizePoints(src_cloud, &normalized_points);

  ASSERT_EQ(normalized_points.size(), src_cloud.size());

  std::vector<double> expected = {1.0, 0.0, 0.0};

  for (size_t i = 0; i < 3; ++i) {
    ASSERT_NEAR(normalized_points[0][i], expected[i], 1E-5);
  }
};

TEST_F(lidarMirrorFOVReshaperTFTest, testNormalizePoints_EmptyCloud)
{
  pcl::PointCloud<pcl::PointXYZI> src_cloud;

  std::vector<std::vector<double>> normalized_points;
  lidarMirrorFOVReshaperTF::normalizePoints(src_cloud, &normalized_points);

  ASSERT_TRUE(normalized_points.empty());
};

TEST_F(lidarMirrorFOVReshaperTFTest, testGetPointMagnitudeFromCloud_PositiveValues)
{
  pcl::PointCloud<pcl::PointXYZI> src_cloud;
  src_cloud.points.push_back(pcl::PointXYZI{3.0, 4.0, 0.0, 0.0});  // mag = 5
  src_cloud.points.push_back(pcl::PointXYZI{1.0, 2.0, 2.0, 0.0});  // mag = 3

  std::vector<double> point_magnitude;
  lidarMirrorFOVReshaperTF::getPointMagnitude(src_cloud, &point_magnitude);

  ASSERT_EQ(point_magnitude.size(), src_cloud.size());

  std::vector<double> expected = {5.0, 3.0};

  for (size_t i = 0; i < point_magnitude.size(); ++i) {
    ASSERT_NEAR(point_magnitude[i], expected[i], 1E-5);
  }
};

TEST_F(lidarMirrorFOVReshaperTFTest, testGetPointMagnitudeFromCloud_NegativeValues)
{
  pcl::PointCloud<pcl::PointXYZI> src_cloud;
  src_cloud.points.push_back(pcl::PointXYZI{-3.0, -4.0, 0.0, 0.0});   // mag = 5
  src_cloud.points.push_back(pcl::PointXYZI{-1.0, -2.0, -2.0, 0.0});  // mag = 3

  std::vector<double> point_magnitude;
  lidarMirrorFOVReshaperTF::getPointMagnitude(src_cloud, &point_magnitude);

  ASSERT_EQ(point_magnitude.size(), src_cloud.size());

  std::vector<double> expected = {5.0, 3.0};

  for (size_t i = 0; i < point_magnitude.size(); ++i) {
    ASSERT_NEAR(point_magnitude[i], expected[i], 1E-5);
  }
};

TEST_F(lidarMirrorFOVReshaperTFTest, testGetPointMagnitude_ZeroPoint)
{
  pcl::PointCloud<pcl::PointXYZI> src_cloud;
  src_cloud.points.push_back(pcl::PointXYZI{0.0, 0.0, 0.0, 0.0});  // mag = 0

  std::vector<double> point_magnitude;
  lidarMirrorFOVReshaperTF::getPointMagnitude(src_cloud, &point_magnitude);

  ASSERT_EQ(point_magnitude.size(), src_cloud.size());
  ASSERT_NEAR(point_magnitude[0], 0.0, 1E-5);
};

TEST_F(lidarMirrorFOVReshaperTFTest, testGetPointMagnitude_SinglePoint)
{
  pcl::PointCloud<pcl::PointXYZI> src_cloud;
  src_cloud.points.push_back(pcl::PointXYZI{5.0, 0.0, 0.0, 0.0});  // mag = 5

  std::vector<double> point_magnitude;
  lidarMirrorFOVReshaperTF::getPointMagnitude(src_cloud, &point_magnitude);

  ASSERT_EQ(point_magnitude.size(), src_cloud.size());
  ASSERT_NEAR(point_magnitude[0], 5.0, 1E-5);
};

TEST_F(lidarMirrorFOVReshaperTFTest, testGetPointMagnitude_EmptyCloud)
{
  pcl::PointCloud<pcl::PointXYZI> src_cloud;

  std::vector<double> point_magnitude;
  lidarMirrorFOVReshaperTF::getPointMagnitude(src_cloud, &point_magnitude);

  ASSERT_TRUE(point_magnitude.empty());
};

TEST_F(lidarMirrorFOVReshaperTFTest, testGetVectorMagnitude_PositiveValues)
{
  std::vector<double> src_vec = {3.0, 4.0};  // mag = 5
  double expected = 5.0;

  double res = lidarMirrorFOVReshaperTF::getVectorMagnitude(src_vec);

  ASSERT_NEAR(res, expected, 1E-5);
};

TEST_F(lidarMirrorFOVReshaperTFTest, testGetVectorMagnitude_NegativeValues)
{
  std::vector<double> src_vec = {-3.0, -4.0};  // mag = 5
  double expected = 5.0;

  double res = lidarMirrorFOVReshaperTF::getVectorMagnitude(src_vec);

  ASSERT_NEAR(res, expected, 1E-5);
};

TEST_F(lidarMirrorFOVReshaperTFTest, testGetVectorMagnitude_MixedValues)
{
  std::vector<double> src_vec = {1.0, -2.0, 2.0};  // mag = 3
  double expected = 3.0;

  double res = lidarMirrorFOVReshaperTF::getVectorMagnitude(src_vec);

  ASSERT_NEAR(res, expected, 1E-5);
};

TEST_F(lidarMirrorFOVReshaperTFTest, testGetVectorMagnitude_ZeroVector)
{
  std::vector<double> src_vec = {0.0, 0.0, 0.0};  // mag = 0
  double expected = 0.0;

  double res = lidarMirrorFOVReshaperTF::getVectorMagnitude(src_vec);

  ASSERT_NEAR(res, expected, 1E-5);
};

TEST_F(lidarMirrorFOVReshaperTFTest, testGetVectorMagnitude_SingleElement)
{
  std::vector<double> src_vec = {5.0};  // mag = 5
  double expected = 5.0;

  double res = lidarMirrorFOVReshaperTF::getVectorMagnitude(src_vec);

  ASSERT_NEAR(res, expected, 1E-5);
};

TEST_F(lidarMirrorFOVReshaperTFTest, testGetVectorMagnitude_EmptyVector)
{
  std::vector<double> src_vec = {};  // mag = 0
  double expected = 0.0;

  double res = lidarMirrorFOVReshaperTF::getVectorMagnitude(src_vec);

  ASSERT_NEAR(res, expected, 1E-5);
};

TEST_F(lidarMirrorFOVReshaperTFTest, testGetPointMagnitude_PositiveValues)
{
  std::vector<std::vector<double>> src_vec = {
    {3.0, 4.0},        // mag = 5
    {1.0, 2.0, 2.0}};  // mag = 3

  std::vector<double> point_magnitude;
  lidarMirrorFOVReshaperTF::getPointMagnitude(src_vec, &point_magnitude);

  ASSERT_EQ(point_magnitude.size(), src_vec.size());

  std::vector<double> expected = {5.0, 3.0};

  for (size_t i = 0; i < point_magnitude.size(); ++i) {
    ASSERT_NEAR(point_magnitude[i], expected[i], 1E-5);
  }
};

TEST_F(lidarMirrorFOVReshaperTFTest, testGetPointMagnitude_NegativeValues)
{
  std::vector<std::vector<double>> src_vec = {
    {-3.0, -4.0},         // mag = 5
    {-1.0, -2.0, -2.0}};  // mag = 3

  std::vector<double> point_magnitude;
  lidarMirrorFOVReshaperTF::getPointMagnitude(src_vec, &point_magnitude);

  ASSERT_EQ(point_magnitude.size(), src_vec.size());

  std::vector<double> expected = {5.0, 3.0};

  for (size_t i = 0; i < point_magnitude.size(); ++i) {
    ASSERT_NEAR(point_magnitude[i], expected[i], 1E-5);
  }
};

TEST_F(lidarMirrorFOVReshaperTFTest, testGetPointMagnitude_MixedValues)
{
  std::vector<std::vector<double>> src_vec = {
    {1.0, -2.0, 2.0},    // mag = 3
    {2.0, -1.0, -2.0}};  // mag = 3

  std::vector<double> point_magnitude;
  lidarMirrorFOVReshaperTF::getPointMagnitude(src_vec, &point_magnitude);

  ASSERT_EQ(point_magnitude.size(), src_vec.size());

  std::vector<double> expected = {3.0, 3.0};

  for (size_t i = 0; i < point_magnitude.size(); ++i) {
    ASSERT_NEAR(point_magnitude[i], expected[i], 1E-5);
  }
};

TEST_F(lidarMirrorFOVReshaperTFTest, testGetPointMagnitude_ZeroVector)
{
  std::vector<std::vector<double>> src_vec = {
    {0.0, 0.0, 0.0},  // mag = 0
    {0.0, 0.0}};      // mag = 0

  std::vector<double> point_magnitude;
  lidarMirrorFOVReshaperTF::getPointMagnitude(src_vec, &point_magnitude);

  ASSERT_EQ(point_magnitude.size(), src_vec.size());

  for (const auto & mag : point_magnitude) {
    ASSERT_NEAR(mag, 0.0, 1E-5);
  }
};

TEST_F(lidarMirrorFOVReshaperTFTest, testGetPointMagnitude_SingleElementVectors)
{
  std::vector<std::vector<double>> src_vec = {
    {5.0},    // mag = 5
    {-3.0}};  // mag = 3

  std::vector<double> point_magnitude;
  lidarMirrorFOVReshaperTF::getPointMagnitude(src_vec, &point_magnitude);

  ASSERT_EQ(point_magnitude.size(), src_vec.size());

  std::vector<double> expected = {5.0, 3.0};

  for (size_t i = 0; i < point_magnitude.size(); ++i) {
    ASSERT_NEAR(point_magnitude[i], expected[i], 1E-5);
  }
};

TEST_F(lidarMirrorFOVReshaperTFTest, testGetPointMagnitude_EmptyVector)
{
  std::vector<std::vector<double>> src_vec;

  std::vector<double> point_magnitude;
  lidarMirrorFOVReshaperTF::getPointMagnitude(src_vec, &point_magnitude);

  ASSERT_TRUE(point_magnitude.empty());
};

TEST_F(lidarMirrorFOVReshaperTFTest, testCalcMirrorPlaneNormal_BasicCase)
{
  std::vector<double> p1 = {0.0, 0.0, 0.0};
  std::vector<double> p2 = {1.0, 0.0, 0.0};
  std::vector<double> p3 = {0.0, 1.0, 0.0};
  std::vector<double> normal_vec;

  lidarMirrorFOVReshaperTF::calcMirrorPlaneNormal(p1, p2, p3, normal_vec);

  std::vector<double> expected = {0.0, 0.0, 1.0};  // normal to xy-plane

  ASSERT_EQ(normal_vec.size(), 3);
  for (size_t i = 0; i < normal_vec.size(); ++i) {
    ASSERT_NEAR(normal_vec[i], expected[i], 1E-5);
  }
};

TEST_F(lidarMirrorFOVReshaperTFTest, testCalcMirrorPlaneNormal_NegativeCoordinates)
{
  std::vector<double> p1 = {-1.0, -1.0, -1.0};
  std::vector<double> p2 = {0.0, -1.0, -1.0};
  std::vector<double> p3 = {-1.0, 0.0, -1.0};
  std::vector<double> normal_vec;

  lidarMirrorFOVReshaperTF::calcMirrorPlaneNormal(p1, p2, p3, normal_vec);

  std::vector<double> expected = {0.0, 0.0, 1.0};  // normal to xy-plane

  ASSERT_EQ(normal_vec.size(), 3);
  for (size_t i = 0; i < normal_vec.size(); ++i) {
    ASSERT_NEAR(normal_vec[i], expected[i], 1E-5);
  }
};

TEST_F(lidarMirrorFOVReshaperTFTest, testCalcMirrorPlaneNormal_RotatedPlane)
{
  std::vector<double> p1 = {0.0, 0.0, 0.0};
  std::vector<double> p2 = {1.0, 0.0, 1.0};
  std::vector<double> p3 = {0.0, 1.0, 1.0};
  std::vector<double> normal_vec;

  lidarMirrorFOVReshaperTF::calcMirrorPlaneNormal(p1, p2, p3, normal_vec);

  // both normal vectors represent a valid orientation of the plane
  std::vector<double> expected = {-0.57735, -0.57735, 0.57735};
  std::vector<double> expected2 = {0.57735, 0.57735, -0.57735};

  ASSERT_EQ(normal_vec.size(), 3);
  for (size_t i = 0; i < normal_vec.size(); ++i) {
    ASSERT_TRUE(
      std::abs(normal_vec[i] - expected[i]) < 1E-5 ||
      std::abs(normal_vec[i] - expected2[i]) < 1E-5);
  }
};

TEST_F(lidarMirrorFOVReshaperTFTest, testCalcMirrorPlaneNormal_SamePoints)
{
  std::vector<double> p1 = {1.0, 1.0, 1.0};
  std::vector<double> p2 = {1.0, 1.0, 1.0};
  std::vector<double> p3 = {1.0, 1.0, 1.0};
  std::vector<double> normal_vec;

  lidarMirrorFOVReshaperTF::calcMirrorPlaneNormal(p1, p2, p3, normal_vec);

  ASSERT_EQ(normal_vec.size(), 3);
  for (const auto & val : normal_vec) {
    ASSERT_TRUE(std::isnan(val));
  }
};

TEST_F(lidarMirrorFOVReshaperTFTest, testCalcMirrorPlaneNormal_CollinearPoints)
{
  std::vector<double> p1 = {0.0, 0.0, 0.0};
  std::vector<double> p2 = {1.0, 1.0, 1.0};
  std::vector<double> p3 = {2.0, 2.0, 2.0};
  std::vector<double> normal_vec;

  lidarMirrorFOVReshaperTF::calcMirrorPlaneNormal(p1, p2, p3, normal_vec);

  ASSERT_EQ(normal_vec.size(), 3);
  for (const auto & val : normal_vec) {
    ASSERT_TRUE(std::isnan(val));
  }
};

TEST_F(lidarMirrorFOVReshaperTFTest, testCalcPointPlaneDist_BasicCase)
{
  pcl::PointCloud<pcl::PointXYZI> src_cloud;
  src_cloud.push_back(pcl::PointXYZI{0.0, 0.0, 1.0});
  src_cloud.push_back(pcl::PointXYZI{0.0, 0.0, -1.0});
  src_cloud.push_back(pcl::PointXYZI{1.0, 1.0, 0.0});

  std::vector<double> plane_support_vec = {0.0, 0.0, 0.0};
  std::vector<double> plane_normal_vec = {0.0, 0.0, 1.0};

  std::vector<double> point_plane_distances;
  lidarMirrorFOVReshaperTF::calcPointPlaneDist(
    src_cloud, plane_support_vec, plane_normal_vec, &point_plane_distances);

  ASSERT_EQ(point_plane_distances.size(), src_cloud.size());

  std::vector<double> expected = {1.0, 1.0, 0.0};

  for (size_t i = 0; i < point_plane_distances.size(); ++i) {
    ASSERT_NEAR(point_plane_distances[i], expected[i], 1E-5);
  }
}

TEST_F(lidarMirrorFOVReshaperTFTest, testCalcPointPlaneDist_OffsetPlane)
{
  pcl::PointCloud<pcl::PointXYZI> src_cloud;
  src_cloud.push_back(pcl::PointXYZI{0.0, 0.0, 6.0});
  src_cloud.push_back(pcl::PointXYZI{0.0, 0.0, 4.0});
  src_cloud.push_back(pcl::PointXYZI{1.0, 1.0, 5.0});

  std::vector<double> plane_support_vec = {0.0, 0.0, 5.0};
  std::vector<double> plane_normal_vec = {0.0, 0.0, 1.0};

  std::vector<double> point_plane_distances;
  lidarMirrorFOVReshaperTF::calcPointPlaneDist(
    src_cloud, plane_support_vec, plane_normal_vec, &point_plane_distances);

  ASSERT_EQ(point_plane_distances.size(), src_cloud.size());

  std::vector<double> expected = {1.0, 1.0, 0.0};

  for (size_t i = 0; i < point_plane_distances.size(); ++i) {
    ASSERT_NEAR(point_plane_distances[i], expected[i], 1E-5);
  }
}

TEST_F(lidarMirrorFOVReshaperTFTest, testCalcPointPlaneDist_HorizontalPlane)
{
  pcl::PointCloud<pcl::PointXYZI> src_cloud;
  src_cloud.push_back(pcl::PointXYZI{0.0, 3.0, 0.0});
  src_cloud.push_back(pcl::PointXYZI{0.0, -3.0, 0.0});
  src_cloud.push_back(pcl::PointXYZI{0.0, 0.0, 0.0});

  std::vector<double> plane_support_vec = {0.0, 0.0, 0.0};
  std::vector<double> plane_normal_vec = {0.0, 1.0, 0.0};

  std::vector<double> point_plane_distances;
  lidarMirrorFOVReshaperTF::calcPointPlaneDist(
    src_cloud, plane_support_vec, plane_normal_vec, &point_plane_distances);

  ASSERT_EQ(point_plane_distances.size(), src_cloud.size());

  std::vector<double> expected = {3.0, 3.0, 0.0};

  for (size_t i = 0; i < point_plane_distances.size(); ++i) {
    ASSERT_NEAR(point_plane_distances[i], expected[i], 1E-5);
  }
}

TEST_F(lidarMirrorFOVReshaperTFTest, testCalcPointPlaneDist_EmptyPointCloud)
{
  pcl::PointCloud<pcl::PointXYZI> src_cloud;

  std::vector<double> plane_support_vec = {0.0, 0.0, 0.0};
  std::vector<double> plane_normal_vec = {0.0, 0.0, 1.0};

  std::vector<double> point_plane_distances;
  lidarMirrorFOVReshaperTF::calcPointPlaneDist(
    src_cloud, plane_support_vec, plane_normal_vec, &point_plane_distances);

  ASSERT_TRUE(point_plane_distances.empty());
}

TEST_F(lidarMirrorFOVReshaperTFTest, testUnwrapVector_CorrectInput)
{
  std::vector<double> src_vec = {1.0, 2.0, 3.0, 0.0, 0.0, 1.0, 4.0, 5.0, 6.0,
                                 1.0, 0.0, 0.0, 7.0, 8.0, 9.0, 0.0, 1.0, 0.0};

  std::vector<double> plane_support_vec, plane_normal_vec;
  std::vector<double> mirror_right_support_vec, mirror_right_normal_vec;
  std::vector<double> mirror_left_support_vec, mirror_left_normal_vec;

  lidarMirrorFOVReshaperTF::unwrapVector(
    src_vec, plane_support_vec, plane_normal_vec, mirror_right_support_vec, mirror_right_normal_vec,
    mirror_left_support_vec, mirror_left_normal_vec);

  ASSERT_EQ(plane_support_vec, std::vector<double>({1.0, 2.0, 3.0}));
  ASSERT_EQ(plane_normal_vec, std::vector<double>({0.0, 0.0, 1.0}));
  ASSERT_EQ(mirror_right_support_vec, std::vector<double>({4.0, 5.0, 6.0}));
  ASSERT_EQ(mirror_right_normal_vec, std::vector<double>({1.0, 0.0, 0.0}));
  ASSERT_EQ(mirror_left_support_vec, std::vector<double>({7.0, 8.0, 9.0}));
  ASSERT_EQ(mirror_left_normal_vec, std::vector<double>({0.0, 1.0, 0.0}));
}

TEST_F(lidarMirrorFOVReshaperTFTest, testUnwrapVector_IncorrectInputSize)
{
  std::vector<double> src_vec = {1.0, 2.0, 3.0};

  std::vector<double> plane_support_vec, plane_normal_vec;
  std::vector<double> mirror_right_support_vec, mirror_right_normal_vec;
  std::vector<double> mirror_left_support_vec, mirror_left_normal_vec;

  testing::internal::CaptureStderr();
  lidarMirrorFOVReshaperTF::unwrapVector(
    src_vec, plane_support_vec, plane_normal_vec, mirror_right_support_vec, mirror_right_normal_vec,
    mirror_left_support_vec, mirror_left_normal_vec);
  std::string output = testing::internal::GetCapturedStderr();

  ASSERT_TRUE(plane_support_vec.empty());
  ASSERT_TRUE(plane_normal_vec.empty());
  ASSERT_TRUE(mirror_right_support_vec.empty());
  ASSERT_TRUE(mirror_right_normal_vec.empty());
  ASSERT_TRUE(mirror_left_support_vec.empty());
  ASSERT_TRUE(mirror_left_normal_vec.empty());

  ASSERT_NE(output.find("Error: input vector is not of size 18"), std::string::npos);
}

TEST_F(lidarMirrorFOVReshaperTFTest, testUnwrapVector_AllZeros)
{
  std::vector<double> src_vec(18, 0.0);

  std::vector<double> plane_support_vec, plane_normal_vec;
  std::vector<double> mirror_right_support_vec, mirror_right_normal_vec;
  std::vector<double> mirror_left_support_vec, mirror_left_normal_vec;

  lidarMirrorFOVReshaperTF::unwrapVector(
    src_vec, plane_support_vec, plane_normal_vec, mirror_right_support_vec, mirror_right_normal_vec,
    mirror_left_support_vec, mirror_left_normal_vec);

  ASSERT_EQ(plane_support_vec, std::vector<double>(3, 0.0));
  ASSERT_EQ(plane_normal_vec, std::vector<double>(3, 0.0));
  ASSERT_EQ(mirror_right_support_vec, std::vector<double>(3, 0.0));
  ASSERT_EQ(mirror_right_normal_vec, std::vector<double>(3, 0.0));
  ASSERT_EQ(mirror_left_support_vec, std::vector<double>(3, 0.0));
  ASSERT_EQ(mirror_left_normal_vec, std::vector<double>(3, 0.0));
}

TEST_F(lidarMirrorFOVReshaperTFTest, testGetStddevPositiveInputVector)
{
  std::vector<double> src_vec = {1.0, 2.0, 3.0, 4.0, 5.0};
  double tgt_stddev = 1.41421;

  double res = lidarMirrorFOVReshaperTF::getStdDev(src_vec);
  ASSERT_NEAR(res, tgt_stddev, 1E-5);
};

TEST_F(lidarMirrorFOVReshaperTFTest, testGetStddevNegativeInputVector)
{
  std::vector<double> src_vec = {-1.0, -2.0, -3.0, -4.0, -5.0};
  double tgt_stddev = 1.41421;

  double res = lidarMirrorFOVReshaperTF::getStdDev(src_vec);
  ASSERT_NEAR(res, tgt_stddev, 1E-5)
    << "Expected results to be equal, but got " << res << " and " << tgt_stddev;
};

TEST_F(lidarMirrorFOVReshaperTFTest, testGetStddevEmptyInputVector)
{
  std::vector<double> emptyVec;
  double resultEmptyVec = lidarMirrorFOVReshaperTF::getStdDev(emptyVec);
  ASSERT_TRUE(std::isnan(resultEmptyVec)) << "Expected result to be NaN for an empty vector.";
}

TEST_F(lidarMirrorFOVReshaperTFTest, testGetStddevSinglePositiveInputVector)
{
  std::vector<double> singleElementVec = {3.0};
  double resultSingleElementVec = lidarMirrorFOVReshaperTF::getStdDev(singleElementVec);
  ASSERT_EQ(resultSingleElementVec, 0.0) << "Expected result to be 0 for a single-element vector.";
};

TEST_F(lidarMirrorFOVReshaperTFTest, testGetStddevSamePositiveElementInputVector)
{
  std::vector<double> identicalElementVec = {2.0, 2.0, 2.0, 2.0};
  double resultIdenticalElementVec = lidarMirrorFOVReshaperTF::getStdDev(identicalElementVec);
  ASSERT_EQ(resultIdenticalElementVec, 0.0)
    << "Expected result to be 0 for a vector of identical elements.";
};

TEST_F(lidarMirrorFOVReshaperTFTest, testDeg2Rad_ZeroDegrees)
{
  double deg = 0.0;
  double expected = 0.0;
  double result = lidarMirrorFOVReshaperTF::deg2rad(deg);
  ASSERT_NEAR(result, expected, 1E-5);
}

TEST_F(lidarMirrorFOVReshaperTFTest, testDeg2Rad_NinetyDegrees)
{
  double deg = 90.0;
  double expected = M_PI / 2.0;  // 90 degrees in radians
  double result = lidarMirrorFOVReshaperTF::deg2rad(deg);
  ASSERT_NEAR(result, expected, 1E-5);
}

TEST_F(lidarMirrorFOVReshaperTFTest, testDeg2Rad_OneEightyDegrees)
{
  double deg = 180.0;
  double expected = M_PI;  // 180 degrees in radians
  double result = lidarMirrorFOVReshaperTF::deg2rad(deg);
  ASSERT_NEAR(result, expected, 1E-5);
}

TEST_F(lidarMirrorFOVReshaperTFTest, testDeg2Rad_ThreeSixtyDegrees)
{
  double deg = 360.0;
  double expected = 2 * M_PI;  // 360 degrees in radians
  double result = lidarMirrorFOVReshaperTF::deg2rad(deg);
  ASSERT_NEAR(result, expected, 1E-5);
}

TEST_F(lidarMirrorFOVReshaperTFTest, testDeg2Rad_NegativeAngle)
{
  double deg = -90.0;
  double expected = -M_PI / 2.0;  // -90 degrees in radians
  double result = lidarMirrorFOVReshaperTF::deg2rad(deg);
  ASSERT_NEAR(result, expected, 1E-5);
}

TEST_F(lidarMirrorFOVReshaperTFTest, testDeg2Rad_FractionalDegree)
{
  double deg = 45.5;
  double expected = 45.5 * M_PI / 180.0;  // 45.5 degrees in radians
  double result = lidarMirrorFOVReshaperTF::deg2rad(deg);
  ASSERT_NEAR(result, expected, 1E-5);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  int test_res = RUN_ALL_TESTS();
  return test_res;
};