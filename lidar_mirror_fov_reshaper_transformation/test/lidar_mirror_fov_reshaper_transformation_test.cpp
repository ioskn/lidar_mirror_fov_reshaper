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

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

/*
 * Ground-Truth values calculated via wolfram alpha
 */
class lidarMirrorFOVReshaperTFTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("open_see_ground_tf_test");

    callback_received_ = false;
    sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "test_topic", 10,
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        callback_received_ = true;
        received_msg_ = *msg;
      });
    pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("test_topic", 10);
  };

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  void publishPointCloud(const sensor_msgs::msg::PointCloud2 &cloud)
  {
    pub_->publish(cloud);
  };

  std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  sensor_msgs::msg::PointCloud2 received_msg_;
  bool callback_received_;
};

TEST_F(lidarMirrorFOVReshaperTFTest, TestPointCloudCallback)
{
  // Create a synthetic PointCloud2 message
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl_cloud.width = 2;
  pcl_cloud.height = 1;
  pcl_cloud.points.resize(pcl_cloud.width * pcl_cloud.height);

  for (size_t i = 0; i < pcl_cloud.points.size(); ++i)
  {
    pcl_cloud.points[i].x = static_cast<float>(i);
    pcl_cloud.points[i].y = static_cast<float>(i * 2);
    pcl_cloud.points[i].z = static_cast<float>(i * 3);
  }

  sensor_msgs::msg::PointCloud2 ros_cloud;
  pcl::toROSMsg(pcl_cloud, ros_cloud);
  ros_cloud.header.frame_id = "test_frame";
  ros_cloud.header.stamp = rclcpp::Clock().now();

  // Publish the synthetic point cloud
  publishPointCloud(ros_cloud);

  // Spin the node to process the callback
  rclcpp::spin_some(node_);

  // Verify that the callback was triggered and processed correctly
  ASSERT_TRUE(callback_received_);
  ASSERT_EQ(received_msg_.header.frame_id, "test_frame");
  ASSERT_EQ(received_msg_.width, 2);
  ASSERT_EQ(received_msg_.height, 1);
};

TEST_F(lidarMirrorFOVReshaperTFTest, testGetIndicesFromAngles) {}

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

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  int test_res = RUN_ALL_TESTS();
  return test_res;
};