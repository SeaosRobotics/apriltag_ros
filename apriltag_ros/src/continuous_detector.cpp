/**
 * Copyright (c) 2017, California Institute of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the California Institute of
 * Technology.
 */

#include "apriltag_ros/continuous_detector.h"

#include <rclcpp_components/register_node_macro.hpp>

namespace apriltag_ros
{
ContinuousDetector::ContinuousDetector(const rclcpp::NodeOptions & options)
: rclcpp::Node("apriltag_ros_continuous_node", options)
{
  tag_detector_ = std::make_shared<TagDetector>(this);
  draw_tag_detections_image_ = getAprilTagOption<bool>(this,
      "publish_tag_detections_image", false);

  std::string transport_hint = getAprilTagOption<std::string>(
      this, "transport_hint", "raw");

  // Using the image_transport free functions (rather than an
  // image_transport::ImageTransport instance) avoids calling
  // shared_from_this() while this node's constructor is still running.
  camera_image_subscriber_ = image_transport::create_camera_subscription(
      this, "image_rect",
      std::bind(&ContinuousDetector::imageCallback, this,
                std::placeholders::_1, std::placeholders::_2),
      transport_hint, rclcpp::QoS(1).get_rmw_qos_profile());
  tag_detections_publisher_ =
      this->create_publisher<AprilTagDetectionArray>("tag_detections", 1);
  if (draw_tag_detections_image_)
  {
    tag_detections_image_publisher_ =
        image_transport::create_publisher(this, "tag_detections_image");
  }
}

void ContinuousDetector::imageCallback (
    const sensor_msgs::msg::Image::ConstSharedPtr& image_rect,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info)
{
  // Lazy updates:
  // When there are no subscribers _and_ when tf is not published,
  // skip detection.
  if (tag_detections_publisher_->get_subscription_count() == 0 &&
      tag_detections_image_publisher_.getNumSubscribers() == 0 &&
      !tag_detector_->get_publish_tf())
  {
    // RCLCPP_INFO_STREAM(this->get_logger(), "No subscribers and no tf publishing, skip processing.");
    return;
  }

  // Convert ROS's sensor_msgs::msg::Image to cv_bridge::CvImagePtr in order
  // to run AprilTag 2 on the image
  try
  {
    cv_image_ = cv_bridge::toCvCopy(image_rect, image_rect->encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  // Publish detected tags in the image by AprilTag 2
  tag_detections_publisher_->publish(
      tag_detector_->detectTags(cv_image_,camera_info));

  // Publish the camera image overlaid by outlines of the detected tags and
  // their payload values
  if (draw_tag_detections_image_)
  {
    tag_detector_->drawDetections(cv_image_);
    tag_detections_image_publisher_.publish(*cv_image_->toImageMsg());
  }
}

} // namespace apriltag_ros

RCLCPP_COMPONENTS_REGISTER_NODE(apriltag_ros::ContinuousDetector)
