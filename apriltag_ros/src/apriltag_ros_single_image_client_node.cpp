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

#include <rclcpp/rclcpp.hpp>
#include "apriltag_ros/srv/analyze_single_image.hpp"

namespace
{
bool getRosParameter(rclcpp::Node* node, const std::string& name, double& param)
{
  // Write parameter "name" from the ROS parameter server into param
  // Return true if successful, false otherwise
  if (node->has_parameter(name))
  {
    node->get_parameter(name, param);
    RCLCPP_INFO_STREAM(node->get_logger(), "Set camera " << name.c_str() << " = " << param);
    return true;
  }
  else
  {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Could not find " << name.c_str() << " parameter!");
    return false;
  }
}
} // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("apriltag_ros_single_image_client");

  node->declare_parameter<std::string>("image_load_path", "");
  node->declare_parameter<std::string>("image_save_path", "");
  node->declare_parameter<double>("fx", 0.0);
  node->declare_parameter<double>("fy", 0.0);
  node->declare_parameter<double>("cx", 0.0);
  node->declare_parameter<double>("cy", 0.0);

  auto client = node->create_client<apriltag_ros::srv::AnalyzeSingleImage>(
      "single_image_tag_detection");

  // Get the request parameters
  auto request = std::make_shared<apriltag_ros::srv::AnalyzeSingleImage::Request>();
  node->get_parameter("image_load_path", request->full_path_where_to_get_image);
  if (request->full_path_where_to_get_image.empty())
  {
    rclcpp::shutdown();
    return 1;
  }
  node->get_parameter("image_save_path", request->full_path_where_to_save_image);
  if (request->full_path_where_to_save_image.empty())
  {
    rclcpp::shutdown();
    return 1;
  }

  // Replicate sensor_msgs/CameraInfo message (must be up-to-date with the
  // analyzed image!)
  request->camera_info.distortion_model = "plumb_bob";
  double fx, fy, cx, cy;
  if (!getRosParameter(node.get(), "fx", fx)) { rclcpp::shutdown(); return 1; }
  if (!getRosParameter(node.get(), "fy", fy)) { rclcpp::shutdown(); return 1; }
  if (!getRosParameter(node.get(), "cx", cx)) { rclcpp::shutdown(); return 1; }
  if (!getRosParameter(node.get(), "cy", cy)) { rclcpp::shutdown(); return 1; }
  // Intrinsic camera matrix for the raw (distorted) images
  request->camera_info.k[0] = fx;
  request->camera_info.k[2] = cx;
  request->camera_info.k[4] = fy;
  request->camera_info.k[5] = cy;
  request->camera_info.k[8] = 1.0;
  request->camera_info.p[0] = fx;
  request->camera_info.p[2] = cx;
  request->camera_info.p[5] = fy;
  request->camera_info.p[6] = cy;
  request->camera_info.p[10] = 1.0;

  if (!client->wait_for_service(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(node->get_logger(),
                 "Service single_image_tag_detection not available");
    rclcpp::shutdown();
    return 1;
  }

  // Call the service (detect tags in the image specified by
  // image_load_path)
  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, future) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = future.get();
    if (response->tag_detections.detections.size() == 0)
    {
      RCLCPP_WARN_STREAM(node->get_logger(), "No detected tags!");
    }
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to call service single_image_tag_detection");
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0; // happy ending
}
