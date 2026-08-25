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
 *
 ** single_image_detector.h ****************************************************
 *
 * Wrapper class of TagDetector class which calls TagDetector::detectTags on a
 * an image stored at a specified load path and stores the output at a specified
 * save path.
 *
 * ROS 2 port: implemented as a composable rclcpp::Node.
 *
 * Originator:        Danylo Malyuta, JPL
 ******************************************************************************/

#ifndef APRILTAG_ROS_SINGLE_IMAGE_DETECTOR_H
#define APRILTAG_ROS_SINGLE_IMAGE_DETECTOR_H

#include "apriltag_ros/common_functions.h"
#include "apriltag_ros/srv/analyze_single_image.hpp"

#include <rclcpp/rclcpp.hpp>

namespace apriltag_ros
{

class SingleImageDetector : public rclcpp::Node
{
 public:
  explicit SingleImageDetector(const rclcpp::NodeOptions & options);

  // The function which provides the single image analysis service
  void analyzeImage(
      const std::shared_ptr<apriltag_ros::srv::AnalyzeSingleImage::Request> request,
      std::shared_ptr<apriltag_ros::srv::AnalyzeSingleImage::Response> response);

 private:
  std::shared_ptr<TagDetector> tag_detector_;
  rclcpp::Service<apriltag_ros::srv::AnalyzeSingleImage>::SharedPtr single_image_analysis_service_;
  rclcpp::Publisher<AprilTagDetectionArray>::SharedPtr tag_detections_publisher_;
};

} // namespace apriltag_ros

#endif // APRILTAG_ROS_SINGLE_IMAGE_DETECTOR_H
