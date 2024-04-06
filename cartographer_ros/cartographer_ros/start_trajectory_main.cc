/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "gflags/gflags.h"
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/node_constants.h"
#include "cartographer_ros/ros_log_sink.h"
#include "cartographer_ros_msgs/srv/start_trajectory.hpp"
#include "cartographer_ros_msgs/srv/finish_trajectory.hpp"
#include "cartographer_ros_msgs/msg/trajectory_options.hpp"
#include "cartographer_ros_msgs/msg/status_code.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include <string>
#include <vector>

using namespace std::chrono_literals;
using std::placeholders::_1;


DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");

/*DEFINE_string(load_state_filename, "",
              "Filename of a pbstream to draw a map from.");


DEFINE_string(initial_pose, "", "Starting pose of a new trajectory");*/

int current_trajectory_id_ = 1;

namespace cartographer_ros {
namespace {

class Cartographer_InitialPose : public rclcpp::Node
{
  public:
    Cartographer_InitialPose() : Node("start_trajectory_main")
    {
      initialPoseSub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10, 
                                                                                                std::bind(&Cartographer_InitialPose::initialpose_callback, this, _1));
      startTrajClient = this->create_client<cartographer_ros_msgs::srv::StartTrajectory>("start_trajectory");
      finishTrajClient = this->create_client<cartographer_ros_msgs::srv::FinishTrajectory>("finish_trajectory");
    }

  private:
    TrajectoryOptions LoadOptions(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose)
    {
      tf2::Quaternion q(pose->pose.pose.orientation.x, 
                        pose->pose.pose.orientation.y,
                        pose->pose.pose.orientation.z,
                        pose->pose.pose.orientation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      std::string initialPoseLua = "{to_trajectory_id = 0, relative_pose = { translation = { "
                                 + std::to_string(pose->pose.pose.position.x) 
                                 + ", "
                                 + std::to_string(pose->pose.pose.position.y)
                                 + ", "
                                 + "0. }, rotation = { 0., 0., "
                                 + std::to_string(yaw)
                                 + ", } } }";
                                 
      auto file_resolver = cartographer::common::make_unique<cartographer::common::ConfigurationFileResolver>(std::vector<std::string>{FLAGS_configuration_directory});
      const std::string code = file_resolver->GetFileContentOrDie(FLAGS_configuration_basename);
      auto lua_parameter_dictionary = cartographer::common::LuaParameterDictionary::NonReferenceCounted(code, std::move(file_resolver));
      auto initial_trajectory_pose_file_resolver = cartographer::common::make_unique<cartographer::common::ConfigurationFileResolver>(std::vector<std::string>{FLAGS_configuration_directory});
      auto initial_trajectory_pose = cartographer::common::LuaParameterDictionary::NonReferenceCounted("return " + initialPoseLua, std::move(initial_trajectory_pose_file_resolver));
      return CreateTrajectoryOptions(lua_parameter_dictionary.get(), initial_trajectory_pose.get(), rclcpp::Node::now());
    }

    void initialpose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose)
    {
      auto finishTraReq = std::make_shared<cartographer_ros_msgs::srv::FinishTrajectory::Request>();
      auto startTrajReq = std::make_shared<cartographer_ros_msgs::srv::StartTrajectory::Request>();

      if(pose->header.frame_id == "map")
      {
        finishTraReq->trajectory_id = current_trajectory_id_++;

        auto finishTrajClientResult = finishTrajClient->async_send_request(finishTraReq);
        // Wait for the result.
        if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), finishTrajClientResult) == rclcpp::FutureReturnCode::SUCCESS)
        {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error finishing trajectory, return.");
          return;
        }

        if(finishTrajClientResult.get()->status.code != cartographer_ros_msgs::msg::StatusCode::OK)
        {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Finish status is failed.");
          return;
        }
        
        // start a new trajectory
        startTrajReq->options = ToRosMessage(LoadOptions(pose));
        startTrajReq->topics.laser_scan_topic = rclcpp::expand_topic_or_service_name(kLaserScanTopic, this->get_name(), this->get_namespace(), true);
        startTrajReq->topics.multi_echo_laser_scan_topic = rclcpp::expand_topic_or_service_name(kMultiEchoLaserScanTopic, this->get_name(), this->get_namespace(), true);
        startTrajReq->topics.point_cloud2_topic = rclcpp::expand_topic_or_service_name(kPointCloud2Topic, this->get_name(), this->get_namespace(), true);
        startTrajReq->topics.imu_topic = rclcpp::expand_topic_or_service_name(kImuTopic, this->get_name(), this->get_namespace(), true);
        startTrajReq->topics.odometry_topic = rclcpp::expand_topic_or_service_name(kOdometryTopic, this->get_name(), this->get_namespace(), true);
        
        auto startTrajClientResult = startTrajClient->async_send_request(startTrajReq);
        // Wait for the result.
        if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), startTrajClientResult) == rclcpp::FutureReturnCode::SUCCESS)
        {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error start trajectory, return.");
          return;
        }

        if(startTrajClientResult.get()->status.code != cartographer_ros_msgs::msg::StatusCode::OK)
        {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "start status is failed.");
          return;
        }
      }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialPoseSub;
    rclcpp::Client<cartographer_ros_msgs::srv::StartTrajectory>::SharedPtr startTrajClient;
    rclcpp::Client<cartographer_ros_msgs::srv::FinishTrajectory>::SharedPtr finishTrajClient;
};
}
}

int main(int argc, char * argv[])
{
  ::rclcpp::init(argc, argv);
  google::AllowCommandLineReparsing();
  google::InitGoogleLogging(argv[0]);
  google::SetUsageMessage(
  "\n\n"
  "Convenience tool around the start_trajectory service. This takes a Lua "
  "file that is accepted by the node as well and starts a new trajectory "
  "using its settings.\n");

  google::ParseCommandLineFlags(&argc, &argv, false);

  CHECK(!FLAGS_configuration_basename.empty())
  << "-configuration_basename is missing.";

  CHECK(!FLAGS_configuration_directory.empty())
  << "-configuration_directory is missing.";

  printf("FLAGS_configuration_basename = %s\n", FLAGS_configuration_basename.c_str());
  printf("FLAGS_configuration_directory = %s\n", FLAGS_configuration_directory.c_str());

  ::rclcpp::spin(std::make_shared<cartographer_ros::Cartographer_InitialPose>());
  ::rclcpp::shutdown();
  return 0;
}