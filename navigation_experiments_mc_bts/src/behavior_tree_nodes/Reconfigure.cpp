// Copyright 2020 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <memory>

#include "navigation_experiments_mc_bts/behavior_tree_nodes/Reconfigure.hpp"
using namespace std::chrono_literals;

namespace navigation_experiments_mc_bts
{

Reconfigure::Reconfigure(
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BT::AsyncActionNode(action_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  client_ = node_->create_client<system_modes::srv::ChangeMode>("/pilot/change_mode");
}

BT::NodeStatus Reconfigure::tick()
{
  auto new_mode = getInput<std::string>("mode").value();
  RCLCPP_INFO(node_->get_logger(), "Reconfiguring system mode to %s", new_mode.c_str());
  if (reconfigure_srv_call(new_mode))
  {
    rclcpp::Rate(1.0).sleep();
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }
}
  

bool Reconfigure::reconfigure_srv_call(std::string new_mode) 
{
  auto request = std::make_shared<system_modes::srv::ChangeMode::Request>();

  request->mode_name = new_mode;
  
  while (!client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return false;
    }
    RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
  }

  auto result = client_->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node_, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(node_->get_logger(), "System mode correctly changed");
    return true;

  } 
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service change_mode");
    return false;
  }

}

}  // namespace navigation_experiments_mc_bts

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<navigation_experiments_mc_bts::Reconfigure>(
        name, config);
    };

  factory.registerBuilder<navigation_experiments_mc_bts::Reconfigure>(
    "Reconfigure", builder);
}
