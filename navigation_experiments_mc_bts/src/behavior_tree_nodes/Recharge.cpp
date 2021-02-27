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

#include "navigation_experiments_mc_bts/behavior_tree_nodes/Recharge.hpp"
using namespace std::chrono_literals;

namespace navigation_experiments_mc_bts
{

Recharge::Recharge(
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BT::AsyncActionNode(action_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  client_ = node_->create_client<std_srvs::srv::Empty>("battery_contingency/battery_charged");
}

BT::NodeStatus Recharge::tick()
{
  while(rclcpp::ok()) {
    RCLCPP_INFO(node_->get_logger(), "Recharging battery for 10 seconds");
    rclcpp::Rate(0.2).sleep();
    srv_call();
    rclcpp::Rate(0.2).sleep();
    break;
  }
  RCLCPP_INFO(node_->get_logger(), "Battery fully recharged");
  auto component_map = 
      config().blackboard->get<std::unordered_map<std::string, bool>>("component_map");

  RCLCPP_ERROR(node_->get_logger(), "Set battery to true");
  
  auto battery_component = component_map.find("battery");
  battery_component->second = true;

  config().blackboard->set("component_map", component_map); 
  return BT::NodeStatus::SUCCESS;
}

void Recharge::srv_call() 
{
  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  while (!client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
  }

  auto result = client_->async_send_request(request);
}

}  // namespace navigation_experiments_mc_bts

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<navigation_experiments_mc_bts::Recharge>(
        name, config);
    };

  factory.registerBuilder<navigation_experiments_mc_bts::Recharge>(
    "Recharge", builder);
}
