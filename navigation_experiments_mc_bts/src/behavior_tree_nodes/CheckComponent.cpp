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

#include "navigation_experiments_mc_bts/behavior_tree_nodes/CheckComponent.hpp"
using namespace std::chrono_literals;

namespace navigation_experiments_mc_bts
{

CheckComponent::CheckComponent(
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(action_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

}

BT::NodeStatus CheckComponent::tick()
{
  auto component_map = config().blackboard->get<std::unordered_map<std::string, bool>>("component_map");
  
  std::string requested_component;

  if (!getInput<std::string>("component", requested_component))
  {
    throw BT::RuntimeError("missing required input [component]");
  }

  if (component_map[requested_component] == true)
  {
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }
}
  

}  // namespace navigation_experiments_mc_bts

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<navigation_experiments_mc_bts::CheckComponent>(
        name, config);
    };

  factory.registerBuilder<navigation_experiments_mc_bts::CheckComponent>(
    "CheckComponent", builder);
}
