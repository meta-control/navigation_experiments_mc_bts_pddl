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

#ifndef navigation_experiments_mc_bts__BEHAVIOR_TREE_NODES__CHECKCOMPONENT_HPP_
#define navigation_experiments_mc_bts__BEHAVIOR_TREE_NODES__CHECKCOMPONENT_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

namespace navigation_experiments_mc_bts
{

class CheckComponent : public BT::SyncActionNode
{
public:
  explicit CheckComponent(
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return { 
      BT::InputPort<std::string>("component") 
    };

  }
  std::string component_name_;


private:
  rclcpp::Node::SharedPtr node_;
  
};

}  // namespace navigation_experiments_mc_bts

#endif  // navigation_experiments_mc_bts__BEHAVIOR_TREE_NODES__CHECKCOMPONENT_HPP_
