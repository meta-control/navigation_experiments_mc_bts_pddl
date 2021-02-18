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

#ifndef navigation_experiments_mc_bts__BEHAVIOR_TREE_NODES__RECONFIGURE_HPP_
#define navigation_experiments_mc_bts__BEHAVIOR_TREE_NODES__RECONFIGURE_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "system_modes/srv/change_mode.hpp"

#include "rclcpp/rclcpp.hpp"

namespace navigation_experiments_mc_bts
{

class Reconfigure : public BT::AsyncActionNode
{
public:
  explicit Reconfigure(
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return { 
      BT::InputPort<std::string>("mode") 
    };

  }

  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    BT::PortsList basic = {
      BT::InputPort<std::chrono::milliseconds>("server_timeout")
    };
    basic.insert(addition.begin(), addition.end());

    return basic;
  }
private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<system_modes::srv::ChangeMode>::SharedPtr client_;
  
  bool reconfigure_srv_call(std::string new_mode);
};

}  // namespace navigation_experiments_mc_bts

#endif  // navigation_experiments_mc_bts__BEHAVIOR_TREE_NODES__RECONFIGURE_HPP_
