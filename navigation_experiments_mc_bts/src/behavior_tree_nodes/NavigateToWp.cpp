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

#include "navigation_experiments_mc_bts/behavior_tree_nodes/NavigateToWp.hpp"


namespace navigation_experiments_mc_bts
{

NavigateToWp::NavigateToWp(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<NavigateToPoseQos>(xml_tag_name, action_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

void NavigateToWp::on_tick()
{
  auto wp_map = 
    config().blackboard->get<std::unordered_map<std::string, geometry_msgs::msg::Pose>>("wp_map");
  
  auto res = getInput<std::string>("goal").value();
  wp_ = wp_map[res];
  RCLCPP_INFO(node_->get_logger(), "Navigating to... [%s -- %f %f]",
    res.c_str(), wp_.position.x, wp_.position.y);
  goal_.pose.pose.position = wp_.position;
  goal_.pose.pose.orientation = wp_.orientation;
  //goal_.pose.pose = wp_;
  goal_.qos_expected.objective_type = "f_navigate"; // should be mros_goal->qos_expected.objective_type = "f_navigate";
  diagnostic_msgs::msg::KeyValue energy_qos;
  energy_qos.key = "energy";
  energy_qos.value = "0.7";
  diagnostic_msgs::msg::KeyValue safety_qos;
  safety_qos.key = "safety";
  safety_qos.value = "0.5";
  goal_.qos_expected.qos.clear();
  goal_.qos_expected.qos.push_back(energy_qos);
  goal_.qos_expected.qos.push_back(safety_qos);
}

void NavigateToWp::on_wait_for_result()
{  
  std::string goal_id = rclcpp_action::to_string(goal_handle_->get_goal_id()); 
  if (!goal_id.compare(feedback_->qos_status.objective_id) == 0){

    RCLCPP_INFO(node_->get_logger(), "goal id and feedback are diferent");
    rclcpp::Rate(1).sleep(); //  Wait for the goal to finish
    return;
  }
  RCLCPP_INFO(node_->get_logger(), "Curr mode: %s ", feedback_->qos_status.selected_mode.c_str()); 
  

  // check selected_mode, f_energy_saving_mode means the robot needs battery.
  if (feedback_->qos_status.selected_mode == "f_energy_saving_mode" && 
      getInput<std::string>("goal").value() != "recharge_station") {
    RCLCPP_ERROR(node_->get_logger(), "Not enough energy");
    
    auto component_map = 
      config().blackboard->get<std::unordered_map<std::string, bool>>("component_map");

    RCLCPP_ERROR(node_->get_logger(), "Set battery to false");
    auto battery_component = component_map.find("battery");
    battery_component->second = false;
    halt();
    config().blackboard->set("component_map", component_map); 
    result_.code = rclcpp_action::ResultCode::ABORTED;
    goal_result_available_ = true;
  }
  else if (feedback_->qos_status.selected_mode == "f_degraded_mode")
  {
    RCLCPP_ERROR(node_->get_logger(), "Laser Scanner failed");
    auto component_map =
      config().blackboard->get<std::unordered_map<std::string, bool>>("component_map");
    RCLCPP_ERROR(node_->get_logger(), "Set laser to false");
     auto battery_component = component_map.find("laser");
    battery_component->second = false;
    halt();
    config().blackboard->set("component_map", component_map); 
    result_.code = rclcpp_action::ResultCode::ABORTED;
    goal_result_available_ = true;
  }

}

BT::NodeStatus NavigateToWp::on_success()
{
  return BT::NodeStatus::SUCCESS;
}

}  // namespace navigation_experiments_mc_bts

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<navigation_experiments_mc_bts::NavigateToWp>(
        name, "navigate_to_pose_qos", config);
    };

  factory.registerBuilder<navigation_experiments_mc_bts::NavigateToWp>(
    "NavigateToWp", builder);
}
