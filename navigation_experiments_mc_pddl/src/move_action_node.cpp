// Copyright 2019 Intelligent Robotics Lab
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

#include <math.h>

#include <memory>
#include <string>
#include <map>
#include <algorithm>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "mros2_msgs/action/navigate_to_pose_qos.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;
using NavigateToPoseQos = mros2_msgs::action::NavigateToPoseQos;

class MoveAction : public plansys2::ActionExecutorClient
{
public:
  MoveAction()
  : plansys2::ActionExecutorClient("move", 500ms)
  {    
    geometry_msgs::msg::PoseStamped wp;
    wp.header.frame_id = "/map";
    wp.pose.position.x = 1.0;
    wp.pose.position.y = -1.0;
    wp.pose.position.z = 0.0;
    wp.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(M_PI_2);

    waypoints_["wp1"] = wp;

    wp.pose.position.x = -1.0;
    wp.pose.position.y = 1.0;
    wp.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(M_PI);
    waypoints_["wp2"] = wp;

    wp.pose.position.x = -3.5;
    wp.pose.position.y = 1.0;
    wp.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(M_PI);
    waypoints_["wp3"] = wp;

    wp.pose.position.x = -6.25;
    wp.pose.position.y = 2.66;
    wp.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(-M_PI_2);
    waypoints_["wp4"] = wp;

    wp.pose.position.x = 4.0;
    wp.pose.position.y = -3.0;
    wp.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(0.0);
    waypoints_["wp_control"] = wp;

    using namespace std::placeholders;
    pos_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose",
      10,
      std::bind(&MoveAction::current_pos_callback, this, _1));
    private_node_ = rclcpp::Node::make_shared("pr_move_node");
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>(private_node_);
    sys_issue_detected_ = false;
  }

  void current_pos_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    current_pos_ = msg->pose.pose;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    sys_issue_detected_ = false;
    send_feedback(0.0, "Move starting");
    navigation_action_client_ =
      rclcpp_action::create_client<NavigateToPoseQos>(
      shared_from_this(),
      "navigate_to_pose_qos");

    bool is_action_server_ready = false;
    do {
      RCLCPP_INFO(get_logger(), "Waiting for navigation action server...");

      is_action_server_ready =
        navigation_action_client_->wait_for_action_server(std::chrono::seconds(5));
    } while (!is_action_server_ready);

    RCLCPP_INFO(get_logger(), "Navigation action server ready");

    wp_to_navigate_ = get_arguments()[2];  // The goal is in the 3rd argument of the action
    current_mode_ = get_arguments()[3];  // The mode is in the 4rd argument of the action

    RCLCPP_INFO(get_logger(), "Start navigation to [%s]", wp_to_navigate_.c_str());

    goal_pos_ = waypoints_[wp_to_navigate_];
    navigation_goal_.pose = goal_pos_;
    navigation_goal_.qos_expected.objective_type = "f_navigate"; // should be mros_goal->qos_expected.objective_type = "f_navigate";
    diagnostic_msgs::msg::KeyValue energy_qos;
    energy_qos.key = "energy";
    energy_qos.value = "0.7";
    diagnostic_msgs::msg::KeyValue safety_qos;
    safety_qos.key = "safety";
    safety_qos.value = "0.5";
    navigation_goal_.qos_expected.qos.clear();
    navigation_goal_.qos_expected.qos.push_back(energy_qos);
    navigation_goal_.qos_expected.qos.push_back(safety_qos);
    dist_to_move = getDistance(goal_pos_.pose, current_pos_);

    auto send_goal_options =
      rclcpp_action::Client<NavigateToPoseQos>::SendGoalOptions();

    if (current_mode_ == "f_normal_mode")
    {
      send_goal_options.feedback_callback = [this](
      NavigationGoalHandle::SharedPtr,
      NavigationFeedback feedback) {
        if (feedback->qos_status.selected_mode == "f_energy_saving_mode") 
        {
          problem_expert_->removePredicate(plansys2::Predicate("(battery_enough r2d2)"));
          problem_expert_->addPredicate(plansys2::Predicate("(battery_low r2d2)"));
          problem_expert_->addPredicate(plansys2::Predicate("(robot_at r2d2 wp_aux)"));
          sys_issue_detected_ = true;
          return;
        } 
        else if (feedback->qos_status.selected_mode == "f_degraded_mode")
        {
          problem_expert_->addPredicate(plansys2::Predicate("(robot_at r2d2 wp_aux)"));
          problem_expert_->removePredicate(plansys2::Predicate("(nav_sensor r2d2)"));
          sys_issue_detected_ = true;
        }
        send_feedback(
          std::min(1.0, std::max(0.0, 1.0 - (feedback->distance_remaining / dist_to_move))),
          "Move running");
      };
    } 
    else
    {
      send_goal_options.feedback_callback = [this](
      NavigationGoalHandle::SharedPtr,
      NavigationFeedback feedback) {
        send_feedback(
          std::min(1.0, std::max(0.0, 1.0 - (feedback->distance_remaining / dist_to_move))),
          "Move running");
      };
    }

    send_goal_options.result_callback = [this](auto) {
        finish(true, 1.0, "Move completed");
      };

    future_navigation_goal_handle_ =
      navigation_action_client_->async_send_goal(navigation_goal_, send_goal_options);
    return ActionExecutorClient::on_activate(previous_state);
  }

private:
  double getDistance(const geometry_msgs::msg::Pose & pos1, const geometry_msgs::msg::Pose & pos2)
  {
    return sqrt(
      (pos1.position.x - pos2.position.x) * (pos1.position.x - pos2.position.x) +
      (pos1.position.y - pos2.position.y) * (pos1.position.y - pos2.position.y));
  }

  void do_work()
  {
    if (sys_issue_detected_)
    {
      RCLCPP_WARN(get_logger(), "System issue detected, cancelling move action ...");
      finish(false, 0.0, "System issue detected");
      navigation_action_client_->async_cancel_all_goals();
    }
  }

  std::map<std::string, geometry_msgs::msg::PoseStamped> waypoints_;

  using NavigationGoalHandle =
    rclcpp_action::ClientGoalHandle<NavigateToPoseQos>;
  using NavigationFeedback =
    const std::shared_ptr<const NavigateToPoseQos::Feedback>;

  rclcpp_action::Client<NavigateToPoseQos>::SharedPtr navigation_action_client_;
  std::shared_future<NavigationGoalHandle::SharedPtr> future_navigation_goal_handle_;
  NavigationGoalHandle::SharedPtr navigation_goal_handle_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pos_sub_;
  geometry_msgs::msg::Pose current_pos_;
  geometry_msgs::msg::PoseStamped goal_pos_;
  NavigateToPoseQos::Goal navigation_goal_;

  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<rclcpp::Node> private_node_;
  double dist_to_move;
  std::string wp_to_navigate_, current_mode_;
  bool sys_issue_detected_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "move"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
