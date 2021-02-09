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

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "behaviortree_cpp_v3/blackboard.h"

#include "ros2_knowledge_graph/GraphNode.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "gtest/gtest.h"

using namespace std::placeholders;

class MoveServer : public rclcpp::Node
{
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ServerGoalHandle<NavigateToPose>;

public:
  MoveServer()
  : Node("move_server") {}

  void start_server()
  {
    move_action_server_ = rclcpp_action::create_server<NavigateToPose>(
      shared_from_this(),
      "navigate_to_pose",
      std::bind(&MoveServer::handle_goal, this, _1, _2),
      std::bind(&MoveServer::handle_cancel, this, _1),
      std::bind(&MoveServer::handle_accepted, this, _1));
  }

  geometry_msgs::msg::PoseStamped goal_;

private:
  rclcpp_action::Server<NavigateToPose>::SharedPtr move_action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const NavigateToPose::Goal>)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleNavigateToPose>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
  {
    std::thread{std::bind(&MoveServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
  {
    auto feedback = std::make_shared<NavigateToPose::Feedback>();
    auto result = std::make_shared<NavigateToPose::Result>();

    goal_ = goal_handle->get_goal()->pose;

    rclcpp::Rate rate(1);
    int counter = 0;
    while (counter++ < 5) {
      RCLCPP_INFO(get_logger(), "Navigating");
      rate.sleep();
    }
    goal_handle->succeed(result);
  }
};

TEST(pilot_test, main)
{
  auto move_server_node = std::make_shared<MoveServer>();
  move_server_node->start_server();

  bool finished = false;
  std::thread t([&]() {
      while (!finished) {rclcpp::spin_some(move_server_node);}
    });

  // Here should go the content of main() in navigation_experiments_mc_bts.cpp

  std::string pkgpath = ament_index_cpp::get_package_share_directory("navigation_experiments_mc_bts");
  std::string xml_file = pkgpath + "/behavior_trees/bt.xml";

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;
  factory.registerFromPlugin(loader.getOSName("navigation_experiments_mc_bts_deliver_order_bt_node"));
  factory.registerFromPlugin(loader.getOSName("navigation_experiments_mc_bts_interact_with_barman_bt_node"));
  factory.registerFromPlugin(loader.getOSName("navigation_experiments_mc_bts_get_order_bt_node"));
  factory.registerFromPlugin(loader.getOSName("navigation_experiments_mc_bts_navigate_to_barman_bt_node"));
  factory.registerFromPlugin(loader.getOSName("navigation_experiments_mc_bts_navigate_to_client_bt_node"));
  factory.registerFromPlugin(loader.getOSName("navigation_experiments_mc_bts_check_order_bt_node"));

  auto blackboard = BT::Blackboard::create();
  auto node = rclcpp::Node::make_shared("pilot_node");
  blackboard->set("node", node);

  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);

  rclcpp::Rate rate(1);
  auto start = node->now();
  while (rclcpp::ok() && !finished) {
    finished = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;

    rclcpp::spin_some(node);
    rate.sleep();
  }

  ASSERT_TRUE(finished);
  RCLCPP_INFO(node->get_logger(), "Pilot execution finished");

  t.join();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
