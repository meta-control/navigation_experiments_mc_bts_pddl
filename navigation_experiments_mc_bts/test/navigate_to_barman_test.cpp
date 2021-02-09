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

#include "nav2_msgs/action/navigate_to_pose.hpp"

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

    goal_handle->succeed(result);
  }
};

TEST(navigate_to_barman_test, basic_usage)
{
  auto move_server_node = std::make_shared<MoveServer>();
  move_server_node->start_server();

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {rclcpp::spin_some(move_server_node);}
    });

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("navigation_experiments_mc_bts_navigate_to_barman_bt_node"));

  std::string bt_xml =
    R"(
  <root main_tree_to_execute = "MainTree" >
    <BehaviorTree ID="MainTree">
       <Sequence name="root_sequence">
           <NavigateToBarman name="navigate_to_barman"/>
       </Sequence>
    </BehaviorTree>
  </root>
  )";

  auto blackboard = BT::Blackboard::create();
  auto node = rclcpp::Node::make_shared("bt_node");
  blackboard->set("node", node);

  BT::Tree tree = factory.createTreeFromText(bt_xml, blackboard);

  auto start = node->now();
  while ((node->now() - start).seconds() < 0.5) {}

  while (!finish) {
    finish = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;
  }

  start = node->now();
  while ((node->now() - start).seconds() < 0.5) {}

  ASSERT_TRUE(finish);

  geometry_msgs::msg::PoseStamped goal;

  goal.pose.position.x = 0.636;
  goal.pose.position.y = 0.545;
  goal.header.frame_id = "map";

  ASSERT_EQ(goal, move_server_node->goal_);

  t.join();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
