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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "gtest/gtest.h"

TEST(interact_with_barman_test, basic_usage)
{
  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("navigation_experiments_mc_bts_interact_with_barman_bt_node"));

  std::string bt_xml =
    R"(
  <root main_tree_to_execute = "MainTree" >
    <BehaviorTree ID="MainTree">
      <InteractWithBarman name="interact_with_barman"/>
    </BehaviorTree>
  </root>
  )";

  auto blackboard = BT::Blackboard::create();
  auto node = rclcpp::Node::make_shared("bt_node");
  blackboard->set("node", node);

  BT::Tree tree = factory.createTreeFromText(bt_xml, blackboard);

  ros2_knowledge_graph::GraphNode graph("test_graph");
  graph.start();

  graph.add_node(ros2_knowledge_graph::Node{"sonny", "robot"});
  graph.add_node(ros2_knowledge_graph::Node{"table_1", "table"});
  graph.add_node(ros2_knowledge_graph::Node{"bar", "table"});
  graph.add_node(ros2_knowledge_graph::Node{"water", "object"});
  graph.add_node(ros2_knowledge_graph::Node{"toastie", "object"});
  graph.add_node(ros2_knowledge_graph::Node{"mango_juice", "object"});

  graph.add_edge(ros2_knowledge_graph::Edge{"wants", "symbolic", "table_1", "water"});
  graph.add_edge(ros2_knowledge_graph::Edge{"wants", "symbolic", "table_1", "toastie"});
  graph.add_edge(ros2_knowledge_graph::Edge{"wants", "symbolic", "table_1", "mango_juice"});

  auto start = node->now();
  while ((node->now() - start).seconds() < 0.5) {}

  auto finish = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;

  start = node->now();
  while ((node->now() - start).seconds() < 0.5) {}

  ASSERT_TRUE(finish);

  ASSERT_EQ(
    blackboard->get<std::string>("text"),
    std::string("The table_1 wants: mango_juice ,toastie ,water ,"));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
