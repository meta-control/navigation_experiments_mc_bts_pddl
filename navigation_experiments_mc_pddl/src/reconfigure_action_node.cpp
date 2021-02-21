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

#include <memory>
#include <algorithm>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/empty.hpp"
#include "system_modes/srv/change_mode.hpp"

using namespace std::chrono_literals;

class ReconfigureAction : public plansys2::ActionExecutorClient
{
public:
  ReconfigureAction()
  : plansys2::ActionExecutorClient("reconfig_system", 500ms)
  {
    mode_ = "";
    client_ = create_client<system_modes::srv::ChangeMode>("/pilot/change_mode");
  }

private:
  void do_work()
  {
    mode_ = get_arguments()[2];
    RCLCPP_INFO(get_logger(), "Reconfiguring system mode to %s", mode_.c_str());
    if (srvCall())
      finish(true, 1.0, "System reconfigured");
  }

  bool srvCall()
  {
    auto request = std::make_shared<system_modes::srv::ChangeMode::Request>();
    request->mode_name = mode_;
  
    while (!client_->wait_for_service(1s)) 
    {
      if (!rclcpp::ok()) 
      {
        RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
        return false;
      }
      RCLCPP_INFO(get_logger(), "service not available, waiting again...");
    }
    auto result = client_->async_send_request(request);
    return true;
  }

  rclcpp::Client<system_modes::srv::ChangeMode>::SharedPtr client_;
  std::string mode_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ReconfigureAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "reconfig_system"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
