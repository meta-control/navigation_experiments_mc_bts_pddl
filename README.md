# navigation_experiments_mc_bts_pddl_base
A tutorial to get the MROS ecosystem fully working to perform the experiments programming a navigation task with metacontrol, with behavior trees and PDDL.

## Table of Contents
1. [Installing MROS Ecosystem](#installing-mros-ecosystem)
2. [Starting with a Turtlebo3 in Gazebo](#starting-with-a-turtlebo3-in-gazebo)
3. [Navigation system](#navigation-launcher)
4. [The metacontroller](#launch-the-mros2-metacontroller)
5. [Patrol mission](#autonomous-navigation-patrol-mission)
6. [MROS managing contingencies](#mros-managing-contingencies)
  - [Laser failure](#laser-failure-management)
  - [Low battery](#low-battery-management)


## Installing MROS Ecosystem
MROS is develop under Ubuntu20.04 and ROS2 Foxy, you can find the ROS2 installation steps and the environment setup [here](https://index.ros.org/doc/ros2/Installation/Foxy/), [here](https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/#colcon) and [here](https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/#create-a-workspace).

The mros_reasoner uses [Owlready2](https://owlready2.readthedocs.io/en/latest/index.html) and Java to handle the ontologies and perform reasoning.
```console
  sudo apt-get install openjdk-13-jre
  pip3 install owlready2
```
We are focus in mobile robots and we are using the navigation2 package.
Fetch, build and install navigation2 stack:

```console
  source /opt/ros/foxy/setup.bash

  sudo apt install ros-foxy-slam-toolbox ros-foxy-gazebo-ros-pkgs python3-vcstool python3-rosdep2 python3-colcon-common-extensions

  cd [ros2_ws]/src
  wget https://raw.githubusercontent.com/meta-control/navigation_experiments_mc_bts_pddl/main/resources.repos
  vcs import < resources.repos
  cd ..
  rosdep install -y -r -q --from-paths src --ignore-src --rosdistro foxy --skip-keys="turtlebot2_drivers map_server astra_camera amcl"
  colcon build --symlink-install
```

## Starting with a Turtlebo3 in Gazebo.
Let's start opening Gazebo with a tb3.
This launcher includes gazebo, pointcloud_to_laser, laser_driver_wrapper, and **[system-modes](https://github.com/micro-ROS/system_modes)**.
The **system_modes mode_manager** takes the modes description from `navigation_experiments_mc_bts_pddl_base/params/pilot_modes.yaml`.

```console
  export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:[ros2_ws]/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models:[ros2_ws]/src/navigation_experiments_mc_bts_pddl/navigation_experiments_mc_bts_pddl_base/worlds/models
  export TURTLEBOT3_MODEL=${TB3_MODEL}
  ros2 launch navigation_experiments_mc_bts_pddl_base tb3_sim_launch.py
```
- After the last command, Gazebo window is opened and you should see a tb3 in a domestic scenario.

## Navigation launcher.
This launcher includes rviz, nav2, amcl, and map-server.

```console
  ros2 launch navigation_experiments_mc_bts_pddl_base nav2_turtlebot3_launch.py
```
- RVIz opens, and the navigation system is waiting for the activation of the laser_driver. This activation will be made automatically by the metacontroller in the next step. It is not necessary to set an initial robot position with the 2D Pose Estimate tool. When the laser_driver is up, the pose will be set automatically.

## Launch the mros2 metacontroller.
This step launches the `mros2_metacontroller`, it launches by default the `kb.owl` ontology and connects to the system_modes created by the pilot_urjc.
- The names of the modes there have been changed to match the `fd` names of the `kb.owl` ontology.
```console
  ros2 launch mros2_reasoner launch_reasoner.launch.py
```
- The reasoner does not have an objective defined at this point. So you will not see anything happenning until you launch the mission in the next step.

## Autonomous navigation. 

![waypoints](resources/waypoints.png)

### Behavior tree patrol mission. 

We have developed a behavior to go through a set of waypoints autonomously. It is implemented using a simple [BehaviorTree](https://github.com/tud-cor/navigation_experiments_mc_bts_pddl_base/blob/main/navigation_experiments_mc_bts/behavior_trees/bt.xml)

```console
  ros2 launch navigation_experiments_mc_bts bt_controller_launch.py
```

### PDDL patrol mission. 

In this case the controller is implemented using [pddl](https://github.com/tud-cor/navigation_experiments_mc_bts_pddl_base/blob/main/navigation_experiments_mc_pddl/pddl/patrol_w_recharge.pddl)

```console
  ros2 launch navigation_experiments_mc_pddl pddl_controller_launch.py
  ros2 run navigation_experiments_mc_pddl patrolling_controller_node
```

## MROS managing contingencies.
Currently, we are supporting two contingencies, a laser sensor failure and battery low.

### Laser failure management.

#### Simulating a laser failure.

We have develop a RVIz tool to simulate a laser failure and its consequences. It injects all-zero laser messages in the system and forces the laser_wrapper to switch to error_processing state.

![rviz_cont_tool](resources/contingency_tool.png)

#### Managing the laser failure.

The [mros_modes_observer](https://github.com/MROS-RobMoSys-ITP/mros_modes_observer) package is used to monitor the status of the components (i.e. laser or other sensors) by subscribing to the [`[component_node]/transition_event`](https://github.com/ros2/rcl_interfaces/blob/master/lifecycle_msgs/msg/TransitionEvent.msg).

When the laser failure is detected, a message is sent to the metacontroller using the `/diagnostic` topic. 

```console
mros2_reasoner_node-1] [INFO] [1603183654.050846253] [mros2_reasoner_node]: Entered timer_cb for metacontrol reasoning
[mros2_reasoner_node-1] [INFO] [1603183654.052244011] [mros2_reasoner_node]:   >> Started MAPE-K ** Analysis (ontological reasoning) **
[mros2_reasoner_node-1] [WARN] [1603183654.625280278] [mros2_reasoner_node]: QA value received for	 	TYPE: laser_resender	VALUE: false
[mros2_reasoner_node-1] [INFO] [1603183654.781046611] [mros2_reasoner_node]: QA value received!	TYPE: laser_resender	VALUE: false
[mros2_reasoner_node-1] [INFO] [1603183654.781165253] [mros2_reasoner_node]:      >> Finished ontological reasoning)
```

- The metacontroller then sets all the modes that use this component as not realisable. This is done through `fd_realisability` with value `false`.
- If the current mode is using this component, a reconfiguration is trigger. This is because the current mode is a function grounding of a not realisable function design.
- The metacontroller searchs the for a new mode that does not use the component in error, for this pilot it's only the `f_degraded_mode`.

### Low battery management.

#### Simulating the battery drining.

The battery of the robot is drining based on the movements of the robot. tb3_sim window shows the battery level (1.0 - 0.0). This battery level represents the Energy QA for the metacontroller.

- The battery level is sent to the metacontroller as a QA value using the `/diagnostic` topic (See https://github.com/ros2/common_interfaces/blob/foxy/diagnostic_msgs/msg/DiagnosticArray.msg)

```
mros2_reasoner_node-1] [INFO] [1603183654.050846253] [mros2_reasoner_node]: Entered timer_cb for metacontrol reasoning
[mros2_reasoner_node-1] [INFO] [1603183654.052244011] [mros2_reasoner_node]:   >> Started MAPE-K ** Analysis (ontological reasoning) **
[mros2_reasoner_node-1] [WARN] [1603183654.625280278] [mros2_reasoner_node]: QA value received for	 	TYPE: energy	VALUE: 0.493473
[mros2_reasoner_node-1] [INFO] [1603183654.781046611] [mros2_reasoner_node]: QA value received!	TYPE: energy	VALUE: 0.493473
[mros2_reasoner_node-1] [INFO] [1603183654.781165253] [mros2_reasoner_node]:      >> Finished ontological reasoning)
```

- When the battery goes over a threshold `0.5`  a reconfiguration is required. The battery_contingency_node sents a diagnostics msg to the metacontroller advertising that the battery is not enough.
- The metacontroller searchs the for a new mode, and marks the current one as failed.

```
[mros2_reasoner_node-1] [INFO] [1613476795.519762331] [mros2_reasoner_node]:   >> Started MAPE-K ** Analysis (ontological reasoning) **
[mros2_reasoner_node-1] [WARN] [1613476796.161156157] [mros2_reasoner_node]: Objective e51f829f591afb02b19efe375bf2f90 in status: IN_ERROR_COMPONENT
[mros2_reasoner_node-1] [INFO] [1613476796.161521288] [mros2_reasoner_node]:   >> Started MAPE-K ** PLAN adaptation **
[mros2_reasoner_node-1] [INFO] [1613476796.161860404] [mros2_reasoner_node]:   >> Reasoner searches an FD 

```

- The new mode is changed using the `/change mode` service provided by the system modes. 

```
[mros2_reasoner_node-1] WARNING:root:			 == Obatin Best Function Design ==
[mros2_reasoner_node-1] WARNING:root:== FunctionDesigns AVAILABLE: ['f_degraded_mode', 'f_energy_saving_mode', 'f_normal_mode', 'f_performance_mode', 'f_slow_mode']
[mros2_reasoner_node-1] WARNING:root:== FunctionDesigns REALISABLE: ['f_energy_saving_mode']
[mros2_reasoner_node-1] WARNING:root:== FunctionDesigns NOT IN ERROR LOG: ['f_energy_saving_mode']
[mros2_reasoner_node-1] WARNING:root:== FunctionDesigns also meeting NFRs: ['f_energy_saving_mode']
[mros2_reasoner_node-1] WARNING:root:== Utility for f_energy_saving_mode : 0.300000
[mros2_reasoner_node-1] WARNING:root:			 == Best FD available f_energy_saving_mode
[mros2_reasoner_node-1] [INFO] [1613476796.162774887] [mros2_reasoner_node]:   >> Started MAPE-K ** EXECUTION **
[mros2_reasoner_node-1] [WARN] [1613476796.163110077] [mros2_reasoner_node]: New Configuration requested: f_energy_saving_mode
[mros2_reasoner_node-1] [INFO] [1613476796.166859840] [mros2_reasoner_node]: Got Reconfiguration result True
[mros2_reasoner_node-1] [INFO] [1613476796.168155123] [mros2_reasoner_node]: Exited timer_cb after successful reconfiguration - Obj set to None
[mros2_reasoner_node-1] [INFO] [1613476797.016818483] [mros2_reasoner_node]: QA value received!	TYPE: energy	VALUE: 0.473608
[mros2_reasoner_node-1] [INFO] [1613476797.274176134] [mros2_reasoner_node]: Cancel Action Callback!
[mros2_reasoner_node-1] WARNING:root:			 >>> Ontology Status   <<<
[mros2_reasoner_node-1] WARNING:root:
[mros2_reasoner_node-1] 	Component Status:	[('battery', 'FALSE')]
[mros2_reasoner_node-1] WARNING:root:
[mros2_reasoner_node-1] 	FG: fg_f_energy_saving_mode  Status: None  Solves: e51f829f591afb02b19efe375bf2f90  FD: f_energy_saving_mode  QAvalues: [('energy', 0.473608)]
[mros2_reasoner_node-1] WARNING:root:
[mros2_reasoner_node-1] 	OBJECTIVE: e51f829f591afb02b19efe375bf2f90   Status: None   NFRs:  [('energy', 0.5), ('safety', 0.5)]

```

#### Recharging behavior.
Once the `f_energy_saving_mode` is set, the current action should be canceled and the recharge protocol takes action. The robot goes to the recharge station, waits until the battery is recharged, and then it returns to the patrolling mission.
