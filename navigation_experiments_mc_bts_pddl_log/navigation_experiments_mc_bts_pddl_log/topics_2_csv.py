# Copyright 2019 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
 
import os
import os.path

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import Float64, Empty, Int16
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster, TransformListener, TransformStamped, Buffer
import math

import csv
from datetime import datetime

class Topics2csv(Node):
    def __init__(self, last_contexts=None):
        super().__init__('topics_2_csv')
        self.tfBuffer = Buffer()
        self.listener = TransformListener(self.tfBuffer, self)
        
        self.reconfig_time_sub_ = self.create_subscription(
          Float64,
          "/navigation_experiments_mc_bts_pddl/reconfig_time",
          self.reconfig_time_cb, 1)

        self.vel_sub_ = self.create_subscription(
          Twist,
          "/cmd_vel",
          self.vel_cb, 1)
          
        self.distance_ = 0.0
        self.old_x_ = 0.0
        self.old_y_ = 0.0
        self.robot_x_ = 0.0
        self.robot_y_ = 0.0
        self.reconfig_time_ = 0.0
        self.vel_ = Twist()

        #self.get_logger().info("DF_CLIENT: Ready!")
        self.fieldnames_ = ['time', 'distance', 'robot_x', 'robot_y', 'vel_x', 'vel_theta', 'reconfig_time']

        username = os.environ['USER']
        self.path = "/home/" + username + "/tud_iros2021/csv/run/"
        self.update_csv_file_name()

        with open(self.path + self.csv_filename , mode='w+') as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=self.fieldnames_)
            writer.writeheader()
        timer_period = 0.1 # seconds
        self.create_timer(timer_period, self.step)

    def destroy(self):
        super().destroy_node()

    def get_robot_position(self):
        dur = Duration()
        dur.sec = 3
        dur.nsec = 0
        try:
            trans = self.tfBuffer.lookup_transform('map', 'base_footprint', rclpy.time.Time(seconds=0), dur)
            self.robot_x_ = trans.transform.translation.x
            self.robot_y_ = trans.transform.translation.y
        except:
            print("tf2 exception")
        
        meters = 0.0
        if self.old_x_ != 0.0 and self.old_y_ != 0.0:
            meters = self.calculate_distance(self.robot_x_, self.robot_y_, self.old_x_, self.old_y_)
            self.old_x_ = self.robot_x_
            self.old_y_ = self.robot_y_
            #miles = metersToMiles(meters_);
        else:
            self.old_x_ = self.robot_x_
            self.old_y_ = self.robot_y_
    
        self.distance_ = meters

    def reconfig_time_cb(self, msg):
        self.reconfig_time_ = msg.data
            
    def path_distance_cb(self, msg):
        self.path_distance_ = msg.data

    def vel_cb(self, msg):
        self.vel_ = msg

    def update_csv_file_name(self):
        filename_list = []
        for file in os.listdir(self.path):
            filename_list.append(int(file[:-4]))
        filename_list_sorted = sorted(filename_list)

        if len(filename_list_sorted) == 0:
            self.csv_filename = str(1) + ".csv"
        else: 
            self.last_file_number = filename_list_sorted[-1]
            self.csv_filename = str(self.last_file_number + 1) + ".csv"
            
    def calculate_distance(self, current_x, current_y, old_x, old_y):
        return math.sqrt((pow(current_x - old_x, 2) + pow(current_y - old_y, 2)))

    def step(self):
        self.get_robot_position()
        with open(self.path + self.csv_filename, mode='a+') as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=self.fieldnames_)
            time = self.get_clock().now()
            writer.writerow({
                'time': time.to_msg().sec + (time.to_msg().nanosec / 1000000000),
                'distance': self.distance_,
                'robot_x' : self.robot_x_,
                'robot_y' : self.robot_y_,
                'vel_x': self.vel_.linear.x,
                'vel_theta':self.vel_.angular.z,
                'reconfig_time': self.reconfig_time_})
            self.distance_ = 0.0
            self.path_distance_ = 0.0
            self.scan_min_ = 0.0
            self.reconfig_time_ = 0.0
          
def main(args=None):
    rclpy.init(args=args)
    node = Topics2csv()
    rclpy.spin(node)
    node.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()