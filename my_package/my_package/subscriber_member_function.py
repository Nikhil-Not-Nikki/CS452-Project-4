# Copyright 2016 Open Source Robotics Foundation, Inc.
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

import rclpy
import math
import std_srvs.srv
from rclpy.node import Node
from turtlesim.msg import Color
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header
from std_msgs.msg import ColorRGBA
from rclpy.clock import Clock
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData

import numpy as np
import matplotlib.pyplot as plt

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            1)
        self.subscription  # prevent unused variable warning

        self.publisher = self.create_publisher(Marker, '/personal', 10)

        self.second_publisher = self.create_publisher(OccupancyGrid, '/map', 1)

        self.reset_client = self.create_client(std_srvs.srv.Empty, 'reset')
        self.reset_req = std_srvs.srv.Empty.Request()
        self.reset_client.call_async(self.reset_req)

        # grid for probability
        self.observation_grid = [ [0]*50 for i in range(50) ]
        self.observation_counter = 0
        self.max_x = float('-inf')
        self.max_y = float('-inf')
        self.min_x = float('inf')
        self.min_y = float('inf')


    def listener_callback(self, msg):
        ranges = msg.ranges

        item_found = False

        for i in range(len(ranges)):
            angle = msg.angle_min + (i * msg.angle_increment)
            magnitude = ranges[i]
            if ranges[i] > msg.range_min and ranges[i] < msg.range_max:
                x = magnitude * math.cos(angle)
                y = magnitude * math.sin(angle)
                self.get_logger().info('I found "x: %f y: %f\n"' % (x, y))
                item_found = True
                
                marker = Marker()
                new_point = Point()
                new_point.x = x
                new_point.y = y
                new_point.z = 0.0

                # set header
                new_header = Header()
                new_header.frame_id = "/world"
                new_header.stamp = Clock().now().to_msg()
                marker.header = new_header

                # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
                marker.type = 2
                marker.id = i

                # Set the scale of the marker
                new_scale = Vector3()
                new_scale.x = 1.0
                new_scale.y = 1.0
                new_scale.z = 1.0

                marker.scale = new_scale

                # Set the color
                new_color = ColorRGBA()
                new_color.r = 0.0
                new_color.g = 1.0
                new_color.b = 0.0
                new_color.a = 1.0

                marker.color = new_color

                # Set the pose of the marker
                new_pose = Pose()
                new_quat = Quaternion()
                new_quat.x = 0.0
                new_quat.y = 0.0
                new_quat.z = 0.0
                new_quat.w = 0.0

                new_pose.position = new_point
                new_pose.orientation = new_quat

                marker.pose = new_pose

                #marker.points = [new_point]
                self.publisher.publish(marker)

                # set up occupancy grid message
                occupancy_grid = OccupancyGrid()

                occ_header = Header()
                occ_header.frame_id = "/world"
                occ_header.stamp = Clock().now().to_msg()
                occupancy_grid.header = occ_header

                occ_metadata = MapMetaData()
                occ_metadata.map_load_time = Clock().now().to_msg()
                occ_metadata.resolution = 1.0
                occ_metadata.width = 50
                occ_metadata.height = 50

                occ_pose = Pose()
                occ_quat = Quaternion()
                occ_quat.x = 0.0
                occ_quat.y = 0.0
                occ_quat.z = 0.0
                occ_quat.w = 0.0

                occ_point = Point()
                occ_point.x = 0.0
                occ_point.y = 0.0
                occ_point.z = 0.0

                occ_pose.position = new_point
                occ_pose.orientation = occ_quat

                occ_metadata.origin = occ_pose

                occupancy_grid.info = occ_metadata

                # each square on the grid is 0.2 m
                x_cell = math.floor(x / 0.2) + 25
                y_cell = math.floor(y / 0.2) + 25
                
                print("X: ", x_cell, " Y: ", y_cell)

                if (x_cell >= 0 and x_cell < 50) and (y_cell >= 0 and y_cell < 50):
                    self.observation_grid[y_cell][x_cell] = self.observation_grid[y_cell][x_cell] + 1

                one_dim_len = len(self.observation_grid) * len(self.observation_grid[0])
                one_dim_arr = [0] * one_dim_len
                for j in range(len(self.observation_grid)):
                    for k in range(len(self.observation_grid[0])):
                        one_dim_arr.append(self.observation_grid[j][k])

                occupancy_grid.data = one_dim_arr
                
                self.second_publisher.publish(occupancy_grid)

                if x > self.max_x:
                    self.max_x = x
                if y > self.max_y:
                    self.max_y = y
                if x < self.min_x:
                    self.min_x = x
                if y < self.min_y:
                    self.min_y = y

        self.observation_counter = self.observation_counter + 1
        print("observation counter: ", self.observation_counter)
        #print("max X = ", self.max_x)
        #print("max Y = ", self.max_y)
        #print("min X = ", self.min_x)
        #print("min Y = ", self.min_y)

        if self.observation_counter > 40:
            # pixel_plot = plt.figure()
            # pixel_plot.add_axes([0, 0, 50, 50])
            plt.title("Map Representation")
            plt.imshow(self.observation_grid)
            plt.colorbar()
            plt.show()

            #for i in range(len(self.observation_grid)):
                #for j in range(len(self.observation_grid[0])):
                    #print(self.observation_grid[i], end=" ")
                #print(" ")
            





def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
