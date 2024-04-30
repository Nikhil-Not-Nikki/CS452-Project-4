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
import std_srvs.srv
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header
from std_msgs.msg import ColorRGBA
from rclpy.clock import Clock


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')

        self.subscription = self.create_subscription(
            Pose2D,
            '/pose',
            self.listener_callback,
            1)
        self.subscription  # prevent unused variable warning

        # Establish publisher on cmd_vel channel
        # This will move the robot
        self.publisher = self.create_publisher(Marker, '/visualization_marker', 10)

        # Establish client of the reset service, and call it on initialization to clear
        # any previous drawing and reset the position of the robot
        self.reset_client = self.create_client(std_srvs.srv.Empty, 'reset')
        self.reset_req = std_srvs.srv.Empty.Request()
        self.reset_client.call_async(self.reset_req)

        self.observation_grid = [ [0]*50 for i in range(50) ]
        self.observation_count = 0

        # Establish client of the set_pen service. This will allow us to chose when the robot
        # draws along its path.
        #self.pen_client = self.create_client(std_srvs.srv.SetBool, 'set_pen')

        # Create a timer that triggers a callback function every 0.1 seconds
        #timer_period = 0.1  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        #self.i = 0
    
    def listener_callback(self, msg):
        #pos = -1
        #for i in range (len(msg.data)):
        #    if i % 50 == 0:
        #        pos += 1
        #    observation_grid[pos][i % 50] = msg.data[i]
        #
        #map_type = String()

        #self.publisher.publish(msg)

        x = msg.x
        y = msg.y

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
        marker.id = self.observation_count
        self.observation_count += 1

        # Set the scale of the marker
        new_scale = Vector3()
        new_scale.x = 0.5
        new_scale.y = 0.5
        new_scale.z = 0.5

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

    """def timer_callback(self):
        # This array contains all the actions the robot needs to take to draw the A&M logo
        # Each element is a triple in which the first element is the x velocity of the robot,
        # the second element is the angular velocity of the robot, and the third element is a
        # boolean that tells us if the pen should be set to draw or not.
        twist_arr = [(-5.0, 0.0, False), (0.0, -1.5708, False), (3.2179, 0.0, False), (0.0, 3.14159, True), (0.92738, 0.0, True),
                     (0.0, -1.5708, True), (0.5, 0.0, True), (0.0, 1.1781, True), (2.0, 0.0, True), (0.0, 1.9635, True), (0.15, 0.0, True),
                     (0.0, -1.5708, True), (0.75, 0.0, True), (0.0, -1.5708, True), (1.5517, 0.0, True), # top of A
                     (0.0, -1.5708, True), (0.75, 0.0, True), (0.0, -1.5708, True), (0.15, 0.0, True), (0.0, 1.9635, True), (2.0, 0.0, True),
                     (0.0, 1.1781, True), (0.5, 0.0, True), (0.0, -1.5708, True), (0.92738, 0.0, True), # start of bottom of A
                     (0.0, -1.5708, True), (1.55710, 0.0, True), (0.0, -1.5708, True), (0.9246, 0.0, True), (0.0, -1.5708, True), (0.124, 0.0, True),
                     (0.0, 1.98968, True), (0.2312, 0.0, True), # start of inner
                     (0.0, -0.3316, False), (0.663879, 0.0, False), (0.0, 0.436332, True), (0.6642, 0.0, True),
                     (0.0, 2.0944, True), (0.6642, 0.0, True), (0.0, 2.0944, True), (0.6642, 0.0, True), (0.0, -1.48353, False), (0.663879, 0.0, False), # end of inner
                     (0.0, -1.65806, True), (0.74857, 0.0, True), (0.0, 1.0472, True), (0.2312, 0.0, True), (0.0, 2.0856685, True), (0.124, 0.0, True), 
                     (0.0, -1.5708, True), (0.9246, 0.0, True), (0.0, -1.5708, True), (1.55710, 0.0, True), # end of A
                     (0.0, -3.22886, False), (3.306741, 0.0, False), (0.0, -1.48353, True), (1.42509, 0.0, True), (0.0, 1.5708, True), (3.22867, 0.0, True),
                     (0.0, 1.5708, True), (1.42509, 0.0, True), (0.0, 1.5708, True), (0.81184, 0.0, True), (0.0, -1.5708, True), (5.0, 0.0, True), (0.0, -1.5708, True),
                     (1.61305, 0.0, True), (0.0, -1.5708, True), (0.77586, 0.0, True), (0.0, 1.5708, True), (1.459414, 0.0, True), (0.0, 1.5708, True), (2.06157, 0.0, True),
                     (0.0, 1.5708, True), (7.6065, 0.0, True), # top of T
                     (0.0, 1.5708, True), (2.06157, 0.0, True), (0.0, 1.5708, True), (1.459414, 0.0, True), (0.0, 1.5708, True), (0.77586, 0.0, True), (0.0, -1.5708, True),
                     (1.61305, 0.0, True), (0.0, -1.5708, True), (5.0, 0.0, True), (0.0, -1.5708, True), (0.85184, 0.0, True), # end of T
                     (0.0, -3.01942, False), (2.93181, 0.0, False), (0.0, 1.44862, True), (0.79652, 0.0, True), (0.0, -1.5708, True), (0.32153, 0.0, True), (0.0, 1.5708, True),
                     (1.93482, 0.0, True), (0.0, 1.5708, True), (0.32153, 0.0, True), (0.0, -1.5708, True), (0.65169, 0.0, True), (0.0, -1.5708, True), (1.1412, 0.0, True),
                     (0.0, -1.15192, True), (1.559776, 0.0, True), (0.0, 2.21657, True), (1.6354655, 0.0, True), (0.0, -1.06465, True), (1.12402, 0.0, True), (0.0, -1.5708, True),
                     (0.65169, 0.0, True), (0.0, -1.5708, True), (0.32153, 0.0, True), (0.0, 1.5708, True), (1.93482, 0.0, True), (0.0, 1.5708, True), (0.32153, 0.0, True), 
                     (0.0, -1.5708, True), (0.79652, 0.0, True), (0.0, -1.5708, True), (1.2906552, 0.0, True), (0.0, -1.5708, True), (0.792, 0.0, True), (0.0, -1.5708, True),
                     (0.326052, 0.0, True), (0.0, 1.5708, True), (1.2949828, 0.0, True), (0.0, 2.68781, True), (1.802138, 0.0, True), (0.0, -2.14675, True), (1.87948, 0.0, True),
                     (0.0, 2.60054, True), (1.2568793, 0.0, True), (0.0, 1.5708, True), (0.326052, 0.0, True), (0.0, -1.5708, True), (0.792, 0.0, True), (0.0, -1.5708, True),
                     (1.2906552, 0.0, True), (0.0, 2.687, False), (4.281552, 0.0, False), (0.0, 3.141529, False)]
        
        # avoid index out of bounds
        if self.i < len(twist_arr):
            
            # Create a Twist message propogate it with the proper values from the array
            msg = Twist()
            msg.linear.x = twist_arr[self.i][0]
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.z = twist_arr[self.i][1] * 10

            # Create a request using the pen client we previously made, set the data member using the array, and make the call
            pen_req = std_srvs.srv.SetBool.Request()
            pen_req.data = twist_arr[self.i][2]
            self.pen_client.call_async(pen_req)

            # Publish the Twist message
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "Linear x: %f y: %f z: %f, Angular z: %f"' % (msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z))
            
            # Counter to keep track of our index in the array
            self.i += 1"""


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
