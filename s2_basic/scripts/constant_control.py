#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# import the message type to use
from std_msgs.msg import Int64 , Bool, Float32

#Twist
from geometry_msgs.msg import Twist


class Heartbeat(Node):
    def __init__(self) -> None:
        # initialize base class (must happen before everything else)
        super().__init__("cmd_vel")
				
        # a heartbeat counter
        self.hb_counter = 0

        # create publisher with: self.create_publisher(<msg type>, <topic>, <qos>)
        # self.hb_pub = self.create_publisher(Twist, "/heartbeat", 10)
        self.hb_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # create a timer with: self.create_timer(<second>, <callback>)
        self.hb_timer = self.create_timer(1.0, self.hb_callback)

        # create subscription with: self.create_subscription(<msg type>, <topic>, <callback>, <qos>)
        self.motor_sub = self.create_subscription(Bool, "/kill", self.health_callback, 10)

    def hb_callback(self) -> None:
        """
        Heartbeat callback triggered by the timer
        """
        # construct heartbeat message
        # msg = Int64()
        # msg.data = self.hb_counter

        msg = Twist()
        msg.linear.x = 1.0
        msg.angular.y = 1.0
        msg.angular.z = 1.0

        # publish heartbeat counter
        self.hb_pub.publish(msg)

        # increment counter
        self.hb_counter += 1

    def health_callback(self, msg: Bool) -> None:
        """
        Sensor health callback triggered by subscription
        """
        # if not msg.data:
        if msg.data == True:
            new_msg = Twist()
            self.hb_pub.publish(new_msg)

            self.get_logger().info("cmd_vel publishing stopped.")
            self.hb_timer.cancel()



if __name__ == "__main__":
    rclpy.init()        # initialize ROS2 context (must run before any other rclpy call)
    node = Heartbeat()  # instantiate the heartbeat node
    rclpy.spin(node)    # Use ROS2 built-in schedular for executing the node
    rclpy.shutdown()    # cleanly shutdown ROS2 context