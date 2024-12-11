import sys

import geometry_msgs.msg
from nav_msgs.msg import Odometry, OccupancyGrid
import rclpy
from rclpy.node import Node


class PathPlanning(Node):
    
    def __init__(self):
        super().__init__('path_planning')
        self.publisher_ = self.create_publisher(geometry_msgs.msg.Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

        self.complete_map = None

    def odom_callback(self, msg: Odometry):
        if self.complete_map is None:
            return

        # run the bot

    def complete_map_callback(self, msg: OccupancyGrid):
        self.complete_map = msg
