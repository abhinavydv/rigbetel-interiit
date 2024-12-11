import sys
import os
import geometry_msgs.msg
from nav_msgs.msg import Odometry, OccupancyGrid
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from message_filters import ApproximateTimeSynchronizer, Subscriber
from threading import Lock
import copy


class MapMerge(Node):
    
    def __init__(self):
        super().__init__('map_merge')

        self.num_robots = 1
        if "NUM_ROBOTS" in os.environ:
            self.num_robots = int(os.environ["NUM_ROBOTS"])

        self.complete_map: OccupancyGrid = None

        self.subs: list[ApproximateTimeSynchronizer] = []
        for i in range(self.num_robots):
            og = Subscriber(self, OccupancyGrid, f'/tortoisebot_simple_{i}/map')
            odom = Subscriber(self, Odometry, f'/tortoisebot_simple_{i}/odom')
            self.subs.append(ApproximateTimeSynchronizer([og, odom], 10, 0.1))
            self.subs[-1].registerCallback(self.map_callback, i)

        self.pub = self.create_publisher(OccupancyGrid, '/complete_map', 10)

        self.map_lock = Lock()

    def map_callback(self, og: OccupancyGrid, odom: Odometry, index):
        with self.map_lock:
            print(index, og.info.height, og.info.width)
            # print(og.data)
            # img = np.array(og.data, dtype=np.int16).reshape((og.info.height, og.info.width))
            # print(img, img.dtype)
            # for i in range(og.info.width):
            #     for j in range(og.info.height):
            #         if img[j, i] != -1:
            #             img[j, i] = 255 - int(img[j, i] * 2.55)
            #         else:
            #             img[j, i] = 200
            # # img = cv2.flip(img, 0)
            # # print(img)
            # cv2.imwrite(f'map_{index}.png', img)
            if self.complete_map is None:
                self.complete_map = og
            else:
                new_map = OccupancyGrid()
                new_map.header = og.header
                new_map.info = copy.deepcopy(og.info)
                new_map.info.origin.position.x = min(self.complete_map.info.origin.position.x, og.info.origin.position.x)
                new_map.info.origin.position.y = min(self.complete_map.info.origin.position.y, og.info.origin.position.y)
                new_map.info.width = int(max(self.complete_map.info.width + self.complete_map.info.origin.position.x, og.info.width + og.info.origin.position.x) - new_map.info.origin.position.x)
                new_map.info.height = int(max(self.complete_map.info.height + self.complete_map.info.origin.position.y, og.info.height + og.info.origin.position.y) - new_map.info.origin.position.y)
                new_map.info.resolution = max(self.complete_map.info.resolution, og.info.resolution)
                new_map.data = [-1] * new_map.info.width * new_map.info.height
                # print(new_map.info.width, new_map.info.height, new_map.info.origin.position.x, new_map.info.origin.position.y)
                # print(self.complete_map.info.width, self.complete_map.info.height, self.complete_map.info.origin.position.x, self.complete_map.info.origin.position.y)
                # print(og.info.width, og.info.height, og.info.origin.position.x, og.info.origin.position.y)
                for i in range(og.info.width):
                    for j in range(og.info.height):
                        # print(len(og.data), i, j, og.info.height, og.info.width)
                        # print(i + j * og.info.width)
                        if og.data[i + j * og.info.width] != -1:
                            new_map.data[int((i + og.info.origin.position.x - new_map.info.origin.position.x) + (j + og.info.origin.position.y - new_map.info.origin.position.y) * new_map.info.width)] = (new_map.data[int((i + og.info.origin.position.x - new_map.info.origin.position.x) + (j + og.info.origin.position.y - new_map.info.origin.position.y) * new_map.info.width)] + og.data[i + j * og.info.width]) // 2

                for i in range(self.complete_map.info.width):
                    for j in range(self.complete_map.info.height):
                        if self.complete_map.data[i + j * self.complete_map.info.width] != -1:
                            new_map.data[int((i + self.complete_map.info.origin.position.x - new_map.info.origin.position.x) + (j + self.complete_map.info.origin.position.y - new_map.info.origin.position.y) * new_map.info.width)] = (new_map.data[int((i + self.complete_map.info.origin.position.x - new_map.info.origin.position.x) + (j + self.complete_map.info.origin.position.y - new_map.info.origin.position.y) * new_map.info.width)] + self.complete_map.data[i + j * self.complete_map.info.width]) // 2

                self.complete_map = new_map

            self.pub.publish(self.complete_map)

def main(args=None):
    rclpy.init(args=args)

    mm = MapMerge()

    rclpy.spin(mm)

    mm.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
