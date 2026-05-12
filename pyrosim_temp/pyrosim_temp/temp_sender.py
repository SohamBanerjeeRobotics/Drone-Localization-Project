#!/usr/bin/env python3
import os
import csv
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64  # csv file datatype
from geometry_msgs.msg import Point, PointStamped
from ament_index_python.packages import get_package_share_directory



class TempLocalizer(Node):
    def __init__(self):
        super().__init__('temp_sender')

        # Load CSV once
        package_share = get_package_share_directory('pyrosim_temp')
        csv_path = os.path.join(package_share, 'data', 'pyrosim_data.csv')
        self.data = self.load_csv(csv_path)  # implement load_csv()
        
        # Subscribers
        self.target_sub = self.create_subscription(Point, '/target_pos', self.target_callback, 10)
        self.gps_sub = self.create_subscription(PointStamped, '/Crazyflie/gps', self.gps_callback, 10)

        # Publisher
        self.temp_pub = self.create_publisher(Float64, '/temp_data', 10)

        # Initialize storage
        self.current_pos = None
        self.target_pos = None
        self.threshold = 0.01  # set your error threshold

    def load_csv(self, file_path):
        with open(file_path, 'r') as f:
            reader = csv.reader(f)
            return np.array(list(reader), dtype = np.float64)

    def target_callback(self, msg):
        self.target_pos = np.array([msg.x, msg.y, msg.z])

    def gps_callback(self, msg):
        self.current_pos = np.array([msg.point.x, msg.point.y, msg.point.z])
        self.check_and_publish()

    def check_and_publish(self):
        if self.current_pos is None or self.target_pos is None:
            return
        ### Need to omit the error calculation part.
        error = np.linalg.norm(self.current_pos - self.target_pos)
        if error < self.threshold:
            # Decide index from target (e.g., x coordinate or some logic)
            matches = np.all(self.target_pos == self.data[:, 0:3], axis = 1)
            idx = np.where(matches)[0][0]
            if idx < len(self.data):
                temp_value = Float64()
                temp_value.data = self.data[idx, 3]
                self.temp_pub.publish(temp_value)
                self.get_logger().info(f"Published: {temp_value.data}")

def main(args=None):
    rclpy.init(args=args)
    node = TempLocalizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

