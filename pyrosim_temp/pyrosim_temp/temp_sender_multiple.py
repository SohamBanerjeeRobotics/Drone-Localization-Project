#!/usr/bin/env python3
import os
import csv
import glob
import rclpy
import numpy as np
import pandas as pd
from rclpy.node import Node
from std_msgs.msg import Float64  # csv file datatype
from geometry_msgs.msg import Point, PointStamped
from ament_index_python.packages import get_package_share_directory



class TempLocalizer(Node):
    def __init__(self):
        super().__init__('temp_sender')

        # Load all the csv files at once
        package_share = get_package_share_directory('pyrosim_temp')
        csv_path = os.path.join(package_share, 'data')
        self.data = self.load_temp_data(csv_path)  # implement load_temp_data()
        
        # Subscribers
        self.target_sub = self.create_subscription(Point, '/target_pos', self.target_callback, 10)
        self.gps_sub = self.create_subscription(PointStamped, '/Crazyflie/gps', self.gps_callback, 10)

        # Publisher
        self.temp_pub = self.create_publisher(Float64, '/temp_data', 10)

        # Initialize storage
        self.current_pos = None
        self.target_pos = None
        self.threshold = 0.01  # set your error threshold

    def load_temp_data(self, file_path):
        files = glob.glob(path + "/temp_*.csv")
        data = {}
        
        for f in files:
            ts = int(re.findall(r"temp_(\d+)\.csv", f)[0])  # extract timestamp
            df = pd.read_csv(f)
            data[ts] = df
        return dict(sorted(data.items()))
    
    def get_interpolated_temp(self, Time, Target_Pos):
        T0, T1 = self.find_bracketing_times(self.data, Time)
        Temp_T0 = self.get_temperature(self.data[T0], Target_Pos)
        Temp_T1 = self.get_temperature(self.data[T1], Target_Pos)
        
        if T0 == T1:
            return Temp_T0
        alpha = (Time - T0) / (T1 - T0)
        return (Temp_T1 * alpha) + (Temp_T0 * (1 - alpha))
        
    def find_bracketing_times(self, Data, Time):
        times = list(Data.keys())

        if Time <= times[0]:
            return times[0], times[0]
        if Time >= times[-1]:
            return times[-1], times[-1]

        for i in range(len(times)-1):
            if times[i] <= Time <= times[i+1]:
                return times[i], times[i+1]
    
    def get_temperature(self, Temp_Array, Target_Pos):
        Temp_Array["dist"] = ((Temp_Array["x"] - Target_Pos[0]) ** 2) + ((Temp_Array["y"] - Target_Pos[1]) ** 2) + ((Temp_Array["z"] - Target_Pos[2]) ** 2)
        row = Temp_Array.loc[Temp_Array["dist"].idxmin()]
        return row["temperature"]
        
    def target_callback(self, msg):
        self.target_pos = np.array([msg.x, msg.y, msg.z])

    def gps_callback(self, msg):
        self.current_pos = np.array([msg.point.x, msg.point.y, msg.point.z])
        self.check_and_publish()

    def check_and_publish(self):
        if self.current_pos is None or self.target_pos is None:
            return
        current_time = self.get_clock().now()
        target_pose_temperatue = Float64()
        target_pose_temperature.data = self.get_interpolated_temp(current_time, self.target_pos)
        
        self.temp_pub.publish(target_pose_temperature)
        self.get_logger().info(f"Published: {target_pose_temperature.data}")

def main(args=None):
    rclpy.init(args=args)
    node = TempLocalizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

