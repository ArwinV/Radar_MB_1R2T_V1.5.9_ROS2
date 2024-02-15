#!/usr/bin/env python3

# Simple ROS2 node that reads data from a Radar_MB_1R2T module
# The device outputs serial data with a baud rate of 152600
# The data packets are built as follows:
# [start1 0xAA] [start2 0x55] 
# [type 0x28] 
# [Length] 
# [start_angleL] [start_angleH] 
# [stop_angleL] [stop_angleH] 
# [unknown] [unknown] 
# {data: [quality?] [distanceL] [distanceH]}

import rclpy
from rclpy.node import Node
from enum import Enum
import serial
from sensor_msgs.msg import LaserScan
import math
import time
import numpy as np

# States
class State(Enum):
    START = 0
    HEADER = 1
    DATA = 2

# Lidar object
class Lidar_MB_1R2T(Node):
    def __init__(self):
        super().__init__("lidar_mb_1r2t")
        self.count = 0
        # Create publisher
        self.lidar_pub = self.create_publisher(LaserScan, "/lidar_mb_1r2t/scan", 10)
        # Open serial
        self.ser = serial.Serial("/dev/ttyUSB0", 153600, timeout=0.1)
        time.sleep(1)

        # Print log message
        self.get_logger().info("Lidar started")

    def read_packet(self):
        # Read packet start bytes, header and data
        state = State.START
        run = True
        while run:
            if state == State.START:
                # Read bytes until start of packet is read (0xAA55)
                data = self.ser.read(1)
                if data[0] == 0xAA:
                    data = self.ser.read(1)
                    if data[0] == 0x55:
                        #self.get_logger().info("Start sequence detected")
                        # Next bytes will be the header
                        state = State.HEADER
            if state == State.HEADER:
                # Read 6 of the 8 header bytes (meaning of last two bytes are unknown)
                data = self.ser.read(8)
                packet_type = data[0]
                data_length = int(data[1])
                # Determine rest of data if packet is data
                #self.get_logger().info("Data type: " + str(packet_type) + " Data length: " + str(data_length))
                if (packet_type == 64) and (data_length > 1):

                    start_angle = float(((data[3] << 8) + int(data[2]))/128.0)
                    stop_angle = float(((data[5] << 8) + int(data[4]))/128.0)
                    # Find angle per sample
                    if stop_angle > start_angle:
                        delta_angle = stop_angle - start_angle
                    else:
                        delta_angle = 0x168 - start_angle + stop_angle
                    delta_angle_per_sample = delta_angle / (data_length-1)
                    # Next bytes will be the data
                    state = State.DATA
                else:
                    # Start again
                    state = State.START
            if state == State.DATA:
                # Read all datapoints, each having 3 bytes (quality?, dataL, dataH)
                data = self.ser.read(data_length*3)
                quality = []
                distance = []
                for i in range(0,data_length):
                    quality.append(data[3*i])
                    distance.append(((data[3*i+2] << 8) + data[3*i+1])/1000)

                # Publish lidar data
                scan = LaserScan()
                scan.header.stamp = self.get_clock().now().to_msg()
                #scan.header.seq = self.count
                scan.header.frame_id = "lidar_frame"
                scan.angle_min = start_angle * math.pi/(180)
                scan.angle_max = stop_angle * math.pi/(180)
                scan.angle_increment = delta_angle_per_sample * math.pi/(180)
                #scan.time_increment = 0
                #scan.scan_time = 0
                scan.range_min = 0.01
                scan.range_max = 20.0
                scan.ranges = distance
                scan.intensities = quality
                self.lidar_pub.publish(scan)
                run = False
                #self.get_logger().info("Packet read")

                #self.count += 1
                state = State.START



def main(args=None):
    # Initialize communications
    rclpy.init(args=args)
    # Create node
    lidar = Lidar_MB_1R2T()
    while(True):
        lidar.read_packet()

    # Shutdown communications
    rclpy.shutdown()
