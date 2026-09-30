#!/usr/bin/env python3

# Simple ROS2 node that reads data from a Radar_MB_1R2T module and publishes
# one LaserScan message per revolution. The packet format is described in
# mb_1r2t_protocol.py.

from lidar_testnode.mb_1r2t_protocol import parse_packets, ScanAssembler
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial

# Packets to inspect before reporting whether the checksum assumption holds
CHECKSUM_REPORT_PACKETS = 200
# Time between attempts to reopen the serial port
RECONNECT_INTERVAL = Duration(seconds=1.0)


class LidarMB1R2T(Node):

    def __init__(self):
        super().__init__('lidar_mb_1r2t')
        self.port = self.declare_parameter('port', '/dev/ttyUSB0').value
        self.baudrate = self.declare_parameter('baudrate', 153600).value
        self.frame_id = self.declare_parameter('frame_id', 'lidar_frame').value
        self.range_min = self.declare_parameter('range_min', 0.01).value
        self.range_max = self.declare_parameter('range_max', 20.0).value
        # Off by default: the checksum layout has not been confirmed on hardware.
        # The node logs how many packets match so it can be checked.
        self.verify_checksum = self.declare_parameter('verify_checksum', False).value
        topic = self.declare_parameter('topic', '/lidar_mb_1r2t/scan').value

        self.lidar_pub = self.create_publisher(LaserScan, topic, 10)
        self.ser = None
        self.last_open_attempt = None
        self.buffer = bytearray()
        self.assembler = ScanAssembler()
        self.scan_start_time = None
        self.packets_seen = 0
        self.checksums_matched = 0

        self.open_serial()
        # Poll the serial port; packets arrive far more often than this
        self.create_timer(0.005, self.poll)
        self.get_logger().info('Lidar started')

    def open_serial(self):
        self.last_open_attempt = self.get_clock().now()
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0)
            self.buffer.clear()
            self.get_logger().info('Opened ' + self.port)
        except serial.SerialException as e:
            self.ser = None
            self.get_logger().error(
                'Could not open ' + self.port + ': ' + str(e),
                throttle_duration_sec=10.0)

    def close_serial(self):
        if self.ser is not None:
            self.ser.close()
            self.ser = None

    def poll(self):
        if self.ser is None:
            if self.get_clock().now() - self.last_open_attempt >= RECONNECT_INTERVAL:
                self.open_serial()
            return
        try:
            waiting = self.ser.in_waiting
            if waiting:
                self.buffer += self.ser.read(waiting)
        except (serial.SerialException, OSError) as e:
            self.get_logger().error('Serial error, reconnecting: ' + str(e))
            self.close_serial()
            return

        for packet in parse_packets(self.buffer):
            self.handle_packet(packet)

    def handle_packet(self, packet):
        self.packets_seen += 1
        if packet.checksum_ok:
            self.checksums_matched += 1
        if self.packets_seen == CHECKSUM_REPORT_PACKETS:
            self.get_logger().info(
                'Checksum matched on %d of %d packets%s' % (
                    self.checksums_matched, self.packets_seen,
                    '' if self.verify_checksum else
                    ' (verification is off, see the verify_checksum parameter)'))
        if self.verify_checksum and not packet.checksum_ok:
            self.get_logger().debug('Dropping packet with bad checksum')
            return

        now = self.get_clock().now()
        if self.scan_start_time is None:
            self.scan_start_time = now
        scan = self.assembler.add_packet(packet)
        if scan is None:
            return
        self.publish_scan(scan, self.scan_start_time, now)
        self.scan_start_time = now

    def publish_scan(self, scan, start_time, end_time):
        msg = LaserScan()
        msg.header.stamp = start_time.to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_increment = scan.angle_increment
        msg.angle_min = 0.0
        msg.angle_max = (len(scan.ranges) - 1) * scan.angle_increment
        msg.scan_time = (end_time - start_time).nanoseconds / 1e9
        msg.time_increment = msg.scan_time / len(scan.ranges)
        msg.range_min = self.range_min
        msg.range_max = self.range_max
        msg.ranges = scan.ranges
        msg.intensities = scan.intensities
        self.lidar_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    lidar = LidarMB1R2T()
    try:
        rclpy.spin(lidar)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        lidar.close_serial()
        lidar.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
