#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from putm_vcl_interfaces.msg import Rtd
import subprocess
import datetime
import os

class RecNode(Node):
    def __init__(self):
        super().__init__('rec_node')
        self.subscription = self.create_subscription(Rtd, '/putm_vcl/rtd', self.rtd_callback, 1)
        self.recording = False
        self.process = None
        self.stop_timer = None

        self.topics_to_record = [
            "/imu/acceleration",
            "/imu/angular_velocity",
            "/parameter_events",
            "/putm_vcl/amk/front/left/actual_values1",
            "/putm_vcl/amk/front/left/actual_values2",
            "/putm_vcl/amk/front/left/setpoints",
            "/putm_vcl/amk/front/right/actual_values1",
            "/putm_vcl/amk/front/right/actual_values2",
            "/putm_vcl/amk/front/right/setpoints",
            "/putm_vcl/amk/rear/left/actual_values1",
            "/putm_vcl/amk/rear/left/actual_values2",
            "/putm_vcl/amk/rear/left/setpoints",
            "/putm_vcl/amk/rear/right/actual_values1",
            "/putm_vcl/amk/rear/right/actual_values2",
            "/putm_vcl/amk/rear/right/setpoints",
            "/putm_vcl/bms_hv_main",
            "/putm_vcl/bms_lv_main",
            "/putm_vcl/current_sensor",
            "/putm_vcl/dashboard",
            "/putm_vcl/frontbox_data",
            "/putm_vcl/frontbox_driver_input",
            "/putm_vcl/lap_timer",
            "/putm_vcl/pdu_channel",
            "/putm_vcl/pdu_data",
            "/putm_vcl/rtd",
            "/putm_vcl/setpoints",
            "/putm_vcl/state_machine",
            "/putm_vcl/steering_wheel",
            "/putm_vcl/xsens_acceleration",
            "/putm_vcl/xsens_dv",
            "/putm_vcl/xsens_euler_publisher",
            "/putm_vcl/xsens_orientation",
            "/putm_vcl/xsens_position",
            "/putm_vcl/xsens_rate_of_turn",
            "/putm_vcl/xsens_temp",
            "/putm_vcl/xsens_utc",
            "/putm_vcl/xsens_velocity",
            "/rosout",
            "/status",
            "/temperature",
            "/tf",
            "/vectornav/gnss",
            #"/vectornav/raw/imu",
            "/vectornav/velocity_body",
            "/yaw_ref",
        ]

    def rtd_callback(self, msg):
        if msg.state:
            if not self.recording:
                self.get_logger().info("Car is ready. Starting data recording.")
                self.start_recording()
            
            if self.stop_timer:
                self.get_logger().info("Car became ready again before timeout. Canceling stop timer.")
                self.stop_timer.cancel()
                self.stop_timer = None
                
        elif not msg.state and self.recording and not self.stop_timer:
            self.get_logger().info("Car is not ready. Stopping data recording after 20 seconds.")
            self.stop_timer = self.create_timer(20.0, self.timer_callback)

    def timer_callback(self):
        if self.recording:
            self.get_logger().info("20 seconds passed. Stopping recording.")
            self.stop_recording()
        self.stop_timer.cancel()
        self.stop_timer = None

    def start_recording(self):
        now = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        putm_dir = "/home/putm/rosbag"
        fallback_dir = "/tmp/rosbag"

        if os.path.isdir(putm_dir):
            base_dir = putm_dir
        else:
            self.get_logger().warn(f"{putm_dir} not found, using fallback: {fallback_dir}")
            os.makedirs(fallback_dir, exist_ok=True)
            base_dir = fallback_dir

        filename = os.path.join(base_dir, f"recording_{now}")

        cmd = ["ros2", "bag", "record", "-s", "mcap", "-o", filename] + self.topics_to_record
        
        try:
            self.process = subprocess.Popen(cmd)
            self.recording = True
            self.get_logger().info(f"Started recording with PID: {self.process.pid}")
        except Exception as e:
            self.get_logger().error(f"Failed to launch ros2 bag record: {e}")

    def stop_recording(self):
        if self.recording and self.process:
            self.get_logger().info(f"Stopping process (PID: {self.process.pid})...")
            self.process.terminate() 
            self.process.wait()     
            self.recording = False
            self.get_logger().info("Process ros2 bag record completed.")

def main(args=None):
    rclpy.init(args=args)
    node = RecNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()