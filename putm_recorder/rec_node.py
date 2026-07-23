#!/usr/bin/env python3

from pathlib import Path
import rclpy
from rclpy.node import Node
from putm_vcl_interfaces.msg import Rtd, AmkActualValues1, AmkActualValues2
import subprocess
import datetime
import os
import yaml

AMK_WHEELS = {
    "/putm_vcl/amk/front/left":  "front_left",
    "/putm_vcl/amk/front/right": "front_right",
    "/putm_vcl/amk/rear/left":   "rear_left",
    "/putm_vcl/amk/rear/right":  "rear_right",
}


class RecNode(Node):
    def __init__(self):
        super().__init__('rec_node')
        self.subscription = self.create_subscription(Rtd, '/putm_vcl/rtd', self.rtd_callback, 1)
        self.recording = False
        self.process = None
        self.stop_timer = None
        self.current_bag_path = None

        self.amk_error_events: dict[str, list] = {w: [] for w in AMK_WHEELS.values()}

        self.declare_parameter('recording_prefix', 'recording')

        for topic_prefix, wheel in AMK_WHEELS.items():
            self.create_subscription(
                AmkActualValues1,
                f"{topic_prefix}/actual_values1",
                self._make_av1_callback(wheel),
                10,
            )
            self.create_subscription(
                AmkActualValues2,
                f"{topic_prefix}/actual_values2",
                self._make_av2_callback(wheel),
                10,
            )

        self._prev_error: dict[str, bool] = {w: False for w in AMK_WHEELS.values()}
        self._prev_warn:  dict[str, bool] = {w: False for w in AMK_WHEELS.values()}

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

    def _make_av1_callback(self, wheel: str):
        """
        Factory method creating a callback function for AmkActualValues1 topic.
        
        Closure concept: 'wheel' is captured by the inner function scope,
        allowing a single factory to handle all 4 independent wheel topics.
        """
        def callback(msg: AmkActualValues1):
            if not self.recording:
                return
            now_ns = self.get_clock().now().nanoseconds
            status = msg.amk_status

            if status.error and not self._prev_error[wheel]:
                self.amk_error_events[wheel].append({
                    "timestamp_ns": now_ns,
                    "type": "error_flag_set",
                    "error": True,
                    "warn": bool(status.warn),
                    "derating": bool(status.derating),
                })
                self.get_logger().warn(f"AMK {wheel}: ERROR flag raised!")

            if status.warn and not self._prev_warn[wheel]:
                self.amk_error_events[wheel].append({
                    "timestamp_ns": now_ns,
                    "type": "warn_flag_set",
                    "error": bool(status.error),
                    "warn": True,
                    "derating": bool(status.derating),
                })
                self.get_logger().warn(f"AMK {wheel}: WARN flag raised!")

            self._prev_error[wheel] = bool(status.error)
            self._prev_warn[wheel]  = bool(status.warn)
        return callback

    def _make_av2_callback(self, wheel: str):
        """
        Factory method for AmkActualValues2 topic.
        
        Logs active error codes (error_info != 0) along with thermal readings 
        from the motor and IGBT power modules.
        """
        def callback(msg: AmkActualValues2):
            if not self.recording:
                return
            if msg.error_info != 0:
                now_ns = self.get_clock().now().nanoseconds
                last = self.amk_error_events[wheel]
                if not last or last[-1].get("error_info") != msg.error_info:
                    self.amk_error_events[wheel].append({
                        "timestamp_ns": now_ns,
                        "type": "error_info_code",
                        "error_info": int(msg.error_info),
                        "temp_motor_raw":    int(msg.temp_motor),
                        "temp_inverter_raw": int(msg.temp_inverter),
                        "temp_igbt_raw":     int(msg.temp_igbt),
                    })
                    self.get_logger().warn(
                        f"AMK {wheel}: error_info=0x{msg.error_info:04X}"
                    )
        return callback

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
        prefix = self.get_parameter('recording_prefix').get_parameter_value().string_value
        putm_dir = "/home/putm/rosbag"
        fallback_dir = "/tmp/rosbag"

        if os.path.isdir(putm_dir):
            base_dir = putm_dir
        else:
            self.get_logger().warn(f"{putm_dir} not found, using fallback: {fallback_dir}")
            os.makedirs(fallback_dir, exist_ok=True)
            base_dir = fallback_dir

        filename = os.path.join(base_dir, f"{prefix}_{now}")

        cmd = ["ros2", "bag", "record", "-s", "mcap", "-o", filename] + self.topics_to_record

        try:
            self.current_bag_path = filename
            for wheel in AMK_WHEELS.values():
                self.amk_error_events[wheel].clear()
            for wheel in AMK_WHEELS.values():
                self._prev_error[wheel] = False
                self._prev_warn[wheel]  = False

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
            self.append_metadata()

    #Function for appending the info about inverters into metadata.yaml file
    def append_metadata(self):
        if not self.current_bag_path:
            return

        yaml_path = Path(self.current_bag_path) / "metadata.yaml"

        if not yaml_path.exists():
            self.get_logger().warn(f"metadata.yaml not found at {yaml_path}")
            return

        try:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)

            errors_summary = {}
            for wheel, events in self.amk_error_events.items():
                if events:
                    errors_summary[wheel] = events

            total_errors = sum(
                1 for events in self.amk_error_events.values()
                for e in events if e.get("type") in ("error_flag_set", "error_info_code")
            )

            data["putm_session"] = {
                "recorded_at": datetime.datetime.now().isoformat(),
                "any_inverter_error": total_errors > 0,
                "total_error_events": total_errors,
                "inverter_errors": errors_summary if errors_summary else None,
            }

            with open(yaml_path, 'w') as f:
                yaml.dump(data, f, default_flow_style=False, allow_unicode=True)

            if total_errors > 0:
                self.get_logger().warn(
                    f"Session ended with {total_errors} inverter error event(s). "
                    f"Details saved to {yaml_path}"
                )
            else:
                self.get_logger().info(f"metadata.yaml updated — no inverter errors. ({yaml_path})")

        except Exception as e:
            self.get_logger().error(f"Failed to update metadata.yaml: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = RecNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
