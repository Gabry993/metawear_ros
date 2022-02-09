# _type: ignore

import ctypes
import math
import os
import time
import yaml

from threading import Event
from typing import Dict, Optional, List, Any

import builtin_interfaces.msg
import geometry_msgs.msg
import metawear_ros.msg

# import tf2_geometry_msgs
# import tf_conversions as tfc
# import tf2_ros
# NOTE(Jerome): not available yet for ROS2

import rclpy.node
import rclpy.time
import rclpy.qos
import sensor_msgs.msg
import std_msgs.msg
import std_srvs.srv

# from mbientlab.metawear.cbindings import BaroBoschOversampling, BaroBoschIirFilter
# from mbientlab.metawear.cbindings import LedPattern

from mbientlab.metawear.cbindings import (
    FnVoid_VoidP_DataP,
    FnVoid_VoidP_VoidP,
    FnVoid_VoidP_VoidP_CalibrationDataP,
    FnVoid_VoidP_VoidP_Int,
    SensorFusionData,
    SensorFusionMode,
    SensorFusionGyroRange,
    SensorFusionAccRange
)

from pymetawear import libmetawear
from pymetawear.client import MetaWearClient
from pymetawear.exceptions import PyMetaWearException

try:
    import PyKDL  # noqa

    USE_PYKDL = True
except ImportError:
    try:
        import numpy as np
        import quaternion  # noqa
        USE_PYKDL = False
    except ImportError:
        print("You either need PyKDL or numpy-quaternion")
        exit(1)


def hexint_presenter(dumper: Any, data: int) -> str:
    return dumper.represent_int(hex(data))


yaml.add_representer(int, hexint_presenter)  # type: ignore


def metawear2ros_time(value_ms: int) -> builtin_interfaces.msg.Time:
    return builtin_interfaces.msg.Time(
        sec=int(value_ms // 1e3), nanosec=int(1e6 * (value_ms % 1000))
    )


def ros2metawear_duration(duration_msg: builtin_interfaces.msg.Duration) -> int:
    return duration_msg.nanosec // 1e6 + duration_msg.sec * 1e3


def ros_duration(secs: float) -> rclpy.time.Duration:
    return rclpy.time.Duration(seconds=int(secs), nanoseconds=int((secs % 1) * 1e9))


class MetaWearRos(rclpy.node.Node):  # type: ignore
    """docstring for MetaWearRos"""

    calibration_fields = {"acc", "gyro", "mag"}
    mwc: MetaWearClient

    def __init__(self) -> None:
        super(MetaWearRos, self).__init__("metawear_ros")
        # topic_name -> number of connected peers
        self.peers_count: Dict[str, int] = {}
        self.publish_rate: int = self.declare_parameter("publish_rate", 25).value
        default_frame_id: str = self.get_name().split("/")[-1]
        self.frame_id: str = self.declare_parameter("frame_id", default_frame_id).value
        self.mimic_myo_frame: str = self.declare_parameter("mimic_myo_frame", True).value
        self.address: str = self.declare_parameter("address", "").value.lower()
        self.interface: str = self.declare_parameter("interface", "hci0").value
        self.is_connected = False

        # CHANGED(Jerome): load/save calibration to .ros, not to the package share folder!
        # CHANGED(Jerome): use directly yaml instead of ros parameters
        # DONE(Jerome): do we still need to expose this info as ros params? -> probably not

        config_default_path = os.path.join(
            os.path.expanduser("~"), ".ros", "metawear_ros", "mwear.yaml"
        )
        self.config_path = self.declare_parameter(
            "config_path", config_default_path
        ).value

        self.should_load_config: bool = self.declare_parameter("load_config", True).value
        self.should_save_config: bool = self.declare_parameter("save_config", True).value
        self.load_config()

        rotation_topic = self.declare_parameter("rotation_topic", "~/rotation").value

        # ### Experimental ####
        # joint_states_topic = self.declare_parameter('joint_states_topic', 'joint_states')
        # self.joint_prefix = self.declare_parameter('joint_prefix', 'shoulder_to_wrist')
        ######################

        # ### Experimental. Feb 17, 2019 ####
        # Allows to adjust zero for yaw angle
        # DONE(Jerome): Is this uses/usefull? -> maybe
        self.yaw_origin = 0.0
        self.srv_set_yaw_origin = self.create_service(
            std_srvs.srv.Empty, "~/set_yaw_origin", self.set_yaw_origin
        )
        ######################

        # DONE(Jerome): do I need the tilde? -> yes but it should be ~/

        accel_topic = self.declare_parameter("accel_topic", "~/accel").value
        gyro_topic = self.declare_parameter("gyro_topic", "~/gyro").value
        calib_state_topic = self.declare_parameter(
            "calib_state_topic", "~/calibration_state"
        ).value
        button_topic = self.declare_parameter("button_topic", "~/button").value
        battery_topic = self.declare_parameter("battery_topic", "~/battery_state").value
        temperature_topic = self.declare_parameter(
            "temperature_topic", "~/temperature"
        ).value
        illuminance_topic = self.declare_parameter(
            "illuminance_topic", "~/illuminance"
        ).value
        # air_pressure_topic = self.declare_parameter('air_pressure_topic', 'air_pressure')
        altitude_topic = self.declare_parameter("altitude_topic", "~/altitude").value
        self.give_button_feedback = self.declare_parameter(
            "give_button_feedback", False
        ).value
        self.republish_button: bool = self.declare_parameter("republish_button", True).value
        self.button_val = std_msgs.msg.Bool(data=False)

        vibration_topic = self.declare_parameter(
            "vibration_topic", "~/vibration2"
        ).value
        vibration_pattern_topic = self.declare_parameter(
            "vibration_pattern_topic", "~/vibration_pattern"
        ).value
        led_topic = self.declare_parameter("led_topic", "~/led").value
        # DONE(Jerome): can I have map as param value?
        # -> I can but I have to define them one by one
        led_color = {}
        for channel, value in zip("rgba", (0.0, 1.0, 1.0, 1.0)):
            led_color[channel] = self.declare_parameter(
                f"led_color.{channel}", value
            ).value
        self.default_led_color = std_msgs.msg.ColorRGBA(**led_color)

        # DONE(Jerome): original code used subscriber_listener to count the number of listeners.
        # Does something like this exists for ROS2 or
        # should I regularly check the number of subscribers?
        # -> added update_subscribers_count at 1 Hz
        # DONE(Jerome): add appropriate qos for queue_size=10
        stream_qos = rclpy.qos.QoSProfile(
            depth=10,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
        )
        state_qos = rclpy.qos.QoSProfile(
            depth=1,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub_rotation = self.create_publisher(
            geometry_msgs.msg.QuaternionStamped, rotation_topic, stream_qos
        )
        self.rotation_topic = self.track_topic(rotation_topic)
        # DONE(Jerome): add appropriate qos for queue_size=10
        # DONE(Gabriele): move the JoinState publisher to a separate node
        # self.pub_joint_state = self.create_publisher(
        #   sensor_msgs.msg.JointState, joint_states_topic)
        # self.joint_states_topic = self.track_topic(joint_states_topic)
        # DONE(Jerome): add appropriate qos for queue_size=10
        self.pub_accel = self.create_publisher(
            geometry_msgs.msg.Vector3Stamped, accel_topic, stream_qos
        )
        self.accel_topic = self.track_topic(accel_topic)
        # DONE(Jerome): add appropriate qos for queue_size=10
        self.pub_gyro = self.create_publisher(
            geometry_msgs.msg.Vector3Stamped, gyro_topic, stream_qos
        )
        self.gyro_topic = self.track_topic(gyro_topic)
        # DONE(Jerome): add appropriate qos for queue_size=10
        self.pub_calib_state = self.create_publisher(
            metawear_ros.msg.CalibrationState, calib_state_topic, state_qos
        )
        self.calib_state_topic = self.track_topic(calib_state_topic)
        self.calib_state: Optional[metawear_ros.msg.CalibrationState] = None
        self.calibrated = False
        self.calib_data: Dict[str, List[int]] = {}
        # DONE(Jerome): add appropriate qos for queue_size=10 and latch=(not self.republish_button)
        self.pub_button = self.create_publisher(
            std_msgs.msg.Bool, button_topic, stream_qos if self.republish_button else rclpy.qos.QoSProfile()
        )
        self.button_topic = self.track_topic(button_topic)
        # DONE(Jerome): add appropriate qos for queue_size=10 and latch=True
        self.pub_battery = self.create_publisher(
            sensor_msgs.msg.BatteryState, battery_topic, stream_qos
        )
        self.battery_topic = self.track_topic(battery_topic)
        # DONE(Jerome): add appropriate qos for queue_size=10 and latch=True
        self.pub_temperature = self.create_publisher(
            sensor_msgs.msg.Temperature, temperature_topic, stream_qos
        )
        self.temperature_topic = self.track_topic(temperature_topic)
        # DONE(Jerome): add appropriate qos for queue_size=10
        self.pub_illuminance = self.create_publisher(
            sensor_msgs.msg.Illuminance, illuminance_topic, stream_qos
        )
        self.illuminance_topic = self.track_topic(illuminance_topic)
        self.illuminance = 0.0

        # self.pub_air_pressure = rospy.Publisher(air_pressure_topic, FluidPressure,
        #     subscriber_listener = self, queue_size = 10)
        # self.air_pressure_topic = self.track_topic(air_pressure_topic)

        # DONE(Jerome): add appropriate qos for queue_size=10
        self.pub_altitude = self.create_publisher(
            geometry_msgs.msg.Vector3Stamped, altitude_topic, stream_qos
        )
        self.altitude_topic = self.track_topic(altitude_topic)

        self.sub_vibration = self.create_subscription(
            builtin_interfaces.msg.Duration, vibration_topic, self.vibration_cb, 1
        )
        # DONE(Jerome): add appropriate qos for queue_size=1
        cmd_qos = rclpy.qos.QoSProfile(depth=1)
        self.sub_vibration_pattern = self.create_subscription(
            metawear_ros.msg.VibrationPattern,
            vibration_pattern_topic,
            self.vibration_pattern_cb,
            cmd_qos,
        )

        self.sub_led = self.create_subscription(
            std_msgs.msg.ColorRGBA, led_topic, self.led_cb, cmd_qos
        )
        self.sub_exec_timer = self.create_subscription(
            std_msgs.msg.UInt8, "~/exec_timer", self.exec_timer_cb, cmd_qos
        )
        self.sub_reset = self.create_subscription(
            std_msgs.msg.Bool, "~/reset_board", self.reset_board_cb, cmd_qos
        )

        # DONE(Jerome): Do we really need this partial TF trasform? -> No
        # self.tf_br = tf2_ros.TransformBroadcaster()
        # Workaround: prevent device streams to disconnect ever
        # self.peers_count[self.rotation_topic] = 1
        # self.peers_count[self.button_topic] = 1
        # self.peers_count[self.altitude_topic] = 1
        # self.peers_count[self.calib_state_topic] = 1
        # self.peers_count[self.button_topic] = int(self.give_button_feedback)

        # CHANGED(Jerome): use a single parameter
        self.pause_streaming_if_no_subscribers: bool = self.declare_parameter(
            "pause_streaming_if_no_subscribers", False
        ).value

        # DONE(Jerome): Move to ROS2
        # rospy.on_shutdown(self.disconnect)
        self.connect()

    def track_topic(self, topic_name: str) -> str:
        self.peers_count[topic_name] = 0
        return topic_name

    def update_subscribers_count(self) -> None:
        peers_count = {
            topic: len(self.get_subscriptions_info_by_topic(topic))
            for topic in self.peers_count
        }
        if peers_count != self.peers_count:
            self.update_enabled_streams()
            self.peers_count = peers_count

    def on_disconnect(self, status: Any) -> None:
        self.get_logger().warning(f"Disconnected from {self.address}...")
        if self.is_connected:
            self.disconnect()
            while not self.is_connected and rclpy.ok():
                self.get_logger().warning("Reconnecting...")
                time.sleep(1.0)
                self.connect()

    def connect(self) -> bool:
        if not self.address:
            self.get_logger().error("No address provided")
            return False

        self.get_logger().info(f"Connecting to {self.address} ...")
        self.mwc = MetaWearClient(self.address, self.interface, connect=False)
        self.mwc.mw.on_disconnect = self.on_disconnect
        try:
            self.mwc.connect()
        except Exception as e:
            self.get_logger().error(f"{e} {type(e)}")
            return False

        self.get_logger().info("Connected")
        self.real_mac_address = self.get_real_mac_address()
        self.get_logger().info(f"Real MAC address: {self.real_mac_address!r}")
        time.sleep(0.1)

        try:
            self.get_logger().info("Resetting data processors")
            libmetawear.mbl_mw_metawearboard_tear_down(self.mwc.board)
            time.sleep(1.0)

            self.get_logger().info("Setting BLE parameters...")
            self.mwc.settings.set_connection_parameters(
                min_conn_interval=7.5, max_conn_interval=10, latency=0, timeout=3000
            )
            time.sleep(0.2)

            self.get_logger().info("Configuring sensors...")
            self.get_logger().info("Setting IMU sensor fusion mode")
            self.mwc.sensorfusion.set_mode(SensorFusionMode.SLEEP)
            # TODO(Jerome): espose as a param (and set NDOF as default value)
            self.mwc.sensorfusion.set_mode(SensorFusionMode.IMU_PLUS)
            # self.mwc.sensorfusion.set_mode(SensorFusionMode.NDOF)
            time.sleep(0.5)
            self.get_logger().info(
                f"Setting IMU sensors range to {SensorFusionAccRange._16G} [acc] and "
                f"{SensorFusionGyroRange._2000DPS} [gyro]")
            # TODO(Jerome): espose as params (and set lower default values)
            # self.mwc.sensorfusion.set_acc_range(SensorFusionAccRange._16G)
            # self.mwc.sensorfusion.set_gyro_range(SensorFusionGyroRange._2000DPS)
            time.sleep(0.2)
            # TODO(Jerome): espose as a parameters, or possibly set a target rate (0 = 100 Hz)
            self.get_logger().info("Setting IMU sensor fusion sampling rates")
            self.mwc.sensorfusion.set_sample_delay(SensorFusionData.QUATERNION, 10)
            self.mwc.sensorfusion.set_sample_delay(SensorFusionData.CORRECTED_ACC, 10)
            self.mwc.sensorfusion.set_sample_delay(SensorFusionData.CORRECTED_GYRO, 10)

            # TODO(Jerome): espose as a parameters
            self.get_logger().info("Setting ambient light sensor mode")
            self.mwc.ambient_light.set_settings(
                gain=8, integration_time=50, measurement_rate=50
            )

            # TODO(Jerome): espose as a parameters
            self.get_logger().info("Setting barometer mode")
            self.mwc.barometer.set_altitude_data(status=True)
            self.mwc.barometer.set_settings(
                oversampling="standard", iir_filter="avg_16"
            )

            self.get_logger().info("Setting disconnect event handler")
            dc_event = self.mwc.settings.disconnected_event()
            blink_pattern = self.mwc.led.load_preset_pattern("blink", repeat_count=5)
            blink_pattern.high_intensity = 31
            blink_pattern.low_intensity = 0
            blink_pattern.high_time_ms = 250
            blink_pattern.pulse_duration_ms = 500

            def event_rec_status(context: Any, event: Any, status: Any) -> None:
                if not status:
                    self.get_logger().info("Disconnect event handler: OK")
                else:
                    self.get_logger().error("Disconnect event handler: FAILED")

            self._evt_rec_status_cb = FnVoid_VoidP_VoidP_Int(event_rec_status)

            libmetawear.mbl_mw_event_record_commands(dc_event)
            self.clear_sensors()
            self.mwc.led.write_pattern(blink_pattern, "g")
            self.mwc.led.write_pattern(blink_pattern, "b")
            blink_pattern.delay_time_ms = 250
            self.mwc.led.write_pattern(blink_pattern, "r")
            self.mwc.led.play()
            self.mwc.haptic.start_motor(100, 1000)
            libmetawear.mbl_mw_event_end_record(dc_event, None, self._evt_rec_status_cb)

            self.setup_vibration_timer()

            if self.calib_data:
                self.write_calibration_data(True)

            self.update_enabled_streams()

            # self.current_color = copy.deepcopy(self.default_led_color)
            self.mwc.led.stop_and_clear()
            self.led_cb(self.default_led_color)

        except PyMetaWearException:
            self.get_logger().error(
                "Unable to configure the sensors. Please try to reset the board"
            )
            # self.reset_board_cb(Bool(True))
            return False

        self.is_connected = True
        self.get_logger().info("Ready")
        return True

    def setup_vibration_timer(self) -> None:
        e = Event()

        def timer_created_cb(context: Any, timer: Any) -> None:
            e2 = Event()

            def events_recorded_cb(context: Any, event: Any, status: Any) -> None:
                self.get_logger().info("Timer event recorded")
                e2.set()

            _events_recorded_cb = FnVoid_VoidP_VoidP_Int(events_recorded_cb)

            if timer:
                self.get_logger().info(
                    f"Timer {libmetawear.mbl_mw_timer_get_id(timer)} created"
                )

                libmetawear.mbl_mw_event_record_commands(timer)
                self.mwc.haptic.start_motor(100, 150)
                libmetawear.mbl_mw_event_end_record(timer, None, _events_recorded_cb)

                # libmetawear.mbl_mw_timer_start(timer)

                while rclpy.ok():
                    e2.wait(0.1)
                    if e2.is_set():
                        break
            else:
                self.get_logger().error("Failed to create timer")

            e.set()

        _timer_created_cb = FnVoid_VoidP_VoidP(timer_created_cb)

        libmetawear.mbl_mw_timer_create(
            self.mwc.board, 300, 1, 0, None, _timer_created_cb
        )

        while rclpy.ok():
            e.wait(0.1)
            if e.is_set():
                break

    def get_real_mac_address(self) -> bytes:
        e = Event()

        mac_data_signal = libmetawear.mbl_mw_settings_get_mac_data_signal(self.mwc.board)

        ret_value: Dict[str, bytes] = {"value": b""}  # mutable

        def mac_data_cb(context: Any, pdata: Any) -> None:
            # CHARP = ctypes.POINTER(ctypes.c_char_p)
            value = ctypes.cast(
                pdata.contents.value, ctypes.c_char_p
            ).value
            if value:
                ret_value["value"] = value
            e.set()

        _mac_data_cb = FnVoid_VoidP_DataP(mac_data_cb)
        libmetawear.mbl_mw_datasignal_subscribe(mac_data_signal, None, _mac_data_cb)
        libmetawear.mbl_mw_datasignal_read(mac_data_signal)

        while rclpy.ok():
            e.wait(0.1)
            if e.is_set():
                break

        libmetawear.mbl_mw_datasignal_unsubscribe(mac_data_signal)

        return ret_value["value"]

    def stream(self, topic: str) -> bool:
        return (
            not self.pause_streaming_if_no_subscribers or
            self.peers_count[self.rotation_topic] > 0
        )

    def update_enabled_streams(self) -> None:
        # self.get_logger().info('update_enabled_streams')
        self.mwc.sensorfusion.notifications(
            quaternion_callback=self.mwc_quat_cb
            if self.stream(self.rotation_topic)
            else None,
            corrected_acc_callback=self.mwc_acc_cb
            if self.stream(self.accel_topic)
            else None,
            corrected_gyro_callback=self.mwc_gyro_cb
            if self.stream(self.gyro_topic)
            else None,
            calibration_state_callback=self.mwc_calib_state_cb
            if self.stream(self.calib_state_topic)
            else None,
        )

        self.mwc.switch.notifications(
            self.mwc_switch_cb if self.stream(self.button_topic) else None
        )

        if self.stream(self.battery_topic):
            self.mwc.settings.notifications(self.mwc_battery_cb)
            # self.mwc.settings.read_battery_state()
        else:
            self.mwc.settings.notifications(None)

        if self.stream(self.temperature_topic):
            self.mwc.temperature.notifications(self.mwc_temperature_cb)
            self.mwc.temperature.read_temperature()
        else:
            self.mwc.temperature.notifications(None)

        self.mwc.ambient_light.notifications(
            self.mwc_ambient_light_cb if self.stream(self.illuminance_topic) else None
        )
        self.mwc.barometer.notifications(
            self.mwc_barometer_cb if self.stream(self.altitude_topic) else None
        )

    def clear_sensors(self) -> None:
        self.mwc.sensorfusion.notifications()
        self.mwc.switch.notifications()
        self.mwc.settings.notifications()
        self.mwc.temperature.notifications()
        self.mwc.ambient_light.notifications()
        self.mwc.barometer.notifications()
        self.mwc.led.stop_and_clear()

        for i in range(8):
            timer = libmetawear.mbl_mw_timer_lookup_id(self.mwc.board, i)
            if timer:
                self.get_logger().warning(f"Deleting timer {i}")
                libmetawear.mbl_mw_timer_remove(timer)

    def disconnect(self) -> None:
        if self.is_connected:
            self.is_connected = False
            self.get_logger().info(f"Disconnecting from {self.address} ...")
            self.clear_sensors()
            time.sleep(0.5)
            # self.get_logger().info('Resetting data processors')
            # libmetawear.mbl_mw_metawearboard_tear_down(self.mwc.board)
            # time.sleep(1.0)
            self.mwc.disconnect()
            time.sleep(0.2)

    def set_yaw_origin(self, request: std_srvs.srv.Empty.Request,
                       response: std_srvs.srv.Empty.Response) -> std_srvs.srv.Empty.Response:
        # TODO(Jerome) Is it correct to return None in ROS2?
        if not self.is_connected:
            self.get_logger().warning("Will not set yaw origin because not connected")
            return response
        self.yaw_origin = self.orig_yaw
        self.get_logger().info("Successfully set yaw to zero")
        return response

    def quaternion_from_metawear(self, value: Any) -> List[float]:
        if self.mimic_myo_frame:
            # Make x-axis point towards elbow
            yaw = math.pi / 2
        else:
            # Make x-axis point forward, along the arm
            yaw = -math.pi / 2

        if USE_PYKDL:
            rot = PyKDL.Rotation.Quaternion(value.x, value.y, value.z, value.w)
            orig_rot = rot * PyKDL.Rotation.RPY(0.0, 0.0, yaw)
            corr_rot = PyKDL.Rotation.RPY(0.0, 0.0, -self.yaw_origin) * orig_rot
            # Tait-Bryan convention
            self.orig_yaw, _, _ = orig_rot.GetEulerZYX()
            q = corr_rot.GetQuaternion()
        else:
            q = np.quaternion(value.w, value.x, value.y, value.z)
            q = q * quaternion.from_rotation_vector([0, 0, yaw])
            q = quaternion.from_rotation_vector([0, 0, -self.yaw_origin]) * q
            q = quaternion.as_float_array(q)
            # DONE(J): check that is still Tait-Bryan convention [probably not, as this seems XYZ]
            # quaternion.as_euler_angles is ~ GetEulerZYZ
            # From https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
            # #Tait–Bryan_angles
            # DONE(J): do we really need `yaw_origin`? -> MAYBE
            siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
            cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3])
            self.orig_yaw = math.atan2(siny_cosp, cosy_cosp)
            q = [q[1], q[2], q[3], q[0]]
        return q

    def vector_from_metawear(self, value: Any) -> List[float]:
        # Why is this transform different than the above one?
        if self.mimic_myo_frame:
            yaw = math.pi / 2
        else:
            yaw = 0
        v = [value.x, value.y, value.z]
        if USE_PYKDL:
            if yaw:
                v = PyKDL.Vector(*v)
                w = PyKDL.Rotation.RPY(0.0, 0.0, yaw) * v
                v = list(w)
        else:
            import quaternion

            if yaw:
                q = quaternion.from_rotation_vector([0.0, 0.0, math.pi / 2])
                v = quaternion.rotate_vectors(q, v)
        return v

    def mwc_quat_cb(self, data: Dict) -> None:
        q = self.quaternion_from_metawear(data["value"])
        time_stamp = metawear2ros_time(data["epoch"])

        # DONE(Gabriele): move the JoinState publisher to a separate node
        # j_state = sensor_msgs.msg.JointState()
        # j_state.header.stamp = time_stamp
        # j_state.name = [self.joint_prefix + '_roll', self.joint_prefix + '_pitch',
        #                 self.joint_prefix + '_yaw']
        # j_state.position = [roll, pitch, yaw]
        # self.pub_joint_state.publish(j_state)

        quat = geometry_msgs.msg.QuaternionStamped()
        quat.header.stamp = time_stamp
        quat.header.frame_id = self.frame_id
        quat.quaternion.x = q[0]
        quat.quaternion.y = q[1]
        quat.quaternion.z = q[2]
        quat.quaternion.w = q[3]

        self.pub_rotation.publish(quat)

        # DONE(Jerome): Do we really need this partial TF trasform? -> probably not
        # t = geometry_msgs.msg.TransformStamped()
        # t.header.stamp = time_stamp
        # t.header.frame_id = 'world'
        # t.child_frame_id = self.frame_id + '_world'
        # t.transform.rotation.x = q[0]
        # t.transform.rotation.y = q[1]
        # t.transform.rotation.z = q[2]
        # t.transform.rotation.w = q[3]
        # self.tf_br.sendTransform(t)
        #
        # self.get_logger().info_throttle(1.0, 'quat: {}'.format(data))

    def mwc_acc_cb(self, data: Dict) -> None:
        a = self.vector_from_metawear(data["value"])
        accel = geometry_msgs.msg.Vector3Stamped()
        accel.header.stamp = metawear2ros_time(data["epoch"])
        accel.header.frame_id = self.frame_id
        accel.vector.x = a[0]
        accel.vector.y = a[1]
        accel.vector.z = a[2]
        self.pub_accel.publish(accel)
        # self.get_logger().info_throttle(1.0, 'acc: {}'.format(data))

    def mwc_gyro_cb(self, data: Dict) -> None:
        g = self.vector_from_metawear(data["value"])
        gyro = geometry_msgs.msg.Vector3Stamped()
        gyro.header.stamp = metawear2ros_time(data["epoch"])
        gyro.header.frame_id = self.frame_id
        gyro.vector.x = g[0]
        gyro.vector.y = g[1]
        gyro.vector.z = g[2]
        self.pub_gyro.publish(gyro)
        # self.get_logger().info_throttle(1.0, 'gyro: {}'.format(data))

    def save_config(self) -> None:
        # self.get_logger().info(
        #     f"Writing calibration data {self.calib_data} to {self.config_path}"
        # )
        os.makedirs(os.path.dirname(self.config_path), exist_ok=True)
        try:
            with open(self.config_path, "w") as config_file:
                yaml.dump(self.config, config_file, default_flow_style=None)
        except Exception as e:
            self.get_logger().warning(
                f"Exception {e} while saving config to {self.config_path}"
            )

    def load_config(self) -> None:
        try:
            with open(self.config_path) as config_file:
                config = yaml.load(config_file, Loader=yaml.FullLoader)
        except FileNotFoundError:
            self.get_logger().warning("Config file not found")
            config = {}

        self.config = {k.lower(): v for k, v in config.items()}

        if self.address in self.config:
            dev_conf = self.config[self.address]
            # TODO(Jerome): where does this info come from? -> probably a convenience alias.
            # Maybe should move it elsewhere (so that config contains just calibration)
            if "conn_address" in dev_conf:
                conn_address = dev_conf["conn_address"]
                self.get_logger().warning("Substituting device address from the config")
                self.address = conn_address

            if "calib_data" in dev_conf:
                if self.calibration_fields <= set(dev_conf["calib_data"]):
                    self.calib_data = dev_conf["calib_data"]
                else:
                    self.get_logger().error(
                        f"calib_data is missing one of the fields {self.calibration_fields}"
                    )

    def update_calibration_data(self, wait: bool = False) -> None:
        if wait:
            e = Event()

        def read_calib_data_cb(context: Any, board: Any, pdata: Any) -> None:
            self.get_logger().info(f'read_calib_data_cb: {pdata.contents}')
            for key in self.calibration_fields:
                self.calib_data[key] = list(getattr(pdata.contents, key))

            # calib_data_param = self.config_param + '/' + self.real_mac_address + '/calib_data'
            # DONE(Jerome) ->ROS2 (removed)
            # rosparam.set_param(calib_data_param, str(pdata.contents))
            # DONE(Jerome) ->ROS2
            self.config[self.address] = self.calib_data
            self.save_config()

            # DONE(Jerome): Why was the line above active?
            # I.e., why do we need to write after reading??
            # -> because data is not automatically saved.
            libmetawear.mbl_mw_sensor_fusion_write_calibration_data(board, pdata)
            libmetawear.mbl_mw_memory_free(pdata)
            if wait:
                e.set()

        self._read_calib_data_cb = FnVoid_VoidP_VoidP_CalibrationDataP(
            read_calib_data_cb
        )
        libmetawear.mbl_mw_sensor_fusion_read_calibration_data(
            self.mwc.board, None, self._read_calib_data_cb
        )
        if wait:
            while rclpy.ok():
                e.wait(0.1)
                if e.is_set():
                    break

    def write_calibration_data(self, wait: bool = False) -> None:
        if wait:
            e = Event()
        # Fake read to just create the underlaying C-struct

        def read_calib_data_cb(context: Any, board: Any, pdata: Any) -> None:
            self.get_logger().info("read_calib_data_cb")
            for field in self.calibration_fields:
                setattr(
                    pdata.contents,
                    field,
                    (ctypes.c_ubyte * 10).from_buffer_copy(
                        bytearray(self.calib_data[field])
                    ),
                )

            libmetawear.mbl_mw_sensor_fusion_write_calibration_data(board, pdata)
            libmetawear.mbl_mw_memory_free(pdata)
            self.get_logger().info("Calibration data successfully updated")
            self.calibrated = True
            if wait:
                e.set()

        self.get_logger().info("Updating calibration data from cache...")
        _read_calib_data_cb = FnVoid_VoidP_VoidP_CalibrationDataP(read_calib_data_cb)
        libmetawear.mbl_mw_sensor_fusion_read_calibration_data(
            self.mwc.board, None, _read_calib_data_cb
        )

        if wait:
            while rclpy.ok():
                e.wait(0.1)
                if e.is_set():
                    break

    def mwc_calib_state_cb(self, data: Dict) -> None:
        self.get_logger().info(f"mwc_calib_state_cb {data}")
        calib = metawear_ros.msg.CalibrationState()
        calib.header.stamp = metawear2ros_time(data["epoch"])
        calib.accelerometer = data["value"].accelrometer  # <<-- typo!
        calib.gyroscope = data["value"].gyroscope
        calib.magnetometer = data["value"].magnetometer

        if not self.calibrated:
            if calib.accelerometer == 3 and calib.gyroscope == 3:
                self.calibrated = True
                if self.should_save_config:
                    self.update_calibration_data()

        self.calib_state = calib
        self.pub_calib_state.publish(calib)

        # self.get_logger().info(data)

    def mwc_switch_cb(self, data: Dict) -> None:
        # self.get_logger().info(f"mwc_switch_cb {data}")
        # stamp = epoch2ros_timestamp(data['epoch'])
        self.button_val = std_msgs.msg.Bool(data=bool(data["value"]))

        if self.button_val.data and self.give_button_feedback:
            self.mwc.haptic.start_motor(100, 80)

        if not self.republish_button:
            self.pub_button.publish(self.button_val)

    def mwc_battery_cb(self, data: Dict) -> None:
        # self.get_logger().info(f"mwc_battery_cb {data}")
        stamp = metawear2ros_time(data["epoch"])

        msg = sensor_msgs.msg.BatteryState()
        msg.header.stamp = stamp
        msg.voltage = data["value"].voltage / 1000.0  # Volts
        msg.current = float("nan")
        msg.charge = float("nan")
        msg.capacity = float("nan")
        msg.design_capacity = float("nan")
        msg.percentage = data["value"].charge / 100.0  # [0.0; 1.0]
        msg.power_supply_status = (
            sensor_msgs.msg.BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        )
        msg.power_supply_health = (
            sensor_msgs.msg.BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        )
        msg.power_supply_technology = (
            sensor_msgs.msg.BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        )
        msg.present = True
        # msg.cell_voltage
        msg.location = "inside"
        msg.serial_number = "n/a"

        self.pub_battery.publish(msg)
        # self.get_logger().info('Battery: {}'.format(data))

    def mwc_temperature_cb(self, data: Dict) -> None:
        # self.get_logger().info(f"mwc_temperature_cb {data}")
        stamp = metawear2ros_time(data["epoch"])
        msg = sensor_msgs.msg.Temperature()
        msg.header.stamp = stamp
        msg.temperature = data["value"]  # degrees Celsius
        msg.variance = 0.0  # n/a

        self.pub_temperature.publish(msg)

    def mwc_ambient_light_cb(self, data: Dict) -> None:
        stamp = metawear2ros_time(data["epoch"])
        self.illuminance = data["value"] / 1000.0  # Lux

        msg = sensor_msgs.msg.Illuminance()
        msg.header.stamp = stamp
        msg.illuminance = self.illuminance
        msg.variance = 0.0  # n/a

        self.pub_illuminance.publish(msg)

    def mwc_barometer_cb(self, data: Dict) -> None:
        stamp = metawear2ros_time(data["epoch"])

        # msg = FluidPressure()
        # msg.header.stamp = stamp
        # msg.fluid_pressure = data['value'] / 1000.0 # Pascal
        # msg.variance = 0 # n/a

        msg = geometry_msgs.msg.Vector3Stamped()
        msg.header.stamp = stamp
        msg.vector.z = data["value"]  # meters

        # self.pub_barometer.publish(msg_bar)

        self.pub_altitude.publish(msg)
        # self.get_logger().info('Barometer: {}'.format(data))

    def vibration_cb(self, msg: builtin_interfaces.msg.Duration) -> None:
        self.mwc.haptic.start_motor(100, ros2metawear_duration(msg))

    def vibration_pattern_cb(self, msg: metawear_ros.msg.VibrationPattern) -> None:
        msg.pattern.reverse()

        while msg.pattern and rclpy.ok():
            cur = msg.pattern.pop()
            duration_ms = ros2metawear_duration(cur.duration)
            self.mwc.haptic.start_motor(cur.power, duration_ms)
            time.sleep(duration_ms * 1e-3)

    def write_led(self, msg: std_msgs.msg.ColorRGBA) -> None:
        # self.mwc.led.stop_and_clear()

        brightness = 31.0 * msg.a

        red = self.mwc.led.load_preset_pattern("solid", repeat_count=255)
        red.high_intensity = int(msg.r * brightness)
        red.low_intensity = int(msg.r * brightness)

        green = self.mwc.led.load_preset_pattern("solid", repeat_count=255)
        green.high_intensity = int(msg.g * brightness)
        green.low_intensity = int(msg.g * brightness)

        blue = self.mwc.led.load_preset_pattern("solid", repeat_count=255)
        blue.high_intensity = int(msg.b * brightness)
        blue.low_intensity = int(msg.b * brightness)

        self.mwc.led.write_pattern(red, "r")
        self.mwc.led.write_pattern(green, "g")
        self.mwc.led.write_pattern(blue, "b")

        self.mwc.led.play()

    def led_cb(self, msg: std_msgs.msg.ColorRGBA) -> None:
        # self.current_color = copy.deepcopy(msg)
        self.write_led(msg)

    def exec_timer_cb(self, msg: std_msgs.msg.UInt8) -> None:
        if self.is_connected:
            self.get_logger().info(f"Executing timer {msg.data}")
            timer = libmetawear.mbl_mw_timer_lookup_id(self.mwc.board, msg.data)
            if timer:
                libmetawear.mbl_mw_timer_start(timer)
            else:
                self.get_logger().error(f"Could not find timer with ID {msg.data}")

    def reset_board_cb(self, msg: std_msgs.msg.Bool) -> None:
        if msg.data:
            self.get_logger().error(
                "Resetting the board. It may take >10 s to reconnect!"
            )
            libmetawear.mbl_mw_debug_reset(self.mwc.board)
            self.disconnect()
            self.get_logger().info("Reconnecting in 3 s...")
            time.sleep(3.0)
            self.connect()
        else:
            self.get_logger().warning("Attempt to reset the board with 'data: false'")

    def blink_led_2Hz(self, color_msg: std_msgs.msg.ColorRGBA, delay: float = 0) -> int:
        if not self.is_connected:
            return 0
        brightness = 31.0 * color_msg.a
        for channel in "rgb":
            pattern = self.mwc.led.load_preset_pattern("blink", repeat_count=2)
            pattern.low_intensity = 0
            pattern.high_time_ms = 250
            pattern.pulse_duration_ms = 500
            pattern.delay_time_ms = int(delay)
            pattern.high_intensity = int(getattr(color_msg, channel) * brightness)
            self.mwc.led.write_pattern(pattern, channel)
        self.mwc.led.play()
        return pattern.repeat_count * (
            pattern.pulse_duration_ms + pattern.delay_time_ms
        )

    def poll_calibration_state(self) -> None:
        if self.is_connected:
            blink_yellow = std_msgs.msg.ColorRGBA(r=0.8, g=1.0, b=0.0, a=1.0)
            blink_red = std_msgs.msg.ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
            t_until = rclpy.time.Time(seconds=0)
            # if self.peers_count[self.calib_state_topic]:
            # self.mwc.sensorfusion.read_calibration_state()

            if self.calib_state:
                cumulative_calib = (
                    self.calib_state.accelerometer + self.calib_state.gyroscope
                )
                # + self.calib_state.magnetometer

                if (
                    self.calib_state.accelerometer < 3 and
                    self.calib_state.gyroscope < 3
                ):
                    # self.calib_state.magnetometer < 3):

                    if self.get_clock().now() > t_until:
                        t = self.blink_led_2Hz(blink_red)
                        t_until = self.get_clock().now() + ros_duration(t / 1000.0)

                elif cumulative_calib < 6:
                    if self.get_clock().now() > t_until:
                        t = self.blink_led_2Hz(blink_yellow)
                        t_until = self.get_clock().now() + ros_duration(t / 1000.0)
                else:
                    self.led_cb(self.default_led_color)

    # CHANGED(Jerome): by default rclpy run everything in a single thread.
    def run(self) -> None:
        # loop_rate = self.create_rate(self.publish_rate, self.get_clock())

        if self.republish_button:
            self.button_timer = self.create_timer(1 / self.publish_rate, self.republish_button_cb)
        self.calibration_timer = self.create_timer(1, self.update)
        self.mwc.sensorfusion.read_calibration_state()
        self.create_timer(1, self.update_battery)
        if self.pause_streaming_if_no_subscribers:
            self.get_logger.info("pause_streaming_if_no_subscribers")
            self.create_timer(1, self.update_subscribers_count)
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            pass
        self.disconnect()

    def update_battery(self) -> None:
        # self.get_logger().info("update_battery start")
        self.mwc.settings.read_battery_state()
        self.mwc.temperature.read_temperature()
        # self.get_logger().info("update_battery end")

    def republish_button_cb(self) -> None:
        self.pub_button.publish(self.button_val)

    def update(self) -> None:
        # self.get_logger().info("update start")
        # if self.republish_button:
        #     self.pub_button.publish(self.button_val)
        # self.poll_calibration_state()
        if self.calibrated:
            self.get_logger().info("Stop calibration timer")
            self.calibration_timer.cancel()
        else:
            self.get_logger().info("Read new calibration")
            self.mwc.sensorfusion.read_calibration_state()

        # self.get_logger().info("update end")


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = MetaWearRos()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
