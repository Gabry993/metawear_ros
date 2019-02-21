#!/usr/bin/env python

import math
import copy
import ctypes
from threading import Event

from pymetawear.client import MetaWearClient
from pymetawear.exceptions import PyMetaWearException

from pymetawear import libmetawear
from mbientlab.metawear.cbindings import SensorFusionData, SensorFusionGyroRange, SensorFusionAccRange, SensorFusionMode
from mbientlab.metawear.cbindings import BaroBoschOversampling, BaroBoschIirFilter
from mbientlab.metawear.cbindings import LedPattern
from mbientlab.metawear.cbindings import FnVoid_VoidP_VoidP_Int, FnVoid_VoidP_VoidP_CalibrationDataP, FnVoid_VoidP_DataP

import rospy
from std_msgs.msg import Bool, Int8, Float32, Duration, ColorRGBA
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import QuaternionStamped, Vector3Stamped, TransformStamped
from sensor_msgs.msg import BatteryState, Temperature, Illuminance, FluidPressure, JointState

from metawear_ros.msg import CalibrationState, Vibration, VibrationPattern

import tf2_ros
import tf2_geometry_msgs
import tf_conversions as tfc
import PyKDL as kdl

class MetaWearRos(rospy.SubscribeListener, object):
    """docstring for MetaWearRos"""

    def __init__(self):
        # topic_name -> number of connected peers
        self.peers_count = {}

        self.publish_rate = rospy.get_param('~publish_rate', 25)

        self.frame_id = rospy.get_param('~frame_id', rospy.get_name().split('/')[-1])
        self.mimic_myo_frame = rospy.get_param('~mimic_myo_frame', True)

        self.address = rospy.get_param('~address').lower()
        self.interface = rospy.get_param('~interface', 'hci0')
        self.mwc = None
        self.is_connected = False

        self.config = rospy.get_param('~config', {})
        self.config = {k.lower():v for k,v in self.config.items()}
        self.calib_data = None

        if self.address in self.config:
            dev_conf = self.config[self.address]
            if 'conn_address' in dev_conf:
                conn_address = dev_conf['conn_address']
                rospy.logwarn('Substituting device address from the config')
                self.address = conn_address

        rotation_topic = rospy.get_param('~rotation_topic', '~rotation')

        #### Experimental ####
        joint_states_topic = rospy.get_param('~joint_states_topic', '~joint_states')
        self.joint_prefix = rospy.get_param('~joint_prefix', 'shoulder_to_wrist')
        ######################

        #### Experimental. Feb 17, 2019 ####
        # Allows to adjust zero for yaw angle
        self.yaw_origin = 0.0
        self.srv_set_yaw_origin = rospy.Service('~set_yaw_origin', Empty, self.set_yaw_origin)
        ######################

        accel_topic = rospy.get_param('~accel_topic', '~accel')
        gyro_topic = rospy.get_param('~gyro_topic', '~gyro')
        calib_state_topic = rospy.get_param('~calib_state_topic', '~calibration_state')
        button_topic = rospy.get_param('~button_topic', '~button')
        battery_topic = rospy.get_param('~battery_topic', '~battery_state')
        temperature_topic = rospy.get_param('~temperature_topic', '~temperature')
        illuminance_topic = rospy.get_param('~illuminance_topic', '~illuminance')
        # air_pressure_topic = rospy.get_param('~air_pressure_topic', '~air_pressure')
        altitude_topic = rospy.get_param('~altitude_topic', '~altitude')
        self.republish_button = rospy.get_param('~republish_button', True)
        self.button_val = False

        vibration_topic = rospy.get_param('~vibration_topic', '~vibration2')
        vibration_pattern_topic = rospy.get_param('~vibration_pattern_topic', '~vibration_pattern')
        led_topic = rospy.get_param('~led_topic', '~led')
        led_color = rospy.get_param('~led_color', {'r': 0.0, 'g': 1.0, 'b': 1.0, 'a': 1.0})
        self.default_led_color = ColorRGBA(**led_color)

        self.pub_rotation = rospy.Publisher(rotation_topic, QuaternionStamped,
            subscriber_listener = self, queue_size = 10)
        self.rotation_topic = self.track_topic(rotation_topic)
        self.pub_joint_state = rospy.Publisher(joint_states_topic, JointState,
            subscriber_listener = self, queue_size = 10)
        self.joint_states_topic = self.track_topic(joint_states_topic)

        self.pub_accel = rospy.Publisher(accel_topic, Vector3Stamped,
            subscriber_listener = self, queue_size = 10)
        self.accel_topic = self.track_topic(accel_topic)

        self.pub_gyro = rospy.Publisher(gyro_topic, Vector3Stamped,
            subscriber_listener = self, queue_size = 10)
        self.gyro_topic = self.track_topic(gyro_topic)

        self.pub_calib_state = rospy.Publisher(calib_state_topic, CalibrationState,
            subscriber_listener = self, queue_size = 10)
        self.calib_state_topic = self.track_topic(calib_state_topic)
        self.calib_state = None

        self.pub_button = rospy.Publisher(button_topic, Bool,
            subscriber_listener = self, queue_size = 10, latch=(not self.republish_button))
        self.button_topic = self.track_topic(button_topic)

        self.pub_battery = rospy.Publisher(battery_topic, BatteryState,
            subscriber_listener = self, queue_size = 10, latch=True)
        self.battery_topic = self.track_topic(battery_topic)

        self.pub_temperature = rospy.Publisher(temperature_topic, Temperature,
            subscriber_listener = self, queue_size = 10, latch=True)
        self.temperature_topic = self.track_topic(temperature_topic)

        self.pub_illuminance = rospy.Publisher(illuminance_topic, Illuminance,
            subscriber_listener = self, queue_size = 10)
        self.illuminance_topic = self.track_topic(illuminance_topic)

        self.illuminance = 0.0

        # self.pub_air_pressure = rospy.Publisher(air_pressure_topic, FluidPressure,
        #     subscriber_listener = self, queue_size = 10)
        # self.air_pressure_topic = self.track_topic(air_pressure_topic)

        self.pub_altitude = rospy.Publisher(altitude_topic, Vector3Stamped,
            subscriber_listener = self, queue_size = 10)
        self.altitude_topic = self.track_topic(altitude_topic)

        self.sub_vibration = rospy.Subscriber(vibration_topic, Duration, self.vibration_cb)
        self.sub_vibration_pattern = rospy.Subscriber(vibration_pattern_topic, VibrationPattern, self.vibration_pattern_cb, queue_size = 1)

        self.sub_led = rospy.Subscriber(led_topic, ColorRGBA, self.led_cb)

        self.sub_reset = rospy.Subscriber('~reset_board', Bool, self.reset_board_cb)

        self.tf_br = tf2_ros.TransformBroadcaster()

        # Workaround: prevent device streams to disconnect ever
        self.peers_count[self.rotation_topic] = 1
        self.peers_count[self.button_topic] = 1
        self.peers_count[self.altitude_topic] = 1
        self.peers_count[self.calib_state_topic] = 1

        rospy.on_shutdown(self.disconnect)
        self.connect()

    def track_topic(self, topic_name):
        resolved_name = rospy.remap_name(rospy.resolve_name(topic_name))
        self.peers_count[resolved_name] = 0
        return resolved_name

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        rospy.loginfo('New subscriber on: %s', topic_name)
        self.peers_count[topic_name] = self.peers_count[topic_name] + 1
        # if self.mwc.mw.is_connected:
        if self.is_connected:
            self.update_enabled_streams()

    def peer_unsubscribe(self, topic_name, num_peers):
        rospy.loginfo('Subscribers left on %s: %d', topic_name, num_peers)
        self.peers_count[topic_name] = num_peers
        # if self.mwc.mw.is_connected:
        if self.is_connected:
            self.update_enabled_streams()

    def on_disconnect(self, status):
        rospy.logwarn('Disconnected from {0}...'.format(self.address))
        if self.is_connected:
            self.is_connected = False
            self.disconnect()
            while not self.is_connected and not rospy.is_shutdown():
                rospy.logwarn('Reconnecting...')
                rospy.sleep(1.0)
                self.connect()

    def connect(self):
        self.mwc = MetaWearClient(self.address, self.interface, connect=False)
        self.mwc.mw.on_disconnect = self.on_disconnect

        rospy.loginfo('Connecting to {0} ...'.format(self.address))
        self.mwc.connect()
        rospy.loginfo('Connected')
        rospy.loginfo('Real MAC address: {}'.format(self.get_real_mac_address()))
        rospy.sleep(0.1)

        try:
            rospy.loginfo('Resetting data processors')
            libmetawear.mbl_mw_metawearboard_tear_down(self.mwc.board)
            rospy.sleep(1.0)

            rospy.loginfo('Setting BLE parameters...')
            self.mwc.settings.set_connection_parameters(min_conn_interval=7.5, max_conn_interval=10, latency=0, timeout=3000)
            rospy.sleep(0.2)
            rospy.loginfo('Done')

            rospy.loginfo('Configuring sensors...')
            rospy.loginfo('Setting IMU sensor fusion mode')
            self.mwc.sensorfusion.set_mode(SensorFusionMode.SLEEP)
            self.mwc.sensorfusion.set_mode(SensorFusionMode.IMU_PLUS)
            #self.mwc.sensorfusion.set_mode(SensorFusionMode.NDOF)
            rospy.sleep(0.5)

            rospy.loginfo('Setting IMU sensors range')
            #self.mwc.sensorfusion.set_acc_range(SensorFusionAccRange._16G)
            #self.mwc.sensorfusion.set_gyro_range(SensorFusionGyroRange._2000DPS)
            rospy.sleep(0.2)

            rospy.loginfo('Setting IMU sensor fusion sampling rates')
            self.mwc.sensorfusion.set_sample_delay(SensorFusionData.QUATERNION, 10)
            self.mwc.sensorfusion.set_sample_delay(SensorFusionData.CORRECTED_ACC, 10)
            self.mwc.sensorfusion.set_sample_delay(SensorFusionData.CORRECTED_GYRO, 10)

            rospy.loginfo('Setting ambient light sensor mode')
            self.mwc.ambient_light.set_settings(gain=8, integration_time=50, measurement_rate=50)
            rospy.loginfo('Setting barometer mode')
            self.mwc.barometer.set_altitude_data(status=True)
            self.mwc.barometer.set_settings(oversampling = 'standard', iir_filter = 'avg_16')
            rospy.loginfo('Done')

            rospy.loginfo('Setting disconnect event handler')
            dc_event = self.mwc.settings.disconnected_event()
            blink_pattern = self.mwc.led.load_preset_pattern('blink', repeat_count=5)
            blink_pattern.high_intensity = 31
            blink_pattern.low_intensity = 0
            blink_pattern.high_time_ms = 250
            blink_pattern.pulse_duration_ms = 500

            def event_rec_status(context, event, status):
                if not status:
                    rospy.loginfo('Disconnect event handler: OK')
                else:
                    rospy.logerr('Disconnect event handler: FAILED')

            self._evt_rec_status_cb = FnVoid_VoidP_VoidP_Int(event_rec_status)

            libmetawear.mbl_mw_event_record_commands(dc_event)
            self.clear_sensors()
            self.mwc.led.write_pattern(blink_pattern, 'g')
            self.mwc.led.write_pattern(blink_pattern, 'b')
            blink_pattern.delay_time_ms = 250
            self.mwc.led.write_pattern(blink_pattern, 'r')
            self.mwc.led.play()
            self.mwc.haptic.start_motor(100, 1000)
            libmetawear.mbl_mw_event_end_record(dc_event, None, self._evt_rec_status_cb)

            self.update_enabled_streams()

            self.current_color = copy.deepcopy(self.default_led_color)
            self.mwc.led.stop_and_clear()
            # self.led_cb(self.default_led_color)

        except PyMetaWearException:
            rospy.logerr('Unable to configure the sensors. Please try to reset the board')
            # self.reset_board_cb(Bool(True))
            return False

        self.is_connected = True
        return True

    def get_real_mac_address(self):
        e = Event()

        mac_data_signal = libmetawear.mbl_mw_settings_get_mac_data_signal(self.mwc.board)

        ret_value = {'value': None} # mutable

        def mac_data_cb(context, pdata):
            CHARP = ctypes.POINTER(ctypes.c_char_p)
            ret_value['value'] = ctypes.cast(pdata.contents.value, ctypes.c_char_p).value
            e.set()

        _mac_data_cb = FnVoid_VoidP_DataP(mac_data_cb)
        libmetawear.mbl_mw_datasignal_subscribe(mac_data_signal, None, _mac_data_cb)
        libmetawear.mbl_mw_datasignal_read(mac_data_signal)
        e.wait()
        libmetawear.mbl_mw_datasignal_unsubscribe(mac_data_signal)

        return ret_value['value']


    def update_enabled_streams(self):
        self.mwc.sensorfusion.notifications(
            quaternion_callback = self.mwc_quat_cb if self.peers_count[self.rotation_topic] or self.peers_count[self.joint_states_topic] else None,
            corrected_acc_callback = self.mwc_acc_cb if self.peers_count[self.accel_topic] else None,
            corrected_gyro_callback = self.mwc_gyro_cb if self.peers_count[self.gyro_topic] else None,
            calibration_state_callback = self.mwc_calib_state_cb if self.peers_count[self.calib_state_topic] else None)

        self.mwc.switch.notifications(self.mwc_switch_cb if self.peers_count[self.button_topic] else None)

        if self.peers_count[self.battery_topic]:
            self.mwc.settings.notifications(self.mwc_battery_cb)
            self.mwc.settings.read_battery_state()
        else:
            self.mwc.settings.notifications(None)

        if self.peers_count[self.temperature_topic]:
            self.mwc.temperature.notifications(self.mwc_temperature_cb)
            self.mwc.temperature.read_temperature()
        else:
            self.mwc.temperature.notifications(None)

        self.mwc.ambient_light.notifications(self.mwc_ambient_light_cb if self.peers_count[self.illuminance_topic] else None)
        self.mwc.barometer.notifications(self.mwc_barometer_cb if self.peers_count[self.altitude_topic] else None)

    def clear_sensors(self):
        self.mwc.sensorfusion.notifications()
        self.mwc.switch.notifications()
        self.mwc.settings.notifications()
        self.mwc.temperature.notifications()
        self.mwc.ambient_light.notifications()
        self.mwc.barometer.notifications()
        self.mwc.led.stop_and_clear()

    def disconnect(self):
        self.is_connected = False
        rospy.loginfo('Disconnecting from {0} ...'.format(self.address))
        self.clear_sensors()
        rospy.sleep(0.5)

        # rospy.loginfo('Resetting data processors')
        # libmetawear.mbl_mw_metawearboard_tear_down(self.mwc.board)
        # rospy.sleep(1.0)

        self.mwc.disconnect()
        self.mwc = None

        rospy.sleep(0.2)

    def set_yaw_origin(self, req):
        if not self.is_connected:
            raise rospy.ServiceException('Not connected')

        self.yaw_origin = self.orig_yaw
        rospy.loginfo('Successfully set yaw to zero')

        return EmptyResponse()

    def mwc_quat_cb(self, data):
        now = rospy.Time.now()

        rot = kdl.Rotation.Quaternion(data['value'].x, data['value'].y, data['value'].z, data['value'].w)
        q = rot.GetQuaternion()

        if self.mimic_myo_frame:
            # Make x-axis point towards elbow
            orig_rot =  rot * kdl.Rotation.RPY(0.0, 0.0, math.pi / 2)
        else:
            # Make x-axis point forward, along the arm
            orig_rot =  rot * kdl.Rotation.RPY(0.0, 0.0, -math.pi / 2)

        corr_rot =  kdl.Rotation.RPY(0.0, 0.0, -self.yaw_origin) * orig_rot

        # Tait-Bryan convention
        (self.orig_yaw, _, _) = orig_rot.GetEulerZYX()
        (yaw, pitch, roll) = corr_rot.GetEulerZYX()
        q = corr_rot.GetQuaternion()

        j_state = JointState()
        j_state.header.stamp = rospy.Time(data['epoch'] / 1000.0) #now
        j_state.name = [self.joint_prefix + '_roll', self.joint_prefix + '_pitch', self.joint_prefix + '_yaw']
        j_state.position = [roll, pitch, yaw]
        self.pub_joint_state.publish(j_state)

        quat = QuaternionStamped()
        quat.header.stamp = rospy.Time(data['epoch'] / 1000.0) #now
        quat.header.frame_id = self.frame_id
        quat.quaternion.x = q[0]
        quat.quaternion.y = q[1]
        quat.quaternion.z = q[2]
        quat.quaternion.w = q[3]

        t = TransformStamped()
        t.header.stamp = quat.header.stamp
        t.header.frame_id = 'world'
        t.child_frame_id = self.frame_id + '_world'
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.pub_rotation.publish(quat)
        self.tf_br.sendTransform(t)
        # rospy.loginfo_throttle(1.0, 'quat: {}'.format(data))

    def mwc_acc_cb(self, data):
        now = rospy.Time.now()

        a = kdl.Vector(data['value'].x, data['value'].y, data['value'].z)

        if self.mimic_myo_frame:
            # Make x-axis point towards elbow
            corr_a =  kdl.Rotation.RPY(0.0, 0.0, math.pi / 2) * a
            a = corr_a

        accel = Vector3Stamped()
        accel.header.stamp = rospy.Time(data['epoch'] / 1000.0) #now
        accel.header.frame_id = self.frame_id
        accel.vector.x = a.x()
        accel.vector.y = a.y()
        accel.vector.z = a.z()

        self.pub_accel.publish(accel)
        # rospy.loginfo_throttle(1.0, 'acc: {}'.format(data))

    def mwc_gyro_cb(self, data):
        now = rospy.Time.now()

        g = kdl.Vector(data['value'].x, data['value'].y, data['value'].z)

        if self.mimic_myo_frame:
            # Make x-axis point towards elbow
            corr_g =  kdl.Rotation.RPY(0.0, 0.0, math.pi / 2) * g
            g = corr_g

        gyro = Vector3Stamped()
        gyro.header.stamp = rospy.Time(data['epoch'] / 1000.0) #now
        gyro.header.frame_id = self.frame_id
        gyro.vector.x = g.x()
        gyro.vector.y = g.y()
        gyro.vector.z = g.z()

        self.pub_gyro.publish(gyro)
        # rospy.loginfo_throttle(1.0, 'gyro: {}'.format(data))

    def mwc_calib_state_cb(self, data):
        now = rospy.Time.now()

        calib = CalibrationState()
        calib.header.stamp = rospy.Time(data['epoch'] / 1000.0) #now
        calib.accelerometer = data['value'].accelrometer # <<-- typo!
        calib.gyroscope = data['value'].gyroscope
        calib.magnetometer = data['value'].magnetometer

        self.calib_state = calib

        self.pub_calib_state.publish(calib)

        # rospy.loginfo(data)

    def mwc_switch_cb(self, data):
        now = rospy.Time.now()

        # rospy.loginfo(data)
        stamp = rospy.Time(data['epoch'] / 1000.0)
        self.button_val = Bool(data['value'])
        if not self.republish_button:
            self.pub_button.publish(self.button_val)

    def mwc_battery_cb(self, data):
        stamp = rospy.Time(data['epoch'] / 1000.0)

        msg = BatteryState()
        msg.header.stamp = stamp
        msg.voltage = data['value'].voltage / 1000.0 # Volts
        msg.current = float('nan')
        msg.charge = float('nan')
        msg.capacity = float('nan')
        msg.design_capacity = float('nan')
        msg.percentage = data['value'].charge / 100.0 # [0.0; 1.0]
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        msg.present = True
        #msg.cell_voltage
        msg.location = 'inside'
        msg.serial_number = 'n/a'

        self.pub_battery.publish(msg)
        # rospy.loginfo('Battery: {}'.format(data))

    def mwc_temperature_cb(self, data):
        stamp = rospy.Time(data['epoch'] / 1000.0)

        msg = Temperature()
        msg.header.stamp = stamp
        msg.temperature = data['value'] # degrees Celsius
        msg.variance = 0 # n/a

        self.pub_temperature.publish(msg)

    def mwc_ambient_light_cb(self, data):
        stamp = rospy.Time(data['epoch'] / 1000.0)
        self.illuminance = data['value'] / 1000.0 # Lux

        msg = Illuminance()
        msg.header.stamp = stamp
        msg.illuminance = self.illuminance
        msg.variance = 0 # n/a

        self.pub_illuminance.publish(msg)

    def mwc_barometer_cb(self, data):
        stamp = rospy.Time(data['epoch'] / 1000.0)

        # msg = FluidPressure()
        # msg.header.stamp = stamp
        # msg.fluid_pressure = data['value'] / 1000.0 # Pascal
        # msg.variance = 0 # n/a

        msg = Vector3Stamped()
        msg.header.stamp = stamp
        msg.vector.z = data['value'] # meters

        # self.pub_barometer.publish(msg_bar)

        self.pub_altitude.publish(msg)

        # rospy.loginfo('Barometer: {}'.format(data))

    def vibration_cb(self, msg):
        d = rospy.Duration(msg.data.secs, msg.data.nsecs)
        self.mwc.haptic.start_motor(100, int(d.to_sec() * 1000))

    def vibration_pattern_cb(self, msg):
        msg.pattern.reverse()

        while msg.pattern and not rospy.is_shutdown():
            cur = msg.pattern.pop()
            d = rospy.Duration(cur.duration.secs, cur.duration.nsecs)
            self.mwc.haptic.start_motor(cur.power, int(d.to_sec() * 1000))
            rospy.sleep(d.to_sec())

    def write_led(self, msg):
        # self.mwc.led.stop_and_clear()

        brightness = 31.0 * msg.a

        red = self.mwc.led.load_preset_pattern('solid', repeat_count=255)
        red.high_intensity = int(msg.r * brightness)
        red.low_intensity = int(msg.r * brightness)

        green = self.mwc.led.load_preset_pattern('solid', repeat_count=255)
        green.high_intensity = int(msg.g * brightness)
        green.low_intensity = int(msg.g * brightness)

        blue = self.mwc.led.load_preset_pattern('solid', repeat_count=255)
        blue.high_intensity = int(msg.b * brightness)
        blue.low_intensity = int(msg.b * brightness)

        self.mwc.led.write_pattern(red, 'r')
        self.mwc.led.write_pattern(green, 'g')
        self.mwc.led.write_pattern(blue, 'b')

        self.mwc.led.play()

    def led_cb(self, msg):
        self.current_color = copy.deepcopy(msg)

        self.write_led(msg)

    def reset_board_cb(self, msg):
        if msg.data == True:
            rospy.logerr('Resetting the board. It may take >10 s to reconnect!')
            libmetawear.mbl_mw_debug_reset(self.mwc.board)
            self.disconnect()
            rospy.loginfo('Reconnecting in 3 s...')
            rospy.sleep(3.0)
            self.connect()
        else:
            rospy.logwarn('Attempt to reset the board with \'data: false\'')

    def blink_led_2Hz(self, color_msg, delay = 0):
        if not self.is_connected:
            return None

        brightness = 31.0 * color_msg.a

        pattern = self.mwc.led.load_preset_pattern('blink', repeat_count=2)
        pattern.low_intensity = 0
        pattern.high_time_ms = 250
        pattern.pulse_duration_ms = 500
        pattern.delay_time_ms = int(delay)

        red = copy.deepcopy(pattern)
        green = copy.deepcopy(pattern)
        blue = copy.deepcopy(pattern)
        red.high_intensity = int(color_msg.r * brightness)
        green.high_intensity = int(color_msg.g * brightness)
        blue.high_intensity = int(color_msg.b * brightness)

        self.mwc.led.write_pattern(red, 'r')
        self.mwc.led.write_pattern(green, 'g')
        self.mwc.led.write_pattern(blue, 'b')

        self.mwc.led.play()

        return pattern.repeat_count * (pattern.pulse_duration_ms + pattern.delay_time_ms)

    def poll_calibration_state(self):
        if self.is_connected:

            blink_yellow = ColorRGBA(r=0.8, g=1.0, b=0.0, a=1.0)
            blink_red    = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)

            t_until = rospy.Time(0.0)

            # if self.peers_count[self.calib_state_topic]:
            self.mwc.sensorfusion.read_calibration_state()

            if self.calib_state:
                cumulative_calib = self.calib_state.accelerometer + self.calib_state.gyroscope

                if self.calib_state.accelerometer < 3 and self.calib_state.gyroscope < 3:
                    if rospy.Time.now() > t_until:
                        t = self.blink_led_2Hz(blink_red)
                        t_until = rospy.Time.now() + rospy.Duration(t / 1000.0)

                elif cumulative_calib < 6:
                    if rospy.Time.now() > t_until:
                        t = self.blink_led_2Hz(blink_yellow)
                        t_until = rospy.Time.now() + rospy.Duration(t / 1000.0)
                else:
                    self.led_cb(self.default_led_color)

    def run(self):
        loop_rate = rospy.Rate(self.publish_rate)

        while not rospy.is_shutdown():
            try:
                loop_rate.sleep()

                if self.republish_button:
                    self.pub_button.publish(self.button_val)

                self.poll_calibration_state()

            except rospy.ROSException, e:
                if e.message == 'ROS time moved backwards':
                    rospy.logwarn('Saw a negative time change, resetting.')

if __name__ == '__main__':
    rospy.init_node('metawear_ros', anonymous = False)

    mw = MetaWearRos()
    mw.run()
