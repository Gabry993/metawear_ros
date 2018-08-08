#!/usr/bin/env python

import math

from pymetawear.client import MetaWearClient
from pymetawear.exceptions import PyMetaWearException
# from ctypes import byref
from pymetawear import libmetawear
from mbientlab.metawear.cbindings import SensorFusionData, SensorFusionGyroRange, SensorFusionAccRange, SensorFusionMode
from mbientlab.metawear.cbindings import LedPattern

import rospy
from std_msgs.msg import Bool, Duration, ColorRGBA
from geometry_msgs.msg import QuaternionStamped, Vector3Stamped, TransformStamped

import tf2_ros
import tf2_geometry_msgs
import tf_conversions as tfc
import PyKDL as kdl

class MetaWearRos(object):
    """docstring for MetaWearRos"""

    def __init__(self):
        self.publish_rate = rospy.get_param('~publish_rate', 25)
        self.frame_id = rospy.get_param('~frame_id', rospy.get_name().split('/')[-1])
        self.mimic_myo_frame = rospy.get_param('~mimic_myo_frame', True)

        self.address = rospy.get_param('~address')
        self.interface = rospy.get_param('~interface', 'hci0')
        self.mwc = MetaWearClient(self.address, self.interface, connect=False)

        rotation_topic = rospy.get_param('~rotation_topic', '~rotation')
        accel_topic = rospy.get_param('~accel_topic', '~accel')
        gyro_topic = rospy.get_param('~gyro_topic', '~gyro')
        button_topic = rospy.get_param('~button_topic', '~button')

        self.republish_button = rospy.get_param('~republish_button', True)
        self.button_val = False

        vibration_topic = rospy.get_param('~vibration_topic', '~vibration2')
        led_topic = rospy.get_param('~led_topic', '~led')

        self.pub_rotation = rospy.Publisher(rotation_topic, QuaternionStamped, queue_size = 10)
        self.pub_accel = rospy.Publisher(accel_topic, Vector3Stamped, queue_size = 10)
        self.pub_gyro = rospy.Publisher(gyro_topic, Vector3Stamped, queue_size = 10)
        self.pub_button = rospy.Publisher(button_topic, Bool, queue_size = 10)

        self.sub_vibration = rospy.Subscriber(vibration_topic, Duration, self.vibration_cb)
        self.sub_led = rospy.Subscriber(led_topic, ColorRGBA, self.led_cb)

        self.sub_reset = rospy.Subscriber('~reset_board', Bool, self.reset_board_cb)

        self.tf_br = tf2_ros.TransformBroadcaster()

        rospy.on_shutdown(self.disconnect)
        self.connect()

    def connect(self):
        rospy.loginfo('Connecting to {0} ...'.format(self.address))
        self.mwc.connect()
        rospy.loginfo('Connected')
        rospy.sleep(0.1)

        try:
            rospy.loginfo('Setting BLE parameters...')
            self.mwc.settings.set_connection_parameters(min_conn_interval=7.5, max_conn_interval=10, latency=0, timeout=3000)
            rospy.sleep(0.2)
            rospy.loginfo('Done')

            rospy.loginfo('Setting sensor fusion mode')
            self.mwc.sensorfusion.set_mode(SensorFusionMode.SLEEP)
            # self.mwc.sensorfusion.set_mode(SensorFusionMode.IMU_PLUS)
            self.mwc.sensorfusion.set_mode(SensorFusionMode.NDOF)
            rospy.sleep(0.2)

            rospy.loginfo('Setting sensors range')
            self.mwc.sensorfusion.set_acc_range(SensorFusionAccRange._16G)
            self.mwc.sensorfusion.set_gyro_range(SensorFusionGyroRange._2000DPS)
            rospy.sleep(0.2)

            rospy.loginfo('Setting sensor fusion sampling rates')
            self.mwc.sensorfusion.set_sample_delay(SensorFusionData.QUATERION, 10)
            self.mwc.sensorfusion.set_sample_delay(SensorFusionData.CORRECTED_ACC, 10)
            self.mwc.sensorfusion.set_sample_delay(SensorFusionData.CORRECTED_GYRO, 10)

            rospy.loginfo('Done')

            self.mwc.sensorfusion.notifications(
                quaternion_callback = self.mwc_quat_cb,
                corrected_acc_callback = self.mwc_acc_cb,
                corrected_gyro_callback = self.mwc_gyro_cb)

            self.mwc.switch.notifications(self.mwc_switch_cb)
        except PyMetaWearException:
            rospy.logerr('Unable to configure the sensors. Please try to reset the board')
            # self.reset_board_cb(Bool(True))

    def disconnect(self):
        rospy.loginfo('Disconnecting from {0} ...'.format(self.address))
        self.mwc.sensorfusion.notifications()
        self.mwc.switch.notifications()
        self.mwc.led.stop_and_clear()

        rospy.loginfo('Resetting data processors')
        libmetawear.mbl_mw_metawearboard_tear_down(self.mwc.board)
        rospy.sleep(1.0)

        self.mwc.disconnect()
        rospy.sleep(0.2)

    def mwc_quat_cb(self, data):
        now = rospy.Time.now()

        rot = kdl.Rotation.Quaternion(data['value'].x, data['value'].y, data['value'].z, data['value'].w)
        q = rot.GetQuaternion()

        if self.mimic_myo_frame:
            # Make x-axis point towards elbow
            corr_rot =  rot * kdl.Rotation.RPY(0.0, 0.0, math.pi / 2)

            # _, pitch, _ = corr_rot.GetRPY()
            # pitch = math.degrees(pitch)
            # # rospy.loginfo()
            # if pitch > 0.0 and pitch < 1.0:
            #     self.vibration_cb(Duration(rospy.Duration.from_sec(0.05)))

            q = corr_rot.GetQuaternion()

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
        # rospy.loginfo(quat)

    def mwc_acc_cb(self, data):
        now = rospy.Time.now()

        accel = Vector3Stamped()
        accel.header.stamp = rospy.Time(data['epoch'] / 1000.0) #now
        accel.header.frame_id = self.frame_id
        accel.vector.x = data['value'].x
        accel.vector.y = data['value'].y
        accel.vector.z = data['value'].z

        self.pub_accel.publish(accel)

    def mwc_gyro_cb(self, data):
        now = rospy.Time.now()

        gyro = Vector3Stamped()
        gyro.header.stamp = rospy.Time(data['epoch'] / 1000.0) #now
        gyro.header.frame_id = self.frame_id
        gyro.vector.x = data['value'].x
        gyro.vector.y = data['value'].y
        gyro.vector.z = data['value'].z

        self.pub_gyro.publish(gyro)
        # rospy.loginfo(data)

    def mwc_switch_cb(self, data):
        now = rospy.Time.now()

        # rospy.loginfo(data)
        stamp = rospy.Time(data['epoch'] / 1000.0)
        self.button_val = Bool(data['value'])
        if not self.republish_button:
            self.pub_button.publish(self.button_val)

    def vibration_cb(self, msg):
        d = rospy.Duration(msg.data.secs, msg.data.nsecs)
        self.mwc.haptic.start_motor(100, int(d.to_sec() * 1000))

    def led_cb(self, msg):
        self.mwc.led.stop_and_clear()
        red = self.mwc.led.load_preset_pattern('solid', repeat_count=255)
        red.high_intensity = int(msg.r * 31)
        red.low_intensity = int(msg.r * 31)

        green = self.mwc.led.load_preset_pattern('solid', repeat_count=255)
        green.high_intensity = int(msg.g * 31)
        green.low_intensity = int(msg.g * 31)

        blue = self.mwc.led.load_preset_pattern('solid', repeat_count=255)
        blue.high_intensity = int(msg.b * 31)
        blue.low_intensity = int(msg.b * 31)

        self.mwc.led.write_pattern(red, 'r')
        self.mwc.led.write_pattern(green, 'g')
        self.mwc.led.write_pattern(blue, 'b')

        self.mwc.led.play()

    def reset_board_cb(self, msg):
        if msg.data == True:
            rospy.logerr('Resetting the board. It may take >10 s to reconnect!')
            libmetawear.mbl_mw_debug_reset(self.mwc.board)
            rospy.loginfo('Reconnecting in 3 s...')
            rospy.sleep(3.0)
            self.connect()
        else:
            rospy.logwarn('Attempt to reset the board with \'data: false\'')

    def run(self):
        loop_rate = rospy.Rate(self.publish_rate)

        while not rospy.is_shutdown():
            try:
                loop_rate.sleep()

                if self.republish_button:
                    self.pub_button.publish(self.button_val)

            except rospy.ROSException, e:
                if e.message == 'ROS time moved backwards':
                    rospy.logwarn('Saw a negative time change, resseting.')

if __name__ == '__main__':
    rospy.init_node('metawear_ros', anonymous = False)

    mw = MetaWearRos()
    mw.run()
