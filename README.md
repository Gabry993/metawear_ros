# metawear_ros

This is a ROS package to interface with MetaWear sensors via Python [pymetawear](https://github.com/hbldh/pymetawear) library.

The package attempts to mimic [myo_ros_osx](https://github.com/bgromov/myo_ros_osx) interface and is primarily developed to replace Myo Armband with MetaMotionR+ sensor.

### Published topics

 * `~/accel` ([geometry_msgs/Vector3Stamped](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Vector3Stamped.html)) - corrected acceleration.
 * `~/altitude` ([geometry_msgs/Vector3Stamped](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Vector3Stamped.html)) - only Z-axis.
 * `~/battery_state` ([sensor_msgs/BatteryState](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/BatteryState.html))
 * `~/button` ([std_msgs/Bool](http://docs.ros.org/melodic/api/std_msgs/html/msg/Bool.html))
 * `~/calibration_state` ([metawear_ros/CalibrationState](msg/CalibrationState.msg)) - calibration state. Although the message is always streamed, the actual hardware calibration is updated ONLY when the appropriate sensor stream are enabled. Thus, to calibrate the accelerometer you MUST subscribe to the `~/accel` topic, same for the `~/gyro`.
 * `~/gyro` ([geometry_msgs/Vector3Stamped](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Vector3Stamped.html)) - corrected gyroscope.
 * `~/illuminance` ([sensor_msgs/Illuminance](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Illuminance.html))
 * `~/rotation` ([geometry_msgs/QuaternionStamped](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/QuaternionStamped.html))
 * `~/temperature` ([sensor_msgs/Temperature](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Temperature.html))
 * `/tf`


### Subscribed topics

 * `~/vibration2` ([std_msgs/Duration](http://docs.ros.org/melodic/api/std_msgs/html/msg/Duration.html))
 * `~/led` ([std_msgs/ColorRGBA](http://docs.ros.org/melodic/api/std_msgs/html/msg/ColorRGBA.html)) - Sets RGB color of LED, uses alpha channel to control brightness.
 * `~/reset_board` ([std_msgs/Bool](http://docs.ros.org/melodic/api/std_msgs/html/msg/Bool.html)) - send `{data: true}` to this topic to perform the board soft reset.

### Parameters

TODO
