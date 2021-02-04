# metawear_ros

This is a ROS2 package to interface with MetaWear sensors via Python [pymetawear](https://github.com/hbldh/pymetawear) library.

The package attempts to mimic [myo_ros_osx](https://github.com/bgromov/myo_ros_osx) interface and is primarily developed to replace Myo Armband with MetaMotionR+ sensor.

### Published topics

 * `~/accel` ([geometry_msgs/msg/Vector3Stamped](http://docs.ros.org/latest/api/geometry_msgs/html/msg/Vector3Stamped.html)) - corrected acceleration.
 * `~/altitude` ([geometry_msgs/msg/Vector3Stamped](http://docs.ros.org/latest/api/geometry_msgs/html/msg/Vector3Stamped.html)) - only Z-axis.
 * `~/battery_state` ([sensor_msgs/msg/BatteryState](http://docs.ros.org/latest/api/sensor_msgs/html/msg/BatteryState.html))
 * `~/button` ([std_msgs/msg/Bool](http://docs.ros.org/latest/api/std_msgs/html/msg/Bool.html))
 * `~/calibration_state` ([metawear_ros/msg/CalibrationState](msg/CalibrationState.msg)) - calibration state. Although the message is always streamed, the actual hardware calibration is updated ONLY when the appropriate sensor stream are enabled. Thus, to calibrate the accelerometer you MUST subscribe to the `~/accel` topic, same for the `~/gyro`.
 * `~/gyro` ([geometry_msgs/msg/Vector3Stamped](http://docs.ros.org/latest/api/geometry_msgs/html/msg/Vector3Stamped.html)) - corrected gyroscope.
 * `~/illuminance` ([sensor_msgs/msg/Illuminance](http://docs.ros.org/latest/api/sensor_msgs/html/msg/Illuminance.html))
 * `~/rotation` ([geometry_msgs/msg/QuaternionStamped](http://docs.ros.org/latest/api/geometry_msgs/html/msg/QuaternionStamped.html))
 * `~/temperature` ([sensor_msgs/msg/Temperature](http://docs.ros.org/latest/api/sensor_msgs/html/msg/Temperature.html))
 * `/tf`


### Subscribed topics

 * `~/vibration2` ([builtin_interfaces/msg/Duration](http://docs.ros.org/latest/api/std_msgs/html/msg/Duration.html))
 * `~/led` ([std_msgs/msg/ColorRGBA](http://docs.ros.org/latest/api/std_msgs/html/msg/ColorRGBA.html)) - Sets RGB color of LED, uses alpha channel to control brightness.
 * `~/reset_board` ([std_msgs/msg/Bool](http://docs.ros.org/latest/api/std_msgs/html/msg/Bool.html)) - send `{data: true}` to this topic to perform the board soft reset.

### Parameters

TODO
