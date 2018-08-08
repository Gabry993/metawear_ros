# metawear_ros

This is a ROS package to interface with MetaWear sensors via Python [pymetawear](https://github.com/hbldh/pymetawear) library.

The package attempts to mimic [myo_ros_osx](https://github.com/bgromov/myo_ros_osx) interface and is primarily developed to replace Myo Armband with MetaMotionR+ sensor.

### Published topics

 * `~/rotation` (geometry_msgs/QuaternionStamped)
 * `~/accel` (geometry_msgs/Vector3Stamped) -- corrected acceleration
 * `~/gyro` (geometry_msgs/Vector3Stamped) -- corrected gyroscope
 * `~/button` (std_msgs/Bool)
 * `tf`

### Subscribed topics

 * `~/vibration2` (std_msgs/Duration)
 * `~/led` (std_msgs/ColorRGBA)
 * `~/reset_board` (std_msgs/Bool) -- performs board soft reset

### Parameters

TODO
