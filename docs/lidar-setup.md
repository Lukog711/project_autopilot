# LiDAR Setup

## Reference Sensor

The reference implementation uses the Livox MID-360.

The corresponding ROS 2 driver is:

https://github.com/Livox-SDK/livox_ros_driver2

The driver requires the Livox SDK2:

https://github.com/Livox-SDK/Livox-SDK2

## Driver Installation

Follow the official Livox SDK2 installation instructions.

Then install the ROS 2 driver according to its documentation.

For ROS 2 Jazzy, use the Jazzy-compatible build procedure supplied by the driver repository.

## Topic

The reference implementation subscribes to:

```text
/livox/lidar
```

using:

```text
livox_ros_driver2/msg/CustomMsg
```

If another LiDAR is used, adapt the subscription to the message type and topic provided by its ROS 2 driver.

## Sensor Timestamp

The safety node checks the timestamp of each incoming point cloud.

Frames older than the configured freshness threshold are rejected.

Current threshold:

```text
200 ms
```

This value is an experimental parameter and should be determined through system-level timing analysis for a production implementation.
