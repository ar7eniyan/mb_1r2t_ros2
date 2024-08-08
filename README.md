# Fork info
- Changed serial protocol handling (my MB_1R2T behaves partly like YDLIDAR S4B, see its datasheet)
- Fix inaccurate LaserScan messages. The prevoius algorithm assumed the uniformness of the angle vaules even between scan packets from serial protocol. That's not true because of a delay between those packets which made LaserScan points drift around (LaserScan itself needs equal angle intervals). The new algorithm clamps each point to a discretized angle value, giving the needed uniformness, but slightly losing precision (can be tweaked through `SAMPLES_PER_SCAN` value in `include/mb_1r2t/mb_1r2t.hpp`).

---

# mb_1r2t_ros2

ROS2 driver for the MB 1R2T LiDAR.

Based on [https://github.com/Vidicon/mb_1r2t_ros](https://github.com/Vidicon/mb_1r2t_ros).

![mb_1r2t](img/mb_1r2t.jpg)

![rviz](img/rviz.png)

## Usage

In order to launch the node:

```bash
ros2 launch mb_1r2t start.launch.py
```

In order to launch the node and see the results via ```rviz```:

```bash
ros2 launch mb_1r2t rviz.launch.py
```

## Notes

By default the node publishes to the following topics:

| Topic | Type |
|-------|------|
|```/laser_scan``` | ```sensor_msgs/msg/LaserScan```|
|```/point_cloud``` | ```sensor_msgs/msg/PointCloud```|

The default parameters of the node are:

+ ```port``` (serial port) = ```/dev/ttyUSB0```

+ ```frame_id``` = ```lidar```

These can be changed using the launch file *launch/start.launch.py*.

Windows is not supported yet.
