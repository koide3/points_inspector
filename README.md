# points_inspector

An inspection tool to see field values of sensor_msgs/msg/PointCloud2 messages.  

[![Build](https://github.com/koide3/points_inspector/actions/workflows/build.yaml/badge.svg)](https://github.com/koide3/points_inspector/actions/workflows/build.yaml)

Usage:
```bash
# ROS1
rosrun points_inspector points_inspector_node points:=/os1_cloud_node1/points

# ROS2
ros2 run points_inspector points_inspector_node -r points:=/os1_cloud_node1/points
```

Output example:
```
--- points ---
frame_id:laser_data_frame
stamp   :1670294980 379087360
size    :512 x 128
x              : datatype=FLOAT32 mean=3.894 first=-0.000 last=-0.000 median=0.000 min=-5.003 max=53.615
y              : datatype=FLOAT32 mean=0.245 first=0.000 last=-0.000 median=0.000 min=-98.566 max=138.273
z              : datatype=FLOAT32 mean=0.406 first=0.000 last=-0.000 median=-0.000 min=-2.003 max=15.989
intensity      : datatype=FLOAT32 mean=120.222 first=0.000 last=0.000 median=4.000 min=0.000 max=9207.000
t              : datatype=UINT32 mean=1352700063.0 first=2690838016 last=2690838016 median=49959936 min=0 max=2690838016
reflectivity   : datatype=UINT16 mean=5.4 first=0 last=0 median=0 min=0 max=184
ring           : datatype=UINT8 mean=63.5 first=0 last=127 median=64 min=0 max=127
ambient        : datatype=UINT16 mean=391.8 first=0 last=0 median=293 min=0 max=6605
range          : datatype=UINT32 mean=6368.6 first=0 last=0 median=0 min=0 max=138284
```

## Docker images

- [koide3/points_inspector:noetic ![Docker Image Size (tag)](https://img.shields.io/docker/image-size/koide3/points_inspector/noetic)](https://hub.docker.com/repository/docker/koide3/points_inspector)
- [koide3/points_inspector:humble ![Docker Image Size (tag)](https://img.shields.io/docker/image-size/koide3/points_inspector/humble)](https://hub.docker.com/repository/docker/koide3/points_inspector)

```bash
# ROS1 noetic
docker run --rm --net host koide3/points_inspector:noetic rosrun points_inspector points_inspector_node points:=/os1_cloud_node1/points
```

```bash
# ROS2 humble (You may need some DDS configuration for ROS2 communication over docker)
docker run --rm koide3/points_inspector:humble ros2 run points_inspector points_inspector_node -r points:=/os1_cloud_node1/points
```
