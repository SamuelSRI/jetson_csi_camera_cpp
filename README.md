# jetson_csi_camera

ROS 2 C++ camera node for NVIDIA Jetson CSI cameras. It uses Argus and a native
GStreamer `appsink`, then copies the BGR buffer directly into
`sensor_msgs/msg/Image`.

OpenCV and `cv_bridge` are not used.

## Dependencies

```bash
sudo apt install \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-tools
```

## Build

```bash
cd ~/ros2_ws
rm -rf build/jetson_csi_camera install/jetson_csi_camera
source /opt/ros/jazzy/setup.bash
colcon build --packages-select jetson_csi_camera --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 launch jetson_csi_camera csi_camera.launch.py
```

The node logs its internal publication rate every two seconds. The default
configuration captures 1640x1232 at 30 FPS and publishes 640x480 BGR8 with
BEST_EFFORT QoS and queue depth 1.
