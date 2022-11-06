## h264_image_transport ![build status](https://github.com/clydemcqueen/h264_image_transport/actions/workflows/build_test.yml/badge.svg?branch=master)

Adds H264 decoding to [ROS2 image transport](https://index.ros.org/p/image_transport/github-ros-perception-image_common/).
Also includes a simple node to copy H264 packets from a `video4linux` camera.

Possible future work: add H264 encoder
 
### Example usage

Command line:
~~~
ros2 run image_transport republish h264 raw --ros-args -r in/h264:=/image_raw/h264 -r out:=/repub_raw
~~~

Launch file:
~~~
ros2 launch h264_image_transport example_launch.py
~~~

### h264_cam_node parameters

| Parameter | Type | Default | Notes |
|---|---|---|---|
| input_fn | string | /dev/video2 | Can be any ffmpeg format |
| fps | string | 30 | Desired frame rate |
| size | string | 800x600 | Width by height |
| frame_id | string | camera_frame | Camera frame ID |
| camera_info_path | string | info.ini | Path to camera info file |

### h264_cam_node topics

| Topic | Message | Notes |
|---|---|---|
| image_raw/h264 | `h264_msgs::msg::Packet` | H264 packet |
| camera_info | `sensor_msgs::msg::CameraInfo` | Camera info message |

### Requirements

Tested on ROS2 Foxy, Galactic and Humble.

Requires `libavdevice`, `libavformat`, `libavcodec`, `libavutil`, `libswscale`.

Here's one way to satisfy these requirements:
~~~
sudo apt install libavdevice-dev libavformat-dev libavcodec-dev libavutil-dev libswscale-dev
~~~

Note: `rosdep` won't find keys for most of these libraries so `package.xml` declares
dependencies on `ffmpeg` and `libavdevice-dev`. Strictly speaking `ffmpeg` is not required.