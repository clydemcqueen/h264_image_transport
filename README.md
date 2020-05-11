## h264_image_transport

Adds H264 decoding to [ROS2 image transport](https://index.ros.org/p/image_transport/github-ros-perception-image_common/).
Also includes a simple node to copy H264 packets from a `video4linux` camera.

Possible future work: add H264 encoder
 
### Example usage
~~~
ros2 launch h264_image_transport example_launch.py
~~~

### v4l_cam_node parameters

| Parameter | Type | Default | Notes |
|---|---|---|---|
| `input_fn` | string | /dev/video2 | Can be any ffmpeg format |
| `fps` | string | 30 | Desired frame rate |
| `size` | string | 800x600 | Width by height |
| `frame_id` | string | camera_frame | Camera frame ID |

### Requirements

Tested on Ubuntu 18.04 with ROS2 Eloquent.

Requires ffmpeg:
~~~
ffmpeg version 3.4.6-0ubuntu0.18.04.1 Copyright (c) 2000-2019 the FFmpeg developers
  libavutil      55. 78.100 / 55. 78.100
  libavcodec     57.107.100 / 57.107.100
  libavformat    57. 83.100 / 57. 83.100
  libavdevice    57. 10.100 / 57. 10.100
  libavfilter     6.107.100 /  6.107.100
  libavresample   3.  7.  0 /  3.  7.  0
  libswscale      4.  8.100 /  4.  8.100
  libswresample   2.  9.100 /  2.  9.100
  libpostproc    54.  7.100 / 54.  7.100
~~~
