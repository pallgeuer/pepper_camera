Pepper Camera Streamer
======================

This is a ROS package developed by Philipp Allgeuer for efficiently streaming camera images from the Pepper robot, and making them available in ROS. It was directly inspired by the GSCam project, and also uses GStreamer. Using `getImageRemote()` from the NAOqi API can obtain VGA (640x480) images at a stable frame rate of 24fps if the network bandwidth supports a constant ~18MB/s data rate, which is often challenging over WiFi (I get ~0.9fps due to very poor WiFi quality). Streaming video with the API like this also costs around 43% extra CPU usage in the `naoqi-service` service, increasing it from ~20% to ~63%. There is thus little headroom to stream a second camera or use the NAOqi API for other e.g. sensor/control-related purposes. Using this package it is possible to stream VGA (640x480) video at a stable 24fps over WiFi, requiring only 0.4-1.2MB/s data rate (depending on the JPEG quality used) and 30% CPU that can run in parallel to the `naoqi-service` service (instead of competing with other NAOqi API calls). This package is observed to produce less delay between image capture and image delivery than the NAOqi API, and does not suffer from the problem that identical frames may be delivered more than once (this happens if you query `getImageRemote()` 'too quickly'). You can filter out such identical frames based on timestamp, but it causes significant disturbance to the regularity with which you receive and process unique new frames.

Installation
------------

Install system dependencies on your local PC (Pepper should already have GStreamer 0.10):
```sh
sudo apt install libgstreamer1.0-dev gstreamer1.0-tools
```
Ensure your ROS installation is sourced (try `which roscore`) then create a workspace to compile this package into:
```sh
mkdir -p PATH/TO/pepper_ros_ws/src
cd PATH/TO/pepper_ros_ws
ln -s PATH/TO/pepper_camera src/pepper_camera
catkin_make
```

Running
-------

Go to the Pepper robot page by entering the robot's IP address in the browser and logging in. In the settings tab disable "Alive by default" and reboot the robot. This is required so that the top camera isn't automatically already in use by the `naoqi-service` when the robot boots, thereby preventing us from using it in a separate process).

SSH into the Pepper robot and start streaming video (it would be best to automate this somehow so that it can be triggered remotely from code):
```sh
gst-launch-0.10 -v v4l2src device=/dev/video0 ! 'video/x-raw-yuv,width=640,height=480' ! jpegenc quality=[QUALITY] ! rtpjpegpay ! udpsink sync=false host=[IP] port=[PORT]
```
Note that:
* `[IP]` should be the IP address of the PC you wish to stream the video to, and `[PORT]` the corresponding port
* `[QUALITY]` should be a number 0-100 (try 75) specifying the quality of the JPEG compression used (balances required network data rate vs quality)
* `/dev/video0` is the top camera and `/dev/video1` is the bottom camera
* Trying to specify a framerate using `framerate=30/1` does not seem to make a difference (whether higher or lower than 24fps)
* Trying to stream resolutions higher than 640x480 results in corrupted images
* Using a video codec like theora instead of JPEG compression results in much better quality for the same network data rate, but it only achieves 15fps or so due to 100% CPU usage

You can test that the network stream arrives at your PC and can be decoded using:
```sh
gst-launch-1.0 -v udpsrc port=[PORT] ! application/x-rtp,encoding-name=JPEG ! rtpjpegdepay ! jpegdec ! fpsdisplaysink
```
You can also in parallel save the last 300 received JPEGs at any time using:
```sh
gst-launch-1.0 -v udpsrc port=[PORT] ! application/x-rtp,encoding-name=JPEG ! rtpjpegdepay ! tee name=jpegstream ! queue ! jpegdec ! fpsdisplaysink jpegstream. ! queue ! multifilesink location="frame%04d.jpg" max-files=300
```
Note that you should apparently also have `payload=26` after `encoding-name=JPEG`, but it does not seem to make a difference. The command will create JPEG files in the current folder.

If you're not getting the framerate you expect, you can check the current WiFi connection speed of the Pepper robot using (on the robot):
```sh
iw dev wlan0 link
```

You can check the capabilities of a camera (doesn't seem to work so well on the Pepper) using:
```sh
v4l2-ctl --list-formats-ext
v4l2-ctl --device=/dev/video0 --all  # <-- Note that this displays sticky information from the last configured use of the camera
```

You can run the Pepper camera node using:
```sh
source PATH/TO/pepper_ros_ws/devel/setup.bash
rosrun pepper_camera node
```
