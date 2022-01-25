Pepper Camera Streamer
======================

**Author:** Philipp Allgeuer <[philipp.allgeuer@uni-hamburg.de](mailto:philipp.allgeuer@uni-hamburg.de)>

This is a ROS package developed by Philipp Allgeuer for efficiently streaming camera images from the Pepper robot and making them available in ROS. With minimal changes as to how the ROS node is launched it can also be used on any other Linux-based robot. This project was directly inspired by the GSCam project, and also uses GStreamer to do the heavy lifting.

Using `getImageRemote()` from the NAOqi API can obtain VGA (640x480) images at a stable frame rate of 24fps if the network bandwidth supports a constant ~18MB/s data rate, which is often challenging if not impossible over WiFi (I get ~0.9fps due to very poor WiFi quality). Streaming video with the API like this also costs around 45% extra CPU usage in the `naoqi-service` service, increasing it from ~20% to ~65%. There is thus little headroom to stream a second camera or use the NAOqi API for other e.g. sensor/control-related purposes.

Using this package it is possible to stream VGA (640x480) video at a stable 24fps over WiFi, requiring only 0.4-1.2MB/s data rate (depending on the JPEG quality used) and 30% CPU that can run in parallel to the `naoqi-service` service (instead of competing with other NAOqi API calls). It is also possible to stream multiple cameras at once. This package is observed to produce less delay between image capture and image delivery than the NAOqi API, and does not suffer from the problem that identical frames may be delivered more than once (this happens if you query `getImageRemote()` 'too quickly'). You can filter out such identical frames based on timestamp, but it causes significant disturbance to the regularity with which you receive and process unique new frames.

Using this package there is also no restriction that forces you to use either Python 2.7 (due to NAOqi Python API) or ROS Kinetic/Melodic (naoqi_driver ROS package limitation), which in turn would limit you to using Ubuntu 18.04 or earlier. Having a ROS node in general, effortlessly allows you to have many tools at your disposal, like for example visualisations or camera calibration routines.

Installation
------------

Install system dependencies on your local PC (Pepper should already have GStreamer 0.10):
```sh
sudo apt install libgstreamer1.0-dev gstreamer1.0-tools
```
Usually your ROS installation is automatically sourced in every bash session due to a line like this in the `~/.bashrc`:
```sh
source "/opt/ros/noetic/setup.bash"
```
If you don't have that then you will need to manually source it in every new terminal/bash session you open. You can always check that ROS has been sourced using `which roscore`, which should return a valid path.

Create a workspace to compile this ROS package into, and symlink the `pepper_camera` folder inside of the workspace (just to avoid a copy):
```sh
mkdir -p PATH/TO/pepper_ros_ws/src
cd PATH/TO/pepper_ros_ws
ln -s PATH/TO/pepper_camera src/pepper_camera
```
Now compile the ROS node using `catkin`:
```sh
catkin_make
```
Every time you enter a new terminal/bash session where you want to do something with the now-compiled ROS node (like launching it or calling the reconfigure service), you need to make sure the ROS workspace you just created is sourced:
```sh
source PATH/TO/pepper_ros_ws/devel/setup.bash
```
The ROS node runs on your computer, not on the robot, but in order to receive images from the robot you will need to launch a GStreamer pipeline on the robot (see Manual Launch below). This can be cumbersome or impractical in some situations, so a daemon/server can be installed onto the robot instead, and made to launch on boot. This waits in deep sleep until it receives a message from the ROS node running on your computer that it needs camera frames. The daemon on the robot then launches a GStreamer process that streams the requested camera feed with the requested parameters (possibly multiple at the same time) to your computer, and keeps it alive until the ROS node exits, at which point it sends a message to the daemon on the robot that it should stop streaming images. To compile and install the daemon, refer to Auto Launch below.

Manual Launch
-------------

Go to the Pepper robot page by entering the robot's IP address in the browser and logging in. In the settings tab disable "Alive by default" and reboot the robot. This is required so that the top camera isn't automatically already in use by the `naoqi-service` when the robot boots, and thereby preventing us from using it in a separate process.

SSH into the Pepper robot and start manually streaming top camera video (to have this happen automatically refer to Auto Launch):
```sh
gst-launch-0.10 v4l2src device=/dev/video0 ! video/x-raw-yuv,width=640,height=480 ! jpegenc quality=QUALITY ! rtpjpegpay ! udpsink sync=false host=IP port=PORT
```
Note that:
* `IP` should be the IP address of the PC you wish to stream the video to, and `PORT` the corresponding target port
* `QUALITY` should be a number 1-100 (try 75 as a good default) specifying the quality level of the used JPEG compression (balances network data rate vs quality)
* `/dev/video0` is the top camera and `/dev/video1` is the bottom camera
* On the Pepper, trying to specify a framerate using `framerate=30/1` does not seem to make a difference (whether higher or lower than 24fps)
* On the Pepper, trying to stream resolutions higher than 640x480 results in corrupted images
* Using a video codec like theora instead of JPEG compression results in much better quality for the same network data rate, but it only achieves 15fps or so due to 100% CPU usage at 640x480

You can *test* that the network stream is arriving at your PC and can be decoded using:
```sh
gst-launch-1.0 udpsrc port=PORT ! application/x-rtp,encoding-name=JPEG ! rtpjpegdepay ! jpegdec ! fpsdisplaysink  # <-- Replace PORT with the correct port
```
You can also in parallel save (in the current folder) the last 300 received JPEGs at any time using:
```sh
gst-launch-1.0 udpsrc port=PORT ! application/x-rtp,encoding-name=JPEG ! rtpjpegdepay ! tee name=jpegstream ! queue ! jpegdec ! fpsdisplaysink jpegstream. ! queue ! multifilesink location="frame%04d.jpg" max-files=300  # <-- Replace PORT with the correct port
```

If you're not getting the framerate you expect, you can check the current WiFi connection speed of the Pepper robot using (on the robot):
```sh
iw dev wlan0 link
```
Here I often get only `19.5 MBit/s MCS 2` (see [MCS table](https://wlanprofessionals.com/mcs-table-and-how-to-use-it)).

You can check the capabilities of a camera (doesn't seem to work so well on the Pepper) using:
```sh
v4l2-ctl --list-formats-ext
v4l2-ctl --device=/dev/video0 --all  # <-- Note that this displays sticky information from the last configured use of the camera
```
For some webcams it might be advantageous to use an MJPEG device format, as opposed to YUV. This potentially provides larger images and higher framerates. Not so for the Pepper though. An example MJPEG GStreamer pipeline would be:
```sh
gst-launch-0.10 v4l2src device=/dev/video0 ! image/jpeg,width=640,height=480 ! rtpjpegpay ! udpsink sync=false host=IP port=PORT
```

Alright, now with the GStreamer pipeline running on the robot (and no GStreamer test pipeline from above still running on your computer), you can launch the Pepper camera node:
```sh
source PATH/TO/pepper_ros_ws/devel/setup.bash         # <-- Only required once per terminal/bash session
roslaunch pepper_camera pepper_top.launch cmd:=false  # <-- Could alternatively launch pepper_bottom.launch
```
We can then check in another terminal that images are arriving:
```sh
rostopic hz /camera/top/rgb -w 72
rosrun rqt_image_view rqt_image_view /camera/top/rgb
```
See Configuration below to see how to configure the launch process and/or reconfigure the running ROS node.

Note that on the Pepper you can stream images larger than 640x480 (up to 2040x1530), but then you must capture at exactly 2560x1920 from the video device and dynamically downscale it to the desired resolution.
```sh
gst-launch-0.10 v4l2src device=/dev/video0 ! video/x-raw-yuv,width=2560,height=1920 ! videoscale ! video/x-raw-yuv,width=2040,height=1530 ! jpegenc quality=70 ! rtpjpegpay ! udpsink sync=false host=134.100.10.212 port=3016
```
This pipeline gives a solid 5.7fps for a suitably good network quality, but there is a noticeable delay between capture and image presentation (albeit essentially fixed). You can also see effects of the rolling shutter if anything moves too fast, because the sensor is just very slow at iterating over its entire sensor array. The `rtpjpegpay` GStreamer element has a hardcoded limit of 2040 for image dimensions for whatever reason, which is why 2040x1530 is the largest size.

Auto Launch
-----------
In order to allow the GStreamer pipeline to be auto-launched on the robot we compile a camera daemon/server. If the Pepper had just any normal Linux OS on it then you could just `scp` the `pepper_camera/server` folder to the robot, and then run `make` inside that folder on the robot (the required `pepper_camera_server` binary appears in the same directory then). But no, the Aldebaran OS doesn't even have a simple compiler like GCC available, so we need to take the unnecessarily hard path of cross-compiling for the Pepper using `qibuild`.

First you will need a Python 2.7 environment, and preferably not a system-wide one. If you're familiar with `conda`:
```sh
conda create -n naoqi python=2.7
conda activate naoqi
```
Install and configure `qibuild` (configuration ends up in `~/.config/qi`):
```sh
pip install qibuild
qibuild config --wizard  # <-- Unix Makefiles / None
```
Create a `qibuild` workspace:
```sh
mkdir PATH/TO/qibuild
cd PATH/TO/qibuild
qibuild init
```
Download and set up the cross-compiling toolchain (and make it the default config):
```sh
mkdir PATH/TO/NAOqi
cd PATH/TO/NAOqi
wget https://community-static.aldebaran.com/resources/2.5.10/CTC/ctc-linux64-atom-2.5.10.7.zip  # <-- Or go to: https://developer.softbankrobotics.com/pepper-naoqi-25-downloads-linux
unzip ctc-linux64-atom-2.5.10.7.zip  # <-- Do NOT extract in file browser graphically as this fails to reproduce the required symlinks!
cd PATH/TO/qibuild  # <-- Important step
qitoolchain create ctc-atom /PATH/TO/NAOqi/ctc-linux64-atom-2.5.10.7/toolchain.xml  # <-- Absolute path AND inside workspace folder are BOTH a must!
qibuild add-config ctc-atom-config -t ctc-atom --default
```
We now have a `ctc-atom` cross-compiling toolchain, and a default `ctc-atom-config` config that uses that toolchain. Let's cross-compile the server:
```sh
cd PATH/TO/qibuild
cp -r PATH/TO/pepper_camera/server/. pepper_camera_server  # <-- Symlinking breaks qibuild so we need to copy!
cd pepper_camera_server
export LC_ALL=C  # <-- Otherwise you get an _nl_intern_locale_data assertion error on build
qibuild configure --release
qibuild make
```
We now copy the produced binary to the robot:
```sh
scp build-ctc-atom-config/sdk/bin/pepper_camera_server nao@PEPPER_IP:~/  # <-- Replace PEPPER_IP with the actual IP or resolvable hostname of the robot
```
We wish for this to launch on boot, so SSH into the Pepper robot and add the following line under the `[program]` section of `~/naoqi/preferences/autoload.ini`:
```
/home/nao/pepper_camera_server
```
Reboot the robot and you should see the daemon in the background idling with zero CPU usage.

Return to your PC and launch the Pepper camera node in auto mode (automatically contacts the Pepper and tells it to start/stop the required GStreamer pipelines). You know what, let's get brave and launch ROS nodes for the top and bottom cameras simultaneously (2 terminals):
```sh
source PATH/TO/pepper_ros_ws/devel/setup.bash               # <-- Only required once per terminal/bash session
roslaunch pepper_camera pepper_top.launch ip:=PEPPER_IP     # <-- Replace PEPPER_IP with the actual IP or resolvable hostname of the robot
roslaunch pepper_camera pepper_bottom.launch ip:=PEPPER_IP  # <-- Replace PEPPER_IP with the actual IP or resolvable hostname of the robot
```
We can then check in another terminal that images are arriving:
```sh
rostopic hz /camera/top/rgb -w 72
rostopic hz /camera/bottom/rgb -w 72
rosrun rqt_image_view rqt_image_view /camera/top/rgb  # <-- You can use the dropdown box to check /camera/bottom/rgb as well
```
In parallel we can check what is happening on the Pepper:
```sh
ps aux | egrep 'pepper_camera_server|gst-launch'
```
We get the following output, showing the Pepper camera server running at 0% CPU (the `--pip` and `--pport` arguments are ignored NAOqi stuff), and two GStreamer pipelines running at 24-30% CPU each:
```sh
nao       3447  0.0  0.0   3352  2408 ?        S    02:12   0:00 /home/nao/allgeuer/pepper_camera_server --pip 127.0.0.1 --pport 9559
nao       3548 30.3  0.2  49912  8928 ?        Sl   03:38   1:04 gst-launch-0.10 v4l2src device=/dev/video0 ! video/x-raw-yuv,width=640,height=480 ! jpegenc quality=70 ! rtpjpegpay ! udpsink sync=false host=134.100.10.212 port=3016
nao       3552 24.2  0.2  49912  8764 ?        Sl   03:39   0:41 gst-launch-0.10 v4l2src device=/dev/video1 ! video/x-raw-yuv,width=640,height=480 ! jpegenc quality=70 ! rtpjpegpay ! udpsink sync=false host=134.100.10.212 port=3017
nao       3559  0.0  0.0   2292   692 pts/0    R+   03:42   0:00 egrep pepper_camera_server|gst-launch
```
Ctrl+C the ROS nodes on your computer and the two GStreamer pipeline processes will automatically be stopped.

From now on it's all easy. Every time you turn on the Pepper robot all you have to do is launch the ROS node on your computer and everything else is managed for you by the daemon/server that's already installed and running:
```sh
source PATH/TO/pepper_ros_ws/devel/setup.bash            # <-- Only required once per terminal/bash session
roslaunch pepper_camera pepper_top.launch ip:=PEPPER_IP  # <-- Replace PEPPER_IP with the actual IP or resolvable hostname of the robot
```
If someone else has already configured the robot for auto-launching, then that is literally all you need to do. You don't need `qibuild` or any of that stuff.

Configuration
-------------
There are many ROS parameters that govern what the ROS node does, e.g. what topics it publishes the images on and in what format, whether it records videos (MJPEG/x264) or jpegs (multifile) to disk in parallel, etc. The best place to see the latest list of ROS parameters is in `src/pepper_camera.cpp` in the definition of the `reset()` function. They are documented there in comments. All ROS parameters starting with `cmd_` are only relevant if `cmd_enabled` is `true`, i.e. only if auto-launching is enabled. Here is an example of setting a ROS parameter on the command line so that the node opens a preview window of the data it is receiving:
```sh
rosparam set /pepper_camera_top/preview true
```
Just note that ROS parameter values set on the command line are overridden by values specified inside the launch file (use args if you need to dynamically change them). If the node is already running then the new parameters are only read in once you call the `reconfigure` service:
```sh
source PATH/TO/pepper_ros_ws/devel/setup.bash  # <-- Only required once per terminal/bash session
rosservice call /camera/top/reconfigure "force: false"
```
By default this only restarts the GStreamer pipeline if a ROS parameter actually changed. You can force it to restart the GStreamer pipeline even if nothing changed by providing `force=true`. Another node can tell when the new GStreamer pipeline is up and running by looking at the return value of the service call (e.g. `config_id: 2`), and waiting until the topic `/pepper_camera_top/config_id` has that value.

If you perform a camera calibration, then you should put the result into a YAML file (see `calibration/pepper_top.yaml` for a template), and then specify a URL to that YAML file using the `camera_info_url` parameter. The camera calibration information will then be available on a topic like `/camera/top/camera_info`.

Feedback
--------
If you have any problems with this repository, feature requests, or feedback, then drop me a line at [philipp.allgeuer@uni-hamburg.de](mailto:philipp.allgeuer@uni-hamburg.de).
