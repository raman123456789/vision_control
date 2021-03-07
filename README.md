# vision_control

cd ~/zed-ros-wrapper/zed_wrapper/params
vim zed2.yaml
at line nubber 20
19 object_detection:
20    od_enabled:                 false
enable Object detection by it by replacing false to true
it shoyld be 

20    od_enabled:                 true

then you can your zed_wrapper using 

roslaunch zed_display_rviz display_zed2.launch 
( i assume you cloned it from https://github.com/stereolabs/zed-ros-examples)

now run (source the terminal)
rosrun vision_control cv_bridge_test.py

then you run the demo customer 
roslaunch demo_customer demo_customer.launch channel:=can0