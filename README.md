# ros2-hexapod-robot
## Introduction
This is a hexapod robot that has a lot of functions, like joystick control, April tag following, SLAM, and Navigation.
## Test
### PC
Run the ros2 joy node 
```
ros2 run joy joy_node
```
twist_mux
```
ros2 run twist_mux twist_mux --ros-args --params-file src/hexapod_control/hexapod_control/twist_mux.yaml -r cmd_vel_out:=spider/cmd_vel
```
