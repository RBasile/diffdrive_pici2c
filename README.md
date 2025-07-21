# diffdrive_pici2c

This node is designed to provide a ros2_control hardware interface for an PIC running a custom firmware.
It is designed to be used with a `diff_drive_controller` from `ros2_control`.
It is expected to communicate via serial and to have two motors, each with velocity control and position/velocity feedback.


It is based on the diffbot example from [ros2_control demos](https://github.com/ros-controls/ros2_control_demos/tree/master/example_2).

For a tutorial on how to develop a hardware interface like this, check out the video below:

https://youtu.be/J02jEKawE5U


## cmd

```
colcon build --symlink-install 
source install/setup.bash 
ros2 launch  diffdrive_pici2c diffbot.launch.py
```

```
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -r /cmd_vel:=/diffbot_base_controller/cmd_vel
```


## To Do

- [ ] remove not used parameters
- [ ] Clean up remaining connections to original demo code