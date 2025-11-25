# Custom Interfaces
This package contains msg/srv/action specfication that we use for our 2025-26 year. Inside this document is also a list of topic that our node interacts with.

## External references
See the list and docs for the common interfaces (msg/srv) here
https://github.com/ros2/common_interfaces/tree/jazzy

The standard interfaces docs is under std_msg (Documented in [Ros2 documentation](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Interfaces.html) as well as the repo)
The other messages are under there respective sub folder in the directory

## References
### Nodes:
TODO: List of **all** nodes with topics that they publish from/listen to.

### Custom message types:
in `Position.msg` file:

```cpp
float32[6] position # [x, y, z, roll, pitch, yaw]
```

in `Pwms.msg` file:

```cpp
int32[8] pwms
```

in `State.msg` file:

```cpp
Position position 
Position velocity 
geometry_msgs/Vector3 acceleration # [x, y, z], type provided by ROS
```

in `Control_mode.srv` file:

```cpp
bool request_mode # matlab = 1, manual = 0
--
bool status 
```

in `Gamepad.msg` file: see https://github.com/Cyclone-Robosub/Web_controller/blob/main/readme.md for details
    
    ```cpp
    float32 x
    float32 y
    float32 rise
    float32 sink
    float32 yaw
    float32 pitch
    ```
### Topics
1. `pwm_cmd` - instantaneous PWM from the mux that is sent the thrusters: Pwms.msg type
2. `pwm_cli` - PWM from manual command line: Pwms.msg type
3. `pwm_ctrl` - PWM from matlab controller: Pwms.msg type
4. `waypoint` - instantaneous waypoint: Position type
5. `state` - instantaneous best state estimate,  typically output by sensor fusion: 
    1. position: list of 6 floats (x, y, z, roll, pitch, yaw)
    2. velocity: list of 6 floats (x, y, z, roll, pitch, yaw)
    3. acceleration: list of 3 floats (x, y, z)
6. `current_control_mode` - Which control mode the robot is currently in (designed to be read by Jason's stuff)

7. Vision Related Topics: TBD
    - seen_objects, includes vector_to_object - list of the objects the camera system has identified with meta data like distances, confidences
    - raw_image - raw image files
    - vision_velocity - velocity estimate from vision algorithms
8. `ps5_controller` - Remote Control Related Topics (Gamepad.msg)
    
9. DVL related topics
    - sensor_ctrl - for all sensors # Probably deprecated
    - ~~sensor_toggles~~ - for all sensors
    - dvl_data: Position type
10. Battery
- bms - battery data: voltage (float32), current (float32)

#### Heartbeats:
These are boolean sent from the mux to the thruster interface to indicate that it's alive and kicking. If not received, the receiver sends a stop command after 1 second. The value passed (true or false) has no effect on the behaviour of the heartbeat, but sending true is probably advised for readability.
1. `mux_heartbeat` - sent from mux to thruster interface
2. `ctrl_heartbeat` - sent from matlab to mux
3. `cli_heartbeat` - sent from cli to mux


### Services & Actions

1. `is_matlab_control_mode` - service return is new mode: Bool (true=matlab, false=manual)
2. `[sensor]_settings` - service containing sensor settings (vision, DVL, maybe BMS)
    1. one of these services per sensor. Each has unique data corresponding to individual sensor specs, plus a toggle field
3. TBD action about updating waypoints
4. `trigger_dropper` - service to trigger manipulations dropper. We're expecting this to be sent to a seperate microcontroller (probably a Pico) to the robot thrusters Pico. Like the heartbeats, this probably contains just a boolean without intrinsic meaning: the receipt of the signal itself contains all the information we need to take action (i.e. release the dropper).
