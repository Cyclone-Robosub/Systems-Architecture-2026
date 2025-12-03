# PS5 Gamepad to control the submarine

# Type name: `Gamepad`
| Name  | Type      | Value range | Notes                 |
| ----- | --------- | ----------- | --------------------- |
| x     | `float32` | [-1.0, 1.0] | Left joystick x axis  |
| y     | `float32` | [-1.0, 1.0] | Left joystick y axis  |
| rise  | `float32` | [0.0, 1.0]  | Activates rise (RT)   |
| sink  | `float32` | [0.0, 1.0]  | Activates sink (LT)   |
| yaw   | `float32` | [-1.0, 1.0] | Right joystick x axis |
| pitch | `float32` | [-1.0, 1.0] | Right joystick y axis |

## Mapping values
- rise: Thrusters[1, 2, 3, 4] pwm = 1500 + 100 * rise
- sink: Thrusters[1, 2, 3, 4] pwm = 1500 - 100 * sink

## how to run the web controller
1. You **MUST** run the rosbridge server first. tis bridge ros to the browser.
```
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```
2. Build and run the web controller.
   1. Go to the workspace containing the web controller, and run:
   ```
   colcon build
   ```
   2. Run:
      ```
      source install/setup.bash
      
      ```
