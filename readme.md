# PS5 Gamepad to control the submarine

## Type name: `Gamepad`
| Name  | Type      | Notes                 |
| ----- | --------- | --------------------- |
| x     | `float32` | Left joystick x axis  |
| y     | `float32` | Left joystick y axis  |
| rise  | `float32` | Activates rise (RT)   |
| sink  | `float32` | Activates sink (LT)   |
| yaw   | `float32` | Right joystick x axis |
| pitch | `float32` | Right joystick y axis |

## On robot
pull the robot branch
```
git clone https://github.com/Cyclone-Robosub/Web_controller.git -b robot
```

## On browser
open [this page](index.html)