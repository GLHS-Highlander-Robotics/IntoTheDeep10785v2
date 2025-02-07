# IntoTheDeep10785v2
# Introduction
*This repository is in a very early stage of development.* 
*This `README.md` may not be appropriately updated to account for recent changes. Please update appropriately if you notice an issue*
- This repository contains the code for version 2 of team 10785's IntoTheDeep robot.

## Future Additions
- Converting currently used LinearOpMode to OpMode
- PID drive
  - Determining position
  - Using position to make autonomous drivetrain path

## Initialization
**NOTE: ALL MOTORS ARE KNOWN AS "GoBILDA 5202/3/4 series"**

### Field Centric
| **Port**  | **Device Type**   | **Name in Config** |
|-----------|-------------------|--------------------|
| 0         | Front Left Motor  | `frontLeft`        |
| 1         | Front Right Motor | `frontRight`       |
| 2         | Back Left Motor   | `backLeft`         |
| 3         | Back Right Motor  | `backRight`        |
| N/A       | Rev Internal IMU  | `imu`              |

*IMU is in `I2C Bus 0`*

## Controls
### Field Centric
| Gamepad 1 input   | Output             |
|-----------------|--------------------| 
| `left_stick_y`  | Forward-Backward   |
| `left_stick_x`  | Left-Right         |
| `right_stick_y` | *Unbound*          |
| `right_stick_x` | Rotation           |
| `dpad_up`       | Forward 30% power  |
| `dpad_down`     | Backward 30% power |
| `dpad_left`     | Left 30% power     |
| `dpad_right`    | Right 30% power    |
| `left_trigger`  | *Unbound*          |
| `right_trigger` | *Unbound*          |
| `left_bumper`   | *Unbound*          |
| `right_bumper`  | *Unbound*          |
| `x`             | Reset Orientation  |
| `y`             | *Unbound*          |
| `b`             | *Unbound*          |
| `a`             | *Unbound*          |
| `back`          | *Unbound*          |
| `start`         | *Unbound*          |

| Gamepad 2 input   | Output             |
|-----------------|--------------------| 
| `left_stick_y`  | Forward-Backward   |
| `left_stick_x`  | Left-Right         |
| `right_stick_y` | *Unbound*          |
| `right_stick_x` | Rotation           |
| `dpad_up`       | Forward 30% power  |
| `dpad_down`     | Backward 30% power |
| `dpad_left`     | Left 30% power     |
| `dpad_right`    | Right 30% power    |
| `left_trigger`  | *Unbound*          |
| `right_trigger` | *Unbound*          |
| `left_bumper`   | *Unbound*          |
| `right_bumper`  | *Unbound*          |
| `x`             | Reset Orientation  |
| `y`             | *Unbound*          |
| `b`             | *Unbound*          |
| `a`             | *Unbound*          |
| `back`          | *Unbound*          |
| `start`         | *Unbound*          |
