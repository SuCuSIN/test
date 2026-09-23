# Experimental contact resistance

This is a grasp cue, not weight measurement, force control, or a safety stop.
No change to gripper endpoints, speed, contact detection, or existing lever hold.
Opt-in launch parameter: `body_resistance:=true` (default false).
Only controller shoulder servo ID 2 is used. Confirm this mapping on your hardware.

Implementation: position-mode servo, SRAM Torque Limit register 48 set to 10
(nominal 1% register setting; not calibrated shaft torque), maximum two encoder
counts of target lag. Fresh movement >=2 counts per update and fresh AnySkin
contact heartbeat are required. At rest, release, stale contact, or invalid read,
torque OFF is attempted. Position mode and output limit are read back before
enabling output. Errors latch the feature off and attempt release; restart to rearm.
No EEPROM/mode changes. The low limit remains in SRAM after shutdown.

This approximates resistance using active position control, NOT guaranteed passive
damping. Small return motion, friction, and weak/imperceptible output are possible.
A stalled/killed host or broken cable can leave torque active. There is no verified
servo-side watchdog. Keep controller power disconnect accessible; do not force it.

## First test

Stop other ROS control publishers first. Keep the Windows Bluetooth bridge running.
Use the normal AnySkin launch arguments plus:

```text
body_resistance:=true body_resistance_bench:=true
```

Bench mode prevents this publisher from connecting to RTDE control or publishing
robot/gripper motion commands. Other running nodes are NOT stopped by bench mode.
The AnySkin process still operates/calibrates the physical gripper through its web
controls, so keep fingers clear. Existing lever contact hold remains enabled.
Use a nonfragile object with web Close Safe to establish contact, then gently move
the supported controller shoulder. Open clears contact and should remove resistance.
Do not proceed to robot tracking if output/release is unexpected or fault logs occur.

During a running test, request OFF with:

```bash
ros2 param set /ur5e_gello_publisher body_resistance false
```

This needs a responsive process and communication link; it is NOT an emergency stop.
After bench verification, restart with `body_resistance_bench:=false` for robot
tracking. This setting is startup-only. The force preview script is not required.

Register reference: FEETECH SDK
https://gitee.com/ftservo/FTServo_Arduino/blob/main/src/SMS_STS.h
