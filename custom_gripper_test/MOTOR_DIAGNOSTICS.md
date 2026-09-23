# Read-only motor diagnostics

Firmware: `esp32_bluetooth/esp32_bluetooth.ino`.
Upload to the gripper ESP32, NOT either AnySkin QT Py sensor.
Arduino board: ESP32 Dev Module. Partition: Huge APP (3MB No OTA).
Stop the ROS launch and Windows Bluetooth bridge before upload. Select the
ESP32's USB upload port, not its Bluetooth COM11 port unless the OS actually
identifies that as the wired programming port. No automatic upload was performed.

## What changes

`DIAG <1|2> <request>` sends one servo READ (instruction 0x02), address 26,
length 45. It never writes EEPROM, torque, position, or motor parameters.
The reply starts `DIAG ` followed by JSON with ID, request correlation token,
status and all 45 raw register bytes. A read failure is explicit (`ok:false`).
The register mapping follows the vendored Waveshare `SMS_STS.h`:
26/27 deadbands, 40 torque enable, 42/43 commanded position,
56/57 present position, 58/59 speed, 60/61 load, 62 voltage,
63 temperature, 69/70 current. Values remain RAW, including sign bits;
do not interpret current/load as calibrated contact force.

Diagnostic mode is OFF by default. When enabled it replaces the position reads
before a confirmed reinforcement and during the existing one-second post-ACK
polling window. No second client, reader thread or independent bus writer is
started. Normal contact-stop reads stay unchanged. More bytes and read latency
can affect experimental timing; the 250ms event expiry is NOT relaxed.
An unsupported command, failed read or timeout disables diagnostics until restart;
no motion is replayed. A failed preflight position read cancels that reinforcement.
Existing communication fault behavior is unchanged.

The full system is NOT read-only: the existing startup opening, calibration,
lever control and enabled reinforcement still move motors. A DIAG command alone
is read-only. Connection behavior of the existing firmware is unchanged.

## Windows bridge

After uploading, start PowerShell:

```powershell
cd "C:\Users\kyss0\OneDrive\Desktop\DocumentsGELLO_Project\gello_software\custom_gripper_test\esp32_bluetooth"
python windows_bluetooth_tcp_bridge.py --port COM11 --host 127.0.0.1 --tcp-port 15555 --trace
```

## WSL control and report

```bash
cd /mnt/c/Users/kyss0/OneDrive/Desktop/DocumentsGELLO_Project/gello_software/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
ls -l /dev/serial/by-id/
sudo chmod a+rw /dev/ttyACM*
GELLO=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B79033706-if00
S1=/dev/serial/by-id/usb-Adafruit_QT_Py_M0_DA493999504B5146342E3120FF0C3227-if00
S2=/dev/serial/by-id/usb-Adafruit_QT_Py_M0_57B17757504B5146342E3120FF0B0225-if00
ros2 launch ur5e_gello_state_publisher ur3_gello_anyskin_control.launch.py \
  gello_port:="$GELLO" robot_ip:=192.168.50.14 anyskin_ports:="$S1,$S2" \
  bluetooth_host:=127.0.0.1 bluetooth_port:=15555 firmware_hold:=false \
  close_speed:=2919 close_acc:=22 contact_margin:=20 \
  haptic_release_delta:=0.03 \
  empty_close_baseline_percentile:=75 \
  slip_regrasp:=true slip_regrasp_mode:=learned regrasp_diagnostics:=true \
  record_anyskin_raw:=false body_resistance:=false body_resistance_bench:=false \
  tracking_report:=true
```

Perform the existing empty calibration with nothing between the jaws. Firmware
upload and diagnostic reads have not been hardware-tested by the agent. Reported
overpressure/reset symptoms are not repaired by adding diagnostics.

## Inspect the result

`Motor DIAG:` console records have `phase:before` or `phase:after`.
`goal_raw` is read from the MOTOR, while `commanded_target_raw` is the PC target.
Compare same-ID records before/after; these are serial observations, not simultaneous
measurements of both motors. `actual_raw` is preserved in usual position telemetry.
The same JSON records go through the existing ROS timing publisher and are saved
to `gripper_motor_diagnostics.jsonl` in the report folder when recording finishes.
These diagnostic records do not enter the joint-latency summary calculations.

- Goal never changes: inspect bus delivery, status, and firmware command path.
- Goal changes then returns: inspect bridge trace for another position/HOLD write.
- Goal stays new but actual stays unchanged: inspect deadbands, torque state,
  raw current/load and voltage changes. This alone does not identify the cause.
- Failed DIAG: retain the failure; do not fabricate zero readings or disable guards.

`motion unverified` is not evidence that the motor applied zero force.
To disable only diagnostics, use `regrasp_diagnostics:=false`.

The integrated launch now shares `haptic_release_delta` (default 0.03) between
lever haptic release and gripper contact release. Small 1-1.2% lever recoil no
longer releases either latch. Full-open input bypasses the deadband. The two
nodes retain their existing local hold anchors; this is a common threshold, not
a synchronized timestamp/anchor. No torque or reinforcement travel limit changed.
