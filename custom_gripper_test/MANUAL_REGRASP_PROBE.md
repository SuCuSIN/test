# Same-connection manual reinforcement probe

This is a trigger-isolation test, not an independent motor driver. It bypasses
the automatic slip trigger and uses the existing single motion worker. Contact,
sensor freshness, opening priority, position limits and the cumulative six-count
limit remain active. The explicit manual request uses the existing contact latch:
a relaxed live contact amplitude alone does not cancel it. Fresh finite sensor
data and the signal-rise cap remain required. This does not prove an object is
still present; the operator must check the supported object before requesting.
Automatic slip reinforcement still requires live sensor contact.
One request is consumed even if subsequently cancelled.
No replay, reconnect, torque setting changes or firmware upload.

Keep the object supported on a surface, fingers clear. Motor goals can remain
active after the program exits. Existing power reliability has not been resolved.

## Windows bridge

Leave the existing bridge running. Only start this if it is not already running:

```powershell
cd "C:\Users\kyss0\OneDrive\Desktop\DocumentsGELLO_Project\gello_software\custom_gripper_test\esp32_bluetooth"
python windows_bluetooth_tcp_bridge.py --port COM11 --host 127.0.0.1 --tcp-port 15555 --trace
```

## WSL control

Stop the old ROS launch with Ctrl+C. This launch starts UR3 body control too.

```bash
cd /mnt/c/Users/kyss0/OneDrive/Desktop/DocumentsGELLO_Project/gello_software/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
sudo chmod a+rw /dev/ttyACM*
GELLO=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B79033706-if00
S1=/dev/serial/by-id/usb-Adafruit_QT_Py_M0_DA493999504B5146342E3120FF0C3227-if00
S2=/dev/serial/by-id/usb-Adafruit_QT_Py_M0_57B17757504B5146342E3120FF0B0225-if00
ros2 launch ur5e_gello_state_publisher ur3_gello_anyskin_control.launch.py \
  gello_port:="$GELLO" robot_ip:=192.168.50.14 anyskin_ports:="$S1,$S2" \
  bluetooth_host:=127.0.0.1 bluetooth_port:=15555 firmware_hold:=false \
  close_speed:=2919 close_acc:=22 contact_margin:=20 haptic_release_delta:=0.03 \
  empty_close_baseline_percentile:=75 slip_regrasp:=true slip_regrasp_mode:=learned \
  regrasp_diagnostics:=true record_anyskin_raw:=false \
  body_resistance:=false body_resistance_bench:=false tracking_report:=true
```

Calibrate empty close, then fully open the lever and wait for the gripper to open.
In a SECOND WSL terminal arm before grasping (this sends no movement):

```bash
curl -sS --max-time 3 -H 'Content-Type: application/json' -d '{"command":"probe_arm"}' http://127.0.0.1:8765/command
```

Require `ok: true`. Automatic reinforcement stays disabled until probe_off or
process restart, including across opening/calibration. Grasp normally and wait
for confirmed contact. Keep lever steady. Request one measured-position +6 raw
closing goal, speed 2919 / acceleration 22:

```bash
curl -sS --max-time 3 -H 'Content-Type: application/json' -d '{"command":"probe_once"}' http://127.0.0.1:8765/command
curl -sS --max-time 3 http://127.0.0.1:8765/command_status
```

`ok: true` means queued, not motion success. Look for `trigger=manual_probe`,
`Regrasp ACK`, then `Regrasp position` closing_delta values. Opening cancels
pending motion; repeated requests do not produce repeated closing steps.
Open fully and arm again for a new trial. Do not increase travel to force motion.

To restore automatic reinforcement, fully open first, then:

```bash
curl -sS --max-time 3 -H 'Content-Type: application/json' -d '{"command":"probe_off"}' http://127.0.0.1:8765/command
```
