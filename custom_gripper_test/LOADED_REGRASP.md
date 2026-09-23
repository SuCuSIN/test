# Loaded reinforcement: evidence and bounded correction

## Controller input recovery

In learned mode, a missing/timed-out controller input before transmission now
pauses reinforcement instead of permanently inhibiting that grasp. Pending
manual probes and ramp continuations are discarded. Recovery requires at least
three distinct fresh input timestamps spanning 0.2 seconds, then three fresh
model samples <=0.30 on both sensors, then a new confirmed slip. No queued slip
is replayed. Attempts, grasp travel origin, cooldown and pressure reference are
retained. Invalid baselines, motor/ACK faults and unverified in-flight motion
remain inhibited; this recovery never clears them.

Fresh lever commands are published before UR3 RTDE read/send calls, independently
of body alignment. Missing servo reads do not renew the command heartbeat from
cached positions. Verbose pre-send state logs now use DEBUG; ACK and physical
motion results remain available. This reduces avoidable delays but does not make
the shared publisher timer independent of a blocked RTDE call.

No new launch arguments are required. Rebuild the ROS publisher and restart the
existing launch after stopping the prior session. Hardware validation is still
required; software tests do not prove loaded motor movement.

## Diagnostic timing

Missing or stale motor diagnostics pause further commands while fresh replies
are collected. The command window remains 2.5 seconds; up to 0.75 additional
seconds are allowed for observation only, never another closing step. Missing
feedback still ends the episode without success. Travel, output, contact and
sensor-rise limits are unchanged. Full motor diagnostic JSON remains on the
recording topic; console copies now use DEBUG to reduce synchronous log output.

## Evidence

The 38babb79 trial registered goals (1748, 2249), speed 2919, acceleration 22,
position mode 0, torque enabled, torque limit 1000. Both goal readbacks stayed
correct but encoder positions stayed (1751, 2244) for two seconds. No persistent
old HOLD/lever target overwrite is present in those observations.

Waveshare's [ST3215 register map V3.7](https://files.waveshare.com/upload/2/27/ST3215%20memory%20register%20map-EN.xls)
identifies decimal 21/22/23 as position P/D/I, 24 as minimum starting output,
and 60 as output PWM duty (not measured torque). The installed settings read
P=32, D=32, I=0, minimum output=16. The vendored SMS_STS.cpp ReadLoad decodes
bit 10 as direction: 1072 means -48, not an output of 107.2%.

After the command, jaw 1 duty was 3.2%; jaw 2 duty was about 4.8-5.6%.
This supports insufficient small-error drive under load as the working
hypothesis, rather than a missing motion instruction. It does not distinguish
static friction from object compliance/blockage, or rule out unsampled power
transients. Speed 2919 and torque ceiling 1000 do not demand full torque.

The older 20260916 tracker paused *new commands* while confirming contact but
did not replace outstanding travel with a measured-position hold until contact
was confirmed. That could produce the earlier unwanted extra squeeze. Do not
restore that behavior on first contact.

## Changed automatic actuation

Initial arming now uses 0.30 seconds of contact settling instead of requiring
three low model scores. Three fresh contacting-sensor scores >=0.80, acquired
after the settling interval, confirm the first event. Repeated timestamps and
samples acquired during settling do not count. The monitor shows CONTACT
SETTLING during this interval. After reinforcement, the existing cooldown and
three fresh scores <=0.30 are still required before another episode. A
sustained false high can therefore cause the first episode, but cannot by
itself rearm repeated reinforcement. Sensor validity and live-contact checks
are unchanged.

For learned automatic reinforcement with `regrasp_diagnostics:=true`:

- Keep first-contact stop, lever lock, speed 2919 and acceleration 22 unchanged.
- One confirmed slip starts one bounded episode, not repeated new slip events.
- Advance goals by 3 counts, then read back the registered goal and encoder.
- Require two fresh observations per jaw and at least 0.4 seconds before advancing.
- A jaw with closing displacement of at least 2 counts in two observations gets
  no more goal advances in this episode. The other jaw can still advance.
- Maximum 4 stages, 12 counts from episode starting measured position,
  24 counts from original contact target per grasp, and 2.5 seconds for progression.
- Fresh sensor/model data, live contact, the existing sensor-rise guard and
  opening priority remain required. A confirmed slip need not stay high in
  every later model prediction while this short episode executes.
- No advance on goal mismatch, motor status fault, disabled torque, non-position
  mode, stale feedback, observed absolute duty >=15%, or current raw >=100.
  These are **observed-output checks, not hardware current/force limits**.
- No blind replay after ACK loss. No EEPROM, PID, torque-limit or firmware changes.
- Manual probe and non-diagnostic/heuristic paths retain their previous bounds.

The larger bounded goal error is intentional: the old fixed 3-count command
could remain stationary under load. This may increase pressure. A rigid object
can prevent encoder motion even when pressure rises; limits terminate escalation,
and success means encoder displacement only, never verified holding force.
The last acknowledged goal remains active after an episode ends or is inhibited.

## Run

Stop the previous ROS launch before restarting. Do not use `probe_arm` for this
automatic test. Firmware re-upload and ROS rebuild are not required for this
Python change. Keep the first test object supported on the table, keep hands
clear, and use a non-fragile object. Do not lift until holding is verified.

Windows bridge (only restart if not already running):

```powershell
cd "C:\Users\kyss0\OneDrive\Desktop\DocumentsGELLO_Project\gello_software\custom_gripper_test\esp32_bluetooth"
python windows_bluetooth_tcp_bridge.py --port COM11 --host 127.0.0.1 --tcp-port 15555 --trace
```

WSL:

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

Complete startup empty-close calibration and open the lever to enable control.
On automatic slip, check `loaded_ramp=True`, then `bounded_step=1/4`, `2/4`, etc.
Stages stop when encoder response is confirmed; they need not reach 4/4.
`closing displacement observed on both jaws; force not verified` is the motion
result. An ACK alone is not success. Ctrl+C saves the tracking report.

Hardware-independent tests exercise the production target/observation methods
with stationary and responding encoder samples. They do not validate motor
breakaway or grip force on the physical gripper.
