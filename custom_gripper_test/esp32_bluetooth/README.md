# ESP32 Bluetooth test firmware

This Arduino sketch adds a Classic Bluetooth Serial Port Profile (SPP) endpoint
named `STS3215_Gripper` to a Waveshare Servo Driver with ESP32 (SKU 21593).
It uses the board's servo UART on GPIO 18/19 at 1 Mbps.

Supported newline-terminated commands:

- `PING`: returns `PONG`.
- `OPEN`: opens both gripper servos to paired position 0.
- `CLOSE`: closes both gripper servos to paired position 500.
- `SET_ZERO`: sets each detected servo's current physical position as its
  calibrated middle position (raw 2047), represented by the application as
  logical position 0.
- `READ <id>`: reads raw and zero-based logical position.
- `MOVE <id> <logical> [speed] [acc]`: moves a servo in calibrated logical
  coordinates. Logical 0 corresponds to raw 2047.
- `RAW_MOVE <id> <raw> [speed] [acc]`: moves directly in the STS3215 raw
  encoder range 0 through 4095.
- `SYNC_MOVE <id1> <pos1> <id2> <pos2> [speed] [acc]`: starts two servo moves
  simultaneously using one STS3215 broadcast sync-write packet.
- `PAIR_MOVE <position> [speed] [acc]`: moves ID 1 to the requested logical
  gripper position from 0 (open) to 500 (closed). Each servo is independently
  mapped to its calibrated raw range.

The configured safe raw ranges include 20 ticks of margin from measured travel:

- ID 1: open 2430, closed 1553
- ID 2: open 1547, closed 2424

Both servos therefore travel exactly 877 raw encoder ticks over a full 0 to 500
gripper command.

Paired moves monitor both servos every 50 ms. If either servo reports current
raw value 100 or greater for two consecutive samples before reaching the
target, the firmware writes both current positions back as a synchronized hold
target. Torque remains enabled, the Bluetooth client receives `LOAD_STOP`, and
the OLED shows `LOAD STOP / Holding position`.

Commands outside these limits are rejected for IDs 1 and 2. When a Bluetooth
controller connects, the gripper automatically moves to open position 0 at
speed 100 and acceleration 10. `SET_ZERO` and `CLEAR_OFFSET` are locked for the
configured gripper IDs because changing calibration would invalidate the raw
limits.
- `TORQUE_ON <id>` / `TORQUE_OFF <id>`: enables or releases a servo.
- `SET_ID <current_id> <new_id>`: permanently changes a servo ID after checking
  that the new ID is unused. Connect only the servo being changed.
- `CLEAR_OFFSET <id>`: permanently clears the stored middle-position offset so
  raw coordinates use the servo's unshifted magnetic encoder reference.

Startup and Bluetooth connection no longer modify servo calibration. `SET_ZERO`
and `CLEAR_OFFSET` are explicit commands because both permanently change servo
settings.

## Compatibility

- Use an original ESP32 with Classic Bluetooth support, such as ESP32-WROOM-32.
- ESP32-S2, ESP32-S3, ESP32-C3, and ESP32-C6 do not support this
  `BluetoothSerial` SPP sketch.
- The CP2102 entry identifies a USB-to-UART bridge, not the exact ESP32 model.
  Confirm the module marking before uploading.

## Upload

1. Open `esp32_bluetooth.ino` in Arduino IDE.
2. Install the Espressif `esp32` board package if it is not already installed.
3. Select `ESP32 Dev Module` and the board's Windows COM port. The OLED driver
   is included in the sketch, so no external Arduino libraries are required.
4. Upload the sketch and open Serial Monitor at `115200` baud.
5. Connect the servo and supply the driver board through its DC jack with a
   voltage suitable for the STS3215. USB alone does not power the servo.
6. Pair with `STS3215_Gripper`, connect using a Bluetooth serial terminal, and
   send `PING` followed by LF/newline.

Expected USB serial output:

```text
Bluetooth started: STS3215_Gripper
Commands: PING, SET_ZERO
```

The onboard 128x32 SSD1306 OLED uses address `0x3C`, SDA GPIO 21, and SCL GPIO
22. It shows boot, servo scan, Bluetooth ready/searching, connected,
disconnected, and startup error states.

Calibration does not rotate the servo. Internally, the STS3215 defines the
calibrated point as raw position 2047; software should subtract 2047 when a
zero-based logical value is required. The initial firmware intentionally scans
only IDs 0..20. Increase `kLastServoId` only if a known servo uses a higher ID.

## Control from VS Code

Pair `STS3215_Gripper` in Windows and identify its outgoing Bluetooth COM port.
Run the following commands in the VS Code PowerShell terminal:

```powershell
python -m pip install pyserial
python bluetooth_servo_control.py --list
python bluetooth_servo_control.py --port COM8
```

Replace `COM8` with the Bluetooth outgoing port. Start with a small movement:

```text
PING
READ 1
MOVE 1 50 100 10
READ 1
MOVE 1 0 100 10
```

To prepare two factory-default ID 1 servos, connect only the first servo and
run `SET_ID 1 2`. Power down the board, connect both servos, and then verify
them independently with `READ 1` and `READ 2`.

## Measure safe travel

Close the interactive controller, then run the range measurement tool. It
releases torque on IDs 1 and 2, displays their raw positions while they are
moved by hand, and records observed minima and maxima.

```powershell
python measure_servo_range.py --port COM10 --ids 1 2
```

Press `Ctrl+C` after moving the mechanism through its intended safe travel.
Torque remains off when measurement finishes.

## GELLO controller bridge

The ROS 2 `bluetooth_gripper_bridge` node subscribes to `gello/gripper_raw`,
maps controller raw 4000 to open position 0 and raw 3459 to closed position 500,
then filters and rate-limits commands before sending them at 20 Hz to COM10.
The firmware load-stop latch rejects continued closing commands after contact
until the GELLO controller moves at least 20 command units toward open.
