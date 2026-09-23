# Checked AnySkin sensor firmware

Target: Adafruit QT Py M0 (SAMD21), NOT the ESP32 gripper board.
Based on local `Downloads/sketch_aug6b`, with its MLX90393 driver preserved
and patched. Original source is backed up in `backups/anyskin_original_*`.
Driver license: MIT, see LICENSE. No automatic upload was performed.

## Fixed

- Never converts an uninitialized raw structure after an I2C failure.
- Checks initialization, configuration cache, sensor status and read results.
- Rejects error-status register responses before caching them.
- Checks the previously ignored Hall configuration result.
- A read/init failure latches that chip invalid until reset or explicit retry.
- If any of five chips is invalid, the entire binary frame is NaN; no fabricated
  zeros, previous readings or copied channels are substituted.
- DRDY wait in the single-shot API has a timeout.

Normal output remains 82 bytes: five little-endian float32 groups T,X,Y,Z,
followed by CR LF. V3 uses single-shot measurement, starts all five chips, waits
10ms, then reads. Frame rate is capped at 50 Hz for diagnosis. Gain=7, filter=2,
OSR=0, OSR2=0, external trigger disabled, 400 kHz I2C. This changes sensor output
cadence, not motor speed. V2 burst reads returned error status 0x90..0x93 on the
user's board. Data-not-ready is one possible cause, NOT proven by these bytes.
The V3 change is a diagnostic fix; validate hardware before resuming control.
Manufacturer status reference:
https://www.melexis.com/-/media/files/documents/datasheets/mlx90393-datasheet-melexis.pdf
V2 addresses match the reported board scan: slots M1=0x0C, M2=0x10,
M3=0x11, M4=0x12, M5=0x13. Verify the second board's scan independently.
These are stream slots; physical locations/model channel order are not yet
verified. Recalibrate after changing firmware; do not reuse the old ordering.
No EEPROM commands, address reassignment, or motor commands are used.

## Upload and diagnose first

1. Stop ROS/control, slip observer, probes and serial monitors. Support/release
   any object first. Do not upload while the gripper is controlling an object.
2. Connect one QT Py sensor board at a time to Windows. If attached to WSL,
   detach that USB device first; confirm its identity rather than guessing COM.
3. Open `anyskin_qtpy_checked.ino` in Arduino IDE. Keep the adjacent .cpp/.h files.
4. Select **Adafruit QT Py M0 (SAMD21)** and that board's USB serial port.
   Compile/upload. Do not select the ESP32 or COM11 Bluetooth bridge.
5. Open Serial Monitor at 115200 and send `D` (newline optional). Initial binary
   text is expected before D. The stream becomes readable diagnostic text.
6. Inspect all five lines: ack=1, init=0x0, ready=1, increasing good count,
   errors=0, finite txyz values. `start` is the single-shot command response;
   do not require every raw status byte to be zero.
7. If a chip fails, capture the full I2C_ACK line and M1..M5 lines. `R` in
   diagnostic mode reruns initialization and resets counters. Missing addresses
   require wiring/address/power investigation; a firmware patch cannot invent
   missing sensors. Do not swap discovered addresses without mapping the board.
   V2 `cause` codes: 0=OK, 1=command write failure, 2=I2C NACK,
   3=short reply, 4=missing byte, 5=sensor error status, 6=cache read failure.
8. Repeat for the other sensor. Restart the board or send `S` to return to
   binary output, then close Serial Monitor before WSL attaches/opens the port.

Only resume grasp tests after BOTH boards pass diagnosis. Run a NEW empty-close
calibration with all sensors unloaded. Old calibration based on invalid M2..M4
is not valid. Existing PC contact parsing is intentionally unchanged (legacy
T,X,Y); the separate pretrained-model input uses correct X,Y,Z. This release
does not certify contact stopping or slip detection. Auto reinforcement stays
OFF. USB/I2C hardware hangs and errors cannot guarantee a physical robot stop.

The new firmware exposes previously hidden failures, so an actually missing
chip will now prevent valid readings instead of appearing to work.
