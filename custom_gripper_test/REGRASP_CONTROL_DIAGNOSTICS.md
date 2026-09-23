# Motor response investigation

Update: the register map has now been checked and the loaded small-error trial
has been audited. See [LOADED_REGRASP.md](LOADED_REGRASP.md) for the bounded
position-error ramp, evidence, remaining uncertainty, and full launch commands.
The read-only firmware extension below is already sufficient; no new upload
is needed for the loaded-ramp Python update.

The latest loaded trial accepted goals (1734, 2266), but measured positions
remained (1739, 2260) during the two-second observation window. This is not
proof of grip-force increase, power failure, or a particular servo tuning fault.

The host contact latch does not resend its previous goal during reinforcement.
The firmware SYNC_MOVE uses immediate sync write at address 41, length 7,
matching the vendored SMS_STS implementation. No separate ACTION is required.
The firmware PAIR_MOVE load monitor is a separate potential writer; no
LOAD_STOP message or persistent goal replacement was observed in that trial.

## Read-only extension

Upload `esp32_bluetooth/esp32_bluetooth.ino` to the gripper ESP32, not the
AnySkin QT Py boards. Stop robot control and the Windows bridge first, remove
the object, and keep fingers clear. Existing connection-time opening remains.

DIAG now reads registers 21 through 70 in one transaction. Its JSON retains
`registers_26_70` and adds `registers_21_25`. The host exposes these extra bytes
as `control_registers_raw`, keyed by decimal address. Interpret these against
the exact motor's register map before changing any setting. No EEPROM or
torque settings are written by this diagnostic extension.

Restart the bridge and the existing automatic ROS launch with
`regrasp_diagnostics:=true`. Do not arm the manual probe. The before/after
Motor DIAG records will include `control_settings_available: true` with this
firmware. Old firmware remains readable but reports false.

Capture one supported-object slip trial, including the before/after DIAG and
Regrasp position records. Do not interpret ACK as physical movement. If a
communication fault occurs, inspect the gripper before restarting; TCP
disconnect does not cancel a motor goal.

This extension collects missing evidence. It does not itself fix loaded
reinforcement or establish a safe grip-force limit.
