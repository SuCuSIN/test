# AnySkin XYZ recording

The Bluetooth AnySkin shell now records decoded magnetic XYZ by default.
ROS launch toggle: `record_anyskin_raw:=false` disables it.
Standalone toggle: `--record-anyskin-raw false`.

Output: `custom_gripper_test/anyskin_raw_records/<timestamp>/`
- `raw_xyz.csv`: sensor index, per-stream sample sequence, PC monotonic receive
  timestamp, then M1..M5 Bx/By/Bz. Signs and duplicate-looking channels are retained.
- `calibration.jsonl`: observed per-sensor zero baselines, their effective PC
  monotonic timestamp and calibration metadata.
- `metadata.json`: sensor ordering, units caveat and timestamp description.
- `summary.json`: written, dropped, invalid counts and recording errors on exit.

These are decoded pre-filter magnetic samples, not original serial bytes, force,
slip direction, or temperature. Do not equate magnetic axes with physical slip
axes. Signed baseline-relative XYZ can be computed offline using the corresponding
calibration epoch. A receive timestamp is not a hardware acquisition timestamp;
the two boards are not synchronized. Compare with report observed_pc_ns / 1e9.

The recorder starts after initial sensor calibration; older startup samples are
excluded. Separate disk-writing thread copies existing bounded stream buffers
without consuming them, reopening ports, changing calibration, updating contact
filters, sending motion, or feeding data into regrasp/haptics. It captures available
new samples rather than polling the magnitude display. Disk delays can cause buffer
overruns; gaps are counted, not silently presented as continuous data. Recording
adds CPU/disk load, so zero impact on timing is not guaranteed.

Recording errors disable recording, not control. Existing decoded-data parsing,
contact/hold/regrasp rules and control settings are unchanged. Ctrl+C normally
flushes CSV and summary; process kill or a hung disk can prevent final flush.
These files are separate from tracking reports and may grow large over long runs.
