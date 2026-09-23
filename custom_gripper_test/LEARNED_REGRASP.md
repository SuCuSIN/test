# Experimental learned slip reinforcement

Enable explicitly with ROS launch arguments:

```bash
slip_regrasp:=true slip_regrasp_mode:=learned
```

Default reinforcement remains OFF. `heuristic` retains the previous sensor-range
algorithm; do not select it when testing the pretrained model trigger.
The standalone `pretrained_anyskin_slip.py` remains read-only and is not required
to run reinforcement. The controller loads the same hash-checked model in a
background thread. Model failure disables model-triggered reinforcement, not
ordinary tracking. Status exposes `regrasp_model_error` and the latest prediction.

Conditions and limits:
- Confirmed contact, two valid fresh sensors, and contact on the triggering sensor.
- Reinforcement-only contact hysteresis: a sensor must first reach the configured
  margin (20 by default); contact is retained down to half that margin (10).
  This does not change initial-contact stopping or lever feedback. Both sensors
  below their release thresholds pause reinforcement immediately; after 0.2s
  reinforcement is inhibited for the grasp.
- Three distinct post-contact samples on that sensor scoring >= 0.9.
- A confirmed event can survive a subsequent lower score during position reads,
  but all confirming timestamps must still be <=250ms old at final preflight.
  The current model stream must remain fresh, finite and in the same calibration
  epoch. An expired event is not replayed; a fresh clear-then-slip cycle is needed.
- Scores expire after 0.25 seconds; zero-reference changes inhibit the episode.
- Each learned-mode request advances the preceding command target by 3 raw
  counts per jaw at speed 80; ordinary speed and first-contact hold are unchanged.
  This avoids measured-position lag cancelling the requested increment. With
  the existing 3-count tracking tolerance, travel from the measured position can
  be up to 6 counts; this is NOT always a 3-count physical movement. A target
  equal to or behind the measured position is rejected. Heuristic mode retains
  its measured-position-relative 2-count step.
- At most three candidate attempts and six raw counts cumulatively per grasp.
  With accurate tracking this permits two 3-count reinforcements, not unlimited
  tightening. Actual-position error may cause earlier rejection.
- Measured positions must match the preceding target within three counts.
- At least 0.7 seconds cooldown, then three distinct fresh scores <= 0.3 on
  the triggering sensor and every currently contacting sensor before a new
  three-high-sample event can trigger. A continuously high signal is one episode.
- After ACK, existing position polling must observe >=2 closing counts twice
  on each jaw within one second before further reinforcement is permitted.
  Missing or unchanged measurements inhibit reinforcement for the current grasp;
  the controller does not increase the target automatically to overcome a stall.
  Encoder movement is not confirmation of grip force or successful slip arrest.
- Opening intent, stale input, invalid positions or sensors cancel reinforcement.
- Existing post-reinforcement sensor-rise cap (+60) remains active. It is not a
  force limit. Communication faults are not bypassed or automatically replayed.

Thresholds are provisional experimental choices, not validated safety limits.
Raw position increments are not measured force. Test non-fragile lightweight
objects near a supported surface, with access to stopping controls. ACK confirms
command handling, not physical motion or a safe grasp.

ROS timing records include reinforcement mode, attempt and triggering prediction;
existing actual-position telemetry remains separate. `regrasp_state` and
`regrasp_attempts` are available through `/command_status`. To disable, restart
with `slip_regrasp:=false`. No firmware change is required for this mode.

`regrasp_motion_result` distinguishes ACK/waiting, observed displacement and
unverified motion. The last result remains visible after release for diagnosis.
`motion_phase` distinguishes `contact paused` from `slip reinforcement`.
Contact pause inhibits ordinary lever-following targets, not model-authorized
reinforcement. The motion worker checks command ownership and newest lever input
before sending. Reinforcement updates the maintained target without changing the
lever lock anchor; completing it does not send another HOLD or restore an old
target. Opening releases contact and returns ownership to lever tracking.
This is not motor torque-off, and does not preempt a packet already sent to the
hardware. Host ownership tests cannot exclude firmware or motor-level problems.
The 3-count increment has software coverage, not hardware validation. Reported
excess pressure, possible resets, and calibration contamination remain unresolved;
do not treat this change as a repair of those problems or a force safety limit.
