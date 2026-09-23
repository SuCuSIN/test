# Experimental sensor-change regrasp

Enable with `slip_regrasp:=true` on `ur3_gello_anyskin_control.launch.py`.
Default false preserves normal behavior. Applies to the ROS position-tracking
controller, including its web targets; not the standalone shell close_safe path.
Keep `body_resistance:=false` while evaluating gripper behavior.

Existing initial contact stop, confirmation margin, and lever feedback latch are
unchanged. After the hold, a separate guard can issue bounded closing targets
without clearing contact or the lever lock. Opening intent cancels new regrasp
targets. It cannot recall a command already sent over Bluetooth.

This is NOT a validated slip detector or force controller. It uses changes in
baseline-subtracted grouped magnetic strengths (M1, M2/M3/M4, M5). Material creep,
external motion and magnetic interference can also satisfy the heuristic.

Provisional guard conditions:
- Both active sensors, fresh samples, valid empty-close calibration.
- At least one sensor still has a group at or above the existing contact margin.
  Both sensor streams must remain valid/fresh. A disconnected sensor is not treated
  as an unloaded sensor. Only an in-contact sensor can trigger reinforcement.
- A per-group P10..P90 range over >=0.5 s and >=5 samples, frozen until regrasp.
  Normal fluctuation is allowed; the user should not pull during range collection.
- >=20 units ABOVE the P90 bound or BELOW the P10 bound in one persistent
  independent group on EITHER sensor for >=0.12 s and >=3 fresh paired observations.
  At least one sensor must still have contact. Returning inside the band resets confirmation;
  switching group or excursion direction starts a new confirmation interval.
  Contact loss is still inhibited, not treated as a request to chase a dropped object.
- Read both servo positions and recheck inputs/sensors before every step.
- Two raw counts toward closed per jaw; no opening correction.
- At most three attempts and six raw counts beyond the first held pose per jaw.
- Separate regrasp speed 80; normal closing speed 2919 and existing acceleration
  are unchanged. Wait 0.7 s and establish a new stable reference after each attempt.
- Large pre-regrasp rises are evaluated by the sustained-exceedance detector,
  not blocked merely for exceeding 60 units from the original contact range.
- >60-unit positive increase from the sensor snapshot immediately BEFORE the first
  reinforcement inhibits further regrasp, including during cooldown. This fixed
  reference is not raised on subsequent attempts. It does not undo a sent motion,
  and is NOT a calibrated force/damage limit or proof of safe first reinforcement.
- Sensor loss, contact loss, input timeout, failed position checks or range limits
  inhibit regrasp for this grasp; no repeated blind tightening/chasing a dropped item.

No proof of secure grasp is inferred when the cap is reached. Additional travel
can damage an object even below these limits. Test with a supported nonfragile
object over a catch surface, not by immediately lifting heavy objects. Keep the
existing opening control and robot stop accessible. No hardware testing performed.

Logs include suspected-slip targets, regrasp flags in timing events, and guard
state/count in sensor records and control status. These are not measured slip or
grip-force values. Existing communication-failure behavior remains unchanged.
