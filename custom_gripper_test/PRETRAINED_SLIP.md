# Pretrained AnySkin slip evaluation

Status: **shadow only, no motor commands and no automatic reinforcement**.
This is Pollen Robotics' reproduction, not a verified release of the paper
authors' checkpoint. It predicts binary slip, not direction or grip force.

## Sources and provenance

- Model: https://huggingface.co/pollen-robotics/anyskin-slip-detection
- Pinned model revision: `775924649e22452b983ada6c4b59b6f928de0241`
- Architecture/prediction: https://github.com/pollen-robotics/anyskin-slip-detection
- Pinned source revision: `24a3728c35985def720ff292a8a45ef7e8662a64`
- Input example: https://github.com/pollen-robotics/tactile_gripper/blob/main/gripper_anyskin/src/main.rs
- Apache-2.0 license included in `models/pollen_anyskin_slip/LICENSE`.
- SHA256 checks for weights and scaler are enforced before loading.
- Uses safetensors, never pickle or remote Python execution.

The adapter matches the Python training/inference path: baseline-subtracted
M1 XYZ through M5 XYZ, exported StandardScaler, shape (1, 1, 15), fresh LSTM
state, linear fc/out, sigmoid. No arbitrary temporal resampling or persistent
LSTM state is added to a model trained with sequence length one.

## Important input discrepancy

The burst protocol used by the public example is T,X,Y,Z per magnetometer.
Our legacy control reader uses T,X,Y. Replacing it globally would change contact
strengths and invalidate existing calibration. This change therefore leaves
that control path unchanged and adds a separate XYZ buffer and calibration.
Existing `anyskin_xyz_v1` raw CSV files from that reader are NOT true XYZ and
must not be fed to this model. Missing Z cannot be reconstructed.

After restarting the updated AnySkin controller and running the existing
empty-gripper calibration, `/command_status` exposes `slip_model_input` using
schema `anyskin_txyz_model_v1`. The observer does not open USB ports or command
calibration, motion, or torque. Its PyTorch inference runs in a separate process.
Calibration for this path requires the full requested sample count.

## Run

Keep the main control launch options `slip_regrasp:=false` and
`record_anyskin_raw:=false`. Calibrate with an empty, unloaded gripper.
In a separate WSL terminal:

```bash
cd /mnt/c/Users/kyss0/OneDrive/Desktop/DocumentsGELLO_Project/gello_software/custom_gripper_test
python3 pretrained_anyskin_slip.py
```

Requires existing WSL `torch`, `safetensors`, and `numpy`. Output goes to
`slip_model_reports/*.jsonl`; Ctrl+C ends only the observer. Both sensors are
evaluated separately; duplicate channels, missing calibration, and samples
older than 250ms produce no score. Values are model scores, not calibrated
probabilities or physical proof of slip. No direction is inferred.

## Before reinforcement

Verify the installed sensor firmware really uses T,X,Y,Z and check all five
magnetometers, particularly previously duplicated M2/M3/M4. Collect supported,
nonfragile-object trials with no slip, pressure without slip, and actual slip.
Evaluate false positives and detection delay before wiring predictions to the
bounded reinforcement guard. The old heuristic must remain OFF during this
evaluation. Existing contact-stop performance is not certified by model tests.
