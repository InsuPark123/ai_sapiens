# SuperMimic research integration

This configuration is for offline development and evaluation. It has not been validated on the robot. It does not change the default K1 configuration.

## Assets

The orientation-only export used in this integration is at `/home/hc/robotis_mujoco/policy/Mimic/taguendo_supermimic_ori_only`. The prepared assets are at `assets/k1/mimic/supermimic_taeguek_ori_only`. That assets directory is gitignored, so a commit or push of this code does **not** transfer the model, configuration, motions, or sidecars to another machine. Preserve or recreate them separately. `manifest.json` records the source and output SHA-256 hashes.

To prepare a new, nonexistent destination (requires Python, NumPy, and PyYAML):

```bash
python3 ai_sapiens_sim2real/tools/prepare_supermimic.py \
  /home/hc/robotis_mujoco/policy/Mimic/taguendo_supermimic_ori_only \
  /path/to/new/supermimic_taeguek_ori_only
```

The converter refuses to overwrite an existing destination. If conversion fails, remove or inspect any partial destination before retrying. Each `taeguek_N.csv.supermimic` file contains one 56-value reference feature vector for each CSV frame. The runtime checks the sidecar length, 50 Hz timing, joint order, and reference positions against the configured default joint positions.

## Observations and selection

The policy input is 1,316 floats: 10-frame term-major histories of relative joint position, joint velocity, body angular velocity, projected gravity, and previous action (750); 10 future reference frames (560); and reference orientation relative to the body (6). It does not consume the robot's global torso position. An IMU marked usable is required. Sensor conventions and behavior on IMU failure still need hardware-specific verification.

`config/k1_supermimic_test.yaml` is an independent draft configuration. Its selector codes 291–297 select Taeguek 1–7; completion returns to Velocity. The active selector entries from `k1_config.yaml` are preserved. The original file has a second, inactive duplicate `selectors` mapping; the draft comments it out to avoid accidentally activating it. Existing RadioMaster decoding includes these numeric codes, but physical controller mapping and labels have not been verified.

## Offline checks

The C++ parity test uses captures produced by `tools/capture_supermimic_mujoco.cpp`, which is built against the local `robotis_mujoco` adapter and MuJoCo/ONNX Runtime libraries. Set `SUPERMIMIC_ASSET` to the prepared asset directory and `SUPERMIMIC_CAPTURES` to the capture directory when running `test_supermimic_parity`. The captures from this evaluation were stored under `/tmp/supermimic-parity`; they are temporary and not in Git. `SUPERMIMIC_CONFIG` and `SUPERMIMIC_ORIGINAL_CONFIG` enable the selector configuration test. `SUPERMIMIC_ASSET` also enables the runtime test.

For the evaluated seven no-push MuJoCo trajectories, 13,432 captured observations were compared to the AI Sapiens observation builder, and 145 corresponding raw ONNX outputs were compared. Maximum absolute differences were approximately 9.54e-7 for observations and 1.67e-6 for raw outputs. These are offline numerical parity results for captured states, not a closed-loop AI Sapiens Sim2Sim test, disturbance assessment, or evidence of safe robot operation. The capture program can stop a trajectory early when its fall condition is met.
