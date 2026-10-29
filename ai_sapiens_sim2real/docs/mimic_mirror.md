# Reusing a K1 Mimic policy with left/right reflection

Set `mirror: true` on an individual ordinary Mimic behavior:

```yaml
state_behaviors:
  mimic_redred_newton:
    kind: mimic
    asset: mimic/redred_newton
    motion: redred.csv
    mirror: true
    policy_action_transition:
      duration: 0.05
```

The default is `false`. Reuse the original ONNX, sim2real configuration and
motion CSV unchanged. No behavior is mirrored until explicitly enabled.

The runtime reflects a private sensor snapshot into the original model's
coordinates before observation encoding, scaling, clipping and history.
Reference motion and previous raw actions stay in the original model frame.
Processed joint targets are reflected back into robot coordinates, including
left/right PD gain exchange. Asymmetric action scales and default positions
are handled by processing actions before reflection.

The K1 joint map follows `ai_sapiens_description/mujoco/k1/k1.xml`: pitch,
knee and elbow angles exchange sides without a sign change; roll and yaw
angles exchange sides with a sign change. Waist yaw changes sign. Orientation
uses sagittal reflection, angular velocity changes its X/Z signs, and projected
gravity changes its Y sign. Unknown joints and policy joint sets missing the
opposite-side counterpart are rejected.

Entry posture validation uses the reflected reference root orientation.
Completion still requests the existing `on_complete` state. Shared sensor data
is never modified. Commands handed to transition blending are always in robot
coordinates; subsequent Velocity or Mimic entries reset action and observation
history through the existing entry lifecycle. Transition duration and damping
behavior are unchanged.

This implementation supports ordinary Mimic only. Mirroring Any2Track,
localization-based policies, steering, dance references, velocity-command
observations or gait-phase observations is rejected during startup. Those
paths require additional command/frame transformations. Omit `mirror` for
existing deployments of those policies.

The integration fixture compares a mirrored runtime against an ordinary
runtime receiving the equivalent virtual robot state, including observation
history, processed actions, completion, transition blending and re-entry.
Physical tracking performance still needs MuJoCo validation with the chosen
trained model; left/right hardware or dynamics asymmetry can affect tracking.
