# Per-policy action transitions

`policy_action_transition` controls command blending when entering a policy.
The root setting provides defaults for every policy. A behavior under
`state_behaviors` can override `enabled`, `duration`, or both. Omitted fields
inherit the root setting. This applies to both `kind: policy` and `kind: mimic`,
including Any2Track.

```yaml
policy_action_transition:
  enabled: true
  duration: 0.1

state_behaviors:
  velocity_policy:
    kind: policy
    asset: your/velocity_asset
    policy_action_transition:
      duration: 0.3

  mimic_standup:
    kind: mimic
    asset: your/standup_asset
    policy_action_transition:
      enabled: false
    # Keep the motion configuration required by this asset here.
```

The destination behavior selects the settings: entering `velocity_policy`
blends for 0.3 seconds, while entering `mimic_standup` switches immediately.
These settings do not define source-to-destination transition permissions or
change Mimic posture entry checks.

Without a root setting, the defaults remain `enabled: false` and
`duration: 0.2`. Duration must be finite and non-negative; zero switches
immediately. Existing configurations retain their behavior when no overrides
are added.

Blending still requires a preceding posture or policy command. Startup, reset,
and entry from damping skip blending, regardless of the destination override.
