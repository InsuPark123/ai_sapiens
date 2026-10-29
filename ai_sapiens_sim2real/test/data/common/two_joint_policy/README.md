# Two-joint policy fixture

This deterministic ONNX model and configuration are shared by policy runtime
integration tests. The model returns the latest two joint-position observations
from a 21-value observation vector. Its two-frame history and two-joint action
layout exercise policy-local observation processing and transitions.

Regenerate `policy.onnx` with `python3 generate_model.py` in an environment with
`onnx` installed. The generator preserves the original graph and weights.
Mirror-specific motion data remains in `test/data/mimic_mirror/motion.csv`.
