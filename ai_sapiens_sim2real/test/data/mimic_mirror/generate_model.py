# Copyright 2026 ROBOTIS CO., LTD.
# SPDX-License-Identifier: Apache-2.0
"""Regenerate the tiny deterministic mirror integration fixture (requires onnx)."""
from pathlib import Path
import onnx
from onnx import TensorProto, helper

# Read the newest joint observation from two-frame term-major history.
weights = [0.0] * (21 * 2)
weights[2 * 2] = 1.0
weights[3 * 2 + 1] = 1.0
graph = helper.make_graph(
    [helper.make_node('MatMul', ['obs', 'weights'], ['action'])],
    'mirror_test',
    [helper.make_tensor_value_info('obs', TensorProto.FLOAT, [1, 21])],
    [helper.make_tensor_value_info('action', TensorProto.FLOAT, [1, 2])],
    [helper.make_tensor('weights', TensorProto.FLOAT, [21, 2], weights)],
)
model = helper.make_model(graph, opset_imports=[helper.make_opsetid('', 13)])
model.ir_version = 8
onnx.checker.check_model(model)
onnx.save(model, Path(__file__).with_name('policy.onnx'))
