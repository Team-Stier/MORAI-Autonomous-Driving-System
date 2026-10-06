#!/usr/bin/env python3
from pathlib import Path
import numpy as np
import onnx
from onnx import TensorProto, helper, numpy_helper

output = np.array([[[640.0], [640.0], [100.0], [60.0], [0.9], [0.0], [0.0], [0.0]]], dtype=np.float32)
def save(name, array, output_type, output_shape):
    graph = helper.make_graph(
        [helper.make_node("Constant", [], ["output0"], value=numpy_helper.from_array(array))],
        "fixture",
        [helper.make_tensor_value_info("images", TensorProto.FLOAT, [1, 3, 1280, 1280])],
        [helper.make_tensor_value_info("output0", output_type, output_shape)],
    )
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 17)])
    model.ir_version = 9
    path = Path(__file__).with_name(name)
    onnx.checker.check_model(model)
    onnx.save(model, path)
    print(path)


save("yolo_fixture.onnx", output, TensorProto.FLOAT, [1, 8, 1])
save("wrong_output_shape.onnx", np.zeros((1, 7, 1), dtype=np.float32),
     TensorProto.FLOAT, [1, 7, 1])
