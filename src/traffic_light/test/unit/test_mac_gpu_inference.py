import http.client
import importlib.util
import json
from pathlib import Path
import socket
import struct
import threading
import unittest
from unittest import mock

import numpy as np


SCRIPT = Path(__file__).resolve().parents[2] / "scripts/mac_gpu_inference.py"
SPEC = importlib.util.spec_from_file_location("mac_gpu_inference", SCRIPT)
gpu = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(gpu)


class FakeGpuSession:
    def __init__(self):
        self.received = None
        self.failure = False
        self.output = np.arange(8 * 33600, dtype=np.float32).reshape(1, 8, 33600)

    def get_inputs(self):
        return [type("Input", (), {"name": "images"})()]

    def run(self, names, inputs):
        if self.failure:
            raise RuntimeError("GPU temporarily unavailable")
        self.received = inputs["images"]
        return [self.output]


class TestMacGpuInference(unittest.TestCase):
    def setUp(self):
        self.server = gpu.HTTPServer(("127.0.0.1", 0), gpu.InferenceHandler)
        self.server.model_path = "/tmp/traffic_light_yolo11m.onnx"
        self.server.backend = FakeGpuSession()
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()

    def request(self, method, path, body=None, headers=None):
        connection = http.client.HTTPConnection(*self.server.server_address, timeout=5)
        try:
            connection.request(method, path, body, headers or {})
            response = connection.getresponse()
            return response.status, response.read()
        finally:
            connection.close()

    def test_health_requires_a_loaded_gpu_backend(self):
        status, body = self.request("GET", "/health")
        self.assertEqual(200, status)
        health = json.loads(body)
        self.assertEqual("CoreMLExecutionProvider", health["backend"])
        self.assertEqual("CPUAndGPU", health["compute_units"])
        self.assertEqual([1, 3, 1280, 1280], health["input_shape"])
        self.assertEqual([1, 8, 33600], health["output_shape"])
        self.assertEqual(self.server.model_path, health["model"])
        self.server.backend = None
        self.assertEqual(503, self.request("GET", "/health")[0])

    def test_inference_preserves_fixed_little_endian_wire_contract(self):
        backend = self.server.backend
        backend.output = backend.output[:, :, ::-1]
        body = struct.pack("<f", 13.5) + bytes(gpu.INPUT_BYTES - 4)
        status, result = self.request("POST", "/infer", body)
        self.assertEqual(200, status)
        self.assertEqual(b"MORAIGPU" + struct.pack("<II", 8, 33600), result[:16])
        self.assertEqual(16 + 8 * 33600 * 4, len(result))
        np.testing.assert_array_equal(
            backend.output.ravel(), np.frombuffer(result[16:], dtype="<f4"))
        self.assertEqual((1, 3, 1280, 1280), backend.received.shape)
        self.assertEqual(13.5, backend.received[0, 0, 0, 0])

    def test_rejects_invalid_lengths_without_running_inference(self):
        for length in ("-1", "invalid", "0", str(gpu.INPUT_BYTES + 1)):
            with self.subTest(length=length):
                status, _ = self.request("POST", "/infer", b"", {"Content-Length": length})
                self.assertEqual(400, status)
        self.assertIsNone(self.server.backend.received)

    def test_rejects_a_truncated_body(self):
        with socket.create_connection(self.server.server_address, timeout=5) as connection:
            connection.sendall(("POST /infer HTTP/1.0\r\nContent-Length: "
                                f"{gpu.INPUT_BYTES}\r\n\r\n").encode() + b"short")
            connection.shutdown(socket.SHUT_WR)
            self.assertIn(b" 400 ", connection.recv(4096))
        self.assertIsNone(self.server.backend.received)

    def test_backend_failure_does_not_stop_http_server(self):
        self.server.backend.failure = True
        with self.assertLogs(level="ERROR"):
            self.assertEqual(503, self.request("POST", "/infer", bytes(gpu.INPUT_BYTES))[0])
        self.assertEqual(503, self.request("GET", "/health")[0])
        self.server.backend = FakeGpuSession()
        self.assertEqual(200, self.request("GET", "/health")[0])

    def test_wrong_output_shape_is_rejected(self):
        self.server.backend.output = np.zeros((1, 7, 33600), dtype=np.float32)
        with self.assertLogs(level="ERROR"):
            self.assertEqual(503, self.request("POST", "/infer", bytes(gpu.INPUT_BYTES))[0])

    def test_nonfinite_input_is_rejected_without_invalidating_gpu(self):
        backend = self.server.backend
        for value in (float("nan"), float("inf"), -float("inf")):
            with self.subTest(value=value):
                body = struct.pack("<f", value) + bytes(gpu.INPUT_BYTES - 4)
                self.assertEqual(400, self.request("POST", "/infer", body)[0])
        self.assertIsNone(backend.received)
        self.assertIs(backend, self.server.backend)

    def test_nonfinite_output_is_rejected(self):
        self.server.backend.output[0, 0, 0] = float("nan")
        with self.assertLogs(level="ERROR"):
            self.assertEqual(503, self.request("POST", "/infer", bytes(gpu.INPUT_BYTES))[0])

    def test_unknown_paths_do_not_run_inference(self):
        self.assertEqual(404, self.request("POST", "/unknown", b"")[0])
        self.assertEqual(404, self.request("GET", "/unknown")[0])
        self.assertIsNone(self.server.backend.received)

    def test_cpu_only_runtime_is_rejected(self):
        runtime = mock.Mock()
        runtime.get_available_providers.return_value = ["CPUExecutionProvider"]
        with mock.patch.dict("sys.modules", {"onnxruntime": runtime}):
            with self.assertRaisesRegex(RuntimeError, "CoreMLExecutionProvider"):
                gpu.load_backend(self.server.model_path)
        runtime.InferenceSession.assert_not_called()

    def test_coreml_backend_is_configured_and_warmed_before_use(self):
        runtime = mock.Mock()
        runtime.get_available_providers.return_value = [gpu.BACKEND, "CPUExecutionProvider"]
        backend = runtime.InferenceSession.return_value
        backend.get_providers.return_value = [gpu.BACKEND, "CPUExecutionProvider"]
        backend.get_inputs.return_value = [type("Input", (), {
            "name": "images", "shape": [1, 3, 1280, 1280], "type": "tensor(float)"})()]
        backend.get_outputs.return_value = [type("Output", (), {
            "shape": [1, 8, 33600], "type": "tensor(float)"})()]
        backend.run.return_value = [np.zeros((1, 8, 33600), dtype=np.float32)]
        with mock.patch.dict("sys.modules", {"onnxruntime": runtime}):
            self.assertIs(backend, gpu.load_backend(self.server.model_path))
        runtime.InferenceSession.assert_called_once_with(
            self.server.model_path, sess_options=runtime.SessionOptions.return_value, providers=[
            (gpu.BACKEND, {"MLComputeUnits": "CPUAndGPU", "ModelFormat": "MLProgram",
                           "RequireStaticInputShapes": "1"}), "CPUExecutionProvider"])
        self.assertEqual(2, runtime.SessionOptions.return_value.intra_op_num_threads)
        backend.disable_fallback.assert_called_once_with()
        backend.run.assert_called_once()
        self.assertEqual((1, 3, 1280, 1280), backend.run.call_args.args[1]["images"].shape)

    def test_cpu_only_session_is_rejected_even_if_coreml_is_available(self):
        runtime = mock.Mock()
        runtime.get_available_providers.return_value = [gpu.BACKEND, "CPUExecutionProvider"]
        runtime.InferenceSession.return_value.get_providers.return_value = ["CPUExecutionProvider"]
        with mock.patch.dict("sys.modules", {"onnxruntime": runtime}):
            with self.assertRaisesRegex(RuntimeError, "CoreMLExecutionProvider"):
                gpu.load_backend(self.server.model_path)
        runtime.InferenceSession.return_value.run.assert_not_called()


if __name__ == "__main__":
    unittest.main()
