#!/usr/bin/env python3
import json
import logging
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
import struct
import time


INPUT_SHAPE = (1, 3, 1280, 1280)
OUTPUT_SHAPE = (1, 8, 33600)
INPUT_BYTES = 19660800
BACKEND = "CoreMLExecutionProvider"
COMPUTE_UNITS = "CPUAndGPU"
ADDRESS = ("127.0.0.1", 18765)


def encode_output(output):
    import numpy as np

    if (tuple(output.shape) != OUTPUT_SHAPE or output.dtype != np.float32
            or not np.isfinite(output).all()):
        raise RuntimeError("GPU output must be finite float32 [1, 8, 33600]")
    return b"MORAIGPU" + struct.pack("<II", 8, 33600) + np.ascontiguousarray(
        output, dtype="<f4").tobytes()


def load_backend(model_path):
    import numpy as np
    import onnxruntime as ort

    if BACKEND not in ort.get_available_providers():
        raise RuntimeError("CoreMLExecutionProvider is unavailable")
    session_options = ort.SessionOptions()
    session_options.intra_op_num_threads = 2
    session = ort.InferenceSession(str(model_path), sess_options=session_options, providers=[
        (BACKEND, {"MLComputeUnits": COMPUTE_UNITS, "ModelFormat": "MLProgram",
                   "RequireStaticInputShapes": "1"}), "CPUExecutionProvider"])
    session.disable_fallback()
    if BACKEND not in session.get_providers():
        raise RuntimeError("CoreMLExecutionProvider was not initialized")
    inputs, outputs = session.get_inputs(), session.get_outputs()
    if (len(inputs) != 1 or tuple(inputs[0].shape) != INPUT_SHAPE
            or inputs[0].type != "tensor(float)" or len(outputs) != 1
            or tuple(outputs[0].shape) != OUTPUT_SHAPE
            or outputs[0].type != "tensor(float)"):
        raise RuntimeError("Model must accept [1, 3, 1280, 1280] and return [1, 8, 33600]")
    encode_output(session.run(None, {inputs[0].name: np.zeros(INPUT_SHAPE, np.float32)})[0])
    return session


class InferenceHandler(BaseHTTPRequestHandler):
    def setup(self):
        super().setup()
        self.connection.settimeout(5)

    def respond(self, status, body, content_type="application/json"):
        try:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Connection", "close")
            self.end_headers()
            self.wfile.write(body)
        except OSError as error:
            logging.warning("Client response failed: %s", error)
        self.close_connection = True

    def do_GET(self):
        if self.path != "/health":
            self.respond(404, b'{"error":"unknown path"}')
            return
        if self.server.backend is None:
            self.respond(503, b'{"error":"GPU backend is not ready"}')
            return
        self.respond(200, json.dumps({"backend": BACKEND, "compute_units": COMPUTE_UNITS,
                                     "input_shape": INPUT_SHAPE, "output_shape": OUTPUT_SHAPE,
                                     "model": str(self.server.model_path)}).encode())

    def do_POST(self):
        if self.path != "/infer":
            self.respond(404, b'{"error":"unknown path"}')
            return
        lengths = self.headers.get_all("Content-Length", [])
        try:
            if (len(lengths) != 1 or int(lengths[0]) != INPUT_BYTES
                    or self.headers.get("Transfer-Encoding") is not None):
                raise ValueError("invalid input length")
        except ValueError:
            self.respond(400, b'{"error":"expected 19660800 raw float32 bytes"}')
            return
        backend = self.server.backend
        if backend is None:
            self.respond(503, b'{"error":"GPU backend is not ready"}')
            return
        try:
            body = self.rfile.read(INPUT_BYTES)
        except OSError:
            body = b""
        if len(body) != INPUT_BYTES:
            self.respond(400, b'{"error":"truncated input body"}')
            return
        try:
            import numpy as np

            tensor = np.frombuffer(body, dtype="<f4").reshape(INPUT_SHAPE)
            if not np.isfinite(tensor).all():
                self.respond(400, b'{"error":"input must contain finite float32 values"}')
                return
            output = backend.run(None, {backend.get_inputs()[0].name: tensor})[0]
            response = encode_output(output)
        except Exception:
            logging.exception("GPU inference failed; reloading the backend")
            self.server.backend = None
            self.respond(503, b'{"error":"GPU inference failed"}')
            return
        self.respond(200, response, "application/octet-stream")

    def log_message(self, format_string, *arguments):
        logging.info("%s %s", self.client_address[0], format_string % arguments)


def main():
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    model_path = Path(__file__).resolve().parents[1] / "models/traffic_light_yolo11m.onnx"
    server = None
    retry_at = 0.0
    while True:
        try:
            if server is None:
                server = HTTPServer(ADDRESS, InferenceHandler)
                server.model_path = model_path
                server.backend = None
                server.timeout = 1
            if server.backend is None and time.monotonic() >= retry_at:
                retry_at = time.monotonic() + 5
                try:
                    server.backend = load_backend(model_path)
                    logging.info("GPU ready: %s %s %s", BACKEND, COMPUTE_UNITS, model_path)
                except Exception:
                    logging.exception("GPU backend initialization failed; retrying")
            server.handle_request()
        except BaseException:
            logging.exception("GPU server failure; continuing")
            if server is not None:
                try:
                    server.server_close()
                except BaseException:
                    logging.exception("GPU server socket close failed; continuing")
                server = None
            try:
                time.sleep(1)
            except BaseException:
                logging.exception("GPU server retry interrupted; continuing")


if __name__ == "__main__":
    main()
