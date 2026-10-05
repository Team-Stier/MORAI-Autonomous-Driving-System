# Traffic Light YOLO Core Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Export the trained YOLO11m detector to ONNX and add a tested C++ traffic-light Core that publishes stable `GREEN`, `LEFT`, `YELLOW`, `RED`, or `UNKNOWN` results from `camera_7` at 5 Hz.

**Architecture:** The existing one-stage `node_pipeline` keeps ownership of ROS input, scheduling, freshness checks, header injection, and publication. A single persistent `YoloTrafficLightCore` owns an ONNX Runtime session and a bounded vote history; OpenCV handles JPEG decoding, letterbox preprocessing, and NMS. Pure decode/mapping/voting helpers remain callable from unit tests without starting ROS or loading a model.

**Tech Stack:** ROS Noetic, C++14, catkin, OpenCV 4.2, ONNX Runtime CUDA Execution Provider, Ultralytics 8.4.172, YOLO11m Detect, GoogleTest

**Spec:** `docs/superpowers/specs/2026-10-05-traffic-light-yolo-core-design.md`

## Global Constraints

- Work only in `/home/stier/MORAI-Autonomous-Driving-System/.worktrees/traffic-light-yolo-core` on `feature/traffic-light-yolo-core`; never commit traffic-light work to `main`.
- Input is `/image/camera_7/compressed`; output is `/traffic_light`; preserve the input image stamp and use `camera_7_optical_frame`.
- Keep the existing 5 Hz periodic, latest-unprocessed-frame, serial-inference, ROS queue-size-1 behavior.
- Do not add ROI selection, stop-line distance, route position, vehicle control, or distance estimation.
- Model classes are exactly `0: go`, `1: left_turn`, `2: prepare_stop`, `3: stop`.
- ROS mapping is `go→GREEN`, `left_turn→LEFT`, `prepare_stop→YELLOW`, `stop→RED`; unsupported, missing, weak, or ambiguous detections become `UNKNOWN`.
- Default inference size is `1280x1280`; default vote policy is five frames with three required votes and must remain configurable.
- Do not modify `node_pipeline`, message definitions, launch files, root README, or submodules.
- Do not commit `.pt`, `.onnx`, rosbag, extracted images, or other large generated artifacts as ordinary Git blobs.
- Add only OpenCV and ONNX Runtime as direct native dependencies; reuse the existing Pipeline and GoogleTest infrastructure.

## Review Focus

- A model exported with the wrong class order must fail during startup, not silently publish the wrong ROS state; Task 1 validates class metadata and Task 3 tests rejection.
- YOLO output may be transposed (`[1,8,N]` versus `[1,N,8]`); Task 2 tests both supported layouts and rejects every other rank/channel count.
- Two different high-confidence states in one frame must become `UNKNOWN` when their scores are within `ambiguity_margin`; Task 2 pins the boundary cases.
- An `UNKNOWN` burst must not leave an old green result stuck indefinitely; Task 2 tests that UNKNOWN participates in the fixed-size vote history.
- Corrupt JPEG and CUDA inference failure have intentionally different behavior: corrupt input yields `UNKNOWN`, while inference failure terminates processing; Tasks 3 and 4 test both paths.

---

### Task 0: Verify the Isolated Branch and Baseline

**Files:**
- Read only: repository and submodule state

**Interfaces:**
- Consumes: worktree `/home/stier/MORAI-Autonomous-Driving-System/.worktrees/traffic-light-yolo-core`.
- Produces: a clean, buildable baseline on `feature/traffic-light-yolo-core`; no source changes.

- [ ] **Step 1: Prove the branch boundary**

  Run: `test "$(git branch --show-current)" = "feature/traffic-light-yolo-core" && git status --short --branch && git -C /home/stier/MORAI-Autonomous-Driving-System status --short --branch`

  Expected: the worktree is on the feature branch and the original checkout remains on an unchanged `main`.

- [ ] **Step 2: Initialize the pinned submodules in this worktree**

  Run: `git submodule update --init --recursive`

  Expected: every submodule is checked out at the gitlink commit recorded by `origin/main`; no parent-repository diff is created.

- [ ] **Step 3: Run the baseline package build and tests**

  Run: `catkin_make --pkg traffic_light && catkin_make run_tests_traffic_light && catkin_test_results --verbose`

  Expected: the unimplemented package builds and its existing tests pass. If not, record the pre-existing failure and stop before attributing it to new code.

### Task 1: Freeze and Export the Model Artifact

**Files:**
- Create: `/home/stier/traffic_light_training/export/export_yolo11m_onnx.py`
- Create: `/home/stier/traffic_light_training/export/traffic_light_yolo11m.json`
- Create: `/home/stier/traffic_light_training/export/README.md`
- Produce, do not commit: `/home/stier/traffic_light_training/export/traffic_light_yolo11m.onnx`
- Read: `/home/stier/traffic_light_training/training_runs/yolo11m_v1/weights/best.pt`
- Read: `/home/stier/traffic_light_training/config/classes.yaml`

**Interfaces:**
- Consumes: completed `best.pt` whose `model.names` is `{0: "go", 1: "left_turn", 2: "prepare_stop", 3: "stop"}`.
- Produces: static-batch ONNX input `images` with shape `[1,3,1280,1280]`, raw output with four box channels plus four class channels, and a JSON manifest containing model SHA-256, input/output names, size, class list, Ultralytics version, opset, and export arguments.

- [ ] **Step 1: Add a failing export preflight check**

  In `export_yolo11m_onnx.py`, first assert that training has reached its configured terminal condition: either the final epoch is present in `results.csv` or Ultralytics early-stopping metadata marks the run complete. Assert the exact four-class order before export. Running it against an incomplete run must exit non-zero before writing ONNX.

- [ ] **Step 2: Run the preflight**

  Run: `/home/stier/traffic_light_training/.venv/bin/python /home/stier/traffic_light_training/export/export_yolo11m_onnx.py --check-only`

  Expected now: either `PASS: completed training, classes 0..3 verified` or a clear `training is not complete` failure. If incomplete, do not export the current checkpoint; wait for or resume the existing training rather than starting a second run.

- [ ] **Step 3: Implement the deterministic export**

  Export with Ultralytics 8.4.172 using `format="onnx"`, `imgsz=1280`, `batch=1`, `dynamic=False`, `simplify=True`, `nms=False`, and `opset=17`. Load the result with `onnx`, run `onnx.checker.check_model`, verify input/output names and dimensions, calculate SHA-256, and write the manifest atomically beside the model.

- [ ] **Step 4: Run export validation**

  Run: `/home/stier/traffic_light_training/.venv/bin/python /home/stier/traffic_light_training/export/export_yolo11m_onnx.py`

  Expected: ONNX checker passes; the report prints class order, tensor shapes, opset, and SHA-256; both artifact files exist under `export/`.

- [ ] **Step 5: Record reproducibility instructions**

  In `export/README.md`, record the source checkpoint, exact command, package versions, generated files, and the rule that generated model artifacts remain outside normal Git history.

### Task 2: Implement and Test Pure Detection Decisions

**Files:**
- Create: `src/traffic_light/include/traffic_light/detection.hpp`
- Create: `src/traffic_light/src/detection.cpp`
- Create: `src/traffic_light/test/unit/test_detection.cpp`
- Modify: `src/traffic_light/include/traffic_light/types.hpp`
- Modify: `src/traffic_light/CMakeLists.txt`

**Interfaces:**
- Consumes: raw YOLO tensor data and `DetectionParameters` defined in `types.hpp` with four confidence thresholds, NMS IoU, ambiguity margin, history size, and minimum votes. This task also defines the containing `TrafficLightParameters` aggregate used by Tasks 3–5, including paths, input dimensions, provider policy, and detection settings.
- Produces: `enum class DetectionState : std::uint8_t { Unknown, Go, LeftTurn, PrepareStop, Stop }`, `std::vector<Detection> decodeYoloOutput(const float*, const std::vector<std::int64_t>&, const DetectionParameters&)`, `DetectionState chooseFrameState(const std::vector<Detection>&, float ambiguity_margin)`, and `class TemporalVoter { DetectionState push(DetectionState); }`.

- [ ] **Step 1: Write failing decode and NMS tests**

  Add tests named `DecodesChannelsFirstTensor`, `DecodesRowsFirstTensor`, `AppliesPerClassConfidence`, `SuppressesSameClassOverlap`, and `RejectsUnexpectedTensorShape`. Fixtures must cover shapes `[1,8,N]` and `[1,N,8]`, four class scores, and class-aware OpenCV NMS.

- [ ] **Step 2: Run the focused test target and confirm failure**

  Run: `catkin_make run_tests_traffic_light_gtest_test_traffic_light_detection`

  Expected: build failure because the detection API is not implemented.

- [ ] **Step 3: Implement the minimal decoder**

  Define `DetectionParameters` and its containing `TrafficLightParameters` in `types.hpp`, plus `Detection { cv::Rect2f box; float confidence; DetectionState state; }` in `detection.hpp`. Decode center-width-height boxes, apply the configured threshold for each class, and invoke `cv::dnn::NMSBoxes` separately per class. Reject null data, non-finite values, unsupported rank, batch other than one, or channel count other than eight with `std::invalid_argument`.

- [ ] **Step 4: Write failing frame-decision and voting tests**

  Add tests named `MapsExactFourClasses`, `ReturnsUnknownWithoutCandidates`, `ReturnsUnknownForAmbiguousClassesAtMargin`, `ChoosesWinnerOutsideAmbiguityMargin`, `RequiresThreeOfFiveVotes`, `BreaksVoteTieByMostRecentOccurrence`, `UnknownEvictsStaleGo`, and `OneOfOneDisablesSmoothing`.

- [ ] **Step 5: Implement frame selection and bounded voting**

  Select the highest-confidence surviving detection. If the best detection of a different state has a confidence difference less than or equal to `ambiguity_margin`, select `Unknown`. Store all states, including `Unknown`, in a deque capped at `history_size`; require `minimum_votes`, and resolve equal vote totals using the state whose last occurrence is newest.

- [ ] **Step 6: Run the detection tests**

  Run: `catkin_make run_tests_traffic_light_gtest_test_traffic_light_detection && catkin_test_results --verbose`

  Expected: all detection and voting tests pass with zero failures.

- [ ] **Step 7: Commit the pure decision layer**

```bash
git add src/traffic_light/include/traffic_light/types.hpp src/traffic_light/include/traffic_light/detection.hpp src/traffic_light/src/detection.cpp src/traffic_light/test/unit/test_detection.cpp src/traffic_light/CMakeLists.txt
git commit -m "feat: add traffic light detection decisions"
```

### Task 3: Add ONNX Runtime Image Inference

**Files:**
- Create: `src/traffic_light/include/traffic_light/cores/yolo_traffic_light_core.hpp`
- Create: `src/traffic_light/src/yolo_traffic_light_core.cpp`
- Create: `src/traffic_light/test/unit/test_yolo_traffic_light_core.cpp`
- Create, test-only: `src/traffic_light/test/fixtures/make_fixture_model.py`
- Produce, do not commit: `src/traffic_light/test/fixtures/yolo_fixture.onnx`
- Modify: `src/traffic_light/CMakeLists.txt`
- Modify: `src/traffic_light/package.xml`

**Interfaces:**
- Consumes: `YoloTrafficLightCore(TrafficLightParameters)` and `interfaces::TrafficLight process(const sensor_msgs::CompressedImage&)`.
- Produces: a payload-only `interfaces::TrafficLight` and persistent ONNX Runtime session/vote history; header remains untouched for the Pipeline injector.

- [ ] **Step 1: Establish the native runtime dependency**

  Check driver, CUDA runtime, and cuDNN compatibility, then install one pinned official ONNX Runtime GPU C/C++ distribution into `/opt/onnxruntime-gpu/<version>` using the required sudo form `echo " " | sudo ...`. Add a small `FindONNXRuntime.cmake` only if the distribution does not provide a usable CMake config. Do not link against files inside the Python virtual environment.

- [ ] **Step 2: Verify CUDA provider availability before code changes**

  Compile and run a minimal probe that creates an ONNX Runtime environment and lists providers.

  Expected: `CUDAExecutionProvider` is listed. If CUDA/cuDNN compatibility cannot be satisfied, stop and report the exact missing shared library; do not silently redesign the node around CPU or Python.

- [ ] **Step 3: Write failing Core tests**

  Generate a tiny deterministic ONNX fixture whose raw output contains one known class candidate. Add `DecodesJpegAndReturnsMappedState`, `CorruptJpegReturnsUnknown`, `DoesNotWriteHeader`, `RejectsManifestClassOrderMismatch`, and `InferenceExceptionPropagates`. Use `history_size=1` in single-frame tests.

- [ ] **Step 4: Run the Core tests and confirm failure**

  Run: `catkin_make run_tests_traffic_light_gtest_test_yolo_traffic_light_core`

  Expected: build failure because `YoloTrafficLightCore` is absent.

- [ ] **Step 5: Implement session initialization**

  Use a private implementation owned by `std::unique_ptr` to keep ONNX Runtime headers out of registration headers. Verify model/manifest existence, exact class order, one input, one supported output, static batch one, and `1280x1280`; the manifest's SHA-256 remains provenance evidence generated in Task 1 rather than requiring another runtime crypto dependency. Append CUDA provider; append CPU only when `allow_cpu_fallback` is true.

- [ ] **Step 6: Implement preprocessing and inference**

  Decode with `cv::imdecode`; a decode failure pushes `Unknown` through the voter. For valid images, perform centered gray letterbox, BGR→RGB, `[0,1]` normalization, NCHW conversion, one synchronous session run, output decode, frame selection, and voting. Do not create worker threads or a second image queue.

- [ ] **Step 7: Run Core tests**

  Run: `catkin_make run_tests_traffic_light_gtest_test_yolo_traffic_light_core && catkin_test_results --verbose`

  Expected: all Core tests pass; the test process reports the expected provider and no header mutation.

- [ ] **Step 8: Commit inference support**

```bash
git add src/traffic_light/include/traffic_light/cores/yolo_traffic_light_core.hpp src/traffic_light/src/yolo_traffic_light_core.cpp src/traffic_light/test/unit/test_yolo_traffic_light_core.cpp src/traffic_light/test/fixtures/make_fixture_model.py src/traffic_light/CMakeLists.txt src/traffic_light/package.xml
git commit -m "feat: run YOLO traffic light inference"
```

### Task 4: Add Configuration and Startup Validation

**Files:**
- Modify: `src/traffic_light/include/traffic_light/types.hpp`
- Create: `src/traffic_light/include/traffic_light/config.hpp`
- Create: `src/traffic_light/test/unit/test_config.cpp`
- Modify: `src/traffic_light/config/traffic_light.yaml`
- Modify: `src/traffic_light/CMakeLists.txt`

**Interfaces:**
- Consumes: private ROS parameters under `~yolo_traffic_light`.
- Produces: `TrafficLightParameters loadTrafficLightParameters(ros::NodeHandle&)` with model/manifest paths, width/height, four confidence thresholds, NMS IoU, ambiguity margin, history size, minimum votes, and CPU-fallback flag.

- [ ] **Step 1: Write failing parameter tests**

  Add tests named `LoadsDocumentedDefaults`, `RejectsMissingPaths`, `RejectsNonPositiveInputSize`, `RejectsThresholdOutsideUnitInterval`, `RejectsZeroHistory`, and `RejectsMinimumVotesGreaterThanHistory`. Include exact default values from the design document.

- [ ] **Step 2: Run the config test and confirm failure**

  Run: `catkin_make run_tests_traffic_light_gtest_test_traffic_light_config`

  Expected: build failure because the parameter loader is absent.

- [ ] **Step 3: Implement the value object and loader**

  Use the `TrafficLightParameters` aggregate from Task 2. Load values in `config.hpp`, validate paths and numeric ranges, and throw `std::invalid_argument` with the parameter name. Do not introduce dynamic reconfigure or another config library.

- [ ] **Step 4: Add production YAML values**

  Set model and manifest paths under `/home/stier/traffic_light_training/export/`, size `1280x1280`, confidence values `{go: 0.40, left_turn: 0.40, prepare_stop: 0.35, stop: 0.40}`, NMS `0.45`, ambiguity `0.05`, history `5`, votes `3`, and CPU fallback `false`.

- [ ] **Step 5: Run config and prior tests**

  Run: `catkin_make run_tests_traffic_light && catkin_test_results --verbose`

  Expected: config, detection, and Core tests all pass.

- [ ] **Step 6: Commit configuration**

```bash
git add src/traffic_light/include/traffic_light/types.hpp src/traffic_light/include/traffic_light/config.hpp src/traffic_light/test/unit/test_config.cpp src/traffic_light/config/traffic_light.yaml src/traffic_light/CMakeLists.txt
git commit -m "feat: configure traffic light YOLO core"
```

### Task 5: Register the Core in the Existing Pipeline

**Files:**
- Modify: `src/traffic_light/include/traffic_light/core_set.hpp`
- Modify: `src/traffic_light/include/traffic_light/core_dispatch.hpp`
- Modify: `src/traffic_light/src/ros_node.cpp`
- Modify: `src/traffic_light/config/traffic_light.yaml`
- Create: `src/traffic_light/test/unit/test_yolo_core_registration.cpp`
- Modify: `src/traffic_light/CMakeLists.txt`

**Interfaces:**
- Consumes: `YoloTrafficLightCore` and `TrafficLightParameters` from Tasks 3–4.
- Produces: `CoreId::YoloTrafficLight`, a `CoreSet` member, name map entry `"yolo_traffic_light_core.hpp"`, dispatch from input slot 0, and selection for section IDs 0–16.

- [ ] **Step 1: Write failing registration tests**

  Add `MissingImageSkipsCoreAndPublication`, `ValidImageCallsCoreAndPublishesOnce`, `OutputHeaderUsesInputStampAndCameraFrame`, and `EverySectionSelectsYoloCore`. Use the deterministic fixture model and Pipeline test IO, not a ROS master.

- [ ] **Step 2: Run the registration test and confirm failure**

  Run: `catkin_make run_tests_traffic_light_gtest_test_yolo_core_registration`

  Expected: failure because the Core ID and dispatch case are unregistered.

- [ ] **Step 3: Apply only the permitted registration diff**

  Add the Core include, enum value, and owned object to `core_set.hpp`. In `core_dispatch.hpp`, check `data.hasInputs({0})`, call `before()`, invoke `cores.yolo_traffic_light.process(data.message<sensor_msgs::CompressedImage>(0))`, and wrap the new message result without an original-slot mapping.

- [ ] **Step 4: Inject config without changing node behavior**

  In `ros_node.cpp`, load `TrafficLightParameters`, add `"yolo_traffic_light_core.hpp"` to the existing name map, construct `CoreSet` with the configured Core, and leave callbacks, topics, timer, checker, policy, header injector, and IO unchanged.

- [ ] **Step 5: Complete section selection**

  Change only `stage1.section_core_map` values 0–16 from empty strings to `yolo_traffic_light_core.hpp`. Keep `health_core_map`, execution mode, frequency, queue, health limits, and input settings unchanged.

- [ ] **Step 6: Run registration and package tests**

  Run: `catkin_make run_tests_traffic_light && catkin_test_results --verbose`

  Expected: all traffic-light tests pass with zero failures.

- [ ] **Step 7: Build the node from a clean package state**

  Run: `catkin_make --pkg traffic_light`

  Expected: `traffic_light_node` compiles and links against OpenCV, ONNX Runtime, CUDA, and existing catkin dependencies without unresolved symbols.

- [ ] **Step 8: Commit Pipeline registration**

```bash
git add src/traffic_light/include/traffic_light/core_set.hpp src/traffic_light/include/traffic_light/core_dispatch.hpp src/traffic_light/src/ros_node.cpp src/traffic_light/config/traffic_light.yaml src/traffic_light/test/unit/test_yolo_core_registration.cpp src/traffic_light/CMakeLists.txt
git commit -m "feat: register YOLO traffic light core"
```

### Task 6: Validate Model Parity and Rosbag Runtime

**Files:**
- Create: `/home/stier/traffic_light_training/evaluation/compare_pt_onnx.py`
- Create: `/home/stier/traffic_light_training/evaluation/reports/yolo11m_onnx_parity.json`
- Create: `/home/stier/traffic_light_training/evaluation/reports/traffic_light_rosbag_runtime.md`
- Modify: `docs/superpowers/specs/2026-10-05-traffic-light-yolo-core-design.md` only if measured deployment facts supersede assumptions

**Interfaces:**
- Consumes: completed `best.pt`, exported ONNX/manifest, held-out foggy-15 images, and the camera-7 rosbag.
- Produces: PT-vs-ONNX parity evidence and ROS topic/runtime measurements; no production API changes.

- [ ] **Step 1: Add the parity check**

  Compare PT and ONNX on the same held-out image list. Record per-image top class, confidence delta, and IoU after identical confidence/NMS settings. Fail when top classes differ, confidence delta exceeds `0.02`, or matched-box IoU is below `0.98`.

- [ ] **Step 2: Run held-out evaluation**

  Run the existing Ultralytics validation on the reserved foggy-15 split, then run `compare_pt_onnx.py`.

  Expected: class-wise precision, recall, mAP, confusion matrix, and a passing parity JSON are written without altering training/validation splits.

- [ ] **Step 3: Run the node against the camera-7 rosbag**

  Start only ROS core, the traffic-light node, and rosbag replay. Record `/traffic_light` rate, input/output stamp pairs, frame IDs, output state sequence, process CPU, `nvidia-smi` GPU utilization/memory, and inference p50/p95.

- [ ] **Step 4: Check runtime acceptance criteria**

  Confirm approximately 5 Hz publication while fresh images arrive, identical input/output stamps, `camera_7_optical_frame`, one publication per processed frame, no overlapping inference, and p95 below 200 ms on the RTX 3090. Review state transitions for the expected smoothing delay and absence of long-lived stale green.

- [ ] **Step 5: Run final branch verification**

  Run: `git diff origin/main --check && catkin_make --pkg traffic_light && catkin_make run_tests_traffic_light && catkin_test_results --verbose`

  Expected: no whitespace errors, successful build, and zero test failures.

- [ ] **Step 6: Preserve evaluation artifacts without polluting the code repository**

  Keep `/home/stier/traffic_light_training/evaluation/` outside this repository. Commit the design document only if measured deployment facts required an actual update; otherwise leave the already committed spec unchanged and create no empty commit.
