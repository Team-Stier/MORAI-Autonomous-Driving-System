# Traffic-light model artifacts

- `traffic_light_yolo11m.onnx`: ROS core deployment model, static input `[1,3,1280,1280]`
- `traffic_light_yolo11m.json`: class order and export manifest
- `checkpoints/best.pt`: best validation checkpoint (epoch 64)
- `checkpoints/last.pt`: final early-stopping checkpoint (89 completed epochs)
- `checkpoints/epoch*.pt`: periodic training checkpoints

Class order is `go`, `left_turn`, `prepare_stop`, `stop`. Binary model files are stored with Git LFS. The training dataset and rosbags are intentionally excluded.
