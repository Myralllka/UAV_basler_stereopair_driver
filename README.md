# Driver for custom stereopair of Basler cameras

Calibration can be done by exactly 12 apriltags, if at least one is not visible - that frame is not counting


To start a calibration process use
```bash
./tmux_scripts/start_calibration.sh
```

To complete (save calibrated pose to `config/camera_poses.yaml`) use
```bash
./tmux_scripts/complete_calibration.sh
```