config:
  backend: {type: string, default: auto, description: "auto uses private Windows SDK under WSL, native otherwise; windows/native force selection."}
  serial_number: {type: string, default: "", description: "Pin USB camera serial; empty requires exactly one camera."}
  color_width: {type: int, default: 1280, description: Original color width; changing it requires compatible calibration.}
  color_height: {type: int, default: 720, description: Original color height.}
  color_format: {type: string, default: MJPG, description: Requested SDK profile format MJPG/RGB/BGR/YUYV.}
  color_fps: {type: int, default: 10, description: USB color stream profile fps; actual calibrated host profile is 1280x720 MJPG at 10 fps.}
  publish_fps: {type: int, default: 8, description: Max processed snapshots per second 1..30.}
  jpeg_quality: {type: int, default: 88, description: JPEG quality 1..100; no resize.}
  startup_timeout_s: {type: float, default: 30.0, description: Activation waits for first valid frame 0.1..60 seconds.}
  max_frame_age_s: {type: float, default: 3.0, description: Refuse stale frame after 0.1..60 seconds.}
  expected_camera_key: {type: string, default: "", description: Optional pin to camera_key from accepted calibration; mismatches fail activation.}
