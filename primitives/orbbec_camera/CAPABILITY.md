# Orbbec USB camera capabilities

Provider `orbbec_camera`; namespace `robonix/primitive/orbbec_camera`.
MCP tools `get_state`, `camera_info`, `snapshot`, all with empty request and
typed response `ok:bool`, `message:string`, `result_json:string`.

Init validates config without opening USB. Activate starts one SDK worker and
waits for an actual original-resolution undistorted RGB JPEG. It validates
serial, dimensions, processing metadata and optional expected_camera_key.
Deactivate/shutdown releases SDK/USB, is idempotent, and issues no robot commands.
There is no depth acquisition, arbitrary file read, runtime command override,
robot enable or motion capability.

get_state returns `connected`, `error`, `source=orbbec_sdk_usb`; if fresh it
includes `frame_id`, `camera_key`, `age_s`, `metadata`. Disconnected is valid
state information, not a fabricated frame. camera_info/snapshot reject unavailable,
stale, failed-worker or invalid frames. No cached frame is returned after failure.

snapshot result includes `mime_type=image/jpeg`, `jpeg_base64`, `frame_id`,
`camera_key`, `age_s` (parent monotonic receive age), `received_at_unix_s`,
`captured_at_unix_s`, `metadata`. It is not sensor_msgs/Image or depth data.
Consumers validate fingerprint, resolution, MIME/base64/JPEG and freshness,
including transport time, before using pixels with a saved calibration.
No resize/letterbox or intrinsic substitution is performed.

metadata includes backend, device, serial, width, height, format, K, D,
processing and actual capture time/fps. K/D are the physical color intrinsics;
output image is undistorted with K unchanged, so downstream must not apply D
again. `camera_key` hashes the same metadata subset as the original calibration.

camera_info returns metadata, camera_key, coordinate convention and
`depth_enabled=false`. get_state/camera_info avoid the large image byte field.
Use grasp detect_object for semantic scene descriptions; snapshot primarily
serves image consumers and diagnostic clients.
