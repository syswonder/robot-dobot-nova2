# Official upstream sources

- `primitives/tg9801_gripper/tg9801/{protocol,controller,driver}.py` is a local
  implementation using the command/register definitions and Python SDK shown at
  https://www.hkvtech.cn/toolbox/UserManual.html#sdk, retrieved 2026-11-18.
  The embedded SDK project is `sdk-ms5r1qp96jla` (机械夹爪Python示例).
  No vendor SDK source or binary is installed into global Python or copied into
  the runtime package. The official SDK/manually listed frames were consulted
  for Modbus RTU, custom framing, raw speed/force/position and status semantics.
  `syswonder/primitive-jaka-mini_with_tg9801_gripper-arm_gripper-rbnx` was consulted
  for adaptive-only public semantics and the hardware finding that release on
  some firmware requires FC10 speed+action rather than FC06 action alone.
  There is no JAKA SDK, ROS or controller-end serial dependency in this provider.
- `primitives/tg9801_gripper/tg9801/windows_serial.py` reuses the existing local
  O6 byte relay with its MulanPSL-2.0 license, preserved as
  `primitives/tg9801_gripper/LICENSE-WINDOWS-SERIAL`. Other new gripper code uses
  the package's MIT license. The relay is self-contained; O6 and its SDK are not
  imported or activated by TG9801.

- `primitives/nova2_arm/nova2_arm/vendor/dobot_api_v3.py` comes from
  [Dobot-Arm/TCP-IP-Python-V3](https://github.com/Dobot-Arm/TCP-IP-Python-V3),
  commit `7fb060c9ad88332ec6fe8e768d72551c65af24f8` (MIT; `vendor/LICENSE-V3`).
  The single headless adaptation replaces the tkinter Text/END import with
  a type annotation and the equivalent string constant; no GUI is used.
  Its command builders and V3 feedback layout are reused. `ControlAPI` builds
  explicit User0/Tool0 GetPose and positional InverseSolution requests because
  this SDK's GetPose has no frame parameters and its variadic InverseSolution
  builder inserts Python repr strings without the required commas. The adapter
  also builds the four-argument SetArmOrientation request because a later
  one-argument definition in this SDK shadows that method.
- `primitives/nova2_arm/nova2_arm/vendor/dobot_api.py` is an unmodified copy of
  [Dobot-Arm/TCP-IP-Python-V4](https://github.com/Dobot-Arm/TCP-IP-Python-V4),
  commit `805247ae5688caeedacd9eb7846c94c76322513e` (MIT; license alongside it).
  This package reuses `DobotApiDashboard` command builders and `MyType` feedback
  layout. Its transport subclass replaces the upstream socket/reconnect code
  with bounded exchanges and never reconnects/replays a command automatically.
  SDK UI, demo execution and alarm-file loading are not used in either version.
- `model/nova2.urdf` and `model/meshes/*.STL` come from
  [Dobot-Arm/DOBOT_6Axis_ROS2_V4](https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V4),
  commit `7807ef8fb31e8f86a76cd474a4a9cddffef6eca9` (MIT; `model/LICENSE`).
  The only URDF change is replacing ROS `package://` mesh paths with local
  relative paths. `nova2_limits.urdf` extracts the same six joint limits into
  the independently runnable primitive package. Approximate URDF limits do
  not override controller limits or protective settings.
- The ROS2 V4 `MovJ.srv`, `MovL.srv`, `cr_robot_ros2.cpp` and
  `dobot_moveit/scripts/send_pose_target.py` were consulted for coordinates
  and units. The quaternion conversion uses extrinsic xyz (Rz*Ry*Rx) and was
  checked against the real controller's pose and actual-quaternion feedback.
  ROS/MoveIt are not runtime dependencies.
- The official [V3 TCP/IP protocol guide](https://github.com/Dobot-Arm/TCP-IP-Protocol-6AXis-V3)
  (2025-05-08 English edition) was checked for native JointMovJ/MovL parameters,
  positional InverseSolution, ResetRobot, CP, SetArmOrientation and TCPSpeedEnd.

The pinned repositories are separate references; no assumption is made
that arbitrary controller firmware implements every upstream SDK command.
The default V3 configuration targets the observed firmware 3.5.8.5 (upstream
V3 SDK supports 3.5.5.0+). `protocol: v4` requires V4-compatible firmware
(upstream states V4.4.0.0+). Both require TCP/IP mode. No automatic protocol
fallback or movement retry is performed.


## Nova2 vision grasp skill

`skills/nova2_grasp` is a new MIT implementation for this deployment. Its capability grouping
and perception/action workflow were informed by the public Robonix packages
[skill-qmzs-grasp-rbnx](https://github.com/syswonder/skill-qmzs-grasp-rbnx) and
[skill-vertical-grasp-object-rbnx](https://github.com/syswonder/skill-vertical-grasp-object-rbnx)
(MulanPSL-2.0); their robot SDKs and implementation files were not copied.
The optional YOLO backend uses the separately installed Ultralytics distribution and user-supplied
OBB weights; neither Ultralytics nor model weights are vendored. The default VLM path does not install it.
Camera frames now come from the local SDK/USB primitive through Atlas/MCP, without the independent
calibration service. The installed calibration JSON and coordinate convention are retained.

## Orbbec SDK USB primitive

`primitives/orbbec_camera` is a new MIT implementation. The lifecycle, validated-first-frame gate,
continuous capture and on-demand snapshot design reference
[primitive-orbbec-camera-rbnx](https://github.com/syswonder/primitive-orbbec-camera-rbnx)
(Apache-2.0); its ROS driver and source files are not copied. The distinct
`robonix/primitive/orbbec_camera/*` contracts return local typed JSON snapshots and do not redefine
the community ROS camera topic or sensor_msgs/Image contracts.

The SDK worker preserves the undistortion/fingerprint convention of the local MIT standalone
calibration service. Official `pyorbbecsdk2` 2.1.3 (Apache-2.0), NumPy and OpenCV are installed
only into the package environment/native private Windows runtime; no SDK source is vendored.
This host's initial Windows runtime was copied once from the existing embedded runtime into a
separate `LocalAppData/Robonix-Nova2-Orbbec` directory and its installed versions/imports verified.
Runtime operation and future normal builds do not read or import the calibration service directory.
SDK method signatures were checked against its installed `pyorbbecsdk.pyi` and official
[Orbbec Python SDK](https://github.com/orbbec/pyorbbecsdk/tree/v2-main).
