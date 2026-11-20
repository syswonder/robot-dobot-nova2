# Nova2 arm

Controls one physical Dobot Nova2 at 192.168.5.1. Firmware 3.5.8.5 uses the
official V3 protocol (29999 control, 30003 motion, 30004 feedback). A separate
`protocol: v4` configuration supports V4 command builders and feedback.
Read `get_state` before moving. Controller must be in TCP/IP mode; no automatic
mode switching, power-on, enable, alarm clearing, or motion occurs at startup.
`queue_running` is the controller's algorithm-queue running flag, not a pending
motion count. It can remain true while RobotMode=5 and the arm is stationary.

- `enable`, `disable`, `clear_error`, `stop`: explicit controller actions.
- `move_joints`: **absolute** J1..J6, six angles in **degrees**.
- `move_pose`: **absolute** `[X,Y,Z,Rx,Ry,Rz]`, **mm and degrees**.
- `move_quaternion`: absolute XYZ in **mm**, unit quaternion **[qx,qy,qz,qw]**.
- All Cartesian commands and kinematics use **User0/Tool0**. Tool0 is the
  controller-defined tool reference; configure/verify it in Dobot software.
- `MovJ` chooses a joint-space path; `MovL` requests a straight TCP path.
  Both use `velocity_percent`; zero means the configured default. On V3,
  joint targets use `JointMovJ`, and a stop uses `ResetRobot`.
  V3 Cartesian MovJ uses the nearby controller IK solution through JointMovJ.
  Before V3 MovL, the driver selects the IK solution's hand configuration and
  ends an existing TCPSpeed override so SpeedL percentages apply.
- `forward_kinematics` and `inverse_kinematics` calculate only; they do not move.

Motion returns a `run_id`. Poll `motion_status(run_id, wait_s=10)` until a
terminal result. Only `SUCCEEDED` confirms arrival from measured feedback;
an accepted command is not completion. Do not submit a second motion while
the current one is running. A failed MCP response (`ok=false`), `FAILED`,
`UNKNOWN`, or `NOT_FOUND` must not be reported as success or retried blindly.

For a relative user request, first read actual state and compute an absolute
target, retaining every unspecified joint or orientation. Do not invent a
home pose, joint zero pose, coordinate frame, target, or missing orientation.
Ask for the missing target information when it cannot be read from live state.
Report the units and frame used. Stop immediately if the user requests it.

No camera, hand-eye calibration, obstacle avoidance, collision-free path
planning, gripper or generic Cartesian servo control is provided. IK success
only checks the endpoint, not the path. The controller's protective functions
and physical emergency stop remain necessary. Chat stop depends on model and
network latency and is not a substitute for the physical emergency stop.

Default velocity is 10%, maximum 20%, for both MovJ and MovL. These are
controller velocity ratios, not degrees/s or mm/s. Acceleration is 10%.
On V3, the package sets CP(0) before a move; V4 uses per-motion cp=0.
The actual controller speed override can further reduce speed. Startup leaves
existing payload, User/Tool definitions and controller settings untouched.
Shutdown requests Stop only for motion owned by this provider; it does not
automatically disable or move home. Do not run another controller concurrently.
