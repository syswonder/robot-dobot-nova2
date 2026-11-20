config:
  robot_ip: {type: string, default: 192.168.5.1, description: Controller IPv4 address.}
  protocol: {type: string, default: v3, description: "v3 for firmware 3.5.8.5; v4 for V4-compatible controllers."}
  dashboard_port: {type: int, default: 29999, description: Dashboard port.}
  motion_port: {type: int, default: 30003, description: Separate V3 motion port; unused for V4.}
  feedback_port: {type: int, default: 30004, description: Real-time feedback port.}
  socket_timeout_s: {type: float, default: 3.0, description: Timeout for each dashboard exchange.}
  feedback_timeout_s: {type: float, default: 2.0, description: Maximum age of feedback used for motion.}
  motion_timeout_s: {type: float, default: 120.0, description: Motion deadline before requesting Stop.}
  default_velocity_percent: {type: int, default: 10, description: Default joint or linear velocity percentage.}
  max_velocity_percent: {type: int, default: 20, description: Maximum allowed joint or linear velocity percentage.}
  acceleration_percent: {type: int, default: 10, description: Per-motion acceleration percentage.}
