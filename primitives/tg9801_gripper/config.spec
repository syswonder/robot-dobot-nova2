config:
  port: {type: string, default: /dev/ttyUSB0, description: Native Linux serial port; ignored when windows_port is set.}
  windows_port: {type: string, default: "", description: WSL private serial relay to Windows COM number.}
  protocol: {type: string, default: auto, description: "auto probes read-only; modbus/custom fix host framing without changing device protocol."}
  adaptive_status_scheme: {type: string, default: modbus_manual, description: "modbus_manual: 2 holding / 3 slipping. three_is_holding: 2 waiting / 3 holding, only after hardware commissioning; slipping unavailable."}
  baudrate: {type: int, default: 1000000, description: Host baudrate; never writes device baudrate.}
  device_id: {type: int, default: 1, description: Unicast device address 1..247; never changes device ID.}
  timeout_s: {type: float, default: 0.5, description: Per-exchange serial timeout 0.05..2 seconds.}
  default_speed: {type: int, default: 300, description: Raw speed 200..max_speed; custom open has no speed field.}
  default_force_percent: {type: int, default: 20, description: Device target-force setting; percent not newtons.}
  max_speed: {type: int, default: 1500, description: Reject raw speed above this ceiling 200..1500.}
  max_force_percent: {type: int, default: 50, description: Reject target force above this ceiling 1..100.}
  motion_enabled: {type: bool, default: true, description: Allow explicit gripper commands; startup remains read-only.}
  position_control_enabled: {type: bool, default: false, description: Optional raw position mode after manual direction/endpoint checks.}
