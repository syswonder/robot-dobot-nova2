# TG9801 capabilities

Provider instance in the body: `tg9801_gripper`.
All tool responses contain `ok`, `message`, `result_json`.
Consumers must inspect `ok` before interpreting result_json.

| Contract suffix | Request | Meaning |
|---|---|---|
| info | none | Protocol, limits, defaults, supported features |
| get_state | none | New measured sample with holding/moving and raw feedback |
| open | speed | Adaptive release; no target width |
| close / grip | speed, force_percent | Same adaptive grasp, not unconditional full travel |
| set_parameters | speed, force_percent | In-memory next-command defaults, no hardware writes |
| move_position | position_raw, speed, force_percent | Optional force-position mode; disabled by default |
| wait_for_state | condition, timeout_s | 3 matching samples; holding / idle / not_moving |

speed=0 selects default; explicit speeds are device raw 200..1500 bounded by
max_speed. force_percent=0 selects default; explicit 1..100 bounded by
max_force_percent. This is an algorithm setting, not calibrated force in N.
position_raw commands are 0..1000, not metres/mm or a normalized opening.
Measured endpoint feedback may exceed that range slightly (accepted through
1010) and is reported without clamping. No direction or raw-to-gap conversion
is assumed. Runtime defaults reset on process restart.
The custom release command has no speed field: open reports open_speed_applied=false.

Each motion request selects its needed mode (adaptive=0, force-position=1),
then applies force for close/grip/position, then sends motion. Mode selection
and force writes occur only after an explicit motion request, never at boot.
Modbus grip AND release use FC10 at 0x000A with [speed, action]. Optional raw
position uses FC10 at 0x000F with [speed, target]; these are two different modes.

Default Modbus adaptive status (`adaptive_status_scheme=modbus_manual`):
0 idle, 1 moving, 2 holding, 3 slipping. Explicit commissioning override
`three_is_holding`: 0 idle, 1 moving, 2 waiting, 3 holding; slipping is unknown
(`null`). The current body uses this override after operator confirmation and
stable raw-position feedback on this device; it is not auto-detected or a
claim about all Modbus firmware. Raw status and selected scheme are returned.
Custom status:
0 idle, 1 moving, 2 waiting for stable grip, 3 holding. The mappings are NOT
interchangeable. Modbus force-position mode uses 0x002F: 0 moving, 1 at target,
2 holding. Current/hardness are raw; custom errors/slipping/mode are unknown
when not exposed by the polled frame, rather than invented values. Custom
7-byte SDK payload and 11-byte manual payload are both checked for length,
checksum, address and command. Extra bytes are retained without a guessed unit.

A Modbus ACK only proves command acknowledgement. Custom motion is fire-and-
forget at device level; host_sent_only never means device ACK or arrival.
Read get_state/wait_for_state afterward. Idle/not_moving does not establish
fully open jaws or a stable grip. Timeout does not cancel, stop or reverse.
No automatic retry, reopen, release on shutdown, ID/baud/protocol changes,
EEPROM writes or fingertip calibration. A possibly sent failed write latches
a fault and blocks further writes until process restart after inspection.
There is no verified jaw halt command. Nova2 stop/disable affects only the arm.

Startup opens the serial link and proves state readback. `auto` probes the two
protocols using only queries, at the configured baudrate, never by writing a
protocol selector. Native Linux port ownership is exclusive. WSL relay owns
one Windows COM handle over private process pipes and a PTY, without TCP ports,
drivers, global Python changes or an O6 runtime dependency.
