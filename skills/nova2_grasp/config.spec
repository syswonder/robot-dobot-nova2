config:
  calibration_path:
    type: string
    default: ../../config/grasp/calibration.json
    description: Accepted exported planar calibration, User0/Tool0.
  ready_pose_path:
    type: string
    default: ../../config/grasp/ready_pose.json
    description: Saved measured ready and return joint pose.
  place_locations_path:
    type: string
    default: ''
    description: Optional JSON map of named User0 XY destinations and release heights above calibrated Z.
  runtime_path:
    type: string
    default: ../../config/grasp/runtime.json
    description: Persisted jaw-axis alignment, no credentials.
  artifacts_dir:
    type: string
    default: ../../.artifacts/grasp
    description: Local annotated images, detection plans and measured task records.
  camera_provider_id:
    type: string
    default: orbbec_camera
    description: Atlas provider ID of the SDK/USB camera primitive; no calibration HTTP dependency.
  camera_max_frame_age_s:
    type: float
    default: 3.0
    description: Reject snapshots older than this including MCP round-trip, 0.1..10 seconds.
  backend:
    type: string
    default: vlm
    description: vlm or yolo; empty per-call backend selects this default.
  approach_height_mm:
    type: float
    default: 200.0
    description: Hover/lift distance above fixed grasp Z, in mm, 20..400.
  velocity_percent:
    type: int
    default: 20
    description: Per-move joint/linear speed percentage, 1..80; user-authorized deployment ceiling is 80.
  gripper_speed:
    type: int
    default: 300
    description: TG9801 raw speed 200..1500, limited also by primitive.
  force_percent:
    type: int
    default: 20
    description: TG9801 force setting percent 1..50, NOT newtons.
  vertical_tolerance_deg:
    type: float
    default: 2.0
    description: Downward Tool0 approach-axis tilt permitted for recovery lifts.
  jaw_axis_offset_deg:
    type: float
    default: 0.0
    description: Two-finger closing-axis offset from Tool0 +X, X=0 Y=90. Persisted
      runtime.json overrides.
  jaw_alignment_confirmed:
    type: bool
    default: false
    description: Initial false until hover check; approach allowed, pick blocked until
      set_jaw_alignment confirms.
  min_confidence:
    type: float
    default: 0.6
    description: Detector confidence threshold, 0..1; model confidence is not calibrated
      probability.
  plan_ttl_s:
    type: float
    default: 120.0
    description: Maximum age of a detection plan at physical task start, 10..300 seconds.
  motion_timeout_s:
    type: float
    default: 305.0
    description: Wait deadline for each measured arm segment, 10..600 seconds; keep above the arm primitive deadline.
  gripper_timeout_s:
    type: float
    default: 8.0
    description: Wait for three matching measured jaw states, 1..10 seconds.
  max_calibration_error_mm:
    type: float
    default: 20.0
    description: Reject calibration whose maximum training residual exceeds this,
      0.1..20 mm.
  square_ratio:
    type: float
    default: 1.12
    description: Near-square faces have two equivalent closure directions; prefer
      nearest reference yaw.
  arm_provider_id:
    type: string
    default: nova2_arm
    description: Atlas arm provider ID.
  gripper_provider_id:
    type: string
    default: tg9801_gripper
    description: Atlas TG9801 provider ID.
  vlm_base_url:
    type: string
    default: ''
    description: Empty uses VLM_BASE_URL; OpenAI-compatible chat/completions endpoint.
  vlm_api_key:
    type: string
    default: ''
    description: Empty uses VLM_API_KEY; keep credentials in existing .env.
  vlm_model:
    type: string
    default: ''
    description: Empty uses VLM_MODEL; must support image input.
  vlm_timeout_s:
    type: float
    default: 45.0
    description: Model request timeout; no automatic retries.
  yolo_model_path:
    type: string
    default: ''
    description: Absolute local YOLO-OBB weights path; no automatic download or model
      fallback.
  yolo_class_aliases_json:
    type: string
    default: '{}'
    description: JSON mapping natural-language labels to YOLO class names, e.g. green
      block -> green_cube.
