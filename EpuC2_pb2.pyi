from google.protobuf.internal import containers as _containers
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

A: FormationType
ABORT: AutopilotMode
ARIELS: PlatformType
ARMED: ArmState
AUTO: AutopilotMode
B: FormationType
BODY_RELATIVE: RemoteSteerMode
C: FormationType
CIRCLE: FormationType
COMPASS_RELATIVE: RemoteSteerMode
COMPLETE: TaskStatus
CRITICAL: ArmState
D: FormationType
DAMOCLES: PlatformType
DESCRIPTOR: _descriptor.FileDescriptor
DIAMOND: FormationType
DISARMED: ArmState
DISTRIBUTE_PERIMETER: DistributeMode
DISTRIBUTE_POLYLINE: DistributeMode
DISTRIBUTE_REGION: DistributeMode
DISTRIBUTE_VOLUME: DistributeMode
DOWN_ARROW: FormationType
DO_DEPLOY_PAYLOAD: ControlPointAction
DO_HDG_HOLD: ControlPointAction
DO_LAND: ControlPointAction
DO_POS_HOLD: ControlPointAction
DO_RETURN: ControlPointAction
E: FormationType
ECHELON_LEFT: FormationType
ECHELON_RIGHT: FormationType
EGOCENTRIC: ControlPointType
EXECUTING: TaskStatus
F: FormationType
FAILED: TaskStatus
FILE: FormationType
G: FormationType
GEODETIC: ControlPointType
H: FormationType
HORRIS: PlatformType
I: FormationType
J: FormationType
JAWBREAKER: PlatformType
K: FormationType
L: FormationType
LAND: AutopilotMode
LEFT_ARROW: FormationType
LINE: FormationType
LOCAL: ControlPointType
LOITER: AutopilotMode
M: FormationType
MANUAL: AutopilotMode
MGRS: ControlPointType
N: FormationType
O: FormationType
OFFBOARD: AutopilotMode
P: FormationType
Q: FormationType
R: FormationType
RIGHT_ARROW: FormationType
ROTATE_PITCH: BodyRelativeCmd
ROTATE_ROW: BodyRelativeCmd
ROTATE_YAW: BodyRelativeCmd
RTL: AutopilotMode
S: FormationType
SQUARE: FormationType
STAGGERED_COLUMN: FormationType
T: FormationType
TASK_DISTRIBUTE: TaskType
TASK_FORMATION: TaskType
TASK_GET_GEOFENCE: TaskType
TASK_GOTO: TaskType
TASK_IDLE: TaskType
TASK_LAND: TaskType
TASK_LAUNCH: TaskType
TASK_LEADER_FOLLOW: TaskType
TASK_PATROL: TaskType
TASK_POS_HOLD: TaskType
TASK_PULSATE: TaskType
TASK_REMOTE_STEER: TaskType
TASK_RETURN: TaskType
TASK_SET_GEOFENCE: TaskType
TASK_SET_RALLY_PT: TaskType
TASK_TELEOP: TaskType
TRANSLATE_FWD_BCK: BodyRelativeCmd
TRANSLATE_LFT_RGT: BodyRelativeCmd
TRANSLATE_VERTICAL: BodyRelativeCmd
TRIANGLE: FormationType
U: FormationType
UNINIT: ArmState
UNKNOWN: AutopilotMode
UP_ARROW: FormationType
V: FormationType
VEE: FormationType
W: FormationType
WEDGE: FormationType
X: FormationType
Y: FormationType
Z: FormationType

class Circle(_message.Message):
    __slots__ = ["origin", "radius"]
    ORIGIN_FIELD_NUMBER: _ClassVar[int]
    RADIUS_FIELD_NUMBER: _ClassVar[int]
    origin: LLA
    radius: float
    def __init__(self, origin: _Optional[_Union[LLA, _Mapping]] = ..., radius: _Optional[float] = ...) -> None: ...

class ControlPoint(_message.Message):
    __slots__ = ["action", "body_rel_cmd", "ctrl_pt_type", "hdg_deg", "mgrs", "pos_lla", "pos_ned", "rotation_cmd_increment_deg", "translation_cmd_increment_m", "vel_ned"]
    ACTION_FIELD_NUMBER: _ClassVar[int]
    BODY_REL_CMD_FIELD_NUMBER: _ClassVar[int]
    CTRL_PT_TYPE_FIELD_NUMBER: _ClassVar[int]
    HDG_DEG_FIELD_NUMBER: _ClassVar[int]
    MGRS_FIELD_NUMBER: _ClassVar[int]
    POS_LLA_FIELD_NUMBER: _ClassVar[int]
    POS_NED_FIELD_NUMBER: _ClassVar[int]
    ROTATION_CMD_INCREMENT_DEG_FIELD_NUMBER: _ClassVar[int]
    TRANSLATION_CMD_INCREMENT_M_FIELD_NUMBER: _ClassVar[int]
    VEL_NED_FIELD_NUMBER: _ClassVar[int]
    action: ControlPointAction
    body_rel_cmd: BodyRelativeCmd
    ctrl_pt_type: ControlPointType
    hdg_deg: float
    mgrs: str
    pos_lla: LLA
    pos_ned: Vec3
    rotation_cmd_increment_deg: int
    translation_cmd_increment_m: int
    vel_ned: Vec3
    def __init__(self, ctrl_pt_type: _Optional[_Union[ControlPointType, str]] = ..., pos_lla: _Optional[_Union[LLA, _Mapping]] = ..., mgrs: _Optional[str] = ..., pos_ned: _Optional[_Union[Vec3, _Mapping]] = ..., body_rel_cmd: _Optional[_Union[BodyRelativeCmd, str]] = ..., vel_ned: _Optional[_Union[Vec3, _Mapping]] = ..., action: _Optional[_Union[ControlPointAction, str]] = ..., hdg_deg: _Optional[float] = ..., translation_cmd_increment_m: _Optional[int] = ..., rotation_cmd_increment_deg: _Optional[int] = ...) -> None: ...

class Distribute_Task_Params(_message.Message):
    __slots__ = ["alt_m", "circular_region", "distribute_mode", "land_on_complete", "max_alt_m", "min_alt_m", "polygon_region"]
    ALT_M_FIELD_NUMBER: _ClassVar[int]
    CIRCULAR_REGION_FIELD_NUMBER: _ClassVar[int]
    DISTRIBUTE_MODE_FIELD_NUMBER: _ClassVar[int]
    LAND_ON_COMPLETE_FIELD_NUMBER: _ClassVar[int]
    MAX_ALT_M_FIELD_NUMBER: _ClassVar[int]
    MIN_ALT_M_FIELD_NUMBER: _ClassVar[int]
    POLYGON_REGION_FIELD_NUMBER: _ClassVar[int]
    alt_m: float
    circular_region: Circle
    distribute_mode: DistributeMode
    land_on_complete: bool
    max_alt_m: float
    min_alt_m: float
    polygon_region: _containers.RepeatedCompositeFieldContainer[LLA]
    def __init__(self, circular_region: _Optional[_Union[Circle, _Mapping]] = ..., polygon_region: _Optional[_Iterable[_Union[LLA, _Mapping]]] = ..., distribute_mode: _Optional[_Union[DistributeMode, str]] = ..., alt_m: _Optional[float] = ..., min_alt_m: _Optional[float] = ..., max_alt_m: _Optional[float] = ..., land_on_complete: bool = ...) -> None: ...

class Formation_Task_Params(_message.Message):
    __slots__ = ["centroid", "formation_type", "leader_id", "match_heading", "scale_factor"]
    CENTROID_FIELD_NUMBER: _ClassVar[int]
    FORMATION_TYPE_FIELD_NUMBER: _ClassVar[int]
    LEADER_ID_FIELD_NUMBER: _ClassVar[int]
    MATCH_HEADING_FIELD_NUMBER: _ClassVar[int]
    SCALE_FACTOR_FIELD_NUMBER: _ClassVar[int]
    centroid: ControlPoint
    formation_type: FormationType
    leader_id: str
    match_heading: bool
    scale_factor: int
    def __init__(self, formation_type: _Optional[_Union[FormationType, str]] = ..., leader_id: _Optional[str] = ..., centroid: _Optional[_Union[ControlPoint, _Mapping]] = ..., scale_factor: _Optional[int] = ..., match_heading: bool = ...) -> None: ...

class Goto_Task_Params(_message.Message):
    __slots__ = ["land_on_completion", "randomized_trajectory", "return_on_completion", "tgt_pos"]
    LAND_ON_COMPLETION_FIELD_NUMBER: _ClassVar[int]
    RANDOMIZED_TRAJECTORY_FIELD_NUMBER: _ClassVar[int]
    RETURN_ON_COMPLETION_FIELD_NUMBER: _ClassVar[int]
    TGT_POS_FIELD_NUMBER: _ClassVar[int]
    land_on_completion: bool
    randomized_trajectory: bool
    return_on_completion: bool
    tgt_pos: ControlPoint
    def __init__(self, tgt_pos: _Optional[_Union[ControlPoint, _Mapping]] = ..., return_on_completion: bool = ..., land_on_completion: bool = ..., randomized_trajectory: bool = ...) -> None: ...

class LLA(_message.Message):
    __slots__ = ["alt_m", "lat_d", "lon_d"]
    ALT_M_FIELD_NUMBER: _ClassVar[int]
    LAT_D_FIELD_NUMBER: _ClassVar[int]
    LON_D_FIELD_NUMBER: _ClassVar[int]
    alt_m: float
    lat_d: float
    lon_d: float
    def __init__(self, lat_d: _Optional[float] = ..., lon_d: _Optional[float] = ..., alt_m: _Optional[float] = ...) -> None: ...

class Leader_Follow_Task_Params(_message.Message):
    __slots__ = ["leader_id", "offset_m"]
    LEADER_ID_FIELD_NUMBER: _ClassVar[int]
    OFFSET_M_FIELD_NUMBER: _ClassVar[int]
    leader_id: str
    offset_m: Vec3
    def __init__(self, leader_id: _Optional[str] = ..., offset_m: _Optional[_Union[Vec3, _Mapping]] = ...) -> None: ...

class Line(_message.Message):
    __slots__ = ["end_pt", "start_pt"]
    END_PT_FIELD_NUMBER: _ClassVar[int]
    START_PT_FIELD_NUMBER: _ClassVar[int]
    end_pt: LLA
    start_pt: LLA
    def __init__(self, start_pt: _Optional[_Union[LLA, _Mapping]] = ..., end_pt: _Optional[_Union[LLA, _Mapping]] = ...) -> None: ...

class Message(_message.Message):
    __slots__ = ["status", "task", "teleop_cmd"]
    STATUS_FIELD_NUMBER: _ClassVar[int]
    TASK_FIELD_NUMBER: _ClassVar[int]
    TELEOP_CMD_FIELD_NUMBER: _ClassVar[int]
    status: _containers.RepeatedCompositeFieldContainer[Status]
    task: _containers.RepeatedCompositeFieldContainer[Task]
    teleop_cmd: _containers.RepeatedCompositeFieldContainer[TeleopCommand]
    def __init__(self, status: _Optional[_Iterable[_Union[Status, _Mapping]]] = ..., task: _Optional[_Iterable[_Union[Task, _Mapping]]] = ..., teleop_cmd: _Optional[_Iterable[_Union[TeleopCommand, _Mapping]]] = ...) -> None: ...

class NavState(_message.Message):
    __slots__ = ["commanded_control_pt", "distr_region", "hdg_d", "origin_lla", "planned_traj", "polygon_fence", "pos_lla", "pose", "rel_alt_m"]
    COMMANDED_CONTROL_PT_FIELD_NUMBER: _ClassVar[int]
    DISTR_REGION_FIELD_NUMBER: _ClassVar[int]
    HDG_D_FIELD_NUMBER: _ClassVar[int]
    ORIGIN_LLA_FIELD_NUMBER: _ClassVar[int]
    PLANNED_TRAJ_FIELD_NUMBER: _ClassVar[int]
    POLYGON_FENCE_FIELD_NUMBER: _ClassVar[int]
    POSE_FIELD_NUMBER: _ClassVar[int]
    POS_LLA_FIELD_NUMBER: _ClassVar[int]
    REL_ALT_M_FIELD_NUMBER: _ClassVar[int]
    commanded_control_pt: _containers.RepeatedCompositeFieldContainer[ControlPoint]
    distr_region: _containers.RepeatedCompositeFieldContainer[LLA]
    hdg_d: float
    origin_lla: LLA
    planned_traj: _containers.RepeatedCompositeFieldContainer[ControlPoint]
    polygon_fence: _containers.RepeatedCompositeFieldContainer[LLA]
    pos_lla: LLA
    pose: Pose
    rel_alt_m: float
    def __init__(self, pose: _Optional[_Union[Pose, _Mapping]] = ..., pos_lla: _Optional[_Union[LLA, _Mapping]] = ..., rel_alt_m: _Optional[float] = ..., hdg_d: _Optional[float] = ..., origin_lla: _Optional[_Union[LLA, _Mapping]] = ..., planned_traj: _Optional[_Iterable[_Union[ControlPoint, _Mapping]]] = ..., commanded_control_pt: _Optional[_Iterable[_Union[ControlPoint, _Mapping]]] = ..., polygon_fence: _Optional[_Iterable[_Union[LLA, _Mapping]]] = ..., distr_region: _Optional[_Iterable[_Union[LLA, _Mapping]]] = ...) -> None: ...

class Patrol_Task_Params(_message.Message):
    __slots__ = ["land_on_completion", "return_on_completion", "tgt_route", "timeout_s"]
    LAND_ON_COMPLETION_FIELD_NUMBER: _ClassVar[int]
    RETURN_ON_COMPLETION_FIELD_NUMBER: _ClassVar[int]
    TGT_ROUTE_FIELD_NUMBER: _ClassVar[int]
    TIMEOUT_S_FIELD_NUMBER: _ClassVar[int]
    land_on_completion: bool
    return_on_completion: bool
    tgt_route: _containers.RepeatedCompositeFieldContainer[ControlPoint]
    timeout_s: int
    def __init__(self, tgt_route: _Optional[_Iterable[_Union[ControlPoint, _Mapping]]] = ..., return_on_completion: bool = ..., land_on_completion: bool = ..., timeout_s: _Optional[int] = ...) -> None: ...

class Pose(_message.Message):
    __slots__ = ["acc_ned", "pitch_r", "pos_ned", "roll_r", "vel_ned", "yaw_r"]
    ACC_NED_FIELD_NUMBER: _ClassVar[int]
    PITCH_R_FIELD_NUMBER: _ClassVar[int]
    POS_NED_FIELD_NUMBER: _ClassVar[int]
    ROLL_R_FIELD_NUMBER: _ClassVar[int]
    VEL_NED_FIELD_NUMBER: _ClassVar[int]
    YAW_R_FIELD_NUMBER: _ClassVar[int]
    acc_ned: Vec3
    pitch_r: float
    pos_ned: Vec3
    roll_r: float
    vel_ned: Vec3
    yaw_r: float
    def __init__(self, yaw_r: _Optional[float] = ..., pitch_r: _Optional[float] = ..., roll_r: _Optional[float] = ..., pos_ned: _Optional[_Union[Vec3, _Mapping]] = ..., vel_ned: _Optional[_Union[Vec3, _Mapping]] = ..., acc_ned: _Optional[_Union[Vec3, _Mapping]] = ...) -> None: ...

class Pulsate_Task_Params(_message.Message):
    __slots__ = ["attack_pt", "inner_radius_m", "outter_radius_m", "pulsate_period_s", "timeout_s"]
    ATTACK_PT_FIELD_NUMBER: _ClassVar[int]
    INNER_RADIUS_M_FIELD_NUMBER: _ClassVar[int]
    OUTTER_RADIUS_M_FIELD_NUMBER: _ClassVar[int]
    PULSATE_PERIOD_S_FIELD_NUMBER: _ClassVar[int]
    TIMEOUT_S_FIELD_NUMBER: _ClassVar[int]
    attack_pt: ControlPoint
    inner_radius_m: int
    outter_radius_m: int
    pulsate_period_s: int
    timeout_s: int
    def __init__(self, attack_pt: _Optional[_Union[ControlPoint, _Mapping]] = ..., pulsate_period_s: _Optional[int] = ..., inner_radius_m: _Optional[int] = ..., outter_radius_m: _Optional[int] = ..., timeout_s: _Optional[int] = ...) -> None: ...

class Remote_Steer_Task_Params(_message.Message):
    __slots__ = ["set_altitude_m", "set_azimuth_deg", "set_bearing_deg", "set_gnd_spd_mps", "steer_mode"]
    SET_ALTITUDE_M_FIELD_NUMBER: _ClassVar[int]
    SET_AZIMUTH_DEG_FIELD_NUMBER: _ClassVar[int]
    SET_BEARING_DEG_FIELD_NUMBER: _ClassVar[int]
    SET_GND_SPD_MPS_FIELD_NUMBER: _ClassVar[int]
    STEER_MODE_FIELD_NUMBER: _ClassVar[int]
    set_altitude_m: float
    set_azimuth_deg: float
    set_bearing_deg: float
    set_gnd_spd_mps: float
    steer_mode: RemoteSteerMode
    def __init__(self, steer_mode: _Optional[_Union[RemoteSteerMode, str]] = ..., set_bearing_deg: _Optional[float] = ..., set_azimuth_deg: _Optional[float] = ..., set_altitude_m: _Optional[float] = ..., set_gnd_spd_mps: _Optional[float] = ...) -> None: ...

class Return_Task_Params(_message.Message):
    __slots__ = ["land_on_completion"]
    LAND_ON_COMPLETION_FIELD_NUMBER: _ClassVar[int]
    land_on_completion: bool
    def __init__(self, land_on_completion: bool = ...) -> None: ...

class Set_GeoFence_Task_Params(_message.Message):
    __slots__ = ["polygon_fence"]
    POLYGON_FENCE_FIELD_NUMBER: _ClassVar[int]
    polygon_fence: _containers.RepeatedCompositeFieldContainer[LLA]
    def __init__(self, polygon_fence: _Optional[_Iterable[_Union[LLA, _Mapping]]] = ...) -> None: ...

class Set_Rally_Pt_Task_Params(_message.Message):
    __slots__ = ["rally_pt"]
    RALLY_PT_FIELD_NUMBER: _ClassVar[int]
    rally_pt: LLA
    def __init__(self, rally_pt: _Optional[_Union[LLA, _Mapping]] = ...) -> None: ...

class Status(_message.Message):
    __slots__ = ["creator", "nav_state", "platform_type", "task_state", "ts_ms", "vehicle_state"]
    CREATOR_FIELD_NUMBER: _ClassVar[int]
    NAV_STATE_FIELD_NUMBER: _ClassVar[int]
    PLATFORM_TYPE_FIELD_NUMBER: _ClassVar[int]
    TASK_STATE_FIELD_NUMBER: _ClassVar[int]
    TS_MS_FIELD_NUMBER: _ClassVar[int]
    VEHICLE_STATE_FIELD_NUMBER: _ClassVar[int]
    creator: str
    nav_state: NavState
    platform_type: PlatformType
    task_state: TaskState
    ts_ms: int
    vehicle_state: VehicleState
    def __init__(self, creator: _Optional[str] = ..., ts_ms: _Optional[int] = ..., nav_state: _Optional[_Union[NavState, _Mapping]] = ..., vehicle_state: _Optional[_Union[VehicleState, _Mapping]] = ..., task_state: _Optional[_Union[TaskState, _Mapping]] = ..., platform_type: _Optional[_Union[PlatformType, str]] = ...) -> None: ...

class Task(_message.Message):
    __slots__ = ["applies_to", "creator", "distribute_task_params", "formation_task_params", "goto_task_params", "leader_follow_task_params", "patrol_task_params", "pulsate_task_params", "remote_steer_task_params", "return_task_params", "set_geo_fence_task_params", "set_rally_pt_task_params", "task_type", "ts_ms"]
    APPLIES_TO_FIELD_NUMBER: _ClassVar[int]
    CREATOR_FIELD_NUMBER: _ClassVar[int]
    DISTRIBUTE_TASK_PARAMS_FIELD_NUMBER: _ClassVar[int]
    FORMATION_TASK_PARAMS_FIELD_NUMBER: _ClassVar[int]
    GOTO_TASK_PARAMS_FIELD_NUMBER: _ClassVar[int]
    LEADER_FOLLOW_TASK_PARAMS_FIELD_NUMBER: _ClassVar[int]
    PATROL_TASK_PARAMS_FIELD_NUMBER: _ClassVar[int]
    PULSATE_TASK_PARAMS_FIELD_NUMBER: _ClassVar[int]
    REMOTE_STEER_TASK_PARAMS_FIELD_NUMBER: _ClassVar[int]
    RETURN_TASK_PARAMS_FIELD_NUMBER: _ClassVar[int]
    SET_GEO_FENCE_TASK_PARAMS_FIELD_NUMBER: _ClassVar[int]
    SET_RALLY_PT_TASK_PARAMS_FIELD_NUMBER: _ClassVar[int]
    TASK_TYPE_FIELD_NUMBER: _ClassVar[int]
    TS_MS_FIELD_NUMBER: _ClassVar[int]
    applies_to: _containers.RepeatedScalarFieldContainer[str]
    creator: str
    distribute_task_params: Distribute_Task_Params
    formation_task_params: Formation_Task_Params
    goto_task_params: Goto_Task_Params
    leader_follow_task_params: Leader_Follow_Task_Params
    patrol_task_params: Patrol_Task_Params
    pulsate_task_params: Pulsate_Task_Params
    remote_steer_task_params: Remote_Steer_Task_Params
    return_task_params: Return_Task_Params
    set_geo_fence_task_params: Set_GeoFence_Task_Params
    set_rally_pt_task_params: Set_Rally_Pt_Task_Params
    task_type: TaskType
    ts_ms: int
    def __init__(self, creator: _Optional[str] = ..., applies_to: _Optional[_Iterable[str]] = ..., ts_ms: _Optional[int] = ..., task_type: _Optional[_Union[TaskType, str]] = ..., goto_task_params: _Optional[_Union[Goto_Task_Params, _Mapping]] = ..., patrol_task_params: _Optional[_Union[Patrol_Task_Params, _Mapping]] = ..., pulsate_task_params: _Optional[_Union[Pulsate_Task_Params, _Mapping]] = ..., set_rally_pt_task_params: _Optional[_Union[Set_Rally_Pt_Task_Params, _Mapping]] = ..., set_geo_fence_task_params: _Optional[_Union[Set_GeoFence_Task_Params, _Mapping]] = ..., leader_follow_task_params: _Optional[_Union[Leader_Follow_Task_Params, _Mapping]] = ..., formation_task_params: _Optional[_Union[Formation_Task_Params, _Mapping]] = ..., distribute_task_params: _Optional[_Union[Distribute_Task_Params, _Mapping]] = ..., remote_steer_task_params: _Optional[_Union[Remote_Steer_Task_Params, _Mapping]] = ..., return_task_params: _Optional[_Union[Return_Task_Params, _Mapping]] = ...) -> None: ...

class TaskState(_message.Message):
    __slots__ = ["task_status", "task_type"]
    TASK_STATUS_FIELD_NUMBER: _ClassVar[int]
    TASK_TYPE_FIELD_NUMBER: _ClassVar[int]
    task_status: TaskStatus
    task_type: TaskType
    def __init__(self, task_status: _Optional[_Union[TaskStatus, str]] = ..., task_type: _Optional[_Union[TaskType, str]] = ...) -> None: ...

class TeleopCommand(_message.Message):
    __slots__ = ["axis_states", "button_states", "server_ts_us", "teleop_src"]
    AXIS_STATES_FIELD_NUMBER: _ClassVar[int]
    BUTTON_STATES_FIELD_NUMBER: _ClassVar[int]
    SERVER_TS_US_FIELD_NUMBER: _ClassVar[int]
    TELEOP_SRC_FIELD_NUMBER: _ClassVar[int]
    axis_states: _containers.RepeatedScalarFieldContainer[int]
    button_states: _containers.RepeatedScalarFieldContainer[bool]
    server_ts_us: int
    teleop_src: str
    def __init__(self, server_ts_us: _Optional[int] = ..., teleop_src: _Optional[str] = ..., button_states: _Optional[_Iterable[bool]] = ..., axis_states: _Optional[_Iterable[int]] = ...) -> None: ...

class Vec3(_message.Message):
    __slots__ = ["down", "east", "north"]
    DOWN_FIELD_NUMBER: _ClassVar[int]
    EAST_FIELD_NUMBER: _ClassVar[int]
    NORTH_FIELD_NUMBER: _ClassVar[int]
    down: float
    east: float
    north: float
    def __init__(self, north: _Optional[float] = ..., east: _Optional[float] = ..., down: _Optional[float] = ...) -> None: ...

class VehicleState(_message.Message):
    __slots__ = ["ap_mode", "arm_state", "battery_voltage", "flight_time_s", "has_geo_fence", "hdop", "num_of_sat"]
    AP_MODE_FIELD_NUMBER: _ClassVar[int]
    ARM_STATE_FIELD_NUMBER: _ClassVar[int]
    BATTERY_VOLTAGE_FIELD_NUMBER: _ClassVar[int]
    FLIGHT_TIME_S_FIELD_NUMBER: _ClassVar[int]
    HAS_GEO_FENCE_FIELD_NUMBER: _ClassVar[int]
    HDOP_FIELD_NUMBER: _ClassVar[int]
    NUM_OF_SAT_FIELD_NUMBER: _ClassVar[int]
    ap_mode: AutopilotMode
    arm_state: ArmState
    battery_voltage: float
    flight_time_s: int
    has_geo_fence: bool
    hdop: float
    num_of_sat: int
    def __init__(self, arm_state: _Optional[_Union[ArmState, str]] = ..., flight_time_s: _Optional[int] = ..., hdop: _Optional[float] = ..., battery_voltage: _Optional[float] = ..., num_of_sat: _Optional[int] = ..., ap_mode: _Optional[_Union[AutopilotMode, str]] = ..., has_geo_fence: bool = ...) -> None: ...

class RemoteSteerMode(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []

class FormationType(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []

class ControlPointType(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []

class BodyRelativeCmd(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []

class DistributeMode(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []

class TaskType(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []

class ControlPointAction(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []

class AutopilotMode(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []

class TaskStatus(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []

class ArmState(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []

class PlatformType(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []
