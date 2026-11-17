"""Explicit SI-unit contracts for the reconstructed reference implementation."""
from dataclasses import dataclass, field, asdict
import math

@dataclass(frozen=True)
class State:
    x: float = 0.0
    y: float = 0.0
    h: float = 20.0
    V: float = 15.5
    gamma: float = 0.0
    psi: float = 0.0
    phi: float = 0.0
    alpha: float = 0.0
    T: float = 0.0
    yaw_rate: float = 0.0

    @property
    def position(self):
        return (self.x, self.y, self.h)

@dataclass(frozen=True)
class Control:
    phi: float
    alpha: float
    T: float
    duration: float = 3.0

@dataclass(frozen=True)
class Config:
    H: float = 3.0
    B: float = 3.0
    dt: float = 0.02
    execution_dt: float = 0.005
    K: int = 12
    min_speed: float = 3.0
    max_speed: float = 45.0
    min_altitude: float = 5.0
    max_altitude: float = 45.0
    max_roll: float = math.radians(35)
    max_gamma: float = math.radians(20)
    min_alpha: float = math.radians(-20)
    max_alpha: float = math.radians(25)
    max_thrust: float = 20.0
    cruise_speed: float = 15.5
    headings: int = 29
    vertical_levels: int = 7
    heading_span: float = math.radians(45)
    compensate_tau: float = 0.8
    margin: float = 2.0
    reserve: float = 1.0
    vertical_clearance: float = 1.0
    voxel_size: float = 2.0
    coverage_weight: float = 0.75
    tie_tolerance: float = 1e-7
    max_state_age: float = 0.5
    max_map_age: float = 1.0
    max_validation_age: float = 0.02

    def __post_init__(self):
        values = asdict(self)
        if not all(math.isfinite(v) for v in values.values()):
            raise ValueError("Nonfinite configuration")
        if min(self.H,self.B,self.dt,self.execution_dt,self.voxel_size) <= 0:
            raise ValueError("Durations, integration steps and voxel size must be positive")
        if any(type(v) is not int for v in (self.K,self.headings,self.vertical_levels)):
            raise ValueError("Library and candidate budgets must be integers")
        if self.K < 0 or self.headings < 1 or self.vertical_levels < 1 or self.vertical_levels % 2 == 0:
            raise ValueError("Invalid library or query count")
        if self.min_altitude >= self.max_altitude or self.min_speed <= 0 or self.min_speed >= self.max_speed:
            raise ValueError("Invalid flight limits")
        if not 0 <= self.coverage_weight <= 1 or min(self.margin,self.reserve,self.tie_tolerance) < 0:
            raise ValueError("Invalid margin or priority parameters")
        if not (0 <= self.max_roll < math.pi/2 and 0 <= self.max_gamma < math.pi/2):
            raise ValueError("Roll and flight-path limits must lie in [0,pi/2)")
        if self.min_alpha>self.max_alpha or self.max_thrust<0 or self.cruise_speed<=0:
            raise ValueError("Invalid command limits")
        if min(self.max_state_age,self.max_map_age,self.max_validation_age,self.vertical_clearance)<0:
            raise ValueError("Freshness and clearance limits must be nonnegative")

@dataclass
class Rollout:
    states: list
    times: list
    valid: bool
    reason: str = "accepted"
    steps: int = 0

@dataclass
class Candidate:
    index: int
    control: Control
    rollout: Rollout
    cost: float
    rank: int = 0

    @property
    def terminal(self):
        return self.rollout.states[-1]

    @property
    def key(self):
        return (self.cost, self.rank)

@dataclass
class Witness:
    candidate: Candidate
    control: Control
    rollout: Rollout
    successor_index: int

@dataclass
class SearchResult:
    status: str
    witness: object = None
    candidates: list = field(default_factory=list)
    schedule: list = field(default_factory=list)
    queries: list = field(default_factory=list)
    steps: int = 0
    primary_steps: int = 0
    inference_calls: int = 0
    map_version: int = 0
    recorded_scores: dict = field(default_factory=dict)
