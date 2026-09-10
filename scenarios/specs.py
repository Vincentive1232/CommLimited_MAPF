# 定义一个场景包含什么，目前包含用axis障碍物，包含机器人类，环境类等等

from dataclasses import dataclass


Vec2 = tuple[float, float]


# 定义obstacle的长方形障碍物形状，是axis-aligned的
@dataclass(frozen=True)
class RectangleSpec:
    center: Vec2
    half_extents: Vec2


# 定义机器人几何形状
@dataclass(frozen=True)
class RobotGeometrySpec:
    geometry_id: str
    radius: float


# 定义边界和obstacles
@dataclass(frozen=True)
class EnvironmentSpec:
    environment_id: str
    lower: Vec2
    upper: Vec2
    obstacles: tuple[RectangleSpec, ...]


# 定义单个机器人，包含其模型类型，起始点和终点，以及几何形状
@dataclass(frozen=True)
class RobotSpec:
    robot_id: str
    model: str
    geometry: RobotGeometrySpec
    start: Vec2
    goal: Vec2


# 定义scenarios的汇总类，包含场景标识、来源信息和有序机器人集合
@dataclass(frozen=True)
class ScenarioSpec:
    schema_version: int
    scenario_id: str
    family: str
    seed: int
    split: str
    environment: EnvironmentSpec
    robots: tuple[RobotSpec, ...]

    @property
    def robot_order(self) -> tuple[str, ...]:
        return tuple(robot.robot_id for robot in self.robots)