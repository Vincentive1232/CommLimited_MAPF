from dataclasses import dataclass, field
from typing import Any

import numpy as np


class PlannerError(RuntimeError):
    """规划接口或输入约定错误"""


@dataclass(frozen=True)
class ExpertPlanResult:
    success: bool
    states: tuple[np.ndarray, ...]
    actions: tuple[np.ndarray, ...]
    dt: float
    robot_order: tuple[str, ...]
    valid_lengths: tuple[int, ...]
    wall_time_s: float
    termination_reason: str
    metadata: dict[str, Any] = field(default_factory=dict)


    def __post_init__(self):
        if not np.isfinite(self.dt) or self.dt <= 0:
            raise PlannerError("dt must be finite and positive")

        if not np.isfinite(self.wall_time_s) or self.wall_time_s < 0:
            raise PlannerError("wall_time_s must be finite and non-negative")

        if not self.robot_order:
            raise PlannerError("robot_order must not be empty")

        if any(
            not isinstance(robot_id, str) or not robot_id.strip()
            for robot_id in self.robot_order
        ):
            raise PlannerError("robot IDs must be non-empty strings")

        if len(set(self.robot_order)) != len(self.robot_order):
            raise PlannerError("robot_order must contain unique IDs")

        if (
            not isinstance(self.termination_reason, str)
            or not self.termination_reason.strip()
        ):
            raise PlannerError("termination_reason must be non-empty")


        if type(self.success) is not bool:
            raise PlannerError("success must be a bool")

        if not self.success:
            if any(
                len(items) != 0
                for items in (self.states, self.actions, self.valid_lengths)
            ):
                raise PlannerError(
                    "failed results must not contain training trajectories"
                )
            return

        robot_count = len(self.robot_order)
        if not (
            len(self.states)
            == len(self.actions)
            == len(self.valid_lengths)
            == robot_count
        ):
            raise PlannerError(
                "successful results need states, actions and valid lengths for every robot"
            )


        for index, (states, actions, valid_lengths) in enumerate(
            zip(self.states, self.actions, self.valid_lengths)
        ):
            for name, array in (("states", states), ("actions", actions)):
                if not isinstance(array, np.ndarray):
                    raise PlannerError(f"robot {index}: {name} must be ndarray")

                if array.ndim != 2 or array.shape[1] == 0:
                    raise PlannerError(f"robot {index}: {name} must have shape [time, dimension]")

                if array.dtype.kind not in "fiu":
                    raise PlannerError(f"robot {index}: {name} must be real numeric")

                if not np.isfinite(array).all():
                    raise PlannerError(f"robot {index}: {name} must be finite")

            if len(states) != len(actions) + 1:
                raise PlannerError(f"robot {index}: states must have one more row than actions")

            if (
                isinstance(valid_lengths, (bool, np.bool_))
                or not isinstance(valid_lengths, (int, np.integer))
                or not 0 <= valid_lengths <= len(actions)
            ):
                raise PlannerError(f"robot {index}: invalid valid_length")


    def stack_homogeneous(self) -> tuple[np.ndarray, np.ndarray]:
        # 把逐机器人数组转换为联合数组
        # 返回形状为 states: [T+1, N, nx], actions: [T, N, nu]
        if not self.success:
            raise PlannerError("cannot stack a failed result")

        self.__post_init__()

        if len({array.shape for array in self.states}) != 1:
            raise PlannerError("states must share the same time length and state dimension")

        if len({array.shape for array in self.actions}) != 1:
            raise PlannerError("actions must share the same time length and action dimension")

        return (
            np.stack(self.states, axis=1),
            np.stack(self.actions, axis=1),
        )