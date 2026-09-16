from abc import ABC, abstractmethod
from typing import Protocol, runtime_checkable

import numpy as np
from .results import ExpertPlanResult, PlannerError


@runtime_checkable
class PlannerProtocol(Protocol):
    """Structural contract for planners used by simulation and data pipelines."""

    def reset(self) -> None:
        """Clear any episode-specific internal state."""
        ...

    def __call__(self, obs: np.ndarray) -> np.ndarray:
        """Compute and return the action for the provided observation."""
        ...


class Planner(ABC):
    """
    Abstract base class for all motion planners.
    """

    @abstractmethod
    def reset(self) -> None:
        """
        Clear internal state (e.g., cached trajectories) for a new episode.
        Stateless planners (like pure MPC) can implement this as a pass/no-op.
        """
        pass

    @abstractmethod
    def __call__(self, obs: np.ndarray) -> np.ndarray:
        """
        Compute and return the next action based on the current observation.
        """
        pass


    def plan_episode(
        self,
        initial_joint_state,
        goals,
        environment,
        *,
        time_limit_s: float,
    ) -> ExpertPlanResult:
        """从一个联合状态（多个机器人的位置）开始规划完整轨迹"""
        raise PlannerError(
            f"{type(self).__name__} does not implement plan_episode"
        )


    def query_action(
        self,
        current_joint_state,
        goals,
        environment,
        *,
        horizon: int,
        time_limit_s: float,
    ) -> ExpertPlanResult:
        """从当前联合状态查询指定控制步数的expert action sequence"""
        raise PlannerError(
            f"{type(self).__name__} does not implement query_action"
        )
