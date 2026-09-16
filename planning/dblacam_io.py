import numpy as np

from .results import PlannerError
from scenarios.specs import EnvironmentSpec


def _read_matrix(trajectory, key, dimension, *, allow_empty=False):
    """读取原始轨迹矩阵，不修复形状，不推算控制"""
    if not isinstance(trajectory, dict):
        raise PlannerError("trajectory must be a mapping")

    if key not in trajectory:
        raise PlannerError(f"trajectory is missing {key}")

    raw = trajectory[key]
    if not isinstance(raw, list):
        raise PlannerError(f"{key} must be a list")

    if not raw:
        if allow_empty:
            return np.empty((0, dimension), dtype=float)
        raise PlannerError(f"{key} must not be empty")

    try:
        array = np.asarray(raw)
    except (TypeError, ValueError) as error:
        raise PlannerError(f"{key} must be a rectangular matrix") from error

    if array.ndim != 2 or array.shape[1] != dimension:
        raise PlannerError(f"{key} must have shape [time, {dimension}]")

    if array.dtype.kind not in "fiu":
        raise PlannerError(f"{key} must contain real numbers")

    if not np.isfinite(array).all():
        raise PlannerError(f"{key} must contain finite values")

    return array.astype(float, copy=True)


def parse_trajectories(result_data, *, state_dims, action_dims):
    """按输出顺序读取原始轨迹，暂不补齐时间长度。"""
    if not isinstance(result_data, dict):
        raise PlannerError("result YAML must be a mapping")

    robot_count = len(state_dims)
    if robot_count == 0 or len(action_dims) != robot_count:
        raise PlannerError("state_dims and action_dims must match robot count")

    for dimension in (*state_dims, *action_dims):
        if type(dimension) is not int or dimension <= 0:
            raise PlannerError("state/action dimensions must be positive integers")

    trajectories = result_data.get("result")
    if not isinstance(trajectories, list) or len(trajectories) != robot_count:
        raise PlannerError(
            f"result must contain exactly {robot_count} robot trajectories"
        )

    states = []
    actions = []

    for index, trajectory in enumerate(trajectories):
        robot_states = _read_matrix(
            trajectory, "states", state_dims[index]
        )

        robot_actions = _read_matrix(
            trajectory, "actions", action_dims[index], allow_empty=True
        )

        if len(robot_states) != len(robot_actions) + 1:
            raise PlannerError(
                f"robot {index}: states must have one more row than actions"
            )

        states.append(robot_states)
        actions.append(robot_actions)

    return tuple(states), tuple(actions)


def environment_to_problem(environment: EnvironmentSpec) -> dict:
    """将场景环境转换为 db-LaCAM 的问题格式 """
    return {
        "min": list(environment.lower),
        "max": list(environment.upper),
        "obstacles": [
            {
                "type": "box",
                "center" : list(obstacle.center),
                "size": [2.0 * half for half in obstacle.half_extents],
            }
            for obstacle in environment.obstacles
        ],
    }