from math import isfinite
from pathlib import Path

import yaml

from .specs import (
    EnvironmentSpec,
    RectangleSpec,
    RobotGeometrySpec,
    RobotSpec,
    ScenarioSpec,
)


# region Check Tools
################################################################################
def _mapping(value, name):
    """ Require a YAML mapping """
    if not isinstance(value, dict):
        raise ValueError(f"{name} must be a mapping")
    return value


def _required(value, fields, name):
    """Check required fields at the loading boundary."""
    value = _mapping(value, name)
    missing = set(fields) - value.keys()
    if missing:
        raise ValueError(f"{name}: missing fields {sorted(missing)}")
    return value


def _text(value, name):
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a non-empty string")
    return value


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be numeric")

    result = float(value)
    if not isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _vec2(value, name):
    if not isinstance(value, list) or len(value) != 2:
        raise ValueError(f"{name} must contain exactly two numbers")
    return tuple(_number(item, name) for item in value)


def _inside(point, lower, upper):
    return all(
        lo <= coordinate <= hi
        for coordinate, lo, hi in zip(point, lower, upper)
    )
################################################################################
# endregion


# region Load environment specification from yaml
################################################################################
def _load_environment(value):
    value = _required(
        value,
        {"environment_id", "min", "max", "obstacles"},
        "environment",
    )

    environment_id = _text(
        value["environment_id"], "environment.environment_id"
    )
    lower = _vec2(value["min"], "environment.min")
    upper = _vec2(value["max"], "environment.max")

    if any(lo >= hi for lo, hi in zip(lower, upper)):
        raise ValueError("environment bounds must have positive width")

    if not isinstance(value["obstacles"], list):
        raise ValueError("environment.obstacles must be a list")

    obstacles = []
    for index, item in enumerate(value["obstacles"]):
        name = f"environment.obstacles[{index}]"
        item = _required(item, {"center", "half_extents"}, name)

        center = _vec2(item["center"], f"{name}.center")
        half_extents = _vec2(item["half_extents"], f"{name}.half_extents")

        if any(size <= 0 for size in half_extents):
            raise ValueError(f"{name}.half_extents must be positive")

        obstacle_lower = tuple(
            c - h for c, h in zip(center, half_extents)
        )
        obstacle_upper = tuple(
            c + h for c, h in zip(center, half_extents)
        )

        if not (
            _inside(obstacle_lower, lower, upper)
            and _inside(obstacle_upper, lower, upper)
        ):
            raise ValueError(f"{name} extends outside environment bounds")

        obstacles.append(RectangleSpec(center, half_extents))

    return EnvironmentSpec(
        environment_id=environment_id,
        lower=lower,
        upper=upper,
        obstacles=tuple(obstacles)
    )
################################################################################
# endregion


# region Load geometries from yaml
################################################################################
def _load_geometries(value):
    value = _mapping(value, "geometries")
    if not value:
        raise ValueError("geometries should not be empty")

    geometries = {}
    for geometry_id, item in value.items():
        geometry_id = _text(geometry_id, "geometry_id")
        name = f"geometries.{geometry_id}"
        item = _required(item, {"radius"}, name)

        radius = _number(item["radius"], f"{name}.radius")
        if radius <= 0:
            raise ValueError(f"{name}.radius must be positive")

        geometries[geometry_id] = RobotGeometrySpec(
            geometry_id=geometry_id,
            radius=radius,
        )

    return geometries
################################################################################
# endregion


# region Load robot specifications from yaml
################################################################################
def _load_robots(value, geometries, environment):
    if not isinstance(value, list) or not value:
        raise ValueError("robots must be a non-empty list")

    robots = []
    seen_ids = set()

    for index, item in enumerate(value):
        name = f"robots[{index}]"
        item = _required(
            item,
            {"robot_id", "model", "geometry", "start", "goal"},
            name,
        )

        robot_id = _text(item["robot_id"], f"{name}.robot_id")
        if robot_id in seen_ids:
            raise ValueError(f"duplicate robot_id: {robot_id}")
        seen_ids.add(robot_id)

        model = _text(item["model"], f"{name}.model")
        if model != "single_integrator":
            raise ValueError(
                f"{name}: currently only single_integrator is supported"
            )

        geometry_id = _text(item["geometry"], f"{name}.geometry")
        if geometry_id not in geometries:
            raise ValueError(f"{name}: unknown geometry {geometry_id}")

        start = _vec2(item["start"], f"{name}.start")
        goal = _vec2(item["goal"], f"{name}.goal")

        for label, position in (("start", start), ("goal", goal)):
            if not _inside(
                position, environment.lower, environment.upper
            ):
                raise ValueError(f"{name}.{label} is outside map bounds")

        robots.append(
            RobotSpec(
                robot_id=robot_id,
                model=model,
                geometry=geometries[geometry_id],
                start=start,
                goal=goal,
            )
        )

    return tuple(robots)
################################################################################
# endregion


# region scenarios from yaml (main entry for loading scenarios)
################################################################################
def load_scenario(path: str | Path) -> ScenarioSpec:
    path = Path(path)
    with path.open(encoding="utf-8") as stream:
        value = yaml.safe_load(stream)

    value = _required(
        value,
        {
            "schema_version",
            "scenario_id",
            "family",
            "seed",
            "split",
            "environment",
            "geometries",
            "robots",
        },
        "scenario",
    )

    version = value["schema_version"]
    if type(version) is not int or version != 1:
        raise ValueError("schema_version must be integer 1")

    scenario_id = _text(value["scenario_id"], "scenario_id")
    if scenario_id != path.stem:
        raise ValueError("scenario_id must match the YAML filename stem")

    seed = value["seed"]
    if type(seed) is not int or seed < 0:
        raise ValueError("seed must be a non-negative integer")

    split = _text(value["split"], "split")
    if split not in {"diagnostic", "train", "validation", "test", "ood"}:
        raise ValueError(f"unsupported split: {split}")

    environment = _load_environment(value["environment"])
    geometries = _load_geometries(value["geometries"])
    robots = _load_robots(value["robots"], geometries, environment)

    if len({robot.geometry.radius for robot in robots}) != 1:
        raise ValueError("the current stage requires homogeneous geometry")

    return ScenarioSpec(
        schema_version=version,
        scenario_id=scenario_id,
        family=_text(value["family"], "family"),
        seed=seed,
        split=split,
        environment=environment,
        robots=robots,
    )
################################################################################
# endregion