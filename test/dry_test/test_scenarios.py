import tempfile
import unittest
from pathlib import Path

import yaml

from scenarios.io import load_scenario


TEMPLATE = (
    Path(__file__).resolve().parents[2]
    / "configs/scenarios/templates/empty_swap__n02__seed000000.yaml"
)


class ScenarioTests(unittest.TestCase):
    def test_load_fixed_swap(self):
        scenario = load_scenario(TEMPLATE)

        self.assertEqual(scenario.robot_order, ("robot_0", "robot_1"))
        self.assertEqual(scenario.environment.lower, (0.0, 0.0))
        self.assertEqual(scenario.environment.upper, (8.0, 8.0))
        self.assertEqual(scenario.environment.obstacles, ())
        self.assertEqual(scenario.robots[0].geometry.radius, 0.30)
        self.assertEqual(
            scenario.robots[0].start, scenario.robots[1].goal
        )
        self.assertEqual(
            scenario.robots[1].start, scenario.robots[0].goal
        )

    def assert_invalid(self, mutate, expected_message):
        data = yaml.safe_load(TEMPLATE.read_text(encoding="utf-8"))
        mutate(data)

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / TEMPLATE.name
            path.write_text(
                yaml.safe_dump(data, sort_keys=False),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, expected_message):
                load_scenario(path)

    def test_duplicate_robot_id(self):
        self.assert_invalid(
            lambda data: data["robots"][1].update(robot_id="robot_0"),
            "duplicate robot_id",
        )

    def test_nonpositive_radius(self):
        self.assert_invalid(
            lambda data: data["geometries"]["disk_r030"].update(radius=0),
            "radius must be positive",
        )

    def test_start_outside_map(self):
        self.assert_invalid(
            lambda data: data["robots"][0].update(start=[-1.0, 4.0]),
            "outside map bounds",
        )

    def test_nonfinite_coordinate(self):
        self.assert_invalid(
            lambda data: data["robots"][0].update(
                start=[float("nan"), 4.0]
            ),
            "must be finite",
        )

    def test_wrong_coordinate_shape(self):
        self.assert_invalid(
            lambda data: data["robots"][0].update(start=[2.0]),
            "exactly two numbers",
        )

    def test_unknown_geometry(self):
        self.assert_invalid(
            lambda data: data["robots"][0].update(geometry="missing"),
            "unknown geometry",
        )

    def test_invalid_map_bounds(self):
        self.assert_invalid(
            lambda data: data["environment"].update(max=[0.0, 8.0]),
            "positive width",
        )

    def test_scenario_id_must_match_filename(self):
        self.assert_invalid(
            lambda data: data.update(scenario_id="wrong_name"),
            "filename stem",
        )

if __name__ == "__main__":
    unittest.main()