import unittest

from planning.dblacam_io import parse_trajectories
from planning.results import PlannerError
from planning.dblacam_io import environment_to_problem
from scenarios.specs import EnvironmentSpec, RectangleSpec


class DbLacamParsingTests(unittest.TestCase):
    def parse(self, trajectories):
        return parse_trajectories(
            {"result": trajectories},
            state_dims=(2,),
            action_dims=(2,),
        )

    def test_reads_original_controls(self):
        states, actions = self.parse([
            {
                "states": [[0.0, 0.0], [0.1, 0.0]],
                "actions": [[1.0, 0.0]],
            }
        ])

        self.assertEqual(states[0].shape, (2, 2))
        self.assertEqual(actions[0].tolist(), [[1.0, 0.0]])

    def test_rejects_missing_actions(self):
        with self.assertRaisesRegex(PlannerError, "missing actions"):
            self.parse([{"states": [[0.0, 0.0]]}])

    def test_rejects_flat_actions(self):
        with self.assertRaisesRegex(PlannerError, "shape"):
            self.parse([
                {
                    "states": [[0.0, 0.0], [0.1, 0.0]],
                    "actions": [1.0, 0.0],
                }
            ])

    def test_rejects_inconsistent_lengths(self):
        with self.assertRaisesRegex(PlannerError, "one more row"):
            self.parse([
                {
                    "states": [[0.0, 0.0]],
                    "actions": [[1.0, 0.0]],
                }
            ])

    def test_rejects_wrong_robot_count(self):
        with self.assertRaisesRegex(PlannerError, "exactly 1"):
            self.parse([])

    def test_environment_preserves_raw_geometry(self):
        environment = EnvironmentSpec(
            environment_id="test",
            lower=(0.0, 0.0),
            upper=(8.0, 8.0),
            obstacles=(
                RectangleSpec((4.0, 4.0), (0.5, 0.25)),
            ),
        )

        problem = environment_to_problem(environment)

        self.assertEqual(problem["min"], [0.0, 0.0])
        self.assertEqual(problem["max"], [8.0, 8.0])
        self.assertEqual(problem["obstacles"], [
            {"type": "box", "center": [4.0, 4.0], "size": [1.0, 0.5]}
        ])
        self.assertEqual(
            environment.obstacles[0].half_extents, (0.5, 0.25)
        )


if __name__ == "__main__":
    unittest.main()