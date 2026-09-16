import unittest

import numpy as np

from planning.planner import Planner, PlannerProtocol
from planning.results import PlannerError


class LegacyPlanner(Planner):
    """只实现旧接口的最小 planner。"""

    def reset(self):
        pass

    def __call__(self, obs):
        return np.zeros(2)


class PlannerCompatibilityTests(unittest.TestCase):
    def test_legacy_interface_still_works(self):
        planner = LegacyPlanner()
        planner.reset()

        self.assertIsInstance(planner, PlannerProtocol)
        np.testing.assert_array_equal(
            planner(np.zeros(2)), np.zeros(2)
        )

    def test_episode_interface_reports_unsupported(self):
        planner = LegacyPlanner()

        with self.assertRaisesRegex(PlannerError, "plan_episode"):
            planner.plan_episode(
                np.zeros((1, 2)),
                np.ones((1, 2)),
                None,
                time_limit_s=1.0,
            )

    def test_query_interface_reports_unsupported(self):
        planner = LegacyPlanner()

        with self.assertRaisesRegex(PlannerError, "query_action"):
            planner.query_action(
                np.zeros((1, 2)),
                np.ones((1, 2)),
                None,
                horizon=5,
                time_limit_s=1.0,
            )


if __name__ == "__main__":
    unittest.main()