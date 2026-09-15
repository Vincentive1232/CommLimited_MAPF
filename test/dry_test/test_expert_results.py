import unittest

import numpy as np

from planning.results import ExpertPlanResult, PlannerError


class ExpertResultTests(unittest.TestCase):
    def make_success(self, **changes):
        fields = dict(
            success=True,
            states=(
                np.array([[0., 0.], [0.1, 0.]]),
                np.array([[2., 0.], [1.9, 0.]]),
            ),
            actions=(
                np.array([[1., 0.]]),
                np.array([[-1., 0.]]),
            ),
            dt=0.1,
            robot_order=("robot_0", "robot_1"),
            valid_lengths=(1, 1),
            wall_time_s=0.2,
            termination_reason="solved",
        )
        fields.update(changes)
        return ExpertPlanResult(**fields)

    def test_stack_preserves_robot_order(self):
        result = self.make_success()
        states, actions = result.stack_homogeneous()

        self.assertEqual(states.shape, (2, 2, 2))
        self.assertEqual(actions.shape, (1, 2, 2))

        for index in range(2):
            np.testing.assert_array_equal(
                states[:, index], result.states[index]
            )
            np.testing.assert_array_equal(
                actions[:, index], result.actions[index]
            )

    def test_failure_has_no_actions_and_cannot_stack(self):
        result = ExpertPlanResult(
            success=False,
            states=(),
            actions=(),
            dt=0.1,
            robot_order=("robot_0", "robot_1"),
            valid_lengths=(),
            wall_time_s=30.0,
            termination_reason="timeout",
        )

        self.assertEqual(result.actions, ())
        with self.assertRaises(PlannerError):
            result.stack_homogeneous()

    def test_stack_rejects_different_state_dimensions(self):
        result = self.make_success(
            states=(np.zeros((2, 2)), np.zeros((2, 3))),
        )

        with self.assertRaises(PlannerError):
            result.stack_homogeneous()


    def test_rejects_invalid_common_fields(self):
        cases = (
            {"dt": 0.0},
            {"dt": float("nan")},
            {"wall_time_s": -1.0},
            {"robot_order": ("robot_0", "robot_0")},
            {"termination_reason": ""},
            {"success": 1},
        )

        for changes in cases:
            with self.subTest(changes=changes):
                with self.assertRaises(PlannerError):
                    self.make_success(**changes)

    def test_rejects_invalid_trajectories(self):
        cases = (
            # 缺少一个机器人的动作序列。
            {"actions": (np.zeros((1, 2)),)},
            # 状态数量必须比动作数量多一。
            {"states": (np.zeros((1, 2)), np.zeros((2, 2)))},
            # 动作必须是二维数组。
            {"actions": (np.zeros(2), np.zeros((1, 2)))},
            # 原始控制不能包含非有限值。
            {"actions": (
                np.array([[float("nan"), 0.]]),
                np.zeros((1, 2)),
            )},
            # 有效控制步数不能超过实际长度。
            {"valid_lengths": (2, 1)},
            # bool 不能作为有效长度。
            {"valid_lengths": (True, 1)},
        )

        for changes in cases:
            with self.subTest(changes=changes):
                with self.assertRaises(PlannerError):
                    self.make_success(**changes)


if __name__ == "__main__":
    unittest.main()