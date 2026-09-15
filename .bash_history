pwd
python -c 'import sys; print(sys.executable)'
test -x /opt/db-lacam/buildRelease/run_dblacam && echo "db-LaCAM ready"
python - <<'PY'
from pathlib import Path
import tempfile
import torch

assert torch.cuda.is_available(), "CUDA unavailable"
print("GPU:", torch.cuda.get_device_name(0))

for name in ("data", "outputs/checkpoints", "outputs/logs"):
    directory = Path("/workspace") / name
    directory.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryFile(dir=directory) as file:
        file.write(b"write-test")
    print("Writable:", directory)

print("Development container OK")
PY

pwd
ls -l /workspace/PLAN_ch.md /workspace/scenarios/specs.py
cd /workspace
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 python -m pytest -q   test/test_simulator_contracts.py   test/test_collision_checker.py   test/test_deep_set_encoder.py   test/dry_test/test_scenarios.py
git add docker/Dockerfile.research notes/docker_info.md
git diff --cached --check
git diff --cached --stat
git status --short
git commit -m "build: add validated Podman research environment"
git status --short
exit
python - <<'PY'
from planning.results import ExpertPlanResult

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

assert not result.success
assert result.actions == ()
print("Failure result OK")
PY

python - <<'PY'
from planning.results import ExpertPlanResult, PlannerError

fields = dict(
    success=False,
    states=(),
    actions=(),
    dt=0.1,
    robot_order=("robot_0", "robot_1"),
    valid_lengths=(),
    wall_time_s=30.0,
    termination_reason="timeout",
)

ExpertPlanResult(**fields)

for change in (
    {"dt": 0.0},
    {"dt": float("nan")},
    {"wall_time_s": -1.0},
    {"robot_order": ("robot_0", "robot_0")},
    {"termination_reason": ""},
):
    try:
        ExpertPlanResult(**(fields | change))
    except PlannerError:
        pass
    else:
        raise AssertionError(f"Invalid input accepted: {change}")

print("Common field checks OK")
PY

python - <<'PY'
from planning.results import ExpertPlanResult, PlannerError

fields = dict(
    success=False,
    states=(),
    actions=(),
    dt=0.1,
    robot_order=("robot_0", "robot_1"),
    valid_lengths=(),
    wall_time_s=30.0,
    termination_reason="timeout",
)

ExpertPlanResult(**fields)

for change in (
    {"dt": 0.0},
    {"dt": float("nan")},
    {"wall_time_s": -1.0},
    {"robot_order": ("robot_0", "robot_0")},
    {"termination_reason": ""},
):
    try:
        ExpertPlanResult(**(fields | change))
    except PlannerError:
        pass
    else:
        raise AssertionError(f"Invalid input accepted: {change}")

print("Common field checks OK")
PY

python - <<'PY'
from planning.results import ExpertPlanResult, PlannerError

fields = dict(
    success=False,
    states=(),
    actions=(),
    dt=0.1,
    robot_order=("robot_0", "robot_1"),
    valid_lengths=(),
    wall_time_s=30.0,
    termination_reason="timeout",
)

ExpertPlanResult(**fields)

for change in (
    {"dt": 0.0},
    {"dt": float("nan")},
    {"wall_time_s": -1.0},
    {"robot_order": ("robot_0", "robot_0")},
    {"termination_reason": ""},
):
    try:
        ExpertPlanResult(**(fields | change))
    except PlannerError:
        pass
    else:
        raise AssertionError(f"Invalid input accepted: {change}")

print("Common field checks OK")
PY

python - <<'PY'
from planning.results import ExpertPlanResult, PlannerError

fields = dict(
    success=False,
    states=(),
    actions=(),
    dt=0.1,
    robot_order=("robot_0", "robot_1"),
    valid_lengths=(),
    wall_time_s=1.0,
    termination_reason="timeout",
)

ExpertPlanResult(**fields)

for change in (
    {"success": 1},
    {"success": True},
    {"valid_lengths": (0, 0)},
):
    try:
        ExpertPlanResult(**(fields | change))
    except PlannerError:
        pass
    else:
        raise AssertionError(f"Invalid result accepted: {change}")

print("Result count checks OK")
PY

python - <<'PY'
import numpy as np
from planning.results import ExpertPlanResult, PlannerError

fields = dict(
    success=True,
    states=(np.array([[0., 0.], [0.1, 0.]]),),
    actions=(np.array([[1., 0.]]),),
    dt=0.1,
    robot_order=("robot_0",),
    valid_lengths=(1,),
    wall_time_s=0.2,
    termination_reason="solved",
)

ExpertPlanResult(**fields)

for change in (
    {"states": (np.zeros((1, 2)),)},
    {"actions": (np.zeros(2),)},
    {"actions": (np.array([[float("nan"), 0.]]),)},
    {"valid_lengths": (2,)},
    {"valid_lengths": (True,)},
):
    try:
        ExpertPlanResult(**(fields | change))
    except PlannerError:
        pass
    else:
        raise AssertionError(f"Invalid trajectory accepted: {change}")

print("Trajectory shape checks OK")
PY

python - <<'PY'
import numpy as np
from planning.results import ExpertPlanResult, PlannerError

fields = dict(
    success=True,
    states=(np.array([[0., 0.], [0.1, 0.]]),),
    actions=(np.array([[1., 0.]]),),
    dt=0.1,
    robot_order=("robot_0",),
    valid_lengths=(1,),
    wall_time_s=0.2,
    termination_reason="solved",
)

ExpertPlanResult(**fields)

for change in (
    {"states": (np.zeros((1, 2)),)},
    {"actions": (np.zeros(2),)},
    {"actions": (np.array([[float("nan"), 0.]]),)},
    {"valid_lengths": (2,)},
    {"valid_lengths": (True,)},
):
    try:
        ExpertPlanResult(**(fields | change))
    except PlannerError:
        pass
    else:
        raise AssertionError(f"Invalid trajectory accepted: {change}")

print("Trajectory shape checks OK")
PY

python - <<'PY'
import numpy as np
from planning.results import ExpertPlanResult, PlannerError

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

result = ExpertPlanResult(**fields)
states, actions = result.stack_homogeneous()

assert states.shape == (2, 2, 2)
assert actions.shape == (1, 2, 2)
np.testing.assert_array_equal(states[:, 1], result.states[1])
np.testing.assert_array_equal(actions[:, 0], result.actions[0])

unequal = ExpertPlanResult(**(
    fields | {"states": (fields["states"][0], np.zeros((2, 3)))}
))
try:
    unequal.stack_homogeneous()
except PlannerError:
    pass
else:
    raise AssertionError("Different state dimensions accepted")

print("Homogeneous stacking OK")
PY

python - <<'PY'
import numpy as np
from planning.results import ExpertPlanResult, PlannerError

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

result = ExpertPlanResult(**fields)
states, actions = result.stack_homogeneous()

assert states.shape == (2, 2, 2)
assert actions.shape == (1, 2, 2)
np.testing.assert_array_equal(states[:, 1], result.states[1])
np.testing.assert_array_equal(actions[:, 0], result.actions[0])

unequal = ExpertPlanResult(**(
    fields | {"states": (fields["states"][0], np.zeros((2, 3)))}
))
try:
    unequal.stack_homogeneous()
except PlannerError:
    pass
else:
    raise AssertionError("Different state dimensions accepted")

print("Homogeneous stacking OK")
PY

python -m pytest -q test/dry_test/test_expert_results.py
python -m pytest -q test/dry_test/test_expert_results.py
python -m pytest -q test/dry_test/test_expert_results.py
exit
