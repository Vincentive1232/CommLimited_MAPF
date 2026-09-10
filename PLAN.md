# Global-to-Distributed Imitation Learning Plan

## 1. Goal and research questions

Train a shared decentralized policy from centralized db-LaCAM demonstrations for continuous-space multi-robot coordination in logistics environments.

For robot $i$ at time $t$:

$$
(a_i^t,\{z_{ij}^t\})=\pi_\theta(o_i^t,\{m_{ji}^t\},\{q_{ij}^t\}),
$$

where $o_i^t$ is local observation, $z_{ij}^t$ is a receiver-conditioned outgoing message, $m_{ji}^t$ is a delivered message, and $q_{ij}^t$ is locally available link information. Messages have no direct labels: expert action loss backpropagates through receiver, differentiable channel/bottleneck and sender.

Main questions:

1. Can MLP and Flow policies imitate a centralized kinodynamic expert using only decentralized observations?
2. Does communication improve success, safety and coordination over the same-backbone no-communication policy?
3. How much message representation and network bandwidth are needed to retain that improvement?
4. How robust is the method to larger fleets, denser/unseen maps, delay, dropout and asynchronous delivery?
5. Does DAgger improve closed-loop performance over offline BC?
6. Optionally, how much does a CBF/safety filter improve deployment safety, and what policy failures does it mask?

## 2. Scope and stage boundaries

- Simulation, homogeneous fleets, 2D continuous space, static obstacles and shared policy parameters.
- Centralized training with db-LaCAM expert access; decentralized execution with local observation, local link information and received P2P messages.
- CasADi remains a regression/reference planner; db-LaCAM is the primary expert because it is faster and scales better, despite jerkier trajectories.
- GLAS reproduction is deferred until the core proof of concept works. The same-backbone communication-disabled model is the primary early baseline.
- A safety filter and physical robots are optional late stages, not prerequisites for learning the policy.

Dynamics stages are serial:

1. `single_integrator`: complete the whole pipeline, including communication and evaluation.
2. `double_integrator`: begin only after Stage 1 works, using a matching Dynobench model and motion primitives.
3. RoboChief: three omniwheels at $120^\circ$, candidate state $[x,y,\theta,v_x,v_y,\omega]$ and control $[a_x,a_y,\alpha]$; finalize from measured platform parameters.

Do not reuse single-integrator primitives for later dynamics. Each new model must first pass model, primitive, timestep, bounds and replay tests.

Planning principle: freeze current-stage interfaces and acceptance gates; keep distant protocol, quantizer and scheduler details provisional until measurements justify them.

### 2.1 Current codebase status

Already present: continuous dynamics, homogeneous multi-robot simulator, CasADi expert, partial db-LaCAM wrapper, ego-relative observation/visibility masks, DeepSet/Transformer encoders, MLP/Flow policies, BC/DAgger, checkpoints, evaluation and trajectory visualization.

Immediate gaps:

- `Planner` currently exposes only `reset()` and `__call__(obs)`; db-LaCAM adapter mixes YAML/subprocess/cache/failure behavior.
- DAgger/evaluator contain CasADi-specific selection or exception handling, and db-LaCAM failure can fall back to a misleading zero action.
- db-LaCAM wrapper currently supports selected systems, assumes $dt=0.1$ and hard-codes an empty obstacle list.
- No common scenario schema, structured expert result, independent replay validator, communication channel/network model, joint communication loader or bit-level communication accounting exists yet.

## 3. System semantics

### 3.1 Robot, action and timing

- Physical RoboChief diameter: approximately $0.50\,\mathrm{m}$.
- Effective collision radius: $r=0.30\,\mathrm{m}$.
- Robot–robot safe center distance: $d_{safe}=0.60\,\mathrm{m}$.
- Provisional near-collision distance: $0.70\,\mathrm{m}$; recalibrate from braking/localization data later.
- Stage 1 action: planar velocity $u_i=[v_x,v_y]$.
- Control/planner period: $dt=0.1\,\mathrm{s}$ (10 Hz).
- Default speed bound: $v_{max}=1.0\,\mathrm{m/s}$; later sensitivity at $0.5$ and $1.5\,\mathrm{m/s}$.
- Stage 2 action is acceleration. Primitive output is only a distant optional policy representation.

### 3.2 Goal and episode termination

- Strict goal tolerance: $\|p_i-p_i^{goal}\|_2\leq0.05\,\mathrm{m}$ for 3 consecutive control steps.
- After confirmation, a robot enters absorbing `goal_reached`: fixed position and forced zero action, but it remains visible, communicable and collision-active.
- Joint success requires every robot to reach the absorbing state. Single integrator has no terminal-speed test; second-order models must add one.
- Default controlled rollout horizon: 30 s / 300 steps. Predeclared corridor/warehouse scenarios may use 60 s. Planner wall time and simulated horizon are separate fields.
- If db-LaCAM cannot use the strict threshold internally, report both planner-native success and the common strict evaluator result.

### 3.3 Collision semantics

- Robot–obstacle checking uses either a radius-$0.30$ m disk against raw obstacles or a point against obstacles inflated once by $0.30$ m—never both.
- `CollisionChecker` is an abstract interface:
  - `DynobenchCollisionChecker`: planner-consistent geometry/FCL path.
  - `AnalyticDiskCollisionChecker`: independent Stage 1 disk–disk and disk–rectangle implementation.
- Check swept motion over the entire control step, not endpoints only. For Stage 1, use continuous-time closest approach for moving disks.
- The two implementations share geometry configuration but not checking code; random segments and boundary cases cross-check them.
- If a candidate action collides during the step, do not commit the next state. Keep the last safe state, record first-contact diagnostics when available and terminate the joint rollout as collision failure.

### 3.4 Local observation

The first policy input contains ego/goal-relative state, visible-neighbor set and visible-obstacle set.

- No communication: directly sense only neighbor relative position; do not expose neighbor velocity, goal or intention.
- All robots may know the same static map/world frame and estimate their own world pose. The static map contains no live fleet positions, goals, expert trajectory or global link matrix.
- Stage 1 does not feed a global map embedding into the policy. `map_context` is a later ablation.
- Robot and obstacle sensing radii default to 2.0 m; evaluate 1.5/2.0/3.0 m and report visible-neighbor/obstacle statistics and fraction of steps where the full fleet is visible.
- Neural inputs contain no fixed robot ID. IDs remain internal for routing, alignment, logging and bandwidth accounting.
- Neighbor sets use masked permutation-invariant aggregation.
- Static obstacle element: $[\Delta x,\Delta y,h_x,h_y]$ for an axis-aligned rectangle.
- First version uses `max_visible_obstacles=8`, nearest by boundary distance, with padding mask. Record truncation; revisit if roughly more than 1–5% of frames are truncated. Ablate 4/6/8/12 later.

## 4. Communication method

### 4.1 Information boundary

The simulator may centrally hold the true link matrix, but robot $i$ receives only locally permitted outgoing candidates, $q_{ij}^t$, capacity and its own sending budget. It never receives the whole fleet link matrix. This is centralized simulation of decentralized information.

The shared scheduler runs per sender:

$$
g_{ij}^t=f_{schedule}(o_i^t,q_{ij}^t,B_i^t),\qquad e_{ij}^t=A_{ij}^tg_{ij}^t.
$$

A global scheduler may exist only as an upper-bound baseline.

### 4.2 Position and message protocol

Local-link proof of concept:

- $A^t=S^t$: communication candidates equal visible neighbors.
- Both endpoints obtain relative position from the virtual local sensor; learned payload does not repeat position.
- Internal IDs align messages to neighbor slots but are not neural features.
- Receiver element: $[r_{ji}^t,z_{ij}^t,q_{ij}^t,age_{ij}^t]$.

Remote network-selected P2P stage:

- $A^t$ is independent of sensing; no central fleet-position broadcast.
- Network discovery exposes candidate addresses/link quality. Scheduler chooses links before pose transfer.
- Only selected links exchange addressed/timestamped world pose; pose/header/setup traffic counts toward bandwidth and is cached locally.
- First link establishment takes at least one control step before personalized payload can use the cached geometry.
- Provisional two-gate design: `pose_query_gate` selects unknown/stale peers; `message_gate` sends learned payload only to peers with valid pose cache. Finalize with mentors when this stage begins.

### 4.3 Message and receiver architecture

- Core message is receiver-conditioned:
  $$z_{ij}^t=f_{msg}(o_i^t,r_{ij}^t,q_{ij}^t).$$
- All edges share encoder parameters; no ID embedding.
- First aggregator is DeepSet with masks and a defined zero-message case. Attention/Transformer is a later comparison.
- Phase 1 full private packet for single integrator: sender goal-relative vector, previous executed action and sender local branch nominal action. It excludes expert action, third-party observation and fixed ID.
- Messages are feed-forward and single-round initially; no recurrent state.

### 4.4 Channel and network interfaces

`CommunicationChannel` subclasses share `reset()`, `exchange(...)` and `get_metrics()`:

- `ImmediateCommunicationChannel`: ideal differentiable same-step delivery.
- `BufferedCommunicationChannel`: queue with arrival time for delay/drop/asynchrony.

Do not detach or serialize tensors in the differentiable early channel. Delayed cross-step gradient policy is decided in the asynchronous stage.

`NetworkModel` is separate from the channel and scheduler:

- `IdealNetwork` for perfect links.
- `DistanceBasedNetwork` first constrained model; default $R_{comm}=4$ m versus $R_{sense}=2$ m; ablate 2/4/6/$\infty$ m.
- `ObstacleAwareNetwork`, `StochasticNetwork` and `TraceDrivenNetwork` are later subclasses.

Network model determines physical feasibility/cost; scheduler determines whether a feasible transmission is useful; channel executes delivery and accounting.

### 4.5 Control-step timing

At time $t$:

1. Build all local observations.
2. Generate receiver-conditioned outgoing messages.
3. Execute one channel exchange.
4. Aggregate messages already delivered.
5. Produce and execute $a^t$.
6. Advance simulator to $t+1$.

Phases 1–3 allow a zero-delay message from $o^t$ to affect $a^t$. Delayed phases permit only messages delivered by decision time. Buffered messages carry generation timestamp and expose normalized age. Expiry is configured when that stage begins.

### 4.6 Communication phases

1. **Information value:** full private information, ideal synchronous channel, no capacity/quantization/receiver constraint. Main topology $A=S$; all-to-all is an oracle.
2. **Learned message:** same topology/channel, learned continuous receiver-conditioned message, initially `message_dim=16`. Dimension is representation width, not real bandwidth.
3. **Representation and quantization:** dimensions 1/2/4/8/16; then fixed bits per element, where payload bits/message = `message_dim × bits_per_element`. Candidate ladder: float32 reference, uniform int8, then optional int4. Report payload separately from pose/address/header overhead.
4. **Bandwidth/network constraint:** per-link capacity $C_{ij}^t$, sender budget $B_i^t$, receiver/pose-query selection and complete traffic accounting. Start with soft gates and bandwidth penalty, then test budget-feasible hard top-$K$/binary selection; STE/Gumbel-style estimators remain candidates to decide in this phase.
5. **Network imperfections:** only after Phase 4 works, add delay, dropout and asynchronous delivery. Track attempted, transmitted and delivered bits; dropped transmissions still consume sent bandwidth.

Do not change content and topology simultaneously in a causal comparison.

### 4.7 Learning objective and initialization

Start with action imitation only:

$$
\mathcal L_{action}=\|\hat u-u^E\|^2\quad\text{or Flow loss}.
$$

Later add one term at a time: bandwidth penalty, receiver sparsity, temporal consistency on persistent edges, then optional information/entropy regularization.

Communication policy warm-starts compatible observation/action weights from the no-communication checkpoint. New communication fusion has near-zero initial effect. Then jointly fine-tune sender, receiver and action network; do not permanently freeze the action policy. Train a same-budget scratch variant as initialization ablation.

## 5. Software architecture and db-LaCAM integration

### 5.1 Module ownership

```text
third_party/db-lacam/   upstream recursive Git submodule
scenarios/              specs, YAML I/O, generators
planning/               Planner, ExpertPlanResult, adapters
collision/              CollisionChecker implementations
validation/             trajectory replay validation
learning/data/          per-robot and joint loaders
configs/                scenarios, planners, experiments
docker/                 build/runtime definitions
```

Create modules only when their vertical slice needs them. Avoid catch-all `utils.py` and cross-module directory nesting.

### 5.2 Scenario/config model

- Human-readable YAML is the external format; loading creates lightweight frozen dataclasses.
- `EnvironmentSpec`: bounds, obstacles, environment ID.
- `ScenarioSpec`: scenario ID, environment, ordered robot model/geometry references, starts/goals, family, seed, split and schema version.
- First obstacle type is axis-aligned rectangle with center and half-extents. `RobotGeometrySpec` is separate so inflation is applied exactly once.
- Validate once at load boundaries: required fields, basic shapes, finite values, unique ordering, positive dimensions and bounds. Do not add Pydantic, auto-repair or speculative migration machinery.
- Template YAML lives in `configs/scenarios/templates/`; generated immutable instances live in `data/scenarios/{split}/`.
- Readable filename pattern: `empty_swap__n04__seed000123.yaml`; `scenario_id` matches the stem and collisions are errors.
- Scenario YAML describes the problem; planner YAML describes solver/primitives/budget; experiment YAML references scenario set, planner, dataset, policy and seed. A scenario is reusable across planners unchanged.

### 5.3 Planner contract

Extend the abstract `Planner` while retaining `__call__(obs)` temporarily for old tests:

```python
plan_episode(initial_joint_state, goals, environment, *, time_limit_s)
query_action(current_joint_state, goals, environment, *, horizon, time_limit_s)
```

New dataset/DAgger/validation code uses explicit joint state rather than reconstructing it from decentralized observation.

Both calls return `ExpertPlanResult` with success, per-robot states/actions, dt, robot order, valid lengths/shared time-grid semantics, wall time, termination reason and compact planner metadata.

- Per-robot representation supports future heterogeneous dimensions.
- `stack_homogeneous()` returns `[T+1,N,nx]` and `[T,N,nu]` for learning. Adapter explicitly holds completed robots at goal with zero actions while preserving original valid lengths.
- No-solution, timeout and planner-invalid output are structured failures. Schema/programming errors raise a common planner exception.
- Never return zero action as a fallback expert label.
- `replan_freq` is an external experiment parameter, not hidden inside the wrapper.

### 5.4 db-LaCAM source, container and call chain

Use a recursive Git submodule for source and Docker for dependencies/build/runtime:

```bash
git clone --recurse-submodules <repo>
git submodule update --init --recursive
git submodule status --recursive
```

Pin db-LaCAM to a tested commit. To modify it, use a fork and update the submodule gitlink; do not rely on edits inside a temporary container. Re-run smoke, adapter, replay and collision tests after every dependency update.

Formal runtime is a unified `research-gpu` service built from `docker/Dockerfile.research`, containing CUDA PyTorch/LeRobot, CasADi, OMPL, db-LaCAM, Dynoplan/Dynobench and motion primitives. Keep existing GPU and db-LaCAM Dockerfiles as regression/fallback until unified smoke tests pass.

First adapter remains subprocess/YAML:

```text
generator/DAgger
  -> PlannerFactory
  -> DbLacamPlanner
  -> ScenarioSpec-to-YAML adapter
  -> /opt/db-lacam/buildRelease/run_dblacam
  -> db-LaCAM/Dynoplan/Dynobench/FCL
  -> result/stats parser
  -> ExpertPlanResult
  -> independent TrajectoryValidator
  -> validated dataset
```

Each call gets an isolated temporary directory. Capture problem, algorithm, result, stats, command, return code, stdout/stderr and timing. Use db-LaCAM internal timeout plus a slightly larger subprocess timeout. Map timeout, missing/malformed output, no-solution and invalid replay to distinct reasons. Measure process/YAML overhead; consider bindings only if it is a demonstrated DAgger bottleneck.

Offline runs archive raw files. Routine DAgger keeps compact records and preserves detailed artifacts mainly on failure.

### 5.5 Container acceptance

- `torch.cuda.is_available()` on RTX A4000.
- CasADi smoke solve.
- db-LaCAM official smoke.
- Both planners created through `PlannerFactory`.
- Writable data/checkpoint/log volumes.

Record repository/dependency commits, image tag/digest, primitives checksum, CUDA/PyTorch/driver, hardware, runtime and peak memory in run metadata—not repeatedly in the episode manifest.

Training, DAgger and evaluation write concise terminal summaries plus TensorBoard scalars/curves. At minimum log losses, success/collision/timeout, expert-query latency/failure, sampled robot counts/scenario families, communication usage and validation checkpoints; detailed per-episode diagnostics remain in structured logs rather than TensorBoard text blobs.

## 6. Expert validation and scenario generation

### 6.1 Expert benchmark

Before training, benchmark db-LaCAM independently on fixed seeds and 2/4/8/16 single-integrator robots across random, swap, crossing, corridor and bottleneck cases, including timeout/invalid cases.

- Report success, wall time, trajectory duration, dynamics/collision validity, action bounds, variation/jerk and scaling.
- Produce coverage/time-to-solution at 1/5/30/60 s.
- Initial offline budget: 30 s per instance. Initial DAgger query budget: 20 s.
- A planner-reported success enters training only after independent replay validation.

### 6.2 Trajectory validation

- Preserve db-LaCAM robot ordering and verify state/action lengths, shapes, $dt$ and semantics.
- Use db-LaCAM’s returned controls as expert labels. Replay them through project dynamics and compare next states to the returned state trajectory.
- State-difference controls are diagnostic only; missing/invalid original controls are adapter/schema failure, even for single integrator.
- Check action/state bounds, swept robot/obstacle collisions, goal criteria and hold behavior.
- If planner and independent collision checks disagree, quarantine the result.

### 6.3 Scenario families

- **A — Empty coordination:** pair swap, 3+ crossing, circle swap and random goal permutation.
- **B — Random clutter:** 8 m × 8 m, axis-aligned obstacles, 10%/20% union-area occupancy, inspired by GLAS.
- **C — Topology stress:** alcove/at-goal, corridor, bottleneck, maze, merge, intersection and warehouse aisle passing.
- **D — MovingAI-derived continuous:** convert selected empty/warehouse/room/random/maze grids to continuous rectangles; do not compare numbers directly with discrete MAPF.
- **E — db-LaCAM reference:** official alcove, at-goal, circle, maze and scalability cases for wrapper validation.

Record workspace, free area, occupancy union ratio, corridor/bottleneck width, robot count, starts/goals, horizon and seed. Empty maps isolate coordination from obstacle avoidance.

### 6.4 Generate scenarios before labeling

1. Generate and freeze a scenario bank independent of planner success.
2. Precheck starts/goals against inflated geometry, pairwise safety, topology constraints and individual static reachability.
3. Run db-LaCAM and classify `expert_valid`, `expert_failed`, `expert_invalid_trajectory`; invalid generation is separate.
4. Train only from `expert_valid`, but evaluate/report the frozen bank without filtering failures after solving.

Report expert coverage $N_{expert\_valid}/N_{precheck\_valid}$ and student/expert joint outcomes to expose selection bias. db-LaCAM failure under a budget means only “not solved by this configuration,” not mathematical infeasibility. A small constructed guaranteed-feasible diagnostic set may be used separately.

Keep raw expert trajectories as the primary dataset. If jerk is harmful, create a separately versioned refined dataset and revalidate dynamics, collisions, goals and bounds.

## 7. Dataset design

### 7.1 Storage layers

1. **Raw planner archive:** problem, algorithm, result, stats and validation per `joint_episode_id`.
2. **Minimal joint manifest:** joint/scenario ID, split, robot-to-LeRobot episode mapping, source/status and raw path.
3. **LeRobot datasets:** decentralized per-robot training frames.

Shared schema/config/version/checksum belongs once in dataset metadata. Detailed failure diagnostics belong in `failures.jsonl` or run logs, not the manifest.

Each frame contains local observation, expert local action chunk, visibility/valid masks, joint/episode mapping and timestep. Global state, IDs and joint expert plans are metadata, not policy input.

### 7.2 Splits and lineage

- Split by frozen scenario/joint episode, never by frame or robot: 70% train, 15% validation, 15% test.
- Independent OOD sets: larger fleet, higher density, unseen map, unseen goal pattern and narrower topology.
- Freeze split manifest before training. All robots from a joint rollout stay in one split.
- Raw, refined and DAgger data retain scenario lineage.
- DAgger uses only registered train scenarios/seeds. Validation/test/OOD are read-only; validation selects models, final test does not drive redesign.

### 7.3 Variable fleet size and sampling

- Store separate datasets by robot count: `dataset_n2` … `dataset_n8`.
- A minibatch contains one $N$ and natural `[batch,N,...]` shapes; no global `max_robots` input padding.
- One shared DeepSet/policy checkpoint trains across loaders. Sample robot count first, then scenario family, then minibatch, initially approximately uniformly.
- Decouple encoder parameter shapes from neighbor-slot count. Test zero neighbors, permutation invariance, cross-count checkpoint load and 12/16-robot OOD inference.
- Train/ID counts: 2–8. OOD: 12/16; 24/32 only if resources permit.

No-communication training reads individual robot frames. Communication training uses `JointTimestepDataset`, which reconstructs all robots at the same joint episode/timestep through the manifest. It does not duplicate stored observations. `active_mask` and `goal_reached_mask` represent runtime state, not maximum-fleet padding.

### 7.4 Temporal labels and normalization

- Raw, frames, control and evaluation remain at 10 Hz; no interpolation or downsampling initially.
- Default action prediction horizon $H=5$ (0.5 s); ablate 1/3/10. Execute only the first action and re-observe every step.
- Near episode end, pad chunks with an action-valid mask; padded actions do not contribute to loss.
- Save `goal_reached_mask`. Before absorbing state, train normally; afterward retain the robot for others’ observations but mask its repeated zero-action policy loss. Add bounded stop/hold examples later only if needed.
- Prefer fixed physical normalization (sensing radius, speed/action bounds). Use train-only statistics only for fields without natural bounds. Freeze normalization through DAgger and store it with checkpoints.

### 7.5 Dataset growth

- Level 0: 1–3 joint episodes for storage/replay/loader checks.
- Level 1: roughly 20 episodes per active scenario/count for overfit and closed-loop smoke tests.
- Then approximately 100/500/2000 joint-episode learning curves. Stop or rebalance based on validation saturation, scenario coverage and expert/storage cost.
- Report joint episodes, per-robot episodes and frames—not frames alone.

## 8. Training plan

### 8.1 Offline BC

1. Overfit a deterministic MLP on tiny fixed data to verify observations, masks, normalization, chunks, loss, checkpoints and evaluation.
2. After smoke success, train full MLP baseline and Flow main policy concurrently on identical splits/settings.
3. Flow produces one sampled action chunk per robot/step; no best-of-$N$ selection. Freeze evaluation sampling seeds/repeats.

Training gate: loss decreases clearly, chunk/mask semantics are correct, denormalized actions obey bounds, checkpoint reload reproduces fixed outputs, fixed-seed evaluation is repeatable and simple training scenarios close successfully. Set numeric thresholds after initial measurements.

### 8.2 DAgger

- Explicit modes: `expert_only`, `mixed`, `learner_only`.
- Query db-LaCAM from the current joint state every control step initially; label the current state with the returned action chunk. Record query count and median/p95/max/cumulative planning time.
- Initial per-query timeout is 20 s. On timeout/no-solution/invalid result, add no fake label, terminate rollout and log the failure.
- Only if cost is unacceptable, test configurable $K$-step replanning. A cached plan requires state-deviation checks and immediate invalidation/replan above threshold.
- Mixed mode samples once per control step for the whole joint action: all unfinished robots execute expert with probability $\beta_k$, otherwise learner; absorbing robots remain zero. Per-agent mixing is a later robustness experiment.
- Regardless of executed source, supervision is the valid expert action from the current state. Record expert action, executed action and source separately.
- If a learner candidate would collide, reject the transition and terminate. The current safe state’s valid expert correction remains trainable; no colliding next state is created.
- If an executed expert action collides without injected noise, quarantine as `expert_execution_collision` and investigate model/order/$dt$/checker mismatch.
- Begin with `action_noise_std=0`. Noisy recovery is a separate, labeled later experiment.
- Use a configurable decaying $\beta_k$ and record configured and actual expert/learner proportions.

## 9. Evaluation and experimental protocol

### 9.1 Proof gates and baselines

- **Gate 1:** no-communication MLP/Flow works closed-loop and beats random and goal-seeking controllers.
- **Gate 2a:** with fixed $A=S$, uncompressed private information improves interaction scenarios over the same no-communication backbone.
- **Gate 2b:** optional all-to-all full-information upper bound.
- **Gate 3:** with the same $A=S$, learned messages improve over no communication and degrade under zero/shuffle/wrong-recipient/disable interventions.

Core baselines:

- Centralized db-LaCAM expert.
- Same-backbone no communication.
- Full private information with fixed topology.
- All-to-all learned communication.
- Network-constrained learned communication.
- MLP versus Flow; BC versus DAgger.
- Later: GLAS official smoke and clearly labeled GLAS-style reproduction; CBF on/off.

### 9.2 Metrics

- Task: strict success, timeout, makespan, sum of costs, path length and time-to-goal.
- Safety: collision, minimum pair/obstacle distance, safe-distance violation, near-collision and safety-filter intervention.
- Imitation: action loss/MSE, variation/jerk, trajectory deviation and expert cost/performance gap. Low MSE alone is not sufficient.
- Communication: dimension, payload bits, bits/message, messages/s, receivers, attempted/transmitted/delivered bits per episode, drops, age and communicating-step fraction.
- Compute: expert planning, policy/sender/filter inference, real-time factor, hardware and memory.

### 9.3 Statistical budget

- Smoke: 10 fixed scenarios, 1 seed.
- Development: about 50 scenarios, 1–2 training seeds.
- Broad ablations: 50–100 scenarios, 3 seeds.
- Final core: 200 frozen scenarios, 5 training seeds.
- Use paired scenario IDs/seeds across methods; report seed distributions, paired effects and predefined 95% confidence intervals for rate metrics.
- RTX A4000 accelerates learning/inference; db-LaCAM remains mainly CPU-bound and is timed separately.

## 10. Implementation roadmap

### Milestone 1 — two-robot vertical slice

Freeze one readable two-robot swap YAML in an empty 8 m × 8 m workspace with Stage 1 dynamics and parameters. Implement:

```text
scenario load
  -> unified GPU container
  -> PlannerFactory/db-LaCAM subprocess
  -> ExpertPlanResult
  -> simulator replay
  -> dynamics/action/swept-collision/strict-goal validation
  -> minimal raw archive
  -> trajectory visualization
```

Acceptance:

- CUDA, CasADi, db-LaCAM and planner-factory smoke tests pass.
- Scenario solves reproducibly; order, shapes, original actions and $dt$ are correct.
- Planner-consistent and independent analytic collision conclusions agree.
- Replay reaches strict 0.05 m goal tolerance.
- Deliberate collision, wrong-$dt$ and timeout cases classify correctly.
- Existing CasADi regression tests still pass.

Do not generate a large dataset, train a policy or implement communication before this gate.

### Subsequent phases

1. Expand db-LaCAM benchmark to 3/4/8/16 robots; build frozen scenario bank and expert datasets.
2. Empty-map no-communication BC and DAgger; MLP/Flow and evaluation.
3. Add random rectangular obstacles and topology/warehouse scenarios.
4. Full private-information communication, then learned continuous messages.
5. Quantization, link/sender budgets and learned receiver selection.
6. Delay, dropout and asynchronous delivery.
7. Full baselines, ablations, OOD and runtime experiments.
8. Extend to double integrator, then RoboChief; optionally add CBF and physical validation.

## 11. Immediate implementation order

1. Commit this plan and preserve the current working tree.
2. Add `third_party/db-lacam` recursive submodule at a tested commit.
3. Add lightweight scenario specs/loader and the fixed swap YAML.
4. Add `docker/Dockerfile.research` and `research-gpu`; keep old services until smoke tests pass.
5. Add `ExpertPlanResult` and extend `Planner` compatibly.
6. Refactor `DbLacamPlanner` subprocess adapter and structured failures.
7. Add analytic collision checker and trajectory validator.
8. Run and document Milestone 1 acceptance tests.

## 12. Risks and responses

- **Expert too slow:** cache offline results; measure stepwise DAgger first; only then test guarded $K$-step replanning or bindings.
- **Jerky expert:** quantify variation/jerk and primitive effects; keep raw data primary and validate any refined derivative separately.
- **Poor expert coverage:** benchmark before training, report coverage and avoid filtering the evaluation bank after solving.
- **Message collapse/ignored:** monitor gradients and variance; run zero/shuffle/wrong-recipient/disable interventions.
- **False bandwidth claims:** continuous dimension is not bits; claim bit bandwidth only after fixed quantization and complete traffic accounting.
- **Safety filter hides policy quality:** always report filtered and unfiltered policy plus intervention rate.
- **Scope growth:** finish the single-integrator vertical pipeline before new dynamics, complex networking, full GLAS or physical robots.

## 13. References

- GLAS paper: https://arxiv.org/abs/2002.11807
- GLAS code: https://github.com/bpriviere/glas
- db-LaCAM paper: https://arxiv.org/abs/2512.06796
- db-LaCAM code: https://github.com/IMRCLab/db-lacam
- Dynobench: https://github.com/quimortiz/dynobench
- MovingAI MAPF benchmarks: https://movingai.com/benchmarks/mapf/index.html
