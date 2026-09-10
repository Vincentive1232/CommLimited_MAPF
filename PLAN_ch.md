# 从全局专家到分布式策略的模仿学习计划

## 1. 研究目标与研究问题

本项目的目标是：从 centralized db-LaCAM expert 生成的示范中，训练一个参数共享的 decentralized policy，用于连续空间物流场景中的多机器人运动协调。

对于时刻 $t$ 的机器人 $i$：

$$
(a_i^t,\{z_{ij}^t\})=\pi_\theta(o_i^t,\{m_{ji}^t\},\{q_{ij}^t\}),
$$

其中，$o_i^t$ 是局部观测，$z_{ij}^t$ 是针对接收者生成的 outgoing message，$m_{ji}^t$ 是实际到达的消息，$q_{ij}^t$ 是机器人本地可获得的链路信息。Message 没有直接的监督标签；expert action loss 的梯度通过 receiver、可微通信信道或瓶颈以及 sender 反向传播。

主要研究问题：

1. MLP 和 Flow policy 能否只使用 decentralized observation 模仿 centralized kinodynamic expert？
2. 在 backbone、数据和训练预算一致时，通信能否相对 no-communication policy 改善成功率、安全性和协调效率？
3. 保留通信增益需要多大的 message representation 和 network bandwidth？
4. 方法能否泛化到更多机器人、更高密度、未见地图和未见目标排列，并应对 delay、dropout 和 asynchronous delivery？
5. DAgger 能否相对 offline BC 改善 closed-loop performance？
6. 可选问题：CBF/safety filter 能提升多少部署安全性，又会掩盖多少 policy 本身的失败？

## 2. 范围与阶段边界

- 当前只考虑 simulation、homogeneous fleet、二维连续空间、静态障碍物和共享 policy 参数。
- 训练阶段可以访问 centralized db-LaCAM expert；部署阶段每个机器人只能使用局部观测、本地链路信息和收到的 P2P messages。
- CasADi 保留为 regression/reference planner。db-LaCAM 因速度和扩展性更好而作为主要 expert，但需单独评估其较 jerky 的轨迹。
- 完整 GLAS reproduction 延后到核心 proof of concept 成立之后。前期最重要的 baseline 是相同 backbone 下关闭通信的模型。
- Safety filter 和真实机器人验证属于后期可选阶段，不阻塞 policy 学习主线。

动力学按顺序扩展：

1. `single_integrator`：完成包含通信与评估在内的整条 pipeline。
2. `double_integrator`：仅在 Stage 1 成立后开始，并使用匹配的 Dynobench model 和 motion primitives。
3. RoboChief：三个间隔 $120^\circ$ 的 omniwheels；候选状态为 $[x,y,\theta,v_x,v_y,\omega]$，候选控制为 $[a_x,a_y,\alpha]$，最终模型根据实测平台参数确定。

后续动力学不能复用 single-integrator primitives。每个新模型必须先通过 model、primitive、timestep、bounds 和 trajectory replay tests。

计划原则：冻结当前阶段所需的接口和验收标准；远期 protocol、quantizer 和 scheduler 细节在进入相应阶段、获得测量结果后再确定。

### 2.1 当前代码库状态

已有能力：多种连续动力学、homogeneous multi-robot simulator、CasADi expert、初步 db-LaCAM wrapper、ego-relative observation 和 visibility masks、DeepSet/Transformer encoders、MLP/Flow policies、BC/DAgger、checkpoint、evaluation 和 trajectory visualization。

当前主要缺口：

- `Planner` 目前只有 `reset()` 和 `__call__(obs)`；db-LaCAM adapter 混合了 YAML、subprocess、cache 和 failure handling。
- DAgger/evaluator 中仍有 CasADi-specific planner 选择或异常处理；db-LaCAM 失败时可能错误地返回零动作。
- 当前 db-LaCAM wrapper 只支持部分系统、假定 $dt=0.1$，并把 obstacles 写死为空。
- 尚无统一 scenario schema、结构化 expert result、独立 replay validator、communication channel/network model、joint communication loader 和 bit-level traffic accounting。

## 3. 系统语义

### 3.1 机器人、动作与时间

- RoboChief 物理直径约为 $0.50\,\mathrm{m}$。
- 有效碰撞半径为 $r=0.30\,\mathrm{m}$。
- 机器人之间的中心安全距离为 $d_{safe}=0.60\,\mathrm{m}$。
- 暂定 near-collision 距离为 $0.70\,\mathrm{m}$，后续根据制动距离和定位误差重新校准。
- Stage 1 action 为平面速度 $u_i=[v_x,v_y]$。
- Control/planner period 为 $dt=0.1\,\mathrm{s}$，即 10 Hz。
- 默认速度上限为 $v_{max}=1.0\,\mathrm{m/s}$；后续测试 $0.5$ 和 $1.5\,\mathrm{m/s}$。
- Stage 2 action 为 acceleration。直接输出 primitive 仅作为远期可选的 policy representation。

### 3.2 到达目标与 episode 终止

- 严格目标容差为 $\|p_i-p_i^{goal}\|_2\leq0.05\,\mathrm{m}$，并且需要连续满足 3 个 control steps。
- 确认到达后，机器人进入 absorbing `goal_reached` 状态：位置固定、action 强制为零，但仍参与感知、通信和碰撞检测。
- 所有机器人都进入 absorbing state 后才算 joint success。Single integrator 不检查终点速度；二阶模型必须增加终点速度条件。
- 默认 rollout horizon 为 30 s / 300 steps。预先声明的 corridor/warehouse 场景可使用 60 s。Planner wall time 与 simulated horizon 是两个独立参数和指标。
- 如果 db-LaCAM 内部无法采用严格阈值，应同时报告 planner-native success 和统一 strict evaluator 结果。

### 3.3 碰撞语义

- Robot–obstacle collision 只能选择以下一种等价实现：半径 $0.30$ m 的 disk 与原始 obstacle 相交；或 robot center 与膨胀一次的 obstacle 相交。禁止重复膨胀。
- 定义抽象 `CollisionChecker`：
  - `DynobenchCollisionChecker`：与 db-LaCAM/Dynobench/FCL 的几何语义一致。
  - `AnalyticDiskCollisionChecker`：独立实现 Stage 1 的 disk–disk 和 disk–rectangle 检查。
- 必须检查整个 control step 内的 swept motion，而不只检查离散端点。Stage 1 对匀速 disk 使用连续时间最近距离。
- 两个 checker 共享同一份 geometry config，但不共享检查实现；使用随机线段和边界案例交叉验证。
- 如果候选 action 在本 step 内产生碰撞，则不提交 next state，保留最后安全状态，记录可获得的 first-contact 信息，并立即以 collision failure 终止 joint rollout。

### 3.4 局部观测

第一版 policy input 包含 ego/goal-relative state、可见邻居集合和可见障碍物集合。

- 无通信时只能直接感知邻居相对位置，不暴露邻居速度、目标或意图。
- 所有机器人可预先知道相同的静态地图和世界坐标系，并估计自己的 world pose；静态地图不包含实时 fleet positions、其他机器人目标、expert trajectory 或全局链路矩阵。
- Stage 1 不把 global map embedding 输入 policy；`map_context` 仅作为后续 ablation。
- Robot 和 obstacle sensing radius 默认均为 2.0 m；测试 1.5/2.0/3.0 m，并报告可见邻居/障碍数量和看到完整 fleet 的 timestep 比例。
- Neural input 不包含固定 robot ID。ID 只用于内部路由、消息对齐、日志和带宽统计。
- Neighbor set 使用带 mask 的 permutation-invariant aggregation。
- 静态矩形障碍的局部元素为 $[\Delta x,\Delta y,h_x,h_y]$。
- 第一版采用 `max_visible_obstacles=8`，按到 obstacle boundary 的距离保留最近障碍并使用 padding mask。记录 truncation；若约 1%–5% 以上 frames 被截断则重新评估。后续 ablate 4/6/8/12。

## 4. 通信方法

### 4.1 信息边界

Simulator 可以集中保存真实 link matrix，但机器人 $i$ 只能获得协议允许的本地 outgoing candidates、$q_{ij}^t$、capacity 和自己的发送预算，不能获得整个 fleet 的 link matrix。这属于 centralized simulation of decentralized information。

共享 scheduler 在每个 sender 上独立运行：

$$
g_{ij}^t=f_{schedule}(o_i^t,q_{ij}^t,B_i^t),\qquad e_{ij}^t=A_{ij}^tg_{ij}^t.
$$

拥有全局信息的 scheduler 只能作为 upper-bound baseline。

### 4.2 位置与消息协议

Local-link proof of concept：

- $A^t=S^t$，即通信候选与当前可见邻居相同。
- 双方都由 virtual local sensor 获得相对位置，因此 learned payload 不重复编码位置。
- 内部 ID 用于将消息和 neighbor slot 对齐，但不作为 neural feature。
- Receiver element 为 $[r_{ji}^t,z_{ij}^t,q_{ij}^t,age_{ij}^t]$。

Remote network-selected P2P 阶段：

- $A^t$ 与 sensing 独立；不存在 central fleet-position broadcast。
- Network discovery 暴露 candidate address/link quality；scheduler 必须先选 link，再传 pose。
- 只有被选中的 link 才交换带 address/timestamp 的 world pose；pose、header 和 link setup 都计入 bandwidth，并写入本地 cache。
- 第一次建立 link 至少需要一个 control step，之后 personalized payload 才能使用缓存几何。
- 暂定两级 gate：`pose_query_gate` 选择未知或过期 peer，`message_gate` 只向已有有效 pose cache 的 peer 发送 learned payload。进入该阶段后与 mentors 再冻结细节。

### 4.3 Message 与 receiver 架构

- 核心 message 是 receiver-conditioned：
  $$z_{ij}^t=f_{msg}(o_i^t,r_{ij}^t,q_{ij}^t).$$
- 所有有向边共享 encoder 参数，不使用 ID embedding。
- 第一版 aggregator 使用 DeepSet，支持 mask 和 zero-message 输入。Attention/Transformer 为后续对照。
- Phase 1 的 full private packet 暂定包含 sender goal-relative vector、previous executed action 和 sender local branch nominal action；不包含 expert action、第三方 observation 或固定 ID。
- 第一版采用 feed-forward、single-round messages，不使用 recurrent state。

### 4.4 Channel 与 network 接口

`CommunicationChannel` 子类共享 `reset()`、`exchange(...)` 和 `get_metrics()`：

- `ImmediateCommunicationChannel`：ideal、differentiable、same-step delivery。
- `BufferedCommunicationChannel`：使用 arrival-time queue 模拟 delay、drop 和 asynchrony。

早期可微信道不得意外 `detach()`、NumPy 转换或序列化。跨 timestep 的梯度策略在 asynchronous 阶段再决定。

`NetworkModel` 与 channel/scheduler 分离：

- `IdealNetwork`：perfect links。
- `DistanceBasedNetwork`：第一版 constrained model；默认 $R_{comm}=4$ m，$R_{sense}=2$ m；测试 2/4/6/$\infty$ m。
- `ObstacleAwareNetwork`、`StochasticNetwork` 和 `TraceDrivenNetwork` 属于后续子类。

Network model 决定物理可行性和成本，scheduler 决定是否值得发送，channel 负责传递和计费。

### 4.5 单个 control step 的时序

时刻 $t$：

1. 构造所有 local observations。
2. 针对 receiver 生成 outgoing messages。
3. Channel 执行一次 exchange。
4. Receiver 聚合决策时刻前已经到达的消息。
5. 生成并执行 $a^t$。
6. Simulator 推进至 $t+1$。

Phase 1–3 中，$o^t$ 产生的 zero-delay message 可以影响同一时刻的 $a^t$。延迟阶段只能使用决策前已经到达的消息。Buffered message 保存生成时间并提供归一化 age；过期规则在进入该阶段后配置。

### 4.6 通信阶段

1. **信息价值验证：** full private information、ideal synchronous channel，无 capacity、quantization 或 receiver constraints。主拓扑为 $A=S$，all-to-all 仅作为 oracle。
2. **Learned message 验证：** 保持相同 topology/channel，使用 receiver-conditioned continuous message；初始 `message_dim=16`。该维度只是 representation width，不能称为真实 bandwidth。
3. **表示容量与量化：** 测试 dimension 1/2/4/8/16，再固定每个元素的 bit 数。Payload bits/message = `message_dim × bits_per_element`。候选顺序为 float32 reference、uniform int8、可选 int4。Payload 与 pose/address/header overhead 分开报告。
4. **Bandwidth/network constraints：** 加入 per-link capacity $C_{ij}^t$、sender budget $B_i^t$、receiver/pose-query selection 和完整 traffic accounting。先使用 soft gates 与 bandwidth penalty，再测试满足预算的 hard top-$K$/binary selection；STE/Gumbel 等 estimator 到该阶段再决定。
5. **网络不完美：** Phase 4 成立后再加入 delay、dropout 和 asynchronous delivery。记录 attempted、transmitted 和 delivered bits；消息即使丢失，也消耗 transmitted bandwidth。

进行因果比较时，不得同时改变 message content 和 topology。

### 4.7 学习目标与初始化

首先只使用 action imitation：

$$
\mathcal L_{action}=\|\hat u-u^E\|^2\quad\text{或 Flow loss}.
$$

后续每次只增加一个目标：bandwidth penalty、receiver sparsity、持续链路上的 temporal consistency，以及可选 information/entropy regularization。

Communication policy 从通过 gate 的 no-communication checkpoint warm-start，复用兼容的 observation/action weights；新 communication fusion 初始化为近零影响。随后联合微调 sender、receiver 和 action network，不能长期冻结 action policy。同时训练相同预算的 scratch variant 作为初始化 ablation。

## 5. 软件架构与 db-LaCAM 集成

### 5.1 模块职责

```text
third_party/db-lacam/   上游 recursive Git submodule
scenarios/              specs、YAML I/O、generators
planning/               Planner、ExpertPlanResult、adapters
collision/              CollisionChecker implementations
validation/             trajectory replay validation
learning/data/          per-robot 与 joint loaders
configs/                scenarios、planners、experiments
docker/                 build/runtime definitions
```

只在对应 vertical slice 需要时创建模块。避免 catch-all `utils.py` 和跨模块目录嵌套。

### 5.2 Scenario 与配置模型

- 外部配置使用 human-readable YAML；加载后转为轻量 frozen dataclass。
- `EnvironmentSpec`：bounds、obstacles、environment ID。
- `ScenarioSpec`：scenario ID、environment、按顺序排列的 robot model/geometry references、starts/goals、family、seed、split 和 schema version。
- 第一种 obstacle 是由 center 和 half-extents 表示的 axis-aligned rectangle。`RobotGeometrySpec` 单独定义，确保 inflation 只发生一次。
- 只在加载边界检查 required fields、基础 shape、finite values、唯一 ordering、正尺寸和 bounds。不引入 Pydantic、自动修复或提前设计复杂 migration。
- Template YAML 放在 `configs/scenarios/templates/`；生成后冻结的 instances 放在 `data/scenarios/{split}/`。
- 可读文件名示例：`empty_swap__n04__seed000123.yaml`；`scenario_id` 与文件 stem 一致，重名直接报错。
- Scenario YAML 只描述问题；planner YAML 描述 solver/primitives/budget；experiment YAML 引用 scenario set、planner、dataset、policy 和 seed。同一个 scenario 不经修改即可交给不同 planners。

### 5.3 Planner 契约

扩展抽象 `Planner`，同时暂时保留 `__call__(obs)` 兼容旧 tests：

```python
plan_episode(initial_joint_state, goals, environment, *, time_limit_s)
query_action(current_joint_state, goals, environment, *, horizon, time_limit_s)
```

新的 dataset、DAgger 和 validation 代码直接使用 joint state，不再让 planner 从 decentralized observation 反解世界状态。

两个入口均返回 `ExpertPlanResult`，包含 success、per-robot states/actions、dt、robot order、valid lengths/shared time-grid semantics、wall time、termination reason 和精简 planner metadata。

- Per-robot 表示支持未来不同 state/action dimensions。
- `stack_homogeneous()` 为 learning 返回 `[T+1,N,nx]` 和 `[T,N,nu]`。Adapter 显式将提前完成的机器人补成目标位置和零动作，同时保留原始 valid lengths。
- No-solution、timeout 和 planner-invalid output 使用结构化 failure；schema/programming errors 抛出统一 planner exception。
- 禁止把 fallback zero action 当作 expert label。
- `replan_freq` 是 experiment parameter，不能隐藏在 wrapper 中。

### 5.4 db-LaCAM 源码、容器与调用链

源码由 recursive Git submodule 管理，依赖、编译和运行由 Docker 管理：

```bash
git clone --recurse-submodules <repo>
git submodule update --init --recursive
git submodule status --recursive
```

db-LaCAM 固定到经过测试的 commit。需要修改时使用 fork 并更新主仓库中的 submodule gitlink，不能依赖临时容器内的修改。每次更新依赖后重新运行 smoke、adapter、replay 和 collision tests。

正式运行环境是由 `docker/Dockerfile.research` 构建的统一 `research-gpu` service，其中包含 CUDA PyTorch/LeRobot、CasADi、OMPL、db-LaCAM、Dynoplan/Dynobench 和 motion primitives。在统一 smoke tests 通过前，保留现有 GPU 和 db-LaCAM Dockerfiles 作为 regression/fallback。

第一版继续使用 subprocess/YAML adapter：

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

每次调用使用独立 temporary directory。记录 problem、algorithm、result、stats、command、return code、stdout/stderr 和 timing。同时使用 db-LaCAM internal timeout 和略大的 subprocess timeout。将 timeout、missing/malformed output、no-solution 和 invalid replay 映射为不同原因。测量 process/YAML overhead；只有确认它是 DAgger 的主要瓶颈后才考虑 binding。

Offline run 保存完整 raw files；正常 DAgger 只保存精简记录，主要在失败时保留详细 artifact。

### 5.5 容器验收

- RTX A4000 上 `torch.cuda.is_available()` 为真。
- CasADi smoke solve 通过。
- db-LaCAM official smoke 通过。
- 两种 planner 都可通过 `PlannerFactory` 创建。
- Data/checkpoint/log volumes 可写。

Run metadata 保存 repository/dependency commits、image tag/digest、primitives checksum、CUDA/PyTorch/driver、hardware、runtime 和 peak memory；这些信息不在每个 episode manifest 中重复。

Training、DAgger 和 evaluation 在 terminal 输出简洁 summary，并写入 TensorBoard scalars/curves。至少记录 losses、success/collision/timeout、expert-query latency/failure、采样的 robot counts/scenario families、communication usage 和 validation checkpoints。详细 per-episode diagnostics 写入结构化日志，不堆入 TensorBoard 文本。

## 6. Expert 验证与场景生成

### 6.1 Expert benchmark

训练前，在固定 seeds 下独立测试 db-LaCAM：使用 2/4/8/16 个 single-integrator robots，覆盖 random、swap、crossing、corridor 和 bottleneck，并包含 timeout/invalid cases。

- 报告 success、wall time、trajectory duration、dynamics/collision validity、action bounds、variation/jerk 和 scaling。
- 给出 1/5/30/60 s budgets 下的 coverage/time-to-solution curve。
- 初始 offline budget 为 30 s/instance；初始 DAgger query budget 为 20 s。
- Planner-reported success 必须通过独立 replay 才能进入训练集。

### 6.2 Trajectory validation

- 保留 db-LaCAM robot ordering，检查 state/action lengths、shapes、$dt$ 和 action semantics。
- 使用 db-LaCAM 原始 controls 作为 expert labels，通过项目 dynamics 重放并与返回的 next states 比较。
- 由 state difference 反算 control 只用于诊断。原始 action 缺失或无效时标为 adapter/schema failure，即使 single integrator 也不例外。
- 检查 action/state bounds、swept robot/obstacle collision、goal condition 和 hold behavior。
- Planner-consistent collision 与独立 checker 不一致时隔离该结果。

### 6.3 场景类型

- **A — Empty coordination：** pair swap、3+ crossing、circle swap、random goal permutation。
- **B — Random clutter：** 8 m × 8 m、axis-aligned obstacles、10%/20% union-area occupancy，参考 GLAS。
- **C — Topology stress：** alcove/at-goal、corridor、bottleneck、maze、merge、intersection、warehouse aisle passing。
- **D — MovingAI-derived continuous：** 将部分 empty/warehouse/room/random/maze grids 转换为连续矩形；不能把结果与离散 MAPF 数字直接比较。
- **E — db-LaCAM reference：** 官方 alcove、at-goal、circle、maze 和 scalability cases，用于验证 wrapper。

记录 workspace、free area、occupancy union ratio、corridor/bottleneck width、robot count、starts/goals、horizon 和 seed。Empty maps 用于将多机器人协调问题与 obstacle avoidance 分开。

### 6.4 先生成 scenario，再调用 expert

1. 独立于 planner success 生成并冻结 scenario bank。
2. Precheck start/goal 与膨胀后 geometry、pairwise safety、topology constraints 和单机器人静态可达性。
3. 运行 db-LaCAM，并分类为 `expert_valid`、`expert_failed`、`expert_invalid_trajectory`；无效生成单独分类。
4. 训练集只使用 `expert_valid`，但正式评估和报告不能在求解后从冻结场景库中删除失败样本。

报告 expert coverage $N_{expert\_valid}/N_{precheck\_valid}$ 和 student/expert joint outcomes，暴露 selection bias。db-LaCAM 在预算内失败只表示“该配置没有求解成功”，不能宣称数学上不可行。可单独建立小规模 guaranteed-feasible diagnostic set。

Raw expert trajectory 是主要数据。如果 jerk 影响学习，则创建单独版本的 refined dataset，并重新检查 dynamics、collision、goal 和 bounds。

## 7. Dataset 设计

### 7.1 三层存储

1. **Raw planner archive：** 每个 `joint_episode_id` 保存 problem、algorithm、result、stats 和 validation。
2. **Minimal joint manifest：** joint/scenario ID、split、robot-to-LeRobot episode mapping、source/status 和 raw path。
3. **LeRobot datasets：** decentralized per-robot training frames。

共享 schema/config/version/checksum 只在 dataset metadata 中保存一次。详细 failure diagnostics 放入 `failures.jsonl` 或 run logs，不放入主 manifest。

每个 frame 包含 local observation、expert local action chunk、visibility/valid masks、joint/episode mapping 和 timestep。Global state、IDs 和 joint expert plans 属于 metadata，不是 policy input。

### 7.2 Split 与 lineage

- 按 frozen scenario/joint episode 划分，不能按 frame 或 robot 随机划分：70% train、15% validation、15% test。
- 独立 OOD sets：更多机器人、更高密度、未见地图、未见目标排列和更窄 topology。
- 训练前冻结 split manifest；同一 joint rollout 的所有 robots 属于同一个 split。
- Raw、refined 和 DAgger data 保留 scenario lineage。
- DAgger 只能使用登记为 train 的 scenarios/seeds。Validation/test/OOD 始终只读；validation 用于模型选择，final test 不用于反复修改方案。

### 7.3 可变 fleet size 与采样

- 按 robot count 保存不同 dataset：`dataset_n2` 到 `dataset_n8`。
- 一个 minibatch 只包含同一个 $N$，自然形成 `[batch,N,...]`；不使用全局 `max_robots` padding。
- 同一个 shared DeepSet/policy checkpoint 跨 loaders 训练。默认先近似均匀采 robot count，再采 scenario family，最后采 minibatch。
- Encoder 参数 shape 与 neighbor-slot count 解耦。测试 zero neighbors、permutation invariance、跨 count checkpoint load 和 12/16 robot OOD inference。
- Train/ID robot counts 为 2–8；OOD 为 12/16；24/32 仅在资源允许时进行。

No-communication training 读取单机器人 frames。Communication training 使用 `JointTimestepDataset`，通过 manifest 重组同一 joint episode、同一 timestep 的所有机器人，不重复保存 observation。`active_mask` 和 `goal_reached_mask` 表示运行状态，不用于最大 fleet padding。

### 7.4 Temporal labels 与 normalization

- Raw、frames、control 和 evaluation 都使用 10 Hz；第一版不 interpolation 或 downsampling。
- 默认 action prediction horizon 为 $H=5$，覆盖 0.5 s；测试 1/3/10。每步只执行第一项 action，然后重新观测。
- Episode 尾部不足 $H$ 时使用 padding 和 action-valid mask；padding 不参与 loss。
- 保存 `goal_reached_mask`。进入 absorbing state 前正常训练；进入后仍保留该机器人供其他机器人观察，但 mask 掉其重复零动作 loss。仅在需要时加入有限 stop/hold samples。
- 优先使用 sensing radius、speed/action bounds 等固定物理 normalization。只有没有自然 bounds 的字段使用 train-only statistics。DAgger 默认冻结 normalization，并随 checkpoint 保存。

### 7.5 Dataset 规模

- Level 0：1–3 个 joint episodes，用于 storage/replay/loader 检查。
- Level 1：每个当前 scenario/count 约 20 episodes，用于 overfit 和 closed-loop smoke。
- 此后生成约 100/500/2000 joint episodes 的 learning curves。根据 validation saturation、scenario coverage 和 expert/storage cost 决定停止或重新平衡。
- 同时报告 joint episodes、per-robot episodes 和 frames，不能只报告 frames。

## 8. 训练计划

### 8.1 Offline BC

1. 首先在少量固定数据上 overfit deterministic MLP，检查 observation、masks、normalization、chunks、loss、checkpoint 和 evaluation。
2. Smoke test 通过后，在完全相同的 splits/settings 下同步训练完整 MLP baseline 和作为主要模型的 Flow policy。
3. Flow 每个 robot/control step 只采样一个 action chunk，不使用 best-of-$N$ selection。冻结 evaluation sampling seeds/repeats。

训练 gate：loss 明显下降；chunk/mask 语义正确；反归一化动作满足 bounds；checkpoint reload 后固定输入输出一致；固定 seed evaluation 可复现；简单训练场景能够 closed-loop 成功。具体数值阈值在初步测量后再确定。

### 8.2 DAgger

- 明确三种模式：`expert_only`、`mixed`、`learner_only`。
- 第一版每个 control step 都从当前 joint state 查询 db-LaCAM，并用返回的 action chunk 标注当前状态。记录 query count、median/p95/max 和累计 planning time。
- 初始 timeout 为 20 s/query。Timeout、no-solution 或 invalid result 时不添加伪标签，立即终止 rollout 并记录失败。
- 只有成本不可接受时，才测试可配置的 $K$-step replanning。缓存计划必须做 state-deviation check，超过阈值立即失效并重规划。
- Mixed mode 每个 control step 只对完整 joint action 抽样一次：所有未到达 robots 以 $\beta_k$ 概率执行 expert，否则都执行 learner；absorbing robots 始终为零。Per-agent mixing 仅作为后续 robustness experiment。
- 无论执行来源如何，监督标签都是当前状态上的有效 expert action。分别记录 expert action、executed action 和 source。
- 如果 learner candidate 将导致碰撞，拒绝该 transition 并终止。当前安全状态上的有效 expert correction 仍可训练，但不创建碰撞后的 next state。
- 如果没有注入 noise 时 expert action 仍导致碰撞，则隔离为 `expert_execution_collision`，检查 model、ordering、$dt$ 和 checker mismatch。
- 第一版 `action_noise_std=0`。Noisy recovery 是单独标记的后续实验。
- 使用可配置的递减 $\beta_k$，并记录设定值和实际 expert/learner action 比例。

## 9. Evaluation 与实验协议

### 9.1 Proof gates 与 baselines

- **Gate 1：** no-communication MLP/Flow 能够 closed-loop 工作，并优于 random 和简单 goal-seeking controllers。
- **Gate 2a：** 固定 $A=S$ 时，uncompressed private information 在交互场景中优于相同 backbone 的 no-communication policy。
- **Gate 2b：** 可选 all-to-all full-information upper bound。
- **Gate 3：** 固定相同 $A=S$ 时，learned message 优于无通信，并且在 zero/shuffle/wrong-recipient/disable interventions 后性能下降。

核心 baselines：

- Centralized db-LaCAM expert。
- 相同 backbone 的 no communication。
- 固定 topology 的 full private information。
- All-to-all learned communication。
- Network-constrained learned communication。
- MLP vs Flow；BC vs DAgger。
- 后续：GLAS official smoke、明确标注差异的 GLAS-style reproduction、CBF on/off。

### 9.2 Metrics

- Task：strict success、timeout、makespan、sum of costs、path length、time-to-goal。
- Safety：collision、minimum pair/obstacle distance、safe-distance violation、near-collision、safety-filter intervention。
- Imitation：action loss/MSE、variation/jerk、trajectory deviation、expert cost/performance gap。低 MSE 不代表 closed-loop 一定好。
- Communication：dimension、payload bits、bits/message、messages/s、receiver count、attempted/transmitted/delivered bits per episode、drop、age 和 communicating-step fraction。
- Compute：expert planning、policy/sender/filter inference、real-time factor、hardware 和 memory。

### 9.3 统计规模

- Smoke：10 个固定 scenarios，1 个 seed。
- Development：约 50 scenarios，1–2 个 training seeds。
- Broad ablations：50–100 scenarios，3 seeds。
- Final core：200 个冻结 scenarios，5 个 training seeds。
- 不同方法使用配对的 scenario IDs/seeds；报告 seeds 分布、paired effects，并为比例指标给出预先确定的 95% confidence intervals。
- RTX A4000 主要加速 learning/inference；db-LaCAM 主要受 CPU 限制并单独计时。

## 10. 实施路线

### Milestone 1 — Two-robot vertical slice

冻结一个可读的 two-robot swap YAML：empty 8 m × 8 m workspace，采用 Stage 1 动力学与参数。实现：

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

验收条件：

- CUDA、CasADi、db-LaCAM 和 planner-factory smoke tests 通过。
- Scenario 可重复求解，ordering、shapes、原始 actions 和 $dt$ 正确。
- Planner-consistent 与 independent analytic collision conclusions 一致。
- Replay 满足严格 0.05 m goal tolerance。
- 人为构造的 collision、wrong-$dt$ 和 timeout cases 能正确分类。
- 现有 CasADi regression tests 不被破坏。

该 gate 通过前，不生成大规模 dataset、不训练 policy、不实现 communication。

### 后续阶段

1. 将 db-LaCAM benchmark 扩展到 3/4/8/16 robots，建立 frozen scenario bank 和 expert datasets。
2. Empty-map no-communication BC/DAgger、MLP/Flow 和 evaluation。
3. 加入随机矩形障碍和 topology/warehouse scenarios。
4. 先实现 full private-information communication，再实现 learned continuous messages。
5. 加入 quantization、link/sender budgets 和 learned receiver selection。
6. 加入 delay、dropout 和 asynchronous delivery。
7. 完成 baselines、ablations、OOD 和 runtime experiments。
8. 扩展 double integrator 和 RoboChief；可选加入 CBF 和真实机器人验证。

## 11. 立即实施顺序

1. 提交本计划并保留当前 working tree。
2. 在经过验证的 commit 上添加 `third_party/db-lacam` recursive submodule。
3. 添加轻量 scenario specs/loader 和固定 swap YAML。
4. 添加 `docker/Dockerfile.research` 与 `research-gpu`；smoke tests 通过前保留旧 services。
5. 添加 `ExpertPlanResult`，并以兼容方式扩展 `Planner`。
6. 重构 `DbLacamPlanner` subprocess adapter 和结构化 failure handling。
7. 添加 analytic collision checker 和 trajectory validator。
8. 运行并记录 Milestone 1 acceptance tests。

## 12. 风险与应对

- **Expert 太慢：** 缓存 offline results；先测量逐步 DAgger；之后才测试 guarded $K$-step replanning 或 bindings。
- **Expert 太 jerky：** 测量 variation/jerk 和 primitive 影响；raw data 始终保留，任何 refined derivative 单独版本化并重新验证。
- **Expert coverage 太低：** 训练前 benchmark，报告 coverage，禁止在求解后过滤 evaluation bank。
- **Message collapse 或被忽略：** 监测 gradient 和 variance；运行 zero/shuffle/wrong-recipient/disable interventions。
- **错误 bandwidth 声明：** continuous dimension 不等于 bits；只有固定 quantization 和完整 traffic accounting 后才报告 bit bandwidth。
- **Safety filter 掩盖 policy 问题：** 同时报告 filtered、unfiltered policy 和 intervention rate。
- **Scope 失控：** single-integrator 全 pipeline 完成前，不扩展新动力学、复杂网络、完整 GLAS 或真实机器人。

## 13. 参考资料

- GLAS paper: https://arxiv.org/abs/2002.11807
- GLAS code: https://github.com/bpriviere/glas
- db-LaCAM paper: https://arxiv.org/abs/2512.06796
- db-LaCAM code: https://github.com/IMRCLab/db-lacam
- Dynobench: https://github.com/quimortiz/dynobench
- MovingAI MAPF benchmarks: https://movingai.com/benchmarks/mapf/index.html
