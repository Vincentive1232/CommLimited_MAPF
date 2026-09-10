# Objective
- 从 centralized db-LaCAM expert 中训练一个参数共享的分布式多机器人控制策略。db-ECBS 纳入 literature review，并保留为资源允许时的可选对照，不作为核心实现依赖。
- 每个机器人根据局部观测和受带宽约束的机器人间消息生成局部动作和待发送消息，消息没有直接监督，而是通过动作模仿目标进行end to end学习。
- 最终输出包含 action 和 message。Message 没有人工提供的直接监督标签，但 action imitation loss 的梯度应通过通信路径反向传播，以端到端训练 sender、可微通信瓶颈、receiver 和 action policy。
- 方法将在包含连续动力学的物流场景中，与无通信策略等baseline进行比较。



# Current Scope and Non-Goals
此处我们明确当前项目和实验的着重点：
目前考虑的问题应该限制在以下的范围内：
    - 目前所有的实验都应该是simulation
    - 使用的机器人暂时假设为homogeneous robot fleet
    - 动力学采用串行阶段门：先用 `single_integrator` 完成 db-LaCAM、dataset、no-communication BC/DAgger、learned communication 和 evaluation 的完整闭环；该闭环成立后再扩展 `double_integrator`，最后扩展完整 RoboChief 二阶全向动力学
    - robochief 是由三个间隔 120 度的 omniwheel 驱动的圆盘机器人；其完整状态、控制输入、物理参数和 db-LaCAM motion primitives 在第一阶段管线稳定后再加入
    - 目前只考虑2D continuous space
    - 采用圆形碰撞模型来建模物体的碰撞，应使用或者借鉴db-LaCAM以及其他主流的collision checking方法
    - 目前只考虑静态障碍物
    - 训练的local policy应该是一个shared policy，这个policy应该被所有机器人分享
    - 最后的训练和部署遵循：centralized training，decentralized execution
    - 通信部分需要考虑到bandwidth limit，packet loss，asynchronous communication等问题。
    - action输出之后允许通过一个CBF/NeuralCBF来保证最后动作的安全
目前阶段不考虑的问题：
    - 异构机器人
    - 三维碰撞几何
    - 真实网络通讯协议
    - 端到端传感器图像，点云，数据作为输入
    - 大规模真机微调和实验
    - 实现多种CBF变体

- Planning granularity principle
  - 当前 PLAN 优先冻结端到端 pipeline、模块接口、阶段依赖、验收 gate 和实验公平性；尚未进入的阶段只记录候选方案，不提前过度固定 protocol header、精确 quantization、scheduler estimator 等实现细节。
  - 每个阶段开始前单独完成 design review，根据上一阶段的测量结果冻结该阶段参数和 acceptance criteria；所有暂定选择允许在有实验依据时修改，但必须更新对应配置和计划中的理由，不把设计历史堆进 episode manifest。
  - 近期工作只实现当前 stage 所需的最小闭环，同时为后续 network/communication/dynamics subclasses 保留统一接口；后期细节不能阻塞 db-LaCAM -> dataset -> no-communication policy 主线。


# Research Questions
此处我们明确我们想要回答的问题/想要达到的目标
- generative model能否实现对centralized MRMP的imitation 
- 相比同 backbone、同数据和同训练预算的 no-communication ablation，learned communication 是否提高以下指标；GLAS/GLAS-style 作为 proof-of-concept 之后的独立外部 baseline：
  - success rate
  - control effort
  - collision safety
  - coordination efficiency
  - reduced communication budget
- 性能能否在不同的通信带宽上实现较为接近的性能：
  - success rate
  - collision count
  - message difference
- 验证泛化能力：
  - 是否能够泛化到从未见过的初始状态
  - 是否能够泛化到从未见过的目标排列
  - 是否能够泛化到更多机器人的场景
  - 更密集的场景
  - 不同的map
- 在以下通信扰动时性能如何：
  - message dropout
  - delievery delay
  - asynchronous update
  - communication radius
  - top-K receiver limit
- 使用DAgger能否显著改善offline behavior cloning的效果
- CBF等Safe Filter作为最终的安全保障能带来多大的性能改善，能否做出一些安全保证。


# Provided Baseline in the codebase
此处我们列出在目前的codebase里什么功能已经具备，不需要重复实现；什么功能还没有实现，需要实现：
- 已有的模块：
  - 多种连续动力学
  - homogeneous multi-robot simulator
  - centralized CasADi expert
  - db-LaCAM wrapper半成品
  - ego-relative observation 第一版局部观测模型
  - visibility mask 邻居是否可见的mask，用于阻止centralize settings下invisible的邻居进入deepset
  - DeepSet/Transformer encoder
  - MLP/Flow imitation policy
  - offline BC 和 DAgger
  - checkpoint 和 evaluation
  - success / collision / trajectory visualization
- 缺失的模块：
  - Behavior Cloning 部分：
    - 内置db-LaCAM并单独测试性能
    - 完整的db-LaCAM expert benchmark
    - db-LaCAM 收集供给离线训练的数据的接口
    - db-LaCAM DAgger接口
    - GLAS baseline
    - 训练一个communication-free的baseline
  - Communication 部分：
    - 真正的learned message
    - sender/receiver 设计
    - communication channel
    - bit-level bandwidth constraint
    - delay/dropout/asynchrony simulation
    - receiver selection
    - 如何将网络的限制输入到我们的通讯模型中
  - 实验验证部分：
    - 完整安全和效率指标
    - 标准的warehouse/logistic benchmark场景
    - ...
- 目前代码存在的问题：
  - `train_dagger.py --planner` 只允许 casadi
  - evaluator 把 expert 限制固定为casadi
  - DAgger只能捕获CasADi的异常类型
  - db-LaCAM 当前注明只支持SingleIntegrator和Unicycle1；
  - db-LaCAM 要求dt=0.1
  - db-LaCAM 配置包含异构机器人，但是MultiRobotSimulator 拒绝异构fleet



# System and Communication Model
- Robot Model
  - RoboChief 的真实外接圆直径暂定约 $0.50\,\mathrm{m}$（physical radius $0.25\,\mathrm{m}$）；规划、仿真碰撞检查和数据验证采用膨胀后的 effective collision radius $0.30\,\mathrm{m}$。
  - 两个 homogeneous robots 的默认中心安全距离为 $d_{safe}=2r_{collision}=0.60\,\mathrm{m}$。这表示每个机器人相对约 $0.25\,\mathrm{m}$ 的真实半径保留约 $0.05\,\mathrm{m}$ 径向 margin；真实平台尺寸确认后应重新校准。
  - Stage 1：使用 `single_integrator` 和上述 $0.30\,\mathrm{m}$ disk footprint 完成整个研究主干：db-LaCAM adapter/benchmark、轨迹重放、scenario/data pipeline、no-communication BC/DAgger、perfect/learned/bandwidth-constrained communication 和完整 evaluation。后续动力学不能阻塞该阶段。
  - Stage 2：仅在 Stage 1 闭环和主要结论成立后，接入 `double_integrator` 与匹配的 dynobench model/motion primitives，并用相同 footprint 复现实验流程，验证二阶连续动力学下的方法表现。
  - Stage 3：实现由三个固定间隔 120 度 omniwheel 驱动的 disk robot（RoboChief）的完整二阶全向模型，并继续使用经真实测量校准后的同一 footprint
  - robochief 的候选状态为 $[x,y,\theta,v_x,v_y,\omega]$，候选控制为 $[a_x,a_y,\alpha]$；最终形式需要结合真实平台参数和 db-LaCAM/dynobench model 确认
  - Stage 2/3 都只有在对应 dynobench model、motion primitives、时间步、动作边界和轨迹重放通过独立测试后，才进入各自动力学的 dataset generation 和 policy training；不能把 single-integrator primitives 强行复用于其他动力学。
  - Robot–obstacle collision 可以实现为半径 $0.30\,\mathrm{m}$ 的 disk 与原始 obstacle 相交，或等价地将 obstacle 膨胀 $0.30\,\mathrm{m}$ 后检查 robot center；只能选择一种方式，避免重复膨胀。
  - Collision threshold 与 near-collision metric 分开：默认 center distance $<0.60\,\mathrm{m}$ 判为 robot collision；near-collision threshold 初始可设为 $0.70\,\mathrm{m}$，后续结合速度、制动距离和定位误差校准。
  - 碰撞检测采用模块化父类/子类设计，例如统一的 `CollisionChecker` 接口。`DynobenchCollisionChecker` 复用 db-LaCAM/Dynobench 的 robot model、geometry 与 FCL collision checking，作为与 expert 一致的主要标准；`AnalyticDiskCollisionChecker` 独立实现 Stage 1 二维 disk–disk 与 disk–obstacle 检查，用于快速仿真、调试和交叉验证。
  - 不能只用 db-LaCAM 内部的同一段碰撞代码验证 db-LaCAM 输出，否则无法独立发现 wrapper 中的 robot ordering、$dt$、坐标转换、采样时刻或半径配置错误。两个 checker 必须共享同一份显式 geometry 配置，但实现路径相互独立，并在随机状态、运动线段和固定边界案例上做一致性测试。
  - 正式 rollout 的碰撞判断必须覆盖一个 control step 内的 swept motion，而非只检查离散时刻的端点。Stage 1 的 analytic checker 对匀速 disk motion 做连续时间最近距离检查；Dynobench checker 按其 motion/model 接口检查相同区间。若二者不一致，该样本进入 validation failure，不得进入训练集。
  - 若候选 action 在当前 control step 内产生 swept collision，simulator 不提交碰撞后的 next state，而是保留最后一个安全状态并立即终止该 joint rollout。事件需记录 collision type、相关 robot/obstacle、当前 timestep，以及 checker 可提供时的预计 first-contact time；evaluation 将该 episode 计为 collision failure。

- Local Observation
  - 目前定义我们的observation为：$o_i^t = [goal_relative_state, ego state, local obstacle observation, visible robot observations, visible obstacle observations]$
  - 无通信时，policy 通过本地传感器只能获得感知半径内邻居的相对位置，不能直接获得邻居的世界坐标。系统可以假设每个 robot 在共享静态地图中估计自己的 world pose，但其他 robots 的 pose 仍需本地感知或后续计费的 P2P pose exchange 获得。
  - 无通信时不能直接感知邻居速度、邻居目标或邻居规划意图。这些信息属于其他机器人的私有状态，只能由 learned message 间接传递，从而保证 communication baseline 相比 no-communication baseline 确实增加了可用信息。
  - 后续可做 `relative position only` 与 `relative position + relative velocity sensing` 的 observation ablation，但第一阶段固定为只直接感知邻居相对位置。
  - 第一阶段默认 robot sensing radius 与 obstacle sensing radius 均为 $2.0\,\mathrm{m}$；该半径约为 4 个真实 robot diameter，并在 $8\times8\,\mathrm{m}$ workspace 中保持局部性。
  - Sensing radius 是显式配置，至少评估 $R_{sense}\in\{1.5,2.0,3.0\}\,\mathrm{m}$，并记录每种设置下每个 robot 的 mean/percentile/max visible-neighbor count、visible-obstacle count，以及“看到全部 fleet”的 timestep fraction。
  - $2.0\,\mathrm{m}$ 是初始工作值而非预设最优值。如果统计显示交互前长期不可见，或大多数 timestep 已近似全局可见，应在正式 dataset generation 前调整并记录依据。
  - 邻居ID应当不可见，这样的话我们可以用deepset把机器人/obstacle看作一致的物品
  - P2P channel 和 simulator 内部保留 sender/receiver ID，用于路由、message buffer、per-link bandwidth 统计和调试；固定数字 ID 或 robot-specific ID embedding 不作为 neural policy 输入。
  - Receiver 将收到的 message 视为无序集合，并可结合本地可获得的相对几何、该链路的局部质量指标、message age 和 valid mask 进行聚合，以保持 homogeneous fleet 的 permutation invariance。
  - 第一阶段的静态障碍物使用局部 axis-aligned bounding-box set，而不是 image/occupancy grid/point cloud。对 robot $i$ 和 obstacle $k$，元素表示为 $b_{ik}=[\Delta x_{ik},\Delta y_{ik},h_x,h_y]$，分别表示障碍物中心的相对位置和半长/半宽。
  - 只暴露与 sensing region 相交的 obstacles，并由独立 obstacle DeepSet 加 mask 聚合；db-LaCAM、simulator collision checker 和 observation builder 必须引用同一份 map geometry。Obstacle count 的可变 batching 采用与 robot neighbor set 一致的思路，不把集合排列或固定 obstacle ID 当作 policy 信息。
  - 由于同一 episode 内 visible-obstacle count 会逐帧变化、而 LeRobot schema 需要固定 shape，第一版采用工程性容量限制 `max_visible_obstacles=8`。不足时 padding 并 mask；超过时按 robot 到 obstacle boundary 的距离保留最近 8 个，不能按 obstacle center 距离代替边界距离。
  - 必须记录每个 split/scenario family 的 obstacle truncation rate 和截断数量分布。初始预警条件为截断 frames 超过约 1%–5%，或干预实验显示被截断障碍影响动作/安全；触发后再评估提高容量、按可见数量分组 batching 或改用 occupancy/point-cloud encoder。
  - `max_visible_obstacles=8` 明确标注为第一阶段的工程折中，不是方法的理论限制。至少保留 $\{4,6,8,12\}$ 的容量 ablation；全局地图障碍物总数不受此值限制。
  - 未来可以在保持上层 policy 接口的前提下增加 occupancy map、LiDAR 或 point-cloud encoder，但不作为第一阶段输入。
  - 保证在我们的observation的construction里面不会让各个robot在大部分时刻都能看到所有的neighbor，不然的话communication会失去意义
  - obstacles 可以用bounding box来建模，同样包含ego position和effective size

- Action：
  - Stage 1 first-order `single_integrator` 的 action 为 planar velocity $u_i=[v_x,v_y]$，默认 control/planner timestep $dt=0.1\,\mathrm{s}$，与当前 db-LaCAM primitives 要求一致。
  - Stage 1 默认 $v_{max}=1.0\,\mathrm{m/s}$，因此一个 control step 的最大标称位移约为 $0.1\,\mathrm{m}$；action normalization、primitive validation、simulator clipping 和 evaluation 必须引用同一速度边界。
  - 速度敏感性测试保留 $v_{max}\in\{0.5,1.0,1.5\}\,\mathrm{m/s}$，但不要求与所有网络/通信 ablations 做完整笛卡尔积；主结果固定 $1.0\,\mathrm{m/s}$。
  - 如果是second order dynamics model 就用acceleration
  - 还有一个最后最后的方法是输出primitives

- Task Success:
  - Stage 1 的正式 goal tolerance 为 $r_{goal}=0.05\,\mathrm{m}$。只有所有机器人同时满足 $\|p_i-p_i^{goal}\|_2\leq0.05\,\mathrm{m}$，并连续保持 3 个 control steps，episode 才判为 success；避免高速掠过目标被算作成功。
  - 单个机器人连续 3 个 control steps 满足 goal tolerance 后进入 absorbing `goal_reached` 状态：其位置固定、执行 action 强制为零，但机器人不能从环境中消失，仍占据空间并参与 robot/obstacle collision checking，也仍可被其他机器人感知并按当前通信阶段参与通信。只有全部机器人都进入 `goal_reached` 后才判定 joint success。
  - 在尚未满足连续 3 步确认条件时，机器人仍按正常 policy/expert action 更新；进入 absorbing state 后 simulator 的零动作约束优先于 policy 输出。db-LaCAM trajectory replay、training rollout 和 evaluation 必须使用相同的 at-goal/hold 语义。
  - `single_integrator` 不额外要求终点速度；进入 `double_integrator` 阶段后必须增加终点速度阈值。
  - db-LaCAM goal threshold、raw-trajectory validator 和 learned-policy evaluator 应尽量统一为 $0.05\,\mathrm{m}$。如果 planner 内部受 motion primitives/discretization 限制只能使用不同阈值，则同时报告 planner-native success 和统一 strict evaluator success，不能混用。
  - 在正式生成数据前，用 standalone benchmark 检查 $dt=0.1\,\mathrm{s}$、现有 motion primitives 和 $v_{max}=1.0\,\mathrm{m/s}$ 能否稳定进入并保持在 0.05 m goal region。
  - Controlled environments 默认 `max_episode_time=30.0 s`，在 $dt=0.1\,\mathrm{s}$ 下对应 `max_episode_steps=300`；达到上限仍未满足全部 goal conditions 时记为 rollout timeout/failure。
  - Planner wall-clock solve timeout 与 30 s simulated episode horizon 是两个不同指标，分别配置和统计，不能用同一个 `timeout` 字段混淆。
  - Corridor/warehouse 等确实需要更长执行时间的场景可以显式配置 60 s 或其他 horizon，但必须按场景预先冻结并报告；同一 benchmark 内的不同方法使用相同 horizon。
  - 同时报告 success rate、time-to-goal/episode steps 和 timeout rate，避免仅通过延长 rollout horizon 提高成功率。
  - collsion happens

- Communication
  - Communication Process:
    - 假设我们可以通过某种方式提供当前各个机器人之间的通信限制，作为我们通讯模块的输入
    - 仿真器/channel 内部可以集中维护完整的真实链路矩阵，用于模拟、约束执行和评估；但不能把完整矩阵输入每个机器人。Robot $i$ 的 scheduler 只接收自己本地可获得的 outgoing-link 状态（或协议允许的 incoming-link metadata），不需要一个中心节点向其广播整个 fleet 的网络质量。
    - 这种实现属于 centralized simulation of decentralized information：全局网络状态只存在于环境内部，每个 policy 的决策仍只依赖自己的局部观测、局部链路信息和已经收到的消息。
    - 对 robot $i$，网络接口只暴露其本地可用的 outgoing-link 切片，例如候选 receiver $j$、可行性 $A_{ij}^t$、本地链路质量估计 $q_{ij}^t$ 和自身总发送预算 $B_i^t$；不能暴露其他 sender 的链路状态或完整的 $A^t/C^t$ 矩阵。
    - 参数共享的 scheduler 在每个 sender 上独立运行，可写为 $g_{ij}^t=f_{schedule}(o_i^t,q_{ij}^t,B_i^t)$，并且只能从本地可行 receiver 中进行选择。核心方法不依赖全局最优分配器或中央 scheduler。
    - 为了分析 decentralized receiver selection 与全局最优选择之间的差距，可以额外实现一个拥有全局链路状态的 oracle/centralized scheduler，但它只作为 upper-bound baseline，不能计入 proposed distributed method。
    - 对每条候选有向边 $(i,j)$，message encoder 根据 sender $i$ 的本地/私有信息、receiver $j$ 相对 sender 的局部几何以及链路状态生成 receiver-conditioned message：$z_{ij}^t=f_{msg}(o_i^t,r_{ij}^t,q_{ij}^t)$。不同 receiver 可以从同一 sender 收到不同的协调意图。
    - 通过scheduler生成一个各个link之间的是否允许通讯的indicator map
    - 然后通过channel发送到各个机器人，在simulation中我们此处需要根据之前我们生成的通讯限制对发送出的消息进行处理
    - 各个Robot 聚合收到的信息 $M_i^t$，并结合自身的state输出action
  - Two-stage position/message protocol:
    - Stage 1（local-link proof of concept）：令 $A_{ij}^t=S_{ij}^t$。Sender $i$ 与 receiver $j$ 都从各自的局部 observation/virtual range sensor 获得对方相对位置，因此 learned payload $z_{ij}^t$ 不重复编码位置。Channel 内部使用 ID 将收到的 message 与对应 local neighbor slot 关联，但 ID 不作为 neural feature；receiver DeepSet element 为 $[r_{ji}^t,z_{ij}^t,q_{ij}^t,age_{ij}^t]$。
    - Stage 1 的完整因果顺序为：构造 local observations/sensing mask -> 为每条可感知链路生成 receiver-conditioned message -> immediate channel -> 按内部 sender ID 与 receiver 的 relative-position observation 对齐 -> DeepSet aggregation -> local action。
    - Stage 2（network-selected remote P2P）：$A_{ij}^t$ 由网络条件产生且不要求等于 $S_{ij}^t$。网络接口先向 sender 暴露可连接 peer addresses 和本地 link quality，但不广播整个 fleet 的 positions；scheduler 先选择少量 P2P links。
    - 只有在 $g_{ij}^t=1$ 的选中链路上，双方点对点交换带 address/timestamp 的 world pose，并写入各自的 local peer cache；未选中 links 不产生 pose traffic。Pose、header 和 link-setup overhead 均计入 bandwidth。
    - 首次建立 link 默认需要一个 control step：时刻 $t$ 完成选择和 P2P pose exchange，最早在 $t+1$ 使用缓存的相对几何生成 receiver-conditioned $z_{ij}^{t+1}$。不在一个 control step 内隐式执行未计时/未计费的多轮 handshake。
    - Peer cache 至少保存 last pose、timestamp/age 和 link quality。超过可配置 `max_pose_age` 后不得把旧 pose 当作当前真值；需要重新选择/交换 pose，或将该 peer 标为 geometry unavailable。
    - Stage 2 的典型顺序为：network discovery -> scheduler selects links -> selected-link pose exchange/cache update -> one-step wait -> receiver-conditioned payload transmission -> aggregation/action。后续 asynchronous channel 可以把每一步的实际 arrival time 显式建模。
    - Stage 2 scheduler 暂定拆成两级：`pose_query_gate` 根据 sender local state、可见的 peer address/link quality、cache validity 和发送预算，选择少量未知/过期 peers 进行计费的 P2P pose query/exchange；`message_gate` 只对已有有效 pose cache 的 peers 决定 learned payload transmission。
    - 未建立 pose cache 的 remote peer 不能先免费暴露位置再供 scheduler 选择。Pose-query exploration、response、header 和 payload 分别计费并记录；首次 query 后仍遵循至少一个 control-step 的建立延迟。
    - 两级 scheduler 是当前暂定设计，需要与 mentors 讨论后冻结；代码接口预留独立 query/message decisions，但该实现不阻塞 Stage 1 local-link learned communication。
    - Stage 1 依赖传感器相对位置而不计 pose bandwidth；Stage 2 依赖选中 P2P 链路上的 pose metadata并计费。二者都不要求中心节点实时广播 fleet positions。
  - Shared static map assumption:
    - 所有 robots 可以在 episode/deployment 前获得同一静态地图和共享世界坐标系，并具备自己的 world-pose estimate；共享静态地图不包含其他 robots 的实时位置、目标、expert trajectory 或全局链路矩阵。
    - Stage 1 只把共享地图用于场景定义、坐标一致性、碰撞几何和定位，不把 global map embedding 输入 policy。Action policy 仍主要使用 goal-relative、neighbor-relative 和 local-obstacle features。
    - Global `map_context` encoder 仅作为后续增强/ablation；若加入，需要与 local-only policy 分开比较，不能仍声称 policy 只使用局部静态环境信息。
  - Message Content:
    - Communication Phase 1 不使用 message encoder，而是直接传输完整 private-information vector（single-integrator 初始定义为 sender goal-relative vector、previous executed action 和由 sender local branch 产生的 preliminary/nominal action）。该向量不包含 expert action、第三方 observation、绝对 robot ID 或动态全局 fleet state。
    - 核心设计采用 receiver-conditioned continuous latent vector $z_{ij}^t$，而不是将同一个 $z_i^t$ 广播给所有 receiver。
    - 所有有向边共享同一个 message encoder 参数，以适用于 homogeneous fleet 和不同机器人数量；receiver identity 通过相对几何/局部上下文表达，不使用固定数字 ID embedding。
    - 第一阶段即实现个性化消息；公共 broadcast message 可作为降低编码开销的 ablation baseline，而不是 proposed method 的默认设置。
  - Message Length:
    - Bandwidth 建模采用三个阶段，避免把 continuous latent dimension 错误等同于真实 bit 数。
    - Communication Phase 1（information-value validation）：直接传输完整 private information，不经过 learned compression，不限制 message dimension/receiver budget，不量化、不加入 bandwidth penalty，并使用 perfect synchronous channel。该阶段只回答额外信息是否提升 local policy，不讨论 bandwidth。
    - Communication Phase 2（learned-message validation）：用端到端 learned continuous $z_{ij}$ 替代完整信息，但仍使用 perfect channel，无量化、receiver budget、delay 或 dropout。初始使用较宽 `message_dim=16`，降低“表示容量不足”对 learned-message 可训练性结论的干扰；此处 dimension 只是 latent width，不称为真实 bit bandwidth。
    - Communication Phase 3（representation/quantization）：不提前假设 4-dimensional/32-bit message 足够。先固定较宽 latent 隔离 quantization effect，再逐级缩小 dimension；测试 `message_dim` $\in\{1,2,4,8,16\}$，其中 4 是目标压缩点而非保证可用的默认结论。
    - Communication Phase 4（network constraints）：在量化 payload 上加入 per-link capacity、sender aggregate budget、receiver/pose-query selection、delay、dropout 和 asynchronous delivery。
    - Phase 3 fixed quantization 中，每个 message element 量化为固定的 `bits_per_element`，一条消息的 payload 定义为 `message_dim * bits_per_element` bits；需要选择可训练的量化近似并在部署时使用硬量化。
    - 第一轮 fixed quantization 使用 encoder `tanh` 将每个 latent element 限制到 $[-1,1]$，再做固定范围 uniform 8-bit quantization，避免每条 message 额外传输动态 scale。训练使用 fake quantization/straight-through gradient，部署/evaluation 使用相同 codebook 的 hard quantization。
    - 推荐的逐级配置为：`dim=16,float32`（512-bit learned-payload reference）-> `dim=16,int8`（128 bits）-> `dim=8,int8`（64 bits）-> `dim=4,int8`（32 bits）-> `dim=4,int4`（16 bits）。先比较 float32/int8 区分量化误差，再比较 dimensions 区分表示容量。
    - 若 4-dimensional message 明显损害主要指标，则允许 8-dimensional/64-bit payload 成为 proposed method 主配置；选择依据必须来自 validation，不根据 final test 结果更改。
    - Learned payload bits 与 address、timestamp、pose、link setup 等 protocol overhead 分开统计，同时报告 payload-only 和 total transmitted bits；不得用 32-bit payload 代替完整 packet size。
    - Phase 4 network capacity 中，外部网络模型为每条有向链路提供 capacity $C_{ij}^t$，并为每个 sender 提供 aggregate budget $B_i^t$；learned scheduler 必须在可行链路和总发送预算内选择 receiver。
    - 每个 $z_{ij}^t$ 都是独立的链路 payload，必须分别计费。完整通信负载按所有实际发送边的 payload bits 求和，并区分 attempted bits、transmitted bits 和 delivered bits。消息即使最终丢失，已使用的发送资源仍计入 transmitted cost。
    - Phase 4 中，一个 control step 的链路预算可以写为 $C_{ij}^t\Delta t$ bits/step；同时满足单链路容量约束和 sender aggregate budget。
  - 通讯对象：
    - 感知图 $G_{sense}^t$ 与物理可通信图 $G_{link}^t$ 分别建模，并在代码接口中使用独立的 sensing mask 和 link mask，不能把“当前看得见”和“当前能通信”写死为同一个概念。
    - 通信拓扑采用分阶段实现。中间验证版本使用 perfect all-to-all peer-to-peer communication，所有机器人均可互相发送消息，不限制 receiver、带宽、delay 或 packet loss，用于验证额外通信信息是否确实改善协调性能。
    - 最终版本由外部网络模型在每个时刻提供 feasible directed link graph $A_{ij}^t$，表示物理网络条件下 sender $i$ 是否能够向 receiver $j$ 通信。
    - 第一版 network-constrained 实验采用 $G_{comm}^t \subseteq G_{sense}^t$：scheduler 只能选择当前也能被本地感知的通信对象，因此 receiver 可用传感器获得的相对位置关联消息，learned message 主要补充速度、目标和意图等私有信息。这是阶段性假设，不是架构硬限制。
    - 接口必须允许后续使用 $G_{comm}^t \nsubseteq G_{sense}^t$。对于“可通信但不可感知”的远端 sender，需要明确提供可关联消息与发送者的路由/几何 metadata，或者把必要空间信息编码进 payload，并明确这些 metadata 是否计入通信预算。
    - Learned scheduler 只能在 $A_{ij}^t=1$ 的候选链路中选择实际接收者，输出通信选择 $g_{ij}^t$；最终发送关系为 $A_{ij}^t g_{ij}^t$。
    - 后续可以通过 GNN、attention score 或其他 scheduler 建模通信必要性，并在 bandwidth budget 下选择 top-K 或超过阈值的 receiver；第一版 receiver selection 从简单、可验证的结构开始。
  - 时间特性：
    - 第一阶段采用同步即时通信：每个 control step 中，所有机器人先并行生成当前 message，经 channel 传递和聚合后，再用当前 local observation 与收到的 message 输出 action。
    - 第二阶段加入随机 delay 和 asynchronous update；机器人使用 channel 在当前时刻实际交付的消息，消息可能来自更早的 timestep。
    - 即时和延迟模型必须通过统一的 communication channel 抽象调用，避免 rollout 或 policy 针对不同 channel 编写不同控制流。

- Communication Channel Interface
  - 新增抽象基类，例如 `CommunicationChannel`，定义所有通信模型必须实现的统一接口；具体 channel 通过子类继承实现。
  - 建议最小接口包含：
    - `reset(...)`：在新 episode 开始时清空 message queue、历史状态和统计量。
    - `exchange(outgoing_messages, feasible_links, link_metadata, timestep)`：接收当前待发送消息和网络状态，返回当前时刻实际交付的消息、delivery mask 及必要 metadata。
    - `get_metrics()`：返回 transmitted bits、message count、drop rate、message age 等通信统计。
  - channel API 分别接收 sensing mask 与 feasible-link mask；具体 channel 可以在第一阶段取二者交集，但抽象基类不应强制二者相等。
  - `ImmediateCommunicationChannel`：`exchange()` 立即返回当前 timestep 的可行消息，用于 perfect synchronous communication 和早期 learned-message 实验。
  - `BufferedCommunicationChannel`：把 outgoing message 加入带 arrival time 的队列，并在 `exchange()` 中返回当前 timestep 已到达的历史消息，用于 delay、dropout 和 asynchronous communication。
  - 上层调用保持统一：`local observation -> sender -> channel.exchange() -> receiver aggregation -> action head`。更换 channel 子类时，不修改 policy/rollout 的主体流程。
  - 第一阶段的 channel 和 sender/receiver 路径应保留 PyTorch tensor 与计算图，不应无意使用 NumPy、序列化或 `detach()` 阻断 action loss 对 sender 的梯度。延迟跨 timestep 时的梯度传播范围需要在异步阶段单独定义并测试。

- Network Model Interface
  - 新增抽象 `NetworkModel`，与 `CommunicationChannel` 分离：network model 根据 robot poses/map/time 生成物理链路真值和各 robot 可获得的局部 link estimates；channel 根据 scheduler decisions 和这些约束执行排队、传输、delay/drop并计费。Policy/rollout 只依赖统一接口。
  - 建议接口包括 `reset(...)`、`observe_links(robot_id, timestep)`、`step(joint_state, timestep)` 和 `get_ground_truth_metrics()`；`observe_links(i)` 只能返回 robot $i$ 本地允许知道的 candidate addresses、outgoing link quality/capacity estimates，不能返回完整 fleet matrix。
  - `IdealNetwork`：所有 P2P links 可用，固定无限/足够大 capacity，无 delay/drop，用于通信 architecture 的 sanity/oracle experiments。
  - `DistanceBasedNetwork`：根据 robot distance 产生 feasibility、link quality 和 capacity；作为第一版 bandwidth-constrained network，参数简单、可解释且可独立单元测试。
  - `DistanceBasedNetwork` 默认 communication radius 为 $R_{comm}=4.0\,\mathrm{m}$，而 sensing radius 为 $R_{sense}=2.0\,\mathrm{m}$：$d\leq2$ m 的 peer 可感知且可通信，$2<d\leq4$ m 的 peer 不可感知但物理链路可用，$d>4$ m 时 $A_{ij}=0$。
  - Communication-radius ablation 使用 $R_{comm}\in\{2,4,6,\infty\}\,\mathrm{m}$；$\infty$ 只表示 all-to-all oracle。各设置报告 network graph degree/connectivity 和 sensed/unsensed feasible-link proportions，避免只按名义半径解释结果。
  - `ObstacleAwareNetwork`：在 distance model 上增加墙体/货架穿越或 line-of-sight attenuation；在基础 distance model 成立后加入。
  - `StochasticNetwork`：加入 time-varying capacity、packet loss 和随机 delay，并使用固定 seeds/reproducible traces；在确定性 network experiments 后加入。
  - `TraceDrivenNetwork`：未来回放真实测量或外部网络模拟 trace，不作为 Stage 1/首个 network-constrained result 的依赖。
  - 所有 network subclasses 对上层返回相同结构；网络模型负责“物理上能否/以何种代价传输”，learned scheduler 负责“是否值得发送”，二者不能合并为一个不可解释模块。



# Proposed Method/Architecture
```
Local observation
      ↓
Sender/message encoder
      ↓
Bandwidth bottleneck / quantizer
      ↓
Communication channel
      ↓
Receiver/message aggregator
      ↓
Action policy head
      ↓
Local robot action
      ↓
Safety Filter/CBF
```
- 我们不需要observation encoder，因为message encoder应该足够可以自己决定应该包含什么information(比如此时位置不重要，只有速度重要等等)，同时起到压缩消息的作用
- action head可以看到自己发送的消息
- 第一版 receiver aggregator 使用 DeepSet。对 robot $j$，每条输入 element 由 receiver-conditioned message $z_{ij}^t$、sender 相对 receiver 的局部几何 $r_{ji}^t$、链路质量、message age 和 valid mask 组成；先逐元素编码，再通过 permutation-invariant pooling 和输出网络得到聚合表示 $M_j^t$。
- DeepSet 必须支持可变数量的发送者以及零消息输入；padding element 必须在 pooling 前通过 mask 排除，不能影响聚合结果。
- 后续增加 attention/Transformer aggregator，研究显式消息重要性加权是否优于 DeepSet；它们作为增强版本和 ablation，不阻塞第一版端到端通信系统。
- sender/receiver ID 只作为 channel 内部路由字段，不输入 neural policy。第一阶段中 receiver 通过本地感知得到 sender 的相对位置，并用它关联对应 message；未来对感知范围外通信再单独定义必要的定位/寻址 metadata 及其 bandwidth overhead。
- communication可以是多轮也可以单轮，理想状态是单轮
- message先不考虑recurrent



# Learning Objective
我们的 idea 的核心要求是不显式监督 message，而是用 imitation learning 的 action supervision 指导 message 的 encoding 和 transmission。Message 没有 ground-truth target 或单独的 supervised message loss，但 action imitation loss 应通过 receiver、可微 channel/bottleneck 和 sender 反向传播，从而端到端学习通信内容。
所以我们的Loss应该包含：
- imitation loss：$\mathcal{L}_{action} = \|\pi_\theta(o_i, M_i) - u_i^E\|^2$ 或者 $\mathcal{L}_{flow}$
- Bandwidth penalty: $\mathcal{L} = \mathcal{L}_{action} + \lambda_B\mathcal{L}_{bandwidth}$
- Receiver Sparsity: $\mathcal{L}_{sparsity} = \lambda_R\sum_{ij}g_{ij}$
- Temporal consistency（仅在同一有向边连续存在时）: $\lambda_T\|z_{ij}^t - z_{ij}^{t-1}\|^2$
- Message entropy或信息瓶颈
我们一项一项验证加入，不要一下用上所有的。



# Baselines
- Centralized expert：db-LaCAM本身
- Primary no-communication ablation：当前 DeepSet/local policy 使用与 proposed policy 相同的 db-LaCAM data、observation、MLP/Flow backbone、training budget 和 evaluator，仅关闭 learned messages；这是验证 communication contribution 的首要公平基线。
- GLAS comparison 分阶段处理：核心方法 proof-of-concept 成立前，不让完整 GLAS 复现阻塞 expert/data/no-communication/communication 主线。之后先独立运行官方 single-integrator example，再实现并明确标注差异的 `GLAS-style reimplementation`；只有足够忠实复现其 observation、policy、safety module 和 training protocol 时才简称 GLAS。
- 原始 GLAS 论文数值只能作为背景，不能与我们不同 expert、地图、robot footprint 和训练预算下的数值直接横向比较。早期与 mentors 确认最终论文所要求的 GLAS fidelity；若要求完整复现，将它与 Safety Filter 阶段作为独立工作包安排。
- Handcrafted communication: 直接发送所有的信息不经过任何encoding
- Unlimited/full-state communication: 不限制带宽，假设ideal communication
- Perfect all-to-all learned communication: 所有 peer-to-peer 链路均可用，不限制带宽、delay 和 packet loss，但 message content 由网络端到端学习；用于验证 learned communication architecture 本身是否有效
- Network-constrained learned communication: 外部网络模型提供 feasible link graph，learned scheduler 在可行链路内选择 receiver，并满足 bandwidth constraint；作为最终 proposed method



# Implementation Phases
- Empty-first implementation order
  - Stage 1A 先只使用 empty $8\times8\,\mathrm{m}$ maps 和 2–4 robot swap/crossing/circle/random-permutation scenarios，跑通 `db-LaCAM -> raw archive/manifest -> replay validation -> per-robot LeRobot datasets -> MLP/Flow smoke -> no-communication BC/DAgger -> evaluation`。
  - Stage 1A 不实现 obstacle encoder，也不让随机障碍生成阻塞数据/训练闭环；但 simulator/map API 保留 obstacles 字段和后续扩展接口。
  - Stage 1B 在 Stage 1A gate 通过后加入 axis-aligned bounding-box observation、obstacle DeepSet、10%/20% random clutter 和相应 collision/reachability validation。
  - Stage 1C 再加入 corridor、bottleneck、alcove/at-goal、maze、warehouse aisle 和 MovingAI-derived continuous layouts。每阶段复用相同 scenario/archive/dataset/evaluator 接口，不为特定地图另写训练主流程。

- Core method proof-of-concept gates
  - Gate 1 — no-communication imitation works：single-integrator MLP 与 Flow 通过 smoke/overfit gate，并在未见过的简单 closed-loop scenarios 上取得非偶然成功率，且优于 random-action 和只朝目标运动的简单 controller。未通过时优先检查 expert data、observation sufficiency 和 covariate shift，不进入通信实现。
  - Gate 2a — local-link perfect private information is useful：第一阶段令 communication feasibility mask 等于 sensing mask，即 $A_{ij}^t=S_{ij}^t$。在完全相同的 data split、backbone 和 evaluation 下，仅向当前可感知邻居发送未压缩 private information；它应在 crossing、swap、bottleneck 等交互场景优于 no-communication。
  - Gate 2b — all-to-all oracle（可选 upper bound）：令 $A_{ij}=1,\forall i\neq j$ 并发送相同定义的完整 private information，用来估计理想网络下的性能上限。它不是最终 decentralized/network-constrained method，也不能替代 Gate 2a。
  - Gate 3 — learned message is used：先保持与 Gate 2a 完全相同的 $A=S$ topology，以端到端 learned receiver-conditioned messages 替代完整 private information；性能应优于 no-communication。同时执行 message-zeroing、message-shuffling、wrong-recipient permutation 和 message-disable tests，干预后性能应下降。
  - Message-content comparison 固定 $A_{ij}$ 和 scheduler；topology/scheduler comparison 固定 message representation。不能在同一个对照中同时改变消息内容、可通信对象和预算，否则无法归因性能变化。
  - 最终 network-constrained 阶段由网络模型独立产生 $A_{ij}^t$，不再要求 $A=S$；scheduler 输出 $g_{ij}^t$，实际发送边为 $e_{ij}^t=A_{ij}^t g_{ij}^t$。分别统计 sensed-and-connected、sensed-but-disconnected、unsensed-but-connected 和 feasible-but-not-selected links。
  - 三个 gates 通过后，才把大量时间投入完整 GLAS reproduction、bit-level bandwidth、复杂 scheduler 和 asynchronous channel。第一轮前不任意写死“提升 10%”等阈值；先用预注册场景得到 effect size/置信区间，再冻结后续正式 gate threshold。

- db-LaCAM Expert Validation Phase
  - Modular planner interface
    - 保留并扩展现有 `planning/planner.py` 中的抽象 `Planner` 父类，不让 BC、DAgger 或 evaluator 直接依赖 `DbLacamPlanner`/`CasadiPlanner` 的具体实现。
    - `CasadiPlanner`、`DbLacamPlanner` 作为该父类的子类，并由 `PlannerFactory` 按配置创建；未来的 db-ECBS 或其他 expert 也通过相同接口接入。
    - 父类接口区分两种用途：`plan_episode(initial_joint_state, goals, environment, *, time_limit_s)` 返回供 offline dataset 使用的完整 joint plan；`query_action(current_joint_state, goals, environment, *, horizon, time_limit_s)` 从 learner 当前 joint state 规划并返回供 DAgger 使用的下一步或 action chunk。Planner API 直接接收显式 global/joint state，不再要求 planner 从 decentralized observation 反解世界状态。
    - 两个入口统一返回结构化 `ExpertPlanResult`，至少包含 `success`、states、actions、`dt`、robot ordering、planning wall time、termination/failure reason 和 planner-specific metadata，避免不同 planner 用隐式 shape 或特殊字段传递结果。
    - `ExpertPlanResult` 的标准轨迹表示采用 per-robot tuple/list，而不是强制全 fleet 使用同一 state/action dimension：`states[i]` 为 `[T_i+1,nx_i]`，`actions[i]` 为 `[T_i,nu_i]`，并保存 `valid_lengths` 与共享 timestamps/time-grid 语义，为未来 heterogeneous robots 保留空间。
    - 为 Stage 1 homogeneous fleet 提供唯一的 `stack_homogeneous()` 辅助转换，输出 `[T+1,N,nx]` states 与 `[T,N,nu]` actions。较早完成的 robot 由 adapter 显式扩展为保持 goal state 和零 action，同时保留原始 `valid_lengths`；training loader 不得自行猜测轨迹长度或直接解析 db-LaCAM YAML。
    - 保留 `__call__(obs) -> action` 作为迁移期兼容入口，逐步迁移现有 CasADi/evaluation tests；新的 dataset generation、DAgger 和 validation 代码只依赖上述显式接口。
    - 正常的 no-solution、timeout 或 planner-invalid-result 通过结构化 failure result 表达；文件损坏、schema violation 和 programming error 等真正异常才抛统一的 planner-agnostic exception。上层代码不能只捕获 CasADi 异常。
    - `DbLacamPlanner` 不得像当前兼容实现一样在 expert failure 时返回零 joint action，因为这会把失败伪装成可训练标签；collector/evaluator 必须收到明确失败并按各自规则终止 rollout和记录事件。
    - `reset()`、episode lifecycle、open-loop plan cache 和 replanning 状态由统一契约定义；具体子类只负责各自输入转换、求解和输出解析。
    - 第一版 `DbLacamPlanner` 保持 subprocess/YAML adapter，不立即开发 Python/C++ binding。每次调用使用隔离的 temporary work directory，写入 problem/algorithm YAML，读取 result/stats，并捕获 command、return code、stdout、stderr 和 wall time；并行 workers 不得共享临时文件名或可写 planner state。
    - db-LaCAM 内部 `-t` time limit 与 Python subprocess 外部 timeout 同时启用，外部 timeout 仅增加明确的小幅启动/文件 I/O grace period。return-code failure、external timeout、missing output、malformed schema、planner no-solution 和 replay-invalid 必须映射为可区分的 termination/failure reasons。
    - Offline generation 将 problem、algorithm、result、stats 和 validation 复制进 raw archive；正常 DAgger query 只保留精简结构化记录，失败时额外保存足够诊断信息，避免每个 control step 都产生庞大 archive。
    - 测量并分别记录 process startup/YAML I/O 与 planner solve time。只有该固定开销在逐步 DAgger 中成为实测显著瓶颈，才考虑 Python/C++ binding；binding 必须保持相同 `Planner`/`ExpertPlanResult` 上层契约。
  - Layer 1 — upstream/official smoke test
    - 先运行 db-LaCAM 自带的官方 example/config，确认依赖、可执行文件、输入输出路径和基本求解流程正确；该层不修改训练代码，也不用于声称算法性能。
  - Layer 2 — thesis fixed benchmark
    - 使用固定配置、公开记录的随机种子和可复现 scenario files，测试 2/4/8/16 个 `single_integrator` robots；初始规模只用于验证趋势，不要求立即达到论文中的大规模结果。
    - 场景至少覆盖 random、swap、crossing、corridor 和 bottleneck，并保留不可解或 timeout case 来验证失败处理。
    - 记录 solve success rate、planning wall time、trajectory duration、collision/dynamic feasibility、action bounds、action variation/jerk，以及这些指标随 robot count 的变化。
    - 先报告 db-LaCAM 在 $\{1,5,30,60\}$ s wall-clock budgets 下的 coverage/time-to-solution curve；正式 offline dataset generation 初始使用 30 s/instance，DAgger 使用 20 s/query。预算可依据 smoke result 调整，但同一比较中的方法必须使用预先冻结且明确报告的预算。
  - Scenario smoke cases
    - 单机器人无障碍
    - 两机器人交换位置
    - 三个机器人交叉交换位置
    - 狭窄通道
    - 多个机器人从圆上出发互相交换位置
    - 狭窄通道
    - bug trap 一个机器人进一个机器人出
    - 不可解或超时任务
  - Validation Test
    - 能够运行
    - input YAML 正确
    - result YAML 正确
    - action shape 正确
    - robot ordering 正确
    - dt 一致
    - action语义一致
    - Expert action label 默认直接使用 db-LaCAM 输出的原始 control sequence；validator 用该 control 和本项目 dynamics 从 $x_t$ 重放到 $x_{t+1}$，检查与 planner state trajectory 的误差。`(x_{t+1}-x_t)/dt` 等 state-difference action 只作为显式诊断比较，不能静默替代缺失或无效的原始 action。
    - 若输出缺少 actions、state/action 长度关系错误或 control 语义无法确认，该结果标记为 schema/adapter failure，不进入训练集。该规则即使在 Stage 1 single-integrator 中也保持不变，以免掩盖 timestep、clipping、primitive sampling 或 hold-padding 错误，并为后续二阶动力学保持一致语义。
    - 离散端点与 step 内 swept collision checking 的语义明确且一致
    - Dynobench 与独立 analytic checker 在 Stage 1 基准案例上结果一致
    - open-loop/replan 行为正确
    - 规划失败能够被统一处理
    - db-LaCAM 返回 success 只是 planner-level success；还必须在本项目 simulator 中逐步重放轨迹，重新检查 robot ordering、dt、状态/动作语义、动力学误差、碰撞和终点误差，二者均通过才可进入 dataset generation。
  - 第一项验收标准：官方 smoke test 可重复运行，固定 benchmark 能输出完整指标，而且成功轨迹能够被本项目 simulator 正确重放并通过独立可行性检查。

- Dataset Generation Plan
  - 总体原则：不重写当前数据系统。继续使用现有 LeRobot Dataset 作为 MLP/Flow 的训练格式，在它之外补充 raw planner archive 和 joint-episode manifest。三层数据承担不同职责，不能互相替代。
  - 当前 codebase 的行为：`MultiRobotSimulator.format_dataset_frame()` 在每个 timestep 为每个 robot 生成一条 decentralized frame；`collect_dagger_rollouts()` 按 robot 分别缓存这些 frames，并把一次 $N$-robot joint rollout 保存为 $N$ 个 LeRobot episodes。训练 loader 再利用 LeRobot 自动提供的 `index`/`episode_index` 动态读取同一 robot 的未来 actions，构造 action chunk。
  - No-communication training 可以继续按单个 robot frame 采样；learned-communication training 必须新增模块化 `JointTimestepDataset`（或等价 joint loader），通过 `joint_episode_id + robot_index + timestep` 同步取回同一 rollout、同一时刻的所有 robot observations、expert action chunks、active/goal masks 和通信图。这样 sender $i$ 生成的 $z_{ij}$ 才能通过 receiver $j$ 的 imitation loss 获得正确梯度。
  - Joint loader 只重组现有 per-robot LeRobot episodes，不复制保存第二份 observation/action 数据。Manifest 维护 `joint_episode_id -> {robot_index: lerobot_episode_index}` 的最小映射，并由 loader 检查 timestep 对齐、robot ordering 和 action-chunk validity。
  - 延续按 robot count 分开的 `dataset_n2` ... `dataset_n8` 方案：单个 joint batch 只来自同一个 $N$，张量自然采用 `[batch,N,...]`，不通过固定 `max_robots` 把不同 $N$ 强行混入同一 batch。训练 sampler 在不同 $N$ 的 loaders 之间切换；`active_mask`/`goal_reached_mask` 表示运行状态而不是为最大 fleet size 做 padding。

  - Layer 1 — Raw planner archive（复现 db-LaCAM 原始结果）
    - 每次 centralized planning/rollout 分配唯一 `joint_episode_id`，并保存 db-LaCAM 原始输入、原始输出和验证结果。建议的概念目录如下（具体扩展名可根据 db-LaCAM 实际输出调整）：
      ```text
      data/raw/joint_episode_000007/
      ├── problem.yaml       # map、所有 robots 的 start/goal 和模型
      ├── algorithm.yaml     # db-LaCAM 参数与 time limit
      ├── result.yaml        # 原始 joint states/actions
      └── validation.json    # replay、collision、dynamics、goal 等检查结果
      ```
    - Raw archive 不直接作为 PyTorch batch，也不能被 refinement 或重新导出覆盖。它回答的是“这个 expert sample 从哪里来、能否重现”。
    - Raw episode 只保存复现该次求解直接需要的 problem/result/validation 和简短 summary；共享的 planner config、db-LaCAM/dynobench commits、dt、motion-primitives checksum 等只在 dataset-level `dataset_metadata.yaml` 记录一次，不在每个 episode 重复。

  - Layer 2 — Joint-episode manifest（关联和管理）
    - Manifest 只是一张供人查看的精简索引表，不输入 neural policy，也不承载完整配置、硬件、checksum 或长错误日志。它只负责把 centralized rollout、raw path 和拆分后的 per-robot LeRobot episodes 关联起来。
    - 例如 3 个 robot 的 `joint_episode_id=7` 可以对应：
      ```json
      {
        "joint_episode_id": 7,
        "scenario_id": "crossing_seed_42",
        "split": "train",
        "status": "expert_valid",
        "raw_path": "raw/joint_episode_000007",
        "lerobot_episodes": {"robot_0": 10, "robot_1": 11, "robot_2": 12}
      }
      ```
    - Planner timeout/无解等失败 episode 即使没有训练 frames，也只在 manifest 保留 `status` 和 raw/log reference。具体 failure type、timestep、runtime 和诊断信息写入独立 `failures.jsonl` 或 raw episode summary，避免污染主索引。
    - `robot_index` 和内部 ID 只用于关联、路由和分析，不作为 shared policy 输入特征。
    - Dataset-level `dataset_metadata.yaml` 保存一次共享 provenance/config；run-level `run_config.yaml`/TensorBoard 保存训练超参数与硬件；episode manifest 保存最小关联字段。三类信息不得无理由重复。

  - Layer 3 — LeRobot training dataset（高效训练）
    - 继续沿用现有每机器人一个 local episode 的设计。以 3 个 robots 的一次 joint rollout 为例，可能保存为 LeRobot episode 10/11/12，分别包含 robot 0/1/2 的连续 local frames；三者通过 manifest 指回同一个 `joint_episode_id=7`。
    - 每个训练 frame 至少包含 local observation、neighbor visibility/mask、expert local action，以及 LeRobot 的 episode/timestep/index 信息。可额外保存 `joint_episode_id`、`robot_index` 和 `scenario_id` 作为非 policy metadata，或只在 manifest 中维护映射。
    - Policy dataloader 只选择明确列入 observation schema 的字段；global state、IDs、expert joint plan 和 scenario metadata 即使被保存，也不能因方便而拼入 decentralized policy input。
    - Action chunk 保持在 loader 中动态构造，因此改变 `prediction_horizon` 不需要重新运行 db-LaCAM。需要修改当前“重复最后 action”的 padding，实现 `action_valid_mask`，使不足 $H$ 的 padding steps 不参与 loss。
    - DAgger 使用 LeRobot `create()/resume()` 或等价 shard append 追加新成功 episodes，不覆盖原始 BC 数据；每轮 aggregation、collector checkpoint 和新增 episode 范围记录在 run-level aggregation log，manifest 只增加对应 episode mapping/status。

  - 推荐的数据生成顺序
    1. 为当前 scenario 分配 `joint_episode_id`，先写入 pending manifest record。
    2. 调用 db-LaCAM，并无论成功或失败都保存输入、输出/错误和 runtime。
    3. 在本项目 simulator 中 replay；只有 planner success 且 validation success 的轨迹可以转换为训练 frames。
    4. 将有效 joint trajectory 按 robot 拆成 LeRobot episodes，并把 episode indices 回填到 manifest。
    5. 数据集划分依据 `scenario_id/joint_episode_id` 完成，再由 loader 生成 frames/action chunks；禁止先打散 frames 再划分。

  - Scenario Bank 与 expert labeling 分离
    - Phase A 先生成并冻结 scenario bank，不在生成过程中根据 db-LaCAM 的求解结果选择保留哪些正常 benchmark instances。每个 instance 先获得稳定的 `scenario_id`、seed、environment family、robot count 和 split。
    - Scenario generation pre-check 包含：start/goal 不与按 collision radius 膨胀后的 obstacles 相交；任意两 starts 和任意两 goals 满足 $d_{safe}$；地图/通道几何满足该场景的参数约束；忽略其他 robots 时，每个 robot 均存在静态可行路径。
    - 未通过 pre-check 的样本标记为 `invalid_generation`，用于监控生成器质量，但不进入 expert coverage 的分母。单机器人静态可达不等于 multi-robot joint feasible，计划和论文中必须明确该区别。
    - Phase B 对冻结的、pre-check-valid scenario bank 运行 db-LaCAM。结果区分 `expert_valid`（求解成功且 replay 验证通过）、`expert_failed`（timeout/no solution/planner error）和 `expert_invalid_trajectory`（返回结果未通过独立验证）。所有结果均写入 manifest。
    - Training dataset 只能由 `expert_valid` trajectories 构造，因为其他 instances 没有可靠监督标签；但 validation/test benchmark 不能事后只保留 db-LaCAM 成功样本。
    - 正式报告 expert coverage：$N_{expert\_valid}/N_{precheck\_valid}$；另外在 expert-valid 子集报告 student-to-expert performance，并分别报告 student-only success、expert-only success、both-success 和 both-failure，暴露 survivorship/selection bias。
    - db-LaCAM 找到解只能说明 instance 在指定 time limit、primitives 和配置下是 `db-LaCAM-solvable`，不能据此宣称所有失败实例在数学上不可解。已知不可解 scenarios 另建 failure-handling test set，不混入正常 success-rate benchmark。
    - 额外建立小规模 `guaranteed_feasible_diagnostic` set：先构造并保存一条满足动力学、action bounds、robot/obstacle collision 和终点条件的 joint trajectory，再用其首尾生成 start/goal，因此每个 instance 都附带 feasibility certificate trajectory。
    - Guaranteed-feasible set 只用于诊断 db-LaCAM adapter、motion primitives、time limit、replay validator 和 failure handling。如果 db-LaCAM 失败，记录为“在给定配置/time limit 下未找回已知可行解”，但仍不能推导一般性的 planner completeness 结论。
    - 由于逆向/构造式场景可能偏简单或带生成偏差，该集合必须与 random/warehouse/bottleneck 主 benchmark 分开报告，不能用于替代主测试集或单独支撑 policy performance claim。

  - 第一版 no-communication BC 使用通过重放验证的 raw trajectory，先建立不受平滑方法影响的基线。
  - 如果 expert jerk 明显影响模仿或控制质量，再由 raw trajectory 生成单独的 refined dataset。refinement 方法及参数写入 metadata，并重新检查动力学可行性、碰撞、终点误差和 action bounds。
  - 评估中分别报告 raw db-LaCAM expert、refined expert（若使用）和 learned policy；不能把 refined trajectory 的性能作为原始 db-LaCAM 性能，也不能在未标注的情况下混合 raw/refined samples。
  - 每个frame至少包含以下信息：
    - local observation
    - expert local action
    - neighbor visibility
    - episode index
    - timestep
  - 下列信息保存在 raw archive/manifest 或标记为非 policy metadata，不要求在每个 frame 中重复：
    - scenario metadata
    - global state(仅供训练分析使用)
    - robot IDs
    - expert joint plan
    - collision margins 
  - 数据拆分：
    - 必须以 `joint_episode_id/scenario_id` 为最小单位划分，而不是随机拆分 frames，防止同一轨迹的相邻时刻或同一 joint rollout 的不同 robots 同时出现在 train 与 test 中。
    - In-distribution 数据使用相同的场景生成规则和 robot-count 范围、但互不重叠的 scenario seeds，第一版比例为 70% train、15% validation、15% test。
    - OOD test 独立于上述 70/15/15，至少按下列因素分别建立集合：更多 robots、更高空间/冲突密度、unseen map、unseen start-goal pattern，以及更窄的 corridor/bottleneck。不要把所有分布偏移只混成一个总分。
    - 在训练前生成并冻结 `split_manifest`，记录每个 split 的 `joint_episode_id/scenario_id`；同一次 joint rollout 拆出的所有 per-robot LeRobot episodes 必须属于同一个 split。
    - Raw、refined 和后续 DAgger 数据都必须保留原始 scenario lineage，并依据同一 split 规则管理，禁止后处理或重新导出导致跨 split 泄漏。
    - DAgger 只能在已登记的 train scenarios，或从同一训练分布新采样并预先登记为 train 的 seeds 上聚合数据。Validation、test 和 OOD scenarios 始终只读，不允许查询其 expert labels 后追加回训练集。
    - Validation 可用于 early stopping、checkpoint selection 和超参数选择；test/OOD 只用于方案冻结后的最终评估，避免根据测试结果反复修改模型。
    - 每个 DAgger episode 记录 `source=dagger`、aggregation iteration、collector policy checkpoint、scenario lineage 和 expert-mixing 配置。新场景必须在 rollout 前确定 split，不能生成结果后再选择归属。
  - 数据质量检验：
    - expert success rate
    - solver timeout
    - collision
    - action bounds
    - episode length
    - state distribution
    - robot count distribution
    - 是否包含足够多真实冲突的场景
  - Dataset scaling strategy
    - 不在 pipeline 验证前一次性生成大规模 expert 数据。Level 0 使用 1–3 个 joint episodes 检查 raw archive、manifest、replay、LeRobot conversion 和 loader；Level 1 每个当前 smoke scenario/robot count 约 20 episodes，用于 MLP/Flow overfit 和初步 closed-loop evaluation。
    - 之后按约 100、500、2000 个 joint episodes 的规模逐级生成/训练并绘制 learning curve；每级复用冻结的 generation config 与 split rule，检查 expert coverage、数据质量、validation performance 和训练/存储成本后，再决定是否继续扩大。
    - 正式 dataset size 不在获得 learning curve 前写死。停止扩展的依据包括 validation improvement 接近饱和、关键场景覆盖不足、expert generation 成本或资源预算；最终论文报告实际 joint episodes、per-robot episodes 和 frames，不能只报 frames。
  - Temporal sampling
    - Stage 1 raw expert trajectory、LeRobot frames、policy control 和 evaluation 统一使用 $dt=0.1\,\mathrm{s}$ / 10 Hz，不对 db-LaCAM trajectory 降采样，也不通过插值构造缺失的中间 expert actions。
    - 默认 `prediction_horizon=5` 因而覆盖 0.5 s 的未来 action window，但 closed-loop 仍每 0.1 s 重观测并只执行 chunk 第一步。Dataset metadata 记录 `fps=10`，loader 必须验证 frame timing/episode boundary。
    - Temporal subsampling 或不同 control rates 仅作为后续独立实验；若加入，需要重新定义 action semantics、horizon duration 和 dynamics replay，不能只跳过 frames。
  - Robot-count ranges
    - Smoke tests 使用 2/3/4 robots；第一阶段正式 train 混合 2–8 robots；in-distribution validation/test 同样覆盖 2–8 robots 但使用未见过的 scenario seeds。
    - Fleet-size OOD test 使用 12 和 16 robots；若 db-LaCAM runtime 和计算资源允许，再扩展至 24/32 robots，但不把该扩展作为第一阶段阻塞条件。
    - 分别报告每个 robot count 的结果，不能只报告由小规模简单场景主导的 aggregate average；同时报告各 split 的 robot-count distribution。
  - Variable fleet-size representation
    - 采用按 robot count 分开的 LeRobot datasets，例如 `dataset_n2/ ... dataset_n8/`。每个 dataset 内 feature schema 固定，每个 minibatch 只包含同一 robot count，因此可以继续使用标准 tensor stacking；不同 robot-count batches 共同更新同一个 shared policy。
    - 不设置全局 `max_robots=16` 作为 policy 输入上限。训练器通过 multi-dataset sampler/loader 在不同 robot-count datasets 之间切换；12/16 robot OOD evaluation 使用对应 shape 的 simulator/dataset view，并加载同一套共享 DeepSet/policy 权重。
    - DeepSet 的逐元素网络 $\phi$、pooling 和 $\rho$ 在数学上不依赖集合大小。代码需要把 encoder 的参数形状与 `neighbor_slots=num_robots-1` 解耦，例如显式传入固定的 ego feature dimension，并在 forward 中从当前 tensor/mask 推断 neighbor count。
    - DeepSet 在 pooling 前应用 visibility mask；需要增加跨 neighbor count 的 checkpoint-load test、零邻居测试和 permutation test，验证改变有效邻居数量或排列不会造成 shape failure，且排列不改变输出（允许数值误差）。
    - 当前 LeRobot 固定 schema 仍意味着不同 robot counts 不能直接混入同一个 minibatch；这是 storage/batching 限制，不是 policy 的理论 fleet-size 上限。未来如有需要可实现 ragged batching 或 batch-local padding，但不作为第一阶段要求。
    - Multi-dataset training 默认先对 $N\in\{2,\ldots,8\}$ 均匀采样 robot count，再从 `dataset_nN` 中采一个 minibatch，而不是把所有 per-robot frames 合并后均匀采样。否则较大 fleet 因每个 joint rollout 产生更多 robot episodes，会天然主导训练。
    - 每轮记录各 robot count 的 sampled batches、frames、joint episodes 和 loss/evaluation 指标。后续可以根据难度、失败率或 curriculum 调整 count weights，但权重必须显式配置和报告。
    - 在 robot count 之下再按 scenario family 分层采样：先选择 $N$，再选择 swap/crossing/circle/random 等当前阶段已启用的 family，最后从对应 dataset/shard 取 minibatch。默认近似均匀，避免容易生成且轨迹较长的 random cases 淹没关键协调场景。
    - Stage 1B/1C 加入 clutter/corridor/bottleneck/warehouse 后扩展相同 sampler；所有 family weights 显式配置，记录实际 batches/frames，并按 family 分别报告 validation 指标。Difficulty reweighting 只能根据 train/validation 结果调整。
  - Normalization
    - 优先使用固定、可解释的物理 bounds 做缩放，例如相对位置除以 sensing radius、速度除以 $v_{max}$、动作除以 $u_{max}$。这样新加入 DAgger 数据时不会改变已有 policy 的输入/输出语义。
    - 对没有合理固定 bounds 的连续字段，才使用初始 train split 计算 mean/std；validation、test 和 OOD 只能复用 train statistics，禁止各自重新拟合。
    - DAgger 默认冻结 normalization。若未来确需更新，必须产生新的 normalization version，并明确迁移/重新训练策略以及每个 checkpoint 对应的版本。
    - `neighbor_mask`、valid mask、IDs、episode index 和其他离散 metadata 不进行普通 mean/std normalization；IDs/episode metadata 也不作为 policy features。
    - 不同物理语义的字段分别缩放，不能为了方便把位置、速度和动作拼在一起计算一个全局统计量。
    - 所有 bounds、mean/std、字段顺序和 normalization version 必须随 checkpoint 保存；evaluation 从 checkpoint 加载同一份参数，不能临时从评估数据计算。
- Training Stages：
  - 首先训练一个Offline BC，这个情况下我们只建立local observation，系统没有communication
    - 利用db-LaCAM offline收集的数据训练一个flow policy
    - 先用 deterministic MLP 在极小数据集上完成快速 pipeline smoke test/overfit test，验证 observation、normalization、expert action chunk、loss、checkpoint 和 closed-loop evaluation 接口。该测试是短期质量门槛，不要求先把 MLP 完整训练收敛。
    - MLP smoke test 通过后，同步开展完整 MLP baseline 与 Flow 主模型训练，二者使用相同的数据 split、observation、action horizon 和 evaluation scenarios。Flow 是预期的最终 policy，MLP 用于诊断、性能/计算成本对照和论文 baseline，不能成为阻塞 Flow 实验的串行前置任务。
    - Offline training pipeline 的正式训练 gate：先在少量固定 episodes 上分别完成 MLP 和 Flow 的 overfit test；训练 loss 应明显下降，action chunk shape/padding mask 正确，反归一化动作满足系统边界，checkpoint 保存和重载后的固定输入输出一致，固定 seed 的 evaluation 可复现，并至少能在训练过的简单场景中完成 closed-loop rollout。
    - 具体 loss/成功率阈值在检查动作量纲、normalization 和初始 smoke result 后写入配置与实验记录；在此之前不随意设一个缺乏依据的数值阈值。
    - policy 输出一个 expert action chunk，但闭环 rollout 只执行预测的第一步，然后重新观测。第一版默认 `prediction_horizon=5`，以降低 policy 输出维度、flow training 难度和显存需求；该参数保持可配置，并保留 $H=1,3,10$ 作为 ablation。
    - Action chunk 长度通常只影响 student 的标签/输出规模，不等同于 db-LaCAM 的全局 planning horizon，因此不能声称减小 $H$ 会直接降低 expert 找到完整可行路径的难度。
    - BC 和 DAgger 都从当前状态对应的 expert plan 截取最多 $H$ 步作为标签。若剩余轨迹不足 $H$，使用 padding 和 loss mask；padding action 不参与 loss，不能简单重复最后一个动作充当有效标签。
    - Dataset frame 显式保存 `goal_reached_mask`。机器人在进入 absorbing `goal_reached` 前（包括连续 3 步的到达确认期）仍正常产生监督样本；进入 absorbing state 后，其状态继续保留供其他机器人构造 observation/message，但默认不再将其后续零动作帧计入 action-policy loss，避免长时间等待产生的大量零标签主导训练。
    - Simulator 仍强制 absorbing robot 执行零动作，因此上述 loss mask 不改变 rollout dynamics。若后续发现 policy 缺少到达后停车能力，再通过少量受控采样或单独的 stop/hold auxiliary loss 训练，不能无上限地复制等待帧。
    - DAgger 在下一 control step 从新的 learner joint state 重新查询 expert；上一次 plan 中的后续 action 只属于上一个训练 chunk，不能直接当作新状态下的当前 expert action。
    - Flow inference 第一版在每个 robot/control step 只生成一个 action chunk，并只执行其第一步；不生成多个 candidates 后按最终效果选择 best-of-N。Policy sampling seed 由预先固定的 scenario/training-seed/rollout-repeat 规则产生，不能因 rollout 失败而重抽 seed。
    - Final evaluation 对 Flow 使用预先冻结的 sampling repeats/seeds 来量化生成随机性；MLP 使用相同 scenarios。多候选 sampling + local critic/ranker 仅作为后续独立增强，若实现需单独报告 selector、sample count 和 inference cost。
  - No-communication DAgger
    - 将db-LaCAM接入之前的训练，作为DAgger可以访问的expert
    - 按标准 DAgger 聚合 learner 实际访问状态上的 expert labels：rollout 即使后来因 learner action 碰撞，碰撞前所有 `expert_query_success=true` 且标签通过验证的 transitions 仍可进入训练集；不能因为 episode 最终失败而整体删除这些 correction samples。
    - Dataset 保存/日志需区分 `expert_action`、`executed_action` 和 `executed_action_source`。实际 learner action 不得误写为监督 label；碰撞后的状态不生成样本。
    - Learner/mixed action 被 swept checker 判定将在本 step 碰撞时，不提交碰撞后的状态并立即终止 joint rollout。由于碰撞检测前的当前状态仍是 learner 实际访问的安全状态，如果该状态上的 expert query 与 label validation 成功，则其 `(current_state, expert_action)` correction sample 仍可保留；被拒绝的 learner collision action 不作为监督 label，也不创建 next-state transition。完整 collision event 必须进入 episode metadata/evaluation metrics。
    - 若在无 execution noise 时实际执行 expert action 仍导致碰撞，则标为 `expert_execution_collision` 并隔离该 episode，优先检查 planner/simulator dynamics、dt、robot ordering 和 collision-check mismatch，而不能当作正常 DAgger 数据。
    - 利用 DAgger 进行训练时，应及时检测 db-LaCAM 是否能够从 learner 当前 joint state 求解。若 expert timeout、报告无解、输入状态无效或返回轨迹未通过验证，则立即终止当前 rollout，避免 learner 继续进入更异常的状态。
    - Expert query 失败的当前 transition 及后续状态均不得生成伪造、随机或 fallback expert label，也不得加入监督训练集。
    - 失败事件必须保留在独立 failure/aggregation log 中，至少包含 failure type、发生 timestep、planner runtime、当前 scenario ID 和可用诊断信息；训练报告和 evaluation 分别统计 expert-query failure rate，不能静默丢弃失败 episode，也不能把长诊断堆入主 manifest。
    - 数据加载器只从 `expert_query_success=true` 的 transition 构造监督样本，但 dataset summary 必须同时读取完整 episode log，以暴露因 planner failure 导致的 selection bias。
    - 第一版小规模 DAgger 在每个 control step 都从 learner 当前 joint state 重新查询 expert，以保证监督标签与实际访问状态匹配；同时记录每次 query latency 和累计 expert planning cost，作为正确性与成本基线。
    - 第一版 DAgger 的单次 db-LaCAM query wall-clock timeout 为 20 s，而不是 5 s；timeout 后遵循前述规则立即终止 rollout并记录失败。该值在 standalone latency benchmark 后可以显式调整，但所有实验必须记录实际配置。
    - 除 per-query latency 外，记录每个 rollout 的累计 expert planning time、query count、median/p95/max query latency，评估逐步 replanning 的真实数据收集成本。
    - 只有在逐步查询的计算成本不可接受时，才测试每 $K$ 步 replanning 并在中间步骤复用缓存计划。`replan_freq` 必须是显式配置和实验参数，不能作为 `DbLacamPlanner` 内部不可见的固定行为。
    - 使用缓存计划时，每一步比较当前 joint state 与缓存轨迹对应状态的偏差；偏差超过明确阈值时立即使 cache 失效并重新规划。阈值定义、触发次数和因此产生的额外 queries 必须记录。
    - Rollout 支持三种显式模式：`expert_only` 用于调试 expert/data pipeline；`mixed` 用于早期 DAgger；`learner_only` 用于后期 DAgger 和最终 policy evaluation。
    - `mixed` 模式在第 $k$ 轮以概率 $\beta_k$ 执行 expert action、以 $1-\beta_k$ 执行 learner action；无论实际执行哪一个，都保存当前访问状态对应的有效 expert label。初始方案采用可配置的几何衰减 $\beta_k=\beta_0\gamma^k$，具体参数由小规模实验确定。
    - 第一版 mixing 在每个 control step 对完整 joint action 只抽样一次：选择 expert 时所有尚未到达的 robots 执行同一份 expert joint action，选择 learner 时则全部执行 learner joint action；absorbing robots 始终执行零动作。暂不对每个 robot 独立抽样 expert/learner，以免组合出 expert 未规划的混合 joint action并人为制造协调冲突。
    - 不论该 step 实际执行 expert 还是 learner joint action，每个尚未到达机器人仍以当前 joint state 对应的有效 expert action 作为监督标签。Per-agent mixing 仅保留为基础 DAgger 稳定后的独立鲁棒性实验，并与 joint-level mixing 分开报告。
    - DAgger 训练过程从 `mixed` 逐步过渡到 `learner_only`。每轮必须记录配置的 $\beta_k$、实际执行的 expert/learner action 比例，以及因 expert failure、collision 或 timeout 提前结束的 rollout 比例。
    - 第一版主 DAgger 固定 `action_noise_std=0`，只研究 expert/learner mixing。标准管线稳定后再加入显式标记 noise level/seed 的 noisy-recovery 数据和 execution-noise robustness evaluation；不得将 noisy 与 clean data 无标记混合。
    - 目标是能够学习到一个成功率接近db-LaCAM的local policy，略低是可以接受的，因为我们的policy是decentralized的
  - Communication Test
    - Learned-communication policy 默认从已经通过 gate 的 no-communication checkpoint warm-start：复用兼容的 local observation encoders 和 action-policy weights，新建 sender encoder、receiver/message aggregator 与 fusion layers。通信分支采用近零影响初始化（例如 residual fusion 的最后一层置零或小权重），使刚接入通信时的行为近似原 no-communication policy，而不是立即破坏已有控制能力。
    - Warm-start 后不长期冻结原 action network；使用同一 expert action imitation objective 对 sender、channel-compatible message path、receiver/fusion 和 action policy 端到端联合微调，使无直接 message labels 的通信表示能够通过最终 action loss 学习。可以仅用很短的 staged unfreezing 处理数值稳定性，但必须显式配置和记录。
    - 额外保留结构、数据和训练预算相同的 from-scratch communication model 作为初始化消融，以区分通信收益、no-communication pretraining 收益和优化稳定性差异；主方法预期采用 warm-start。
    - Phase 1 假设通信网络完美，不存在 bandwidth、delay、dropout、quantization 或 receiver-budget constraint；直接传输完整 local/private information 并聚合到 policy 输入，检验通信是否提升表现。主比较先使用 $A=S$，perfect all-to-all 作为额外 oracle topology。
    - Phase 2 保持与对应 Phase 1 实验完全相同的 perfect topology/channel，但用较宽的端到端 learned continuous message 替代完整信息，验证 sender/receiver architecture；不得同时改变 topology 归因 learned-message 效果。
    - Phase 3 才研究 message dimension 和 fixed quantization；`message_dim=4` 是该压缩阶段的默认候选，不是 Phase 1 的约束。
    - Phase 4 才加入 network-derived feasible links、receiver/pose-query selection、bandwidth budget、delay/dropout 和 asynchronous delivery。
    - 单个 control step 的统一通信时序为：在时刻 $t$ 构造所有 local observations；每个 sender 针对候选 receiver 生成 outgoing messages；channel 执行一次传递；receiver 聚合本轮已经到达的 messages；policy 输出并执行 $a^t$；simulator 再推进到 $t+1$。每步只进行一轮消息传递，不在同一 control step 内迭代多轮通信。
    - Phase 1–3 的 ideal synchronous channel 允许由 $o^t$ 生成的零延迟 message 影响同一时刻的 $a^t$。Phase 4/5 加入 delay/asynchrony 后，$a^t$ 只能使用截至决策时刻已经到达的消息；尚未到达的本轮消息留在 channel buffer，不能提前读取。
    - 每条 buffered message 保存 generation timestep/timestamp，并向 receiver 提供可归一化的 `message_age`；zero-delay 与 delayed channel 使用相同上层返回接口。过期策略和最大缓存年龄作为后续网络阶段的显式配置，在进入该阶段时再细化。
    - 目标是验证通讯可以提升规划的成功率，这样才能进一步确认通过正确的压缩减少通讯仍然能够提升规划成功率
  - Design communication module
    - 外部网络模型提供当前机器人之间的 feasible directed links、bandwidth limit 和估计 delay 等链路质量指标
    - learned scheduler 只在 feasible link graph 内决定当前哪些机器人之间应该通信
    - 通过另外一个网络决定在当前的网络带宽限制下应该如何压缩/浓缩/选择目前的local observation传输给目标接收机器人。
    - 按照之前的whom和what进行一轮消息传递
    - 结合消息，可以用decoder解码消息，并且接入policy输入
    - 先定义统一的 `CommunicationChannel` 抽象，再分别实现即时 channel 和带 message buffer 的 delay/asynchronous channel；两者对上层暴露相同的调用方法和返回结构
    - Bandwidth 实现顺序为：continuous message dimension -> fixed bit quantization -> per-link capacity 与 per-sender aggregate budget 下的 learned receiver selection
    - Scheduler 训练方案暂定为：先在 perfect all-to-all 条件下预训练 sender/receiver/action policy，再引入 continuous soft gates 和 bandwidth penalty，最后使用满足预算的 hard top-K/binary selection 并联合微调。hard selection 的梯度估计可比较 straight-through estimator 与 Gumbel-Softmax。
    - 上述 scheduler 方案是当前工作假设，并非不可修改的设计决定；应在 db-LaCAM、dataset、no-communication BC/DAgger 管线完成后，再根据实验结果复核。
  - Safety Filter
    - 用safety filter来对policy输出的action来进行过滤，保证action永远安全，这一步更大程度上属于工程实现，保证机器人在现实中可以安全执行

- 当前实施优先级：`db-LaCAM standalone benchmark -> expert adapter/trajectory replay -> dataset generation -> no-communication BC -> no-communication DAgger -> perfect communication -> learned/bandwidth-constrained communication -> asynchronous channel -> safety filter`。通信架构的预先设计不能阻塞前面的 expert 和无通信基线工作。
- GLAS 官方代码复现/移植放在上述主方法 proof-of-concept 通过之后；但同框架 communication-disabled ablation 属于主线且必须同步保留，因为没有它就无法判断 communication 的实际贡献。


# Evaluation Scenarios
所有环境与 scenario 均以 human-readable YAML 作为外部定义格式，并在加载后转换成统一、只读的 `EnvironmentSpec`/`ScenarioSpec`。Planner adapter、simulator、collision checker、observation builder 和 visualizer 只能消费该内存对象或其标准转换，不能各自从不同配置重新构造地图。

- YAML 分为两类：`configs/scenarios/templates/` 保存便于人工修改的 family/template 配置（workspace、obstacle pattern、robot count range、生成规则等）；`data/scenarios/{split}/` 保存生成后冻结的具体 instance，显式包含 bounds、obstacles、每个 robot 的 start/goal、seed、family 和 schema version。正式 dataset/evaluation 只能引用冻结 instance，不能在运行时从 template 随机重生成。
- 配置职责进一步分为三类且不交叉：scenario YAML 只定义问题（bounds、obstacles、robot model/geometry reference、ordered starts/goals）；planner YAML 只定义求解方法（planner name、motion primitives、search 参数和 solve budget）；experiment YAML 只引用 scenario set、planner config、dataset/policy/training config 和 run seed。Scenario 文件不得嵌入 db-LaCAM 专属搜索参数。
- 同一个冻结 scenario 必须能够不经内容修改地交给 db-LaCAM、CasADi 或未来 planner。每次 raw planner archive 保存实际 scenario 与 planner configuration snapshot/引用，从而可复现“哪个 solver 用什么参数求解了哪个问题”，但主 manifest 仍保持此前约定的最小字段。
- 文件名与 scenario 内容对应并保持简洁可读，例如 `empty_swap__n04__seed000123.yaml`、`clutter10_random__n08__seed000417.yaml`。`scenario_id` 默认与不含扩展名的文件名一致；若重名则生成阶段直接报错，不能静默覆盖。完整参数留在 YAML 内，不把所有参数或长 checksum 堆进文件名/manifest。
- `EnvironmentSpec` 至少包含 `bounds_min`、`bounds_max`、`obstacles` 和 `environment_id`；`ScenarioSpec` 在其上增加 `scenario_id`、robot model/geometry reference、ordered starts/goals、family、seed、split 和 schema version。第一阶段 obstacle schema 采用 axis-aligned rectangle 的 center/half-extents；未来 polygon、occupancy 或 3D geometry 通过扩展 obstacle type 接入。
- YAML loader 使用 Python frozen dataclasses 与集中的 `from_dict()`/`to_dict()`，PyYAML 只负责序列化。仅在文件加载边界执行一次必要的 fail-fast validation：必需字段、基础 shape、finite numeric values、robot count/order、正尺寸和 workspace bounds；内部模块接收已验证 spec，不再层层重复检查原始 dictionaries。
- 该 schema 层保持轻量，不引入 Pydantic、复杂自动修复、宽泛兼容逻辑或尚未需要的 migration framework；错误直接给出包含文件名与字段路径的清晰信息。只有实际出现第二个 schema version 时才实现迁移，不为假设需求提前增加防御性代码。
- `RobotGeometrySpec` 与 obstacle geometry 分开定义；effective collision radius 只能在 checker/planner model 的标准转换中应用一次，scenario YAML 不同时保存“原始障碍”和“已膨胀障碍”两套容易混淆的几何。

Scenarios 不能完全用随机起点，可以包含以下场景：
- empty/open-space family：不放置静态障碍物，用于隔离纯 multi-robot coordination；包含 pair swap、circle swap、crossing 和随机 start-goal permutation
- swap：两机器人交换位置
- crossing：多机器人交叉
- corridor：狭窄通道
- intersection：仓库十字路口
- bottleneck：单入口/出口；
- merge：多车道汇合；
- aisle passing：货架通道会车；
- dense random：高密度随机任务；
- pickup/drop-off：物流取放任务的简化形式

## Benchmark Environment Families
- 第一阶段 empty 和 GLAS-style random-clutter controlled environments 默认使用 $8\times8\,\mathrm{m}$ workspace，以便与 GLAS 的实验尺度对照；workspace size 必须是显式配置，不能写死在 simulator 中。
- Corridor、warehouse 和 MovingAI-derived maps 可以使用各自合理尺寸，但必须同时报告实际尺寸、robot radius、free-space area 和 sensing radius。更大 workspace 作为单独的 OOD factor，避免与 robot-density OOD 混在一起解释。
- 障碍物难度必须同时用 geometry density 和 topology 描述；不能只用 `sparse/dense` 标签，因为相同占用面积的独立小障碍和一条封闭瓶颈会产生完全不同的规划难度。
- 定义 obstacle area ratio：$\rho_{obs}=\mathrm{area}(\cup_k O_k)/\mathrm{area}(W)$。第一阶段采用 0%（empty）、10%（sparse）和 20%（dense）三个主等级，与 GLAS 的随机环境设置对齐；若障碍重叠，按 union area 计算，不能重复计数。
- Family A — Empty coordination：0% obstacles；pair swap、3+ robot crossing、circle swap、random permutation。用于判断失败究竟来自机器人协调还是静态避障。
- Family B — GLAS-style random clutter：在固定 workspace/grid cells 中随机放置 axis-aligned rectangles，分别生成 10% 和 20% occupancy；保留生成 seed，并检查 start/goal 位于 free space 且 instance 不是明显静态不可达。
- Family C — Topology stress：alcove/at-goal、single-lane corridor、bottleneck、maze、merge 和 warehouse aisle passing。单独记录 corridor width、bottleneck width、可并行通过的 robot 数量等拓扑参数。
- Family D — External map templates：选择 MovingAI 的 `empty`、`warehouse`、`room`、`random`、`maze` 类地图作为布局来源，将 occupied grid cells 合并/转换为连续 axis-aligned rectangles，并按 robot radius 做必要的 collision geometry 处理。转换后的实验必须称为 “MovingAI-derived continuous maps”，不能直接与离散 MAPF benchmark 数字比较。
- Family E — db-LaCAM native/reference instances：优先复用其官方 `alcove`、`atgoal`、circle、maze 和 random scalability problems，用于验证 wrapper 并与 db-LaCAM 报告的场景类型保持联系；只在动力学、primitive、参数和硬件差异明确记录后讨论数值可比性。
- Dynobench 用作 robot/environment schema、collision checking 和 motion-primitives 的主要技术参照，而不是强行要求所有 learning experiments 使用其全部 benchmark。
- 论文参照：GLAS（随机 $8\times8\,m$、10%/20% obstacles 和 robot/obstacle density evaluation）；db-LaCAM 官方 paper/repository（alcove、atgoal、circle、maze、random scalability）；MovingAI MAPF Benchmarks（标准 grid map families）；Dynobench（kinodynamic planning benchmark infrastructure）。

每个场景需要明确：
- robot count；
- map；
- start/goal generation；
- obstacle layout；
-最大 episode steps；
- test seeds


# Evaluation Metrics
- Statistical protocol and compute budget
  - 可用训练硬件包含一张 RTX A4000；它主要用于 MLP/Flow/DeepSet 训练与 policy inference。db-LaCAM 搜索主要受 CPU、单次 time limit 和并行进程资源影响，因此 expert generation/DAgger wall time 必须独立测量。
  - Smoke/debug：每个配置 10 个固定 scenarios、1 个 training seed，只用于接口和明显错误检查。
  - Development：每个配置约 50 个固定 scenarios、1–2 个 training seeds，用于选择结构、超参数和淘汰无效 ablations。
  - Final core results：每个实验条件 200 个冻结 scenarios、5 个独立 training seeds。核心条件至少包括 no communication、perfect private information、learned communication 和 bandwidth-constrained communication。
  - 大规模 ablation 初始使用 50–100 scenarios、3 个 training seeds；只有影响主要结论的关键配置再补到 200 scenarios、5 seeds，避免对所有超参数做完整笛卡尔积。
  - 不同方法使用完全相同的 evaluation scenario IDs/seeds，进行 paired comparison。报告各 training seeds 的分布和 paired effect；success/collision 等比例指标给出预先固定方法的 95% confidence interval，不能只展示单次最佳 seed。
  - 所有 run 保存 GPU/CPU 型号、软件版本、训练时长、peak GPU memory 和 expert wall time；A4000 是当前资源假设，不是方法运行的最低硬件要求。
- Task performance
  - success rate；
  - timeout rate；
  - makespan；
  - sum of costs；
  - path length；
  - time-to-goal。
- Safety
  - collision rate；
  - minimum pairwise distance；
  - minimum obstacle distance；
  - d_safe violation；
  - near-collision rate；
  - safety-filter intervention frequency。
- Imitation quality
  - action MSE；
  - action variation / jerk，用于衡量 db-LaCAM expert 轨迹和 learned policy 的控制平滑性；
  - trajectory deviation；
  - expert cost ratio；
  - performance gap to db-LaCAM。
  - 注意 action MSE 低不一定表示闭环性能好。
- Communication
  必须定义实际预算：
  - message dimension；
  - bits/message；
  - messages/second；
  - receivers/message；
  - total transmitted bits/episode；
  - dropped-message fraction；
  - average message age；
  - fraction of timesteps with communication。
- Computation
  - expert planning time；
  - policy inference time；
  - communication encoder time；
  - safety filter time；
  - real-time factor；
  - GPU/CPU hardware说明。



# Ablations
- 模块消融实验
  - no communication
  - learned communication
  - no limit communication
  - observation radius
  - receiver aggregation sum/mean/max/attention
  - dropout rate
  - delay
  - train robot count vs. test robot count
- 方法消融实验：
  - BC vs DAgger
  - MLP vs Flow
  - prediction horizon 1 vs 3 vs 5 vs 10
  - with/without safety filter



# 实施过程中需要注意的代码规则和工具偏好
- 目标模块边界为：`third_party/db-lacam/` 只放上游 submodule；`scenarios/` 放 specs、YAML I/O 与 generators；`planning/` 放抽象 planner、`ExpertPlanResult` 和具体 adapters；`collision/` 放抽象 checker 与 Dynobench/analytic implementations；`validation/` 放独立 trajectory replay；`learning/data/` 放 per-robot/joint loaders；`configs/` 按 scenarios/planners/experiments 分类；`docker/` 只负责构建运行环境。
- 上述目录按 vertical slice 实施到对应功能时再创建，不为尚未实现的远期阶段一次性生成大量空目录/占位类。第一轮只建立 db-LaCAM source/build 与 standalone validation 所需的 `third_party/`、`scenarios/`、`planning/`、`collision/`、`validation/` 和相关 configs/tests；communication/data 模块在其依赖 gate 通过后接入。
- db-LaCAM 采用“Git submodule 管理源码版本 + Docker 管理依赖、编译和运行”的组合方式，二者职责不同而非二选一。上游源码放在 `third_party/db-lacam/` recursive submodule 中并固定到经过验证的 commit；不得把第三方 C++ 源码复制进 `planning/`，也不得让正式镜像在每次 build 时无版本约束地 clone 上游 `main`。
- 第一次获取项目使用 `git clone --recurse-submodules <repo>`；已有 checkout 使用 `git submodule update --init --recursive`；用 `git submodule status --recursive` 检查 db-LaCAM、dynoplan/dynobench 等嵌套依赖的实际 commit。构建前应检测 submodule 是否完整，并在缺失时给出明确错误，而不是在 CMake 深处失败。
- 更新 db-LaCAM 必须显式进入 `third_party/db-lacam` fetch/checkout 到待验证 commit，再回到主仓库提交更新后的 gitlink；更新后重新运行 official smoke、adapter、trajectory replay 和 collision cross-check tests。若未来需要修改第三方源码，优先 fork db-LaCAM 并让 submodule 指向该 fork，不能把临时容器内修改当作正式实现。
- 统一 `research-gpu` Docker build 从 `third_party/db-lacam` 的固定源码构建并把 executable 安装/保存在 `/opt/db-lacam/buildRelease/run_dblacam`（实际安装位置可在实现时统一，但必须由配置显式提供并经 smoke test 检查）。宿主机不需要安装 OMPL、FCL、Dynobench 等 C++ dependencies，源码仍可直接在 IDE 中阅读。
- 固定调用链为：dataset generator/DAgger → `PlannerFactory` → `DbLacamPlanner.plan_episode()`/`query_action()` → `ScenarioSpec` 到 db-LaCAM YAML adapter → `/opt/db-lacam/buildRelease/run_dblacam` subprocess → db-LaCAM/Dynoplan/Dynobench/FCL → result/stats parser → `ExpertPlanResult` → 独立 `TrajectoryValidator` → validated dataset。Training、DAgger 和 dataset 模块不得直接依赖或解析 db-LaCAM 内部 C++/YAML 结构。
- 正式实验统一使用 Docker Compose；本机直接运行只用于快速开发。固定并记录 container image/tag 或 digest、repository commit、db-LaCAM commit、dynobench commit、OMPL revision 以及 motion-primitives version/checksum。
- 训练容器通过 NVIDIA Container Toolkit 使用 RTX A4000；实验启动时记录 CUDA/PyTorch/driver 可见版本和实际 device。容器化用于复现环境，不改变 A4000 主要加速 learning、db-LaCAM 主要消耗 CPU 的事实。
- 正式实验使用单一集成 `research-gpu` container，而不是在训练和 planner 容器之间做逐步 RPC。该容器同时包含 CUDA PyTorch/LeRobot、CasADi、db-LaCAM executable、dynobench、OMPL 和 motion primitives；`PlannerFactory` 通过配置在 `casadi`/`dblacam` subclasses 之间切换，DAgger 可在同一 Python process 中直接调用对应 planner wrapper。
- 新增正式的 `docker/Dockerfile.research` 与 Compose `research-gpu` service，优先采用 multi-stage/分层构建：planner build stage 编译固定 submodule 中的 OMPL/db-LaCAM/Dynoplan/Dynobench，最终 runtime 同时提供 CUDA PyTorch、LeRobot、CasADi、db-LaCAM runtime dependencies/executable 和 primitives。具体 base image 与 library copy/install 以 smoke tests 和可维护性为准，不为了追求最小镜像牺牲清晰性。
- 统一容器采用渐进迁移：在 `research-gpu` 的 CUDA/CasADi/db-LaCAM/factory/workspace smoke tests 全部通过前，保留现有 `Dockerfile.gpu` 与 `Dockerfile.db-lacam` 作为 development/regression 和回退入口，不在第一轮直接删除。通过 gate 后将 `research-gpu` 设为正式实验默认；旧入口是否清理由后续实际使用情况决定。
- 实现时优先把现有 `Dockerfile.db-lacam` 重构为可参数化的共同 planner/runtime build，并为 GPU service 安装 CUDA PyTorch、在 Compose 中声明 GPU access；避免长期维护两份重复的 db-LaCAM/OMPL 编译步骤。现有轻量 `csvil`/CPU service 可保留用于 regression tests，但不作为正式实验环境。
- `research-gpu` build 必须执行最小验收：`torch.cuda.is_available()`、CasADi smoke solve、db-LaCAM executable smoke、两种 planner 均可由 `PlannerFactory` 创建，以及 workspace/data/checkpoint volumes 可写。
- Raw archives、manifests、LeRobot datasets、configs、checkpoints 和 logs 通过明确的 workspace/data volumes 持久化，不能只保存在临时 container filesystem 中。
- 目前codebase的结构非常好，主要模块都有一个顶层抽象类，所有不同的方法都可以继承这个类使用，后续的实现也要按照这种方式来
- 我想在tensorboard中监视训练，后面不仅要在terminal中输出训练/DAgger情况，还要在tensorboard中记录下
- 各个部分要严格进行模块化处理，不要出现一个模块的文件夹出现在另外一个模块的文件夹里
- 减少不必要的维度检查，不要过于保守
- 尽量从简单的结构做起，这样方便逐模块分析，逐模块改进，还能研究出哪一部分encoder会导致latent collapse等问题


# Risks
- db-LaCAM 速度太慢：先离线数据并缓存可复用结果；第一版仍保留逐步 DAgger query 正确性基线，只有实测成本不可接受时才按已定义的 state-deviation guard 测试 $K$-step replanning。
- db-LaCAM 轨迹或动作较 jerky：在 expert benchmark 中量化 action variation / jerk；先确认问题来自 motion primitives、离散时间分辨率还是 replanning。必要时比较不改变动力学可行性的轨迹 refinement、动作平滑或 student regularization，并分别报告原始 expert 与处理后 expert 的性能。
- db-LaCAM 不支持目标动力学: 先用已支持模型，再扩展 motion primitives
- 同时开发新动力学与通信方法会扩大调试范围：先在 `single_integrator` 上完成整个方法闭环，再串行扩展 `double_integrator`，最后通过相同接口迁移到 RoboChief；后两者不得阻塞 Stage 1。
- Expert 本身成功率不足: 先独立 benchmark，不合格则不训练 student
- 消息发生 collapse： 测试梯度、消息方差、干预/置乱消息
- Learned policy 忽略消息： message masking、counterfactual evaluation
- Continuous message dimension 被错误解释为真实带宽：Phase 2 只报告 latent dimension；只有 Phase 3/4 加入明确的 bits-per-element、发送频率和 receiver 数量后才报告 bit-level communication load。
- DAgger 成本过高： BC 预训练、减少 round、选择困难状态查询
- Safety filter 掩盖策略缺陷：同时报告过滤前后性能及 intervention rate


# Timeline
- Implementation Milestone 1 — two-robot vertical slice：在 $8\times8\,\mathrm{m}$ empty workspace 中冻结一个 two-robot swap YAML，使用 homogeneous single-integrator、$dt=0.1\,\mathrm{s}$、$v_{max}=1.0\,\mathrm{m/s}$、collision radius $0.30\,\mathrm{m}$，贯通 scenario load → unified GPU container → `PlannerFactory`/db-LaCAM subprocess → `ExpertPlanResult` → simulator replay → dynamics/action/swept-collision/strict-goal validation → minimal raw archive → trajectory visualization。
- Milestone 1 acceptance gate：容器内 CUDA、CasADi 与 db-LaCAM smoke 均通过；固定 scenario 可重复求解；robot ordering、state/action shape、原始 action label 和 $dt$ 正确；db-LaCAM/Dynobench 与 independent analytic collision conclusions 一致；replay 满足统一 $0.05\,\mathrm{m}$ goal tolerance；故意构造的 collision、wrong-$dt$ 和 planner-timeout cases 能被正确分类；现有 CasADi regression tests 不被破坏。
- Milestone 1 不生成大规模 dataset、不训练 policy、不实现 communication。只有该 vertical slice 通过后，才扩展到 3/4/8/16 robots、scenario bank 和批量 expert generation，避免同时调试 container、adapter、数据和 learning。
- Phase 1：db-LaCAM teacher
  - 独立 smoke tests；
  - benchmark；
  - 统一 planner interface；
  - expert dataset generator。
- 现有 CasADi/repository baseline 已由当前项目阶段验证，不再作为独立前置里程碑；后续 unit/integration tests 仍作为每次接口修改后的 regression checks，但不阻塞进入 Phase 1。
- Phase 2：No-communication baseline
  - offline BC；
  - DAgger；
  - db-LaCAM expert
  - 完整 evaluation metrics。
- Phase 3：Learned communication
  - synchronous continuous messages；
  - sender/receiver；
  - end-to-end training；
  - message intervention tests。
- Phase 4：Bandwidth constraints
  - quantization；
  - message frequency；
  - receiver sparsity；
  - communication metrics。
- Phase 5：Network imperfections
  - dropout；
  - delay；
  - asynchronous delivery。
- Phase 6：Experiments
  - baseline comparison；
  - GLAS official smoke/reproduction and GLAS-style baseline（在核心 proof-of-concept 后）；
  - ablations；
  - generalization；
  - runtime。
- Phase 7：Optional safety/robots
  - CBF；
  - physical robot validation。
