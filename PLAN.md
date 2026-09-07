# Objective
- 从centralized expert(db-LaCAM/db-ECBS)中训练一个参数共享的分布式多机器人控制策略。
- 每个机器人根据局部观测和受带宽约束的机器人间消息生成局部动作和待发送消息，消息没有直接监督，而是通过动作模仿目标进行end to end学习。
- 最终输出包含action和message，但message不参与训练的反向传播。
- 方法将在包含连续动力学的物流场景中，与无通信策略等baseline进行比较。



# Current Scope and Non-Goals
此处我们明确当前项目和实验的着重点：
目前考虑的问题应该限制在以下的范围内：
    - 目前所有的实验都应该是simulation
    - 使用的机器人暂时假设为homogeneous robot fleet
    - 机器人模型计划使用的是一个由三个间隔120度的omniwheel驱动的圆盘机器人模型，后续会加入其运动学参数
    - 考虑的动力学模型可以包含single_integrator/unicycle1/double_integrator/unicycle_2，需要根据db-LaCAM能够处理的dynamics形式进行限制
    - 目前只考虑2D continuous space
    - 采用圆形碰撞模型来建模物体的碰撞，应使用或者借鉴db-LaCAM以及其他主流的collision checking方法
    - 目前只考虑把静态障碍物
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


# Research Questions
此处我们明确我们想要回答的问题/想要达到的目标
- generative model能否实现对centralized MRMP的imitation 
- 相比起无通信的Baseline（GLAS）相比，learned communication是否提高了以下的指标：
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
  - 由三个固定间隔120度排列的omniwheel驱动的disk robot，我们称之为robochief
  - 最好保证是二阶运动学模型
  - 具体的运动学和模型参数稍后加入：
    - 

- Local Observation
  - 目前定义我们的observation为：$o_i^t = [goal_relative_state, ego state, local obstacle observation, visible robot observations, visible obstacle observations]$
  - 其无法直接获得邻居位置，只能获得相对位置，这是因为从机器人本机的sensor出发我们无法得知自身在目前全局坐标系的位置。
  - 如果有能力的话可以先假定我们可以获得邻居的速度，后续可以做没有的ablation
  - 邻居ID应当不可见，这样的话我们可以用deepset把机器人/obstacle看作一致的物品
  - 局部地图目前在simualtion中怎么方便怎么来
  - 保证在我们的observation的construction里面不会让各个robot在大部分时刻都能看到所有的neighbor，不然的话communication会失去意义
  - obstacles 可以用bounding box来建模，同样包含ego position和effective size

- Action：
  - 如果是first order dynamics model我们就用velocity
  - 如果是second order dynamics model 就用acceleration
  - 还有一个最后最后的方法是输出primitives

- Task Success:
  - 所有机器人到达目标
  - 到达允许的goal area之内
  - 到达最大时间限制仍然没有找到目标
  - collsion happens

- Communication
  - Communication Process:
    - 假设我们可以通过某种方式提供当前各个机器人之间的通信限制，作为我们通讯模块的输入
    - 将这个通讯网络的限制作为输入输入到一个encoder/diffusion/flow matching model中，将我们的bandwidth作为constraints，生成message $z_i^t$
    - 通过scheduler生成一个各个link之间的是否允许通讯的indicator map
    - 然后通过channel发送到各个机器人，在simulation中我们此处需要根据之前我们生成的通讯限制对发送出的消息进行处理
    - 各个Robot 聚合收到的信息 $M_i^t$，并结合自身的state输出action
  - Message Content:
    - continuous latent vector
  - Message Length:
    - 衡量bandwidth constraints的方式应该按照这样的方式计算：bits per message x messages per second x number of receivers
  - 通讯对象：
    - 最理想的状态是learned receiver selection
    - 但是初步搭建的时候可以通过GNN来构造各个机器人之间的通讯必要性，只有在必要性大于一定阈值的时候才允许通讯
  - 时间特性：
    - 每个control step 通信，但是每个机器人都独立决定和谁通讯
    - 后面需要考虑随机delay
    - 然后还要考虑asynchronous update



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
- receiver aggregation用什么我还不确定，用DeepSet， Transformer还是GNN都可以，原则是我们从简单的开始/效果最好的开始
- message需要带上sender ID和sender relative pose
- communication可以是多轮也可以单轮，理想状态是单轮
- message先不考虑recurrent



# Learning Objective
我们的idea的核心要求是不要显示监督消息，而是用我们的imitation learning的监督信号来指导消息的encoding和transmitt
所以我们的Loss应该包含：
- imitation loss：$\mathcal{L}_{action} = \|\pi_\theta(o_i, M_i) - u_i^E\|^2$ 或者 $\mathcal{L}_{flow}$
- Bandwidth penalty: $\mathcal{L} = \mathcal{L}_{action} + \lambda_B\mathcal{L}_{bandwidth}$
- Receiver Sparsity: $\mathcal{L}_{sparsity} = \lambda_R\sum_{ij}g_{ij}$
- Temporal consistency: $\lambda_T\|z_i^t - z_i^{t-1}\|^2$
- Message entropy或信息瓶颈
我们一项一项验证加入，不要一下用上所有的。



# Baselines
- Centralized expert：db-LaCAM本身
- No communication Imitation Policy: 当前 DeepSet/local policy，或者更接近 GLAS 的实现
- Handcrafted communication: 直接发送所有的信息不经过任何encoding
- Unlimited/full-state communication: 不限制带宽，假设ideal communication



# Implementation Phases
- db-LaCAM Expert Validation Phase
  - Smoke Test
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
    - open-loop/replan 行为正确
    - 规划失败能够被统一处理
- Dataset Generation Plan
  - 注意这里数据点要首先参考目前codebase是怎么做的来决定做什么修改
  - 每个frame至少包含以下信息：
    - local observation
    - expert local action
    - neighbor visibility
    - episode index
    - timestep
    - scenario metadata
  - 可能还需要包含以下信息：
    - global state(仅供训练分析使用)
    - robot IDs
    - expert joint plan
    - collision margins 
  - 数据拆分：
    - 必须按scenario/episode 


# Evaluation Metrics
# Ablations
# Risks
# Timeline
# Open Questions