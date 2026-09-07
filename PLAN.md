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
- CBF等Safe Filter能带来多大的性能改善，能否做出一些安全保证。
# Current Baseline
# System and Communication Model
# Proposed Method
# Baselines
# Implementation Phases
# Evaluation Metrics
# Ablations
# Risks
# Timeline
# Open Questions