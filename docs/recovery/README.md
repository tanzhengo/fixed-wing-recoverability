# 终端续飞可行性原型 / Finite-horizon continuation prototype

第二阶段新增候选覆盖和剩余方案复验，见[最新方法与运行说明](V2.md)。本页保留第一阶段设置和结果。

本版本是第一阶段代码与仿真基础，不含神经网络，不是论文完成版。基于上游 `ccb6005c73992214a86cc0cff4fe648f1443a119` 的 ROS 演示代码；该代码与 Ubuntu 原工作区 `/home/tan/FW_mp` 的核心规划头文件内容一致。原工作区未修改。

## 方法与边界

1. 使用原模型从当前状态展开三维主基元，原代价函数评分。
2. 启用实验模式时，硬过滤碰撞、模型非法、飞行包线违规以及可选已知区域之外的候选。
3. 按代价排序，最多检查 `recovery/max_candidates` 个主候选。从每个终态完整继承位置、速度、航向、航迹角、滚转、迎角、推力等状态，生成独立于目标点的 5 个航向 × 3 个高度目标的后续控制。
4. 找到一条满足模型与采样约束的后续段，返回主段及后续证据。动态障碍预测使用主段时长加后续段时间，不从零重新计时。
5. 候选预算用尽与完整候选检查失败分开报告。无已验证后续但有主段可行时，降级执行最佳主段。无主段可行时，沿用明确标记为未保证安全的应急控制；固定翼不能把停发命令当作安全停止。

这是**有限候选、有限时域、离散采样的续飞证据**。没有构造不变安全集，没有证明递归可行性，也没有承诺执行后总能规划成功。青色后续轨迹目前是选择证据，不是自动接管执行的备份控制器；下一周期仍从实际状态重规划。

原公开代码使用配置文件提供的几何障碍和膨胀质点检查，不包含原论文完整的 LLS、相机 FOV/遮挡建图、Dubins/Clothoid 或整机扫掠体。不得把这些模块归功于本次实现。可选 `known_space_radius` 仅表示以本周期实际状态为球心、验证期间冻结的理想化观测球；0 表示关闭。即使开启，也不代表真实可见空间。

## 配置与状态

默认 `mp.yaml` 的 `recovery/enabled: false`，保留原演示选择；非法初态和越界命令新增显式拒绝。新启动文件会开启模块，并将主候选检查上限设为203，覆盖默认203个主候选；`max_candidates:=12` 是有限预算对照。完整覆盖仍然只检查有限备份库，不代表穷尽可行控制。

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `recovery/enabled` | false | 是否启用后续检查 |
| `recovery/horizon_time` | 3.0 s | 后续段时长 |
| `recovery/max_candidates` | 12 | 每周期检查的主候选上限，按数量而非墙钟时间预算 |
| `recovery/known_space_radius` | 0 m | 理想已知球半径；0 关闭 |

`/suav1/planning_status` 为 `std_msgs/String` JSON，包含状态和计数；`/suav1/traj` 中青色 `traj_continuation` 表示后续证据。

| 状态 | 解释 |
|---|---|
| `RECOVERY_VERIFIED` | 找到有限时域后续证据 |
| `BUDGET_EXHAUSTED` | 尚有未检查主候选，不能断言无后续 |
| `DEGRADED_NO_CONTINUATION` | 已检查全部可行主候选，有限备份库未找到证据 |
| `NO_FEASIBLE_PRIMARY` | 启用模式下无可行主段；应急命令不保证安全 |
| `INVALID_STATE` | 初态非法，不发布新控制；同样不能解释成安全停止 |
| `BASELINE_FEASIBLE` / `BASELINE_UNSAFE` | 关闭模块时的原选择状态 |

## 在 Ubuntu 20.04 / ROS Noetic 运行

以下命令在包含本仓库的 catkin 工作区根目录执行。本次目录为本聊天 `outputs/two_phase_ws`，后续可把仓库克隆到任意 catkin 的 `src/` 中。

```bash
source /opt/ros/noetic/setup.bash
catkin_make -j2 -DCATKIN_WHITELIST_PACKAGES="motion_primitives_planning;simple_airplane_plugins;simple_airplane_gazebo;simple_airplane_description"
source devel/setup.bash
catkin_make motion_primitives_3d_test terminal_recovery_test -j2
rostest motion_primitives_planning motion_primitives_3d.test
rostest motion_primitives_planning terminal_recovery.test
roslaunch motion_primitives_planning terminal_recovery.launch paused:=false
```

无界面集成检查使用 `gui:=false rviz:=false`。`enabled:=false` 运行同一场景的原选择。配置或 C++ 修改后请重启相关节点；C++ 修改需重编译。

## 可复现实验

绘图依赖 `python3-matplotlib`（本机已安装，已在包依赖中声明）。

```bash
mkdir -p results
roslaunch -p 11331 motion_primitives_planning recovery_benchmark.launch output_prefix:=$PWD/results/recovery
rosrun motion_primitives_planning summarize_recovery_benchmark.py $PWD/results/recovery
rosrun motion_primitives_planning plot_recovery_benchmark.py --prefix $PWD/results/recovery --output-dir $PWD/results
```

四个固定场景（开放空间、错列柱体、晚出现的横向墙体、死胡同）× 三个初始横向偏移（-2/0/2 m）× 五个变体，共 60 回合。这里“晚出现”指障碍进入短规划时域，地图本身始终已知，不是传感器遮挡。几何随程序导出到 `_scenarios.csv`，绘图直接读取该文件。

| 变体 | 主段时长 | 主候选 | 后续检查 |
|---|---:|---:|---|
| `baseline_3s` | 3 s | 29 × 7 | 无 |
| `dense_3s` | 3 s | 57 × 9 | 无 |
| `long_6s` | 6 s | 29 × 7 | 无 |
| `recovery_3plus3s` | 3 s | 29 × 7 | 3 s，最多12个主候选 |
| `recovery_full` | 3 s | 29 × 7 | 3 s，最多203个主候选 |

它是同模型闭环机制实验：每 0.2 s 执行一次控制，使用与规划相同的积分模型作为被控对象；不注入模型偏差、感知误差或计算延迟。地图完全已知，静态圆柱占满高度，飞行高度带为 16–24 m，目标为 (250,0,20) m。到目标水平15 m内且高度误差3 m内判成功，30 s未完成判超时。无可行主段立即结束并记录 `no_feasible_primary`，这是仿真评估规则，不是物理上安全停车。

全部回合均记录，包括失败。耗时为选择函数的墙钟时间，不包含ROS发布、绘图、感知与真实控制延迟。100 ms计数仅为线上10 Hz参考预算，离线5 Hz实验不模拟真实deadline miss。地图数量很少且非统计独立的随机样本，成功率不能外推到一般环境。

## 首轮结果

完整数据、28项测试记录、Gazebo集成检查与图表见[首轮实验记录](RESULTS.md)。

## 下一阶段

根据实际对照结果决定候选排序和预算机制是否值得继续；若完整检查有效而12候选无效，应先解决多方向候选覆盖，而不是直接加入网络。进一步工作需要：真正的已观测自由空间、可执行备份策略与递归条件、独立动力学与延迟、更多未参与设计的地图、与原论文完整算法的公平对照。只有这些证据充分后，才把研究主张写入论文。
