# 审稿修订：方法、实验和论文

本目录对应修订后的[单文件论文](../../../paper/main.tex)和[编译 PDF](../../../paper/main.pdf)。[逐条审稿回复](RESPONSE_TO_REVIEW.txt)列出每项实际修改、结果及证据位置。修订前结论保存在[历史结果记录](ARCHIVED_RESULTS_BEFORE_REVIEW.md)，冻结代码、原模型和原实验没有覆盖。

## 实际方法变化

- 训练消融：State/Geometry输入 × BCE/BCE+0.1Huber × 三种子，共12次训练，检查点统一按验证BCE选取。
- 模型选择：在32张验证地图、四种条件的512个回合上比较四配置，冻结State+Aux后才访问新测试地图。
- 计算优化：在线只输出可行性；State模型删除75个零输入列和几何探针，采用28–64–32–1、3,969参数。共同最小代价候选先检查，必要时才执行网络；该改动保持原决策与预算消耗。
- 数值一致性：所有方法可使用相同0.2 s标称RK4前缀复核，步长0.005 s。它不读取真实风速、模型扰动或延迟。长期Euler预测、任务代价和15控制延续库保持一致。

## 新测试结果

32张独立新地图，四种执行条件，九种方法，前缀复核开/关两组，共2,304个回合。每格128项任务。

| 方法 | 关闭：完成/裕度侵入 | 开启：完成/裕度侵入 |
| --- | ---: | ---: |
| Cost | 43 / 2 | 44 / 0 |
| Diverse | 88 / 1 | 89 / 0 |
| Full | 94 / 4 | 96 / 0 |
| 原Geometry | 90 / 4 | 92 / 1 |
| 原State | 100 / 10 | 108 / 0 |
| Geometry+BCE | 90 / 13 | 100 / 1 |
| Geometry+Aux | 95 / 5 | 100 / 0 |
| State+BCE | 103 / 14 | 113 / 3 |
| Selected：State+Aux | 107 / 12 | 118 / 0 |

共同开启复核时，Selected相对Diverse完成率差22.66个百分点，95%地图分组bootstrap区间14.84–30.47；相对Full差17.19个百分点，区间8.59–25.78。Selected自身开启复核增加11个完成任务，无完成任务丢失。其余10个任务因无可行主基元结束。

匹配输入的发现率、独立端到端计时和分项耗时见[runtime/analysis.json](runtime/analysis.json)及[开启复核结果](runtime/prefix/analysis.json)。端到端计时与埋点分解为不同运行，不能相互替代。操作预算不是墙钟截止时间。

## 失败机制与城市对照

原512个任务全部精确复现。21次裕度侵入对应候选仍有3–10条可行延续；同状态、同控制的0.2 s重积分中，Euler0.02检测0次、Euler0.005检测15次、标称RK4检测18次，其余3次需要记录的侧风条件。共同轨迹首次分歧分析显示Geometry往往选择更低代价和较少延续，但主轨迹净空没有一致下降。详细数据见[diagnostics](diagnostics/README.md)。

城市采用五种调度×三个配置，共15个试验：粗高度库/48.825 m门限均未完成首个航点；相对高度库/48.825 m门限全部完成循环；相对高度库/15 m门限仅原Geometry完成，Cost、Diverse和Selected在完成3个航点后无可行主基元，原State出现裕度侵入。该比较为同一城市布局，每配置一次；[结果表](city/summary.csv)保留全部失败。

城市规划读取预先提供的建筑盒，LiDAR及体素图记录观察环境。城市运行适配器保留原推理路径，未启用新RK4前缀检查；紧凑推理计时来自独立数值基准。原生Gazebo/RViz截图和森林图保留在稿件中，并注明原始配置。

## 证据和复现入口

| 内容 | 协议/结果 | 运行与分析脚本 |
| --- | --- | --- |
| 训练消融 | [training](training/README.md) | `train_recovery_revision.py` |
| 模型选择 | `runtime/VALIDATION_SELECTION_PROTOCOL.json`, `EXECUTION_MODEL_FREEZE.json` | `select_review_execution.py` |
| 独立测试及计时 | [runtime](runtime/README.md) | `run_review_runtime.py`, `analyze_review_runtime.py`, `compare_review_prefix.py` |
| 回放及特征诊断 | [diagnostics](diagnostics/README.md) | `run_recovery_review_diagnostics.py`, `run_recovery_review_branchpoints.py`, `analyze_recovery_review_diagnostics.py` |
| 城市配置 | [city](city/README.md) | `run_review_city.py`, `analyze_review_city.py` |

脚本均位于`motion_primitives_planning/scripts/`。先在catkin工作区构建对应review目标，source ROS与工作区环境，按各子目录说明运行。运行清单记录命令、种子、模型和源码哈希。所有性能计时应在其他仿真与训练停止后重新测量；原始计时仅代表记录的硬件和软件环境。

数值一致性和实现检查包含：原匹配输入3,456次优化配对零差异、10,368项原始决策复现、四个部署配置的C++输出配对零差异、七项前缀复核检查及八项运行版高度/门限回归检查。训练导出与C++/PyTorch浮点误差分别保留于对应检查文件。

## 文档编辑

正文、公式、算法、表格、图注与15条参考文献全部在`paper/main.tex`。ZIP只包含一个`.tex`，另有图片、IEEEtran类文件、预览PDF和中文编辑说明；不依赖ROS或原始数据即可编译。
