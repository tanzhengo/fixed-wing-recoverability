# 交给 Ubuntu 上 Codex 的任务

在本分支的仓库根目录启动 Codex，然后复制下面整段文字。它不依赖原 Windows 对话记录。当前进度见 [Ubuntu 验证摘要](validation/ubuntu_20261117_summary.json) 和 [恢复证据包](../evidence_recovered/USAGE.txt)。

```text
请接续这个固定翼 terminal continuation 项目的 Ubuntu 验证与开发。

先阅读 reconstruction/AGENTS.md、reconstruction/README.txt、
reconstruction/UBUNTU_GUIDE.md、reconstruction/PUBLICATION_STATUS.json，
以及 reconstruction/ros/INTEGRATION_CONTRACT.txt、
evidence_recovered/USAGE.txt 和
reconstruction/validation/ubuntu_20261117_summary.json。

当前工作分支应为 codex/tro-revision-2026-11-17。
先检查 Git 状态和远端更新，保留已有未提交工作，不修改 main。
当前 reconstruction/ 是依据用户提供的 LaTeX main11.tex 重建的
Python/NumPy 参考实现。paper/main.tex 是历史稿件。
2026-11-17 已找回原稿 C++/ROS 源码、58 维权重、九次训练记录和实验档案，
并核验哈希、重算主表结果；这些原件单独保存在 evidence_recovered/。
恢复档案和聚合重算不等于全部历史闭环实验已重新运行，
也不表示本目录新 Python 参考实现已接入 ROS。

UBUNTU_GUIDE.md 第 3–7 节已于 2026-11-17 在 Ubuntu 完成，
76 项检查全部通过，短同步回合结果为 time_limit。先核对摘要和当前源码；
如代码或运行环境有变化，再在新目录按以下步骤复验：
1. 检查 Ubuntu/Python 版本，建立独立 Python 3.10+ 虚拟环境。
2. 为本次运行新建 runs/ 目录，记录提交、工作区状态和依赖版本。
3. 执行软件检查和命令入口检查；基线有 76 项测试。
4. 运行构造地图诊断，再实际标注、训练一个 obs_bce smoke 模型，
   用这个新模型生成 learned 查询比较，并做单帧重放和短数值回合。
5. 检查实际输出和失败状态，保留全部记录，不能只凭退出码宣布成功。
   修复遇到的可复现代码问题，补必要回归检查，不篡改测试或结果。

继续原稿实验时优先使用 evidence_recovered/ 中的正式模型、最终选模记录、
分析与评估入口，保留恢复档案；已有首个预定 cul-de-sac 的原始失败轨迹。
若继续新 Python 实现的 ROS 接入，检查现有 ROS/Gazebo 环境，
按指南第 8 节继续新适配器：
先旧接口核验、离线重放和旁路提案，再接入真实时间的重验证与仿真闭环。
旧 launch 文件运行的是旧规划器，不代表 reconstruction/ 已接入。
若环境缺失或版本冲突，记录具体缺项并继续可独立完成的适配开发；
只有需要我提供登录、密码、缺失文件或实质性的环境选择时再询问。

保留当前 58 维特征、H=B=3 s、K=12 等基线。改动必须有代码或实验依据。
地图真值不能提供给规划器；时间和控制历史必须真实，未知区域不能当空闲。
禁止伪造飞行数据、性能表、失败案例或“Selected”模型。
同步零计算延迟诊断不能标为异步实时验证；time_limit 不能标为任务成功。

在 runs/ 中保存每阶段报告和接续信息，说明哪些实际执行、哪些尚待验证。
把完成的代码/文档修复提交并推送当前开发分支，不合并 main。
如需继续改论文，先确认我提供的当前 LaTeX 工程，缺失时告诉我具体需要什么；
不要直接修改旧 paper/main.tex 作为最新稿，也不要用新诊断值替换原论文表格。
```

仅需复验 Python 环境时，可把上面 ROS 段落替换为：“核对已有 Ubuntu 摘要，并在当前环境复验第 3–7 节后报告结果；本次先不启动 ROS 接入。”
