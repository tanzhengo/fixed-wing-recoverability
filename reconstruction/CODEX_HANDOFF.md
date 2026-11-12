# 交给 Ubuntu 上 Codex 的任务

在本分支的仓库根目录启动 Codex，然后复制下面整段文字。它不依赖原 Windows 对话记录。

```text
请接续这个固定翼 terminal continuation 项目的 Ubuntu 验证与开发。

先阅读 reconstruction/AGENTS.md、reconstruction/README.txt、
reconstruction/UBUNTU_GUIDE.md、reconstruction/PUBLICATION_STATUS.json，
以及 reconstruction/ros/INTEGRATION_CONTRACT.txt。

当前工作分支应为 codex/manuscript-reconstruction-2026-11-12。
先检查 Git 状态和远端更新，保留已有未提交工作，不修改 main。
当前 reconstruction/ 是依据用户提供的 LaTeX main11.tex 重建的
Python/NumPy 参考实现。paper/main.tex 和旧 C++/ROS 结果是历史版本。
原始最新代码、58 维权重和实验记录尚未找回，不能声称已复现。

请实际操作，先完成 UBUNTU_GUIDE.md 第 3–7 节：
1. 检查 Ubuntu/Python 版本，建立独立 Python 3.10+ 虚拟环境。
2. 为本次运行新建 runs/ 目录，记录提交、工作区状态和依赖版本。
3. 执行软件检查和命令入口检查；基线有 76 项测试。
4. 运行构造地图诊断，再实际标注、训练一个 obs_bce smoke 模型，
   用这个新模型生成 learned 查询比较，并做单帧重放和短数值回合。
5. 检查实际输出和失败状态，保留全部记录，不能只凭退出码宣布成功。
   修复遇到的可复现代码问题，补必要回归检查，不篡改测试或结果。

这些完成后，检查现有 ROS/Gazebo 环境，按指南第 8 节继续新适配器：
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

首次仅想完成 Python 验证时，可把上面 ROS 段落替换为：“完成第 3–7 节后先报告结果；本次先不启动 ROS 接入。”
