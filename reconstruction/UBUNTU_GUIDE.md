# Ubuntu 运行与后续开发指南

对应分支：`codex/tro-revision-2026-11-17`。首次代码发布提交：`cfda580`。2026-11-17 已在 Ubuntu 主机完成本指南第 3–7 节：76 项软件检查全部通过，并运行了诊断、标注、smoke 训练、单帧重放和短同步数值回合。回合结果为 `time_limit`；具体环境、指标及哈希见 [Ubuntu 验证摘要](validation/ubuntu_20261117_summary.json)。Windows 原始检查记录保持不变。

同日找回并核验了原稿的 C++/ROS 源码、58 维权重、训练记录和实验档案，入口为 [恢复证据包](../evidence_recovered/USAGE.txt)。恢复的原始实验系统与本目录的新 NumPy 参考实现分别保存；档案核验与结果重算不表示全部历史闭环实验已重新运行。

需要让 Ubuntu 上的 Codex 接手时，先看 [交接任务](CODEX_HANDOFF.md)。本指南也可手动逐步执行。切换前如计划继续改论文或运行城市演示，还需带上当前 LaTeX 工程、修订工作稿及原机使用的 Gazebo 城市模型；这些不是运行第一阶段 Python 检查的前提。

## 1. 先判断要运行哪部分

| 目的 | 环境与入口 | 当前状态 |
| --- | --- | --- |
| 检查重建代码、生成诊断图、标注和训练 | Python 3.10+，NumPy、Matplotlib；本目录 | 已完成 Ubuntu 软件与管线验证；不需要 ROS、CUDA 或 PyTorch |
| 重算原稿结果、编译恢复的原始评估器 | [evidence_recovered](../evidence_recovered/USAGE.txt)；分析用 NumPy，C++ 编译用 ROS Noetic | 原始记录已恢复并核验，五个评估器编译通过；与本目录参考实现分开 |
| 运行旧 Gazebo 演示、核验旧消息接口 | 历史工程：Ubuntu 20.04、ROS Noetic、Gazebo Classic 11 | 是旧实现；运行成功不表示新规划器已接入 |
| 用观测地图驱动新规划器做异步闭环实验 | 新 ROS 适配器或 C++ 移植，以及新启动文件 | 尚待实现；参考 [接口约定](ros/INTEGRATION_CONTRACT.txt) |

重建依据是用户另行提供的 LaTeX 工程 `main11.tex`。仓库 `paper/main.tex` 是旧稿，不能据此改回旧模型、103 维输入或 6 s 延续时域。当前重建使用 58 维输入、H=B=3 s、K=12、最多 406 个主候选、15 个后继控制。详细选择见 [README.txt](README.txt)。

新装系统做第一阶段可使用 Ubuntu 24.04 的独立 Python 虚拟环境；这不表示本项目已验证该系统兼容性。若你已经有旧 Ubuntu 20.04/Noetic 环境，先保留它。第一阶段需要另行选用 Python 3.10+ 解释器，不能替换系统 Python 来迁就项目。

ROS Noetic 已于 2025-05-31 结束支持，Gazebo Classic 11 已于 2025-01-29 结束支持。因此旧演示的环境和新装 Ubuntu 的环境必须分别处理；不能把 Noetic 的安装命令直接套到 Ubuntu 22.04/24.04，也不能仅将 `gazebo` 换成 `gz` 就视为迁移完成。[ROS 官方说明](https://discourse.openrobotics.org/t/ros-noetic-end-of-life-may-31-2025/43160)、[Gazebo 官方说明](https://classic.gazebosim.org/)。

## 2. 获取分支和启动 Codex

尚未下载仓库时，在 Ubuntu 的 Bash 终端运行：

```bash
sudo apt update
sudo apt install -y git python3 python3-venv python3-pip curl
mkdir -p "$HOME/projects"
cd "$HOME/projects"
git clone --branch codex/tro-revision-2026-11-17 --single-branch \
  https://github.com/tanzhengo/fixed-wing-recoverability.git
cd fixed-wing-recoverability
git status --short --branch
git rev-parse HEAD
```

已有仓库时，先检查 `git status` 并保存自己的未提交修改，再获取这个分支；不要覆盖原目录或执行硬重置。公开仓库可以直接读取；向分支推送仍需要你自己的 GitHub 写入授权。

已安装 Codex 时，从仓库根目录运行 `codex`，粘贴 [CODEX_HANDOFF.md](CODEX_HANDOFF.md) 的任务文字。尚未安装时，按 [OpenAI 官方 Codex CLI 文档](https://learn.chatgpt.com/docs/codex/cli) 安装并登录；当前官方 Linux 安装方式为：

```bash
curl -fsSL https://chatgpt.com/codex/install.sh | sh
```

安装后按提示更新终端环境，再从仓库目录运行 `codex`。这一步安装的是操作项目的工具，不是项目的 Python 依赖。

## 3. 建立独立 Python 环境并记录版本

以下步骤从仓库根目录开始。后面的命令都在 `reconstruction/` 内、同一个 Bash 会话中运行；`FW_RUN` 保存本次输出目录。中途换终端时，重新激活环境，并把 `FW_RUN` 设为先前记录的完整目录。

```bash
cd reconstruction
python3 -c 'import sys; assert sys.version_info >= (3, 10), "Need Python 3.10+"; print(sys.version)'
```

先单独执行上述检查，**确认成功后**才执行下一组。版本不满足时跳到本节末尾的处理说明，不继续安装。

```bash
python3 -m venv .venv && \
  source .venv/bin/activate && \
  python -m pip install --upgrade pip && \
  python -m pip install -r requirements.txt && \
  python -m pip check
```

环境创建、激活或安装的任一步失败，先解决该错误。全部成功后再建立本次运行记录：

```bash
mkdir -p runs
export FW_RUN="$(mktemp -d "$PWD/runs/ubuntu-$(date -u +%Y%m%dT%H%M%SZ)-XXXXXX")"
printf '%s\n' "$FW_RUN"
git rev-parse HEAD > "$FW_RUN/commit.txt"
git status --porcelain > "$FW_RUN/worktree_status.txt"
python -m pip freeze > "$FW_RUN/requirements_resolved.txt"
cat /etc/os-release > "$FW_RUN/os-release.txt"
python - <<'PY'
import json, os, platform, sys
from pathlib import Path
import numpy, matplotlib
payload = {
    "python": sys.version, "executable": sys.executable,
    "platform": platform.platform(), "machine": platform.machine(),
    "numpy": numpy.__version__, "matplotlib": matplotlib.__version__,
}
(Path(os.environ['FW_RUN']) / 'environment.json').write_text(
    json.dumps(payload, indent=2) + '\n', encoding='utf-8')
PY
```

如果 Python 版本检查失败，先停止这一组命令，由 Codex 选择独立的 3.10+ 解释器或隔离环境，再用该解释器创建 `.venv`。已有虚拟环境与计划解释器不符时，新建另一个有明确名称的环境，保留原环境供回查。Python 依赖装入虚拟环境，不使用 `sudo pip` 或 `--break-system-packages`。[Ubuntu 官方 Python 环境指南](https://ubuntu.com/developers/docs/tutorials/python-use/)。

依赖文件指定的是版本下限，未声称锁定任意未来版本的行为。首次运行应保留上述解析版本；遇到兼容问题，应记录错误并说明版本调整依据。

## 4. 软件检查

```bash
python scripts/run_checks.py --output "$FW_RUN/software_checks.json"
python scripts/run_diagnostics.py --help
python scripts/replay_query.py --help
python scripts/label_dataset.py --help
python scripts/train_model.py --help
python scripts/run_episode.py --help
```

2026-11-17 的 Ubuntu 验证运行了 76 项检查，failures/errors/skipped 均为 0，见 [验证摘要](validation/ubuntu_20261117_summary.json)。后续代码或环境变更后，按上述命令在新目录复验；新增测试时记录新计数和原因。失败时先定位和修复，不跳过测试或手改结果。原有 `validation/software_checks.json` 保留为 Windows 原发布记录。

## 5. 生成基础诊断和新训练样例

```bash
python scripts/run_diagnostics.py --output "$FW_RUN/diagnostics"
```

这是明确构造的观测空间数值案例，会实际计算主轨迹和后继验证结果。其输出包括：

- `diagnostic_manifest.json`：配置、源码哈希、状态与查询工作量。
- `query_traces.csv`、`feasible_candidates.csv`：可追溯的查询与候选记录。
- `train_frames.jsonl`、`validation_frames.jsonl`、`diagnostic_test_frames.jsonl`：三组独立用途的帧文件。
- 诊断图：`query_work_diagnostic.*`，以及满足两类候选条件时的 `continuation_and_map_diagnostic.*`。

程序使用非交互绘图后端，可在无桌面环境生成图。不提供模型时，诊断不会凭空生成 learned 曲线。核查 manifest 中是否真正生成了所需比较；缺少一类候选时保留缺失状态，不补造轨迹。数值样例不是 LiDAR 实测、历史飞行或代表性 cul-de-sac 失败案例。输出包含较大的地图快照，应先检查磁盘空间；重跑使用新目录。

## 6. 跑通标注、训练与 learned 查询流程

这一阶段仅验证训练管线。以下只训练 `obs_bce` 的一个种子；正式消融与模型选择另行进行。

```bash
python scripts/label_dataset.py "$FW_RUN/diagnostics/train_frames.jsonl" \
  --output "$FW_RUN/train_labels.npz" --split train
python scripts/label_dataset.py "$FW_RUN/diagnostics/validation_frames.jsonl" \
  --output "$FW_RUN/validation_labels.npz" --split validation
python scripts/train_model.py \
  --train "$FW_RUN/train_labels.npz" \
  --validation "$FW_RUN/validation_labels.npz" \
  --output-dir "$FW_RUN/smoke_training" \
  --dataset-description "New constructed observed-box fixtures; pipeline smoke check only" \
  --variants obs_bce --seeds 20261101 --epochs 60
python scripts/run_diagnostics.py \
  --output "$FW_RUN/learned_diagnostics" \
  --model "$FW_RUN/smoke_training/obs_bce_20261101/compact_model.json" \
  --model-label "Learned (new smoke model)"
```

保留 `train_labels.manifest.json`、`validation_labels.manifest.json`、训练目录的 `summary.json`，以及种子目录中的 `training.json`、`training_model.npz`、`compact_model.json` 和 `compact_model.npz`。标注输出必须以 `.npz` 结尾；训练与验证场景 ID 必须不相交。程序检查 ID 不能替代几何/场景种子的人工核验。

新权重不是原稿的 `Selected` 模型，不能加载旧 103 维权重代替。不要用 `diagnostic_test_frames.jsonl` 训练；BCE 最佳检查点也不等于已完成闭环调度选择。

新核心可用模式为 `cost/diverse/learned/combined/observed/full`。历史 launch 中的 `learned_diverse` 不是这些脚本的合法模式名。本目录没有可安装的 Python 包定义，使用上面的脚本入口，不执行 `pip install -e .`。

## 7. 单帧重放和同步数值回合

重放接口接收一个 JSON 对象，不接收整个 JSONL。先取出一帧：

```bash
python - <<'PY'
import json, os
from pathlib import Path
run = Path(os.environ['FW_RUN'])
with (run / 'diagnostics/diagnostic_test_frames.jsonl').open(encoding='utf-8') as stream:
    frame = json.loads(next(line for line in stream if line.strip()))
(run / 'replay_frame.json').write_text(json.dumps(frame), encoding='utf-8')
PY
python scripts/replay_query.py "$FW_RUN/replay_frame.json" \
  --mode cost --output "$FW_RUN/replay_cost.json"
python scripts/replay_query.py "$FW_RUN/replay_frame.json" \
  --mode diverse --output "$FW_RUN/replay_diverse.json"
python scripts/replay_query.py "$FW_RUN/replay_frame.json" \
  --mode learned \
  --model "$FW_RUN/smoke_training/obs_bce_20261101/compact_model.json" \
  --output "$FW_RUN/replay_learned.json"
python scripts/run_episode.py configs/open_scene_diagnostic.json \
  --output-dir "$FW_RUN/open_episode" --mode diverse
```

最后一条命令使用配置中的 2.2 s 仿真时长；墙钟耗时可能远长于 2.2 s。默认传感器为 512×32 射线、5 Hz。若为排错降低扫描规模，必须单独标记，不能与默认协议混报。

检查 `open_episode/summary.json`、`manifest.json` 和全部 `*.jsonl`。程序正常退出不等于任务成功，`outcome=time_limit` 不能写成到达目标；初始化失败、执行拒绝、碰撞等结果同样保留。该运行器使用同步、零计算延迟策略，并包含显式标记的初始化控制阶段，不能作为异步 ROS 实时性证据。

## 8. 接入旧 ROS/Gazebo 环境时需要做什么

第一阶段不需要执行本节。准备进入仿真时，先读取 [INTEGRATION_CONTRACT.txt](ros/INTEGRATION_CONTRACT.txt)，检查实际 Ubuntu、ROS、Gazebo 和 Python 版本，保存版本输出。先查现有可用环境；若需要新建遗留环境，记录镜像/软件包版本和来源。不要用一套未经核验的过期安装命令覆盖现有系统。

下面是**已有 Ubuntu 20.04 + Noetic + Gazebo Classic 11 且依赖完整时**的旧工程基线检查。它只验证旧工程和接口，未启动新参考实现：

```bash
# 在仓库根目录、一个未激活 reconstruction/.venv 的新终端中运行。
source /opt/ros/noetic/setup.bash
export FW_REPO="$(pwd)"
export FW_ROS_WS="$(mktemp -d "$HOME/fw-reconstruction-ros-XXXXXX")"
mkdir -p "$FW_ROS_WS/src"
ln -s "$FW_REPO" "$FW_ROS_WS/src/fixed-wing-recoverability"
cd "$FW_ROS_WS"
catkin_make -j2 \
  -DCATKIN_WHITELIST_PACKAGES="motion_primitives_planning;simple_airplane_plugins;simple_airplane_gazebo;simple_airplane_description"
source devel/setup.bash
catkin_make motion_primitives_3d_test -j2
rostest motion_primitives_planning motion_primitives_3d.test
```

依赖缺失时核对各包 `package.xml`、`CMakeLists.txt` 和根 README 的历史依赖清单，再针对性补齐。不要直接编译整个保留的 ROSplane/MAVROS 工作区。图形演示的 mesh、城市模型等资源还可能依赖原机 `~/.gazebo/models`，需逐项检查；不能假定这个 Git 仓库包含所有城市资产。

接下来由 Codex 在新 package/target 中完成以下适配，并逐阶段记录实际测试：

1. 明确旧自定义 `Odometry/Twist` 字段、姿态符号、单位和传感器外参；这些字段不是普通速度指令。
2. 缓存同一 ROS 时钟中的状态、实际下发控制和扫描，按扫描时间插值；缺失/无效射线不能被当成无回波清空空间。
3. 原子获取三状态观测地图快照，将场景真值隔离在传感器与独立评估器之外。只执行旧的已知几何规划器不能算接入成功。
4. 先离线重放真实录制帧，再运行仅记录提案、不向飞机发指令的旁路节点，核对状态与几何信息流。
5. 将 `plan` 提案接入 `execution.dispatch` 重验证，使用实际下发时刻维护缓存；失败交给明确的监督逻辑，不能把停止记录当成飞机可悬停。
6. 新建单独的启动入口完成仿真闭环，保存真实规划耗时、观测到指令延迟、地图/状态/验证年龄、缓存复用及失败原因。

纯 Python `.venv` 不会自动解决 Noetic Python 模块兼容性。跨版本 Python 导入不应作为默认接入方式；根据实际环境选择独立进程通信或 C++ 移植，先证明接口正确再做性能优化。重建代码尚未证明满足 20 ms 下发时限，不能通过把计算耗时记为零来满足它。

## 9. 下一阶段实验与论文接续

Ubuntu 软件与数值诊断已完成。接续原稿实验时，先使用 [恢复证据包](../evidence_recovered/USAGE.txt) 中的三组正式 compact 模型、九次训练记录、最终选模记录及原始结果。包内分析脚本可重算主表与按地图配对的 bootstrap；首个预定 cul-de-sac 案例的原始轨迹和逐周期日志也已恢复。进一步实验使用新输出目录并冻结协议，保留既有档案。

若继续开发本目录的 NumPy 参考实现，下一步仍是第 8 节的新适配器、异步数据链路和真实时间记录。原始 C++/ROS 系统的恢复没有自动完成这项接入。新模型需要独立的训练、验证和闭环选模，不能将本次 smoke 模型视为原稿 Selected。

完整当前 LaTeX 工程和原修订工作稿没有随本次代码提交上传。做论文修改前，先从用户提供的文件中找到并确认当前版本；缺失时可以继续代码验证，但不能用仓库旧 `paper/main.tex` 冒充最新稿。原稿主表的聚合结果已由恢复档案独立重算，核验记录见 [PACKAGING_CHECKS.json](../evidence_recovered/PACKAGING_CHECKS.json)；这与重新执行全部历史闭环任务是不同步骤。新运行结果独立报告，附源码/配置/模型/输入哈希和失败记录。

每阶段结束，在 `FW_RUN` 中写一份简短交接记录：运行命令、版本、输入输出路径、成功/失败状态、实际指标、未完成工作及下一条操作。`runs/` 已被忽略，不会自动进入 Git。提交修复和说明时检查差异，按当前用户授权推送分支；不要把大体积原始记录、权重或机器路径混入普通代码提交。
