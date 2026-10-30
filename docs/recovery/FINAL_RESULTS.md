# 审稿修订：论文、实验与复现

当前稿件：[论文 PDF](../../paper/main.pdf) · [单一 main.tex](../../paper/main.tex)。

本轮新增训练消融、验证集闭环选择、延迟推理、紧凑网络、公共数值前缀检查、独立新测试地图和城市配置对照。完整结果及复现入口集中在[修订说明](revision/README.md)，逐项回应见[审稿回复](revision/RESPONSE_TO_REVIEW.txt)。

主要结果：选定State+Aux在32张新地图×4种条件下，关闭/开启公共复核分别完成107/118项任务；开启后未出现膨胀边界侵入。同样开启复核的Diverse完成89项，Full完成96项。所有方法的失败均进入统计。模型选择不使用新测试结果。

城市15个配置试验明确区分原高度库、相对高度层和48.825/15 m航点门限。规划仍使用已知建筑几何；LiDAR记录与体素显示不作为传感器地图驱动规划的证据。

先前论文对应的结果保存在[修订前结果记录](revision/ARCHIVED_RESULTS_BEFORE_REVIEW.md)，原始数据和模型仍在[learning](learning/README.md)。森林、城市LiDAR图和原生Gazebo/RViz截图均保留，并注明各自配置与实验作用。

编译与单文件ZIP制作见[paper/README.md](../../paper/README.md)。
