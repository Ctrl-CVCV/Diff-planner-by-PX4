# YOLO 2026-11-24 best_v8n 模型检查点

本检查点记录 2026-11-24 将检测模型更新为 `20261124_best_v8n` 的过程与校验结果。
与上一检查点 `yolo_20261123_fix` 不同，本次 `.engine` 随 `yolo_trt_ros` 包入库并作为
launch 默认引擎路径，上一检查点自此被本版本取代。

## 模型与部署

- 源权重：`/home/nvidia/yolo/20261124_best_v8n.pt`（训练元数据 date `2026-11-23T22:16`，
  ultralytics 8.4.54；SHA256 见 `verification.json`）。
- 导出产物：`20261124_best_v8n.onnx`、`20261124_best_v8n.engine`。
- **入库路径（生效）**：`src/yolo_trt_ros/models/20261124_best_v8n.engine`，
  `launch/yolo_trt.launch` 的 `engine_path` 默认值改为
  `$(find yolo_trt_ros)/models/20261124_best_v8n.engine`；`.gitignore` 为该目录开例外
  （其余 `*.engine` 仍忽略）。
- 兼容副本：`/home/nvidia/yolo/best.pt`、`/home/nvidia/yolo/best.engine` 同步更新，
  供仍引用该路径的旧 launch（如 `qwen_vl_ros_ws` 内的副本）使用。
- 类别：`0=danger_sign, 1=other_warning, 2=head`；显示名映射仍为
  `危险品标识,其它标识,救助人员`。

## 导出参数（与前两版引擎保持一致）

```bash
cd /home/nvidia/yolo
YOLO_AUTOINSTALL=false PYTHONNOUSERSITE=1 ./venv/bin/python -u - <<'EOF'
from ultralytics import YOLO
m = YOLO('./20261124_best_v8n.pt')
m.export(format='engine', device=0, half=True, imgsz=640, workspace=4, batch=4, nms=True)
EOF
```

环境：Jetson Orin，ultralytics 8.4.192，torch 2.1.0a0+41361538.nv23.06，
TensorRT 8.5.2.2。引擎构建耗时约 599 s；无异常告警（仅常规 FP16 转换信息）。

## 验证

- `roslaunch --dump-params yolo_trt_ros yolo_trt.launch` 确认
  `/yolo_trt_detector/engine_path` 解析为包内 `models/20261124_best_v8n.engine`。
- TensorRT 反序列化通过；绑定 `images (4,3,640,640) FLOAT`、`output0 (4,300,6) FLOAT`，
  满足 `scripts/yolo_trt_node.py` 的静态形状与 float32 绑定要求。
- 用节点自身 `YoloEngine` 代码路径（系统 Python 3.8 + numpy 1.17.4 + TRT 8.5.2.2）实跑
  合成帧，batch=4 耗时约 38.8 ms；合成图无真实目标，未产生检测框符合预期。
- 入库副本与 `/home/nvidia/yolo` 源文件 SHA256 一致（`62c612a9…`）。

## 回滚

替换前文件已备份（保留原 mtime）：

```bash
cp -p /home/nvidia/yolo/best.pt.bak-20261124-1609 /home/nvidia/yolo/best.pt
cp -p /home/nvidia/yolo/best.engine.bak-20261124-1609 /home/nvidia/yolo/best.engine
git checkout 6189167 -- src/yolo_trt_ros/launch/yolo_trt.launch   # 恢复上一版默认路径
```

备份对应 `20261123_fix` 版本，SHA256 见 `verification.json`。engine 与 GPU 架构及
TensorRT 版本绑定（本检查点为 Orin + TRT 8.5.2.2），在其他机器部署时需用 `.pt`
按上述参数重新导出；ONNX 仅作中间产物，不入库。