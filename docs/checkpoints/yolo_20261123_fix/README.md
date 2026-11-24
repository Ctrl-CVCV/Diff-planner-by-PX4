# YOLO 2026-11-23 fix_v8n 模型检查点

> 本检查点已被 [2026-11-24 检查点](../yolo_20261124/README.md) 取代，仅作历史记录。

本检查点记录 2026-11-23 将检测模型从 `20261123_best_v8n` 更新为 `20261123_fix_best_v8n`
的过程与校验结果；不包含模型二进制（见 `.gitignore` 对 `*.engine` 的忽略策略），
也不表示已完成飞行验收。

## 模型与部署

- 源权重：`/home/nvidia/yolo/20261123_fix_best_v8n.pt`（SHA256 见 `verification.json`）。
- 导出产物：`20261123_fix_best_v8n.onnx`、`20261123_fix_best_v8n.engine`。
- 生效路径：`/home/nvidia/yolo/best.pt`、`/home/nvidia/yolo/best.engine`，与
  `yolo_trt_ros` 的 `launch/yolo_trt.launch` 默认 `engine_path` 一致。
- 类别：`0=danger_sign, 1=other_warning, 2=head`；launch 显示名映射为
  `危险品标识,其它标识,救助人员`。

## 导出参数（与上一版引擎保持一致）

```bash
cd /home/nvidia/yolo
PYTHONNOUSERSITE=1 ./venv/bin/python -u - <<'EOF'
from ultralytics import YOLO
m = YOLO('./20261123_fix_best_v8n.pt')
m.export(format='engine', device=0, half=True, imgsz=640, workspace=4, batch=4, nms=True)
EOF
```

环境：Jetson Orin，ultralytics 8.4.192，torch 2.1.0a0+41361538.nv23.06，
TensorRT 8.5.2.2，CUDA 0。引擎构建耗时约 562 s，构建期仅有 FP16 subnormal
权重告警（与上一版同类）。

## 验证

- TensorRT 反序列化通过；绑定 `images (4,3,640,640) FLOAT`、`output0 (4,300,6) FLOAT`，
  满足 `scripts/yolo_trt_node.py` 的静态形状与 float32 绑定要求。
- 用节点自身 `YoloEngine` 代码路径（系统 Python 3.8 + numpy 1.17.4 + TRT 8.5.2.2）实跑
  合成帧，batch=4 耗时约 30–38 ms；合成图无真实目标，未产生检测框符合预期。
- 元数据内嵌类别名与部署标签一致，`nms=true`、`quantize=16`、`workspace=4`、
  `batch=4`、`imgsz=[640,640]` 与旧引擎相同。

## 回滚

替换前的模型已备份（保留原 mtime）：

```bash
cp -p /home/nvidia/yolo/best.pt.bak-20261123-2032 /home/nvidia/yolo/best.pt
cp -p /home/nvidia/yolo/best.engine.bak-20261123-2032 /home/nvidia/yolo/best.engine
```

备份文件 SHA256 见 `verification.json`。engine 与 GPU 架构及 TensorRT 版本绑定
（本检查点为 Orin + TRT 8.5.2.2），换机部署需用 `.pt` 按上述参数重新导出；
ONNX 仅作中间产物，无需入库。