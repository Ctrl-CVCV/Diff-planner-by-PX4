# 摄像头 120 FPS 预览、可见高度测试与 YOLO 图片采集

更新：2026-10-25。以下工具仅打开摄像头、显示图像、保存照片，不解锁、不起飞、不发送 PX4Ctrl 或 Planner 指令。

脚本：`~/gcs_ws/src/Diff-Planner/src/diff_planner/plan_manage/scripts/camera_yolo_capture.py`。

## 1. 先明确本次测试范围

- 默认按 Linux USB/UVC/V4L2 摄像头编写，设备 `/dev/video0`，请求 `MJPG / 1280×720 / 120 FPS`。如果实际是 CSI、深度相机或网络摄像头，须先确认其接口，不能直接假定 `/dev/video0` 可用。
- **软件明确请求 120 FPS，但不能把不支持的摄像头强制变为 120 FPS。** 驱动协商值不匹配时默认拒绝继续；协商成功后还会统计实际读取帧率。二者都不是硬件曝光时间戳验证。
- 当前可访问的开发环境未发现 `/dev/video*`，因此尚未验证相机型号、可用模式或真实 120 FPS；本文的默认分辨率是测试起点，不是已验证的硬件规格。
- 本机另一个工作区 `~/qwen_vl_ros_ws` 的 `camera_publisher.py` 当前参数校验上限为 60 FPS，直接给那个节点传 `fps:=120` 会被拒绝。本工具独立运行，不修改它，也不运行 Qwen/YOLO 推理。**同一摄像头不要同时被两个采集程序打开。**
- 优先在卸桨/锁桨条件下把相机举到计划高度测试视野、清晰度和帧率。此前带动力时存在定位/链路风险；不要用视觉测试脚本顺便验证自动飞行。

## 2. 查找设备和支持的 120 FPS 模式

在连接相机的 Jetson/Ubuntu 上运行：

```bash
ls -l /dev/video* /dev/v4l/by-id/
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --list-formats-ext
v4l2-ctl -d /dev/video0 --list-ctrls-menus
```

如果缺少命令或 OpenCV，可在地面安装相应依赖：

```bash
sudo apt install v4l-utils python3-opencv
PYTHONNOUSERSITE=1 /usr/bin/python3 -c 'import cv2; print(cv2.__version__)'
```

从 `--list-formats-ext` 中找出**同时支持指定像素格式、分辨率和约 120 FPS** 的组合；不是看到某个模式有 120，就代表所有分辨率都支持。若仅 `320×240` 有 120 FPS，应同步把命令的 `--width/--height` 改成该模式。若支持的格式不是 MJPG，使用设备实际列出的四字符格式，例如 `--fourcc YUYV`，不要凭猜测修改。

默认请求已提高到 `1280×720`。若设备列出的模式还支持 `1920×1080 / 120 FPS`，可把下面预览和采集命令中的分辨率同时改为 `--width 1920 --height 1080`；更高分辨率同样通过这两个参数指定。脚本保留相机原始尺寸，不把低分辨率图像放大冒充高清；驱动协商尺寸不匹配会报错。若高清模式只能达到 30/60 FPS，就不能同时满足高清和真实 120 FPS，需要改用设备确实支持的模式。

多个 `/dev/videoN` 可能属于同一设备，其中并非每个都是图像采集节点。尽可能使用 `/dev/v4l/by-id/` 中对应图像节点的稳定路径传给 `--device`。打不开时先检查用户所属组、设备权限和占用程序；不要默认用 sudo 跑整个 Python 脚本。

## 3. 持续观看实时图像

在连接显示器的 Jetson 桌面终端执行：

```bash
CAMERA_SCRIPT="$HOME/gcs_ws/src/Diff-Planner/src/diff_planner/plan_manage/scripts/camera_yolo_capture.py"
PYTHONNOUSERSITE=1 /usr/bin/python3 "$CAMERA_SCRIPT" \
  --mode preview \
  --device /dev/video0 \
  --width 1280 --height 720 \
  --fps 120 --fourcc MJPG
```

- 预览一直持续，按窗口中的 `q`、`Esc` 或终端 `Ctrl+C` 退出并释放相机；预览模式不保存照片。
- 不需要启动 EKF、PX4Ctrl、Planner 或 ROS master。图像直接来自相机，脚本本身不发布 `/usb_cam/image_raw`。
- 启动日志同时显示 `requested`（请求值）、`actual`（驱动报告值）及各属性设置是否被驱动接受；运行时每秒打印读取 FPS。
- 相机请求 120 FPS；窗口默认最多绘制 30 FPS，以降低显示开销。`--preview-fps 120` 可把软件显示上限改为 120，但屏幕刷新率、桌面、解码和 CPU 负载仍可能限制实际可见刷新率。不能用窗口流畅度证明采集达到 120。
- 驱动报告 30/60 FPS 时默认报错退出，不会静默宣称成功。仅为排查图像、明确接受非 120 FPS 时可加 `--allow-fps-fallback`；数据记录仍保留实际协商值。
- 普通无图形 SSH 会话不能弹出本机预览窗口。优先在机载电脑本地桌面/远程桌面查看；X11 转发会额外消耗带宽，不能据此判断本地最高帧率。无显示环境可以运行第 5 节不带 `--preview` 的采集命令。

### 实测只有 60 多 FPS：先区分抓取、解码、显示与写盘

当前 `camera_yolo_capture.py` 没有 60 FPS 的限速器：`--fps 120` 用于请求驱动帧率，正常取帧循环没有按 60 Hz sleep。`--preview-fps` 仅控制窗口绘制频率。但取帧和窗口绘制仍在同一个线程内，绘制耗时会延迟下一次取帧；拍照时还存在图像复制、后台 JPEG 编码和写盘负载。因此运行日志的 `read_fps` 是整个循环得到解码图像的吞吐，不是独立测得的相机曝光帧率。

如果观察的是另一个工作区的 `/usb_cam/image_raw`，则须先确认发布程序：当前 `~/qwen_vl_ros_ws/src/qwen_vl_ros/scripts/camera_publisher.py` 参数校验最高为 60 FPS，循环也使用 `rospy.Rate(self.fps)`。这条旧 ROS 链路不能用于验证当前工具是否达到 120 FPS。不要仅凭 `rostopic hz` 或屏幕刷新效果认定相机本体只能跑 60。

先在地面关闭相机预览、ROS 相机节点及 YOLO/Qwen 推理，保持同一设备、分辨率、格式、光照和尽量相同的系统负载，**依次**运行下面两条命令，不要同时打开设备：

```bash
CAMERA_SCRIPT="$HOME/gcs_ws/src/Diff-Planner/src/diff_planner/plan_manage/scripts/camera_yolo_capture.py"

# A：只 grab，不取出解码图像，不显示、不写照片
PYTHONNOUSERSITE=1 /usr/bin/python3 "$CAMERA_SCRIPT" \
  --mode benchmark --grab-only --device /dev/video0 \
  --width 1280 --height 720 --fps 120 --fourcc MJPG --duration 10

# B：正常 read 取出解码图像，不显示、不写照片
PYTHONNOUSERSITE=1 /usr/bin/python3 "$CAMERA_SCRIPT" \
  --mode benchmark --device /dev/video0 \
  --width 1280 --height 720 --fps 120 --fourcc MJPG --duration 10
```

测速会打印 `结果` JSON，不创建图片目录：A 看 `grab_fps`，B 看 `read_fps`。同时报告 OpenCV 版本/模块路径、驱动协商值、单次 grab/read 平均和最大耗时 `receive_mean_ms`/`receive_max_ms`。其中耗时包括等待下一帧及后端处理，不能直接当成 CPU 解码耗时。`--grab-only` 只能用于 benchmark，不能用于图片采集，也不允许同时预览。

结果判读是排查方向，不是未经复测的最终归因：

| 现象 | 优先检查 |
| --- | --- |
| 驱动协商报告约 60，未达到请求 120 | 当前格式/分辨率的支持模式、帧间隔协商；不是改一个显示参数就能解决 |
| A 接近 120，B 只有约 60 | 图像取出/解码、颜色转换或相关软件处理开销；优先排查 OpenCV 及 CPU 负载 |
| A/B 都接近 120，开启预览后约 60 | 图像复制、窗口绘制、桌面/远程显示开销；降低 `--preview-fps` 再对照，不要降低相机请求 FPS |
| B 接近 120，拍照时保存数明显不足且 `queue_dropped > 0` | JPEG 编码或写盘吞吐；接收数、入队数和保存数要分开看，可用 `--save-fps 10` 对照 |
| A/B 都只有约 60 | 仍不能断言相机损坏；继续隔离 OpenCV、驱动、曝光策略、设备模式和传输链路 |

若 A/B 都低，退出脚本后再用 V4L2 工具绕过 Python/OpenCV/窗口/JPEG 重编码做十秒左右的缓冲接收对照（运行时间取决于实际帧率，1200 帧在 60 FPS 时约需 20 秒）：

```bash
v4l2-ctl -d /dev/video0 \
  --set-fmt-video=width=1280,height=720,pixelformat=MJPG \
  --set-parm=120 \
  --stream-mmap=4 --stream-count=1200 --stream-to=/dev/null
```

先确认设备确实列出这个模式；命令修改该设备的采集设置，勿在其他程序采集中运行。若工具流接收接近 120 而 A/B 不足，应优先检查软件后端/运行负载；若都不足，则继续查询 `--list-ctrls-menus`、相机说明及连接状态，不要在不知道控制项单位和取值含义时直接强制写曝光参数。`grab_fps` 和工具流 FPS 均仅表示缓冲接收速度，不能证明每帧曝光独立、没有重复帧或内容质量达标。

请保留两次测速完整输出及 `v4l2-ctl --list-formats-ext` 的结果；没有这些数据，不能把“60 多”直接归因于相机硬件或宣称已通过软件修好。

### 如果相机已经通过 ROS 发布图像

先确认实际话题，再用 ROS 图像查看器订阅，不要同时启动上面的直连工具：

```bash
source ~/gcs_ws/devel/setup.bash
rostopic list | grep -E 'image_raw|image_rect|image/compressed'
rostopic type /usb_cam/image_raw
rostopic hz /usb_cam/image_raw --window=120
rosrun image_view image_view image:=/usb_cam/image_raw
```

把话题替换为真实的 `sensor_msgs/Image` 话题。这里的查看器不设置相机采样率，`rostopic hz` 测量的是 ROS 消息到达率；上游如果只有 30 FPS，查看器不会将它变为真实 120 FPS。本次新增采集脚本使用 V4L2 直连，不订阅该 ROS 话题；需要先停相机发布节点，或保留原驱动并另行使用 ROS 数据采集方案。

## 4. 怎么找“目标仍可看清”的无人机高度

不要只判断“有画面”，还要确认目标足够大、清晰且完整：

1. 将实际比赛目标/标志物放到真实背景和光照下，使用正式安装角度、镜头及分辨率。先在地面手持相机/卸桨机体测试。
2. 从低处逐级增加离地高度或目标距离，例如每次增加 0.2 m；每个位置固定观察数秒，检查目标是否完整进入画面、最小目标的像素尺寸、运动模糊、过曝和暗部噪点。
3. **下视相机**主要检查覆盖范围和目标像素大小；**前视相机**还必须记录目标距离及相机俯仰角，高度单独一个数不能决定能否看见。
4. 在每个高度采一组图，给命令加 `--height-m 0.8` 等人工标签。这个参数只记录高度，不控制飞机、不读取测距，也不等于自动测得的 AGL。
5. 记录下表，最后用真正训练/验证的 YOLO 模型验证；肉眼可见不等于模型可靠检出。不要把“本次最高能看见的高度”直接当成比赛安全高度。

| 人工测量离地高度 | 目标距离/相机角度 | 最小目标像素宽高 | 是否完整清晰 | 读取 FPS / 是否掉图 | YOLO 验证结果 |
|---|---|---|---|---|---|
| 例如 0.8 m | 按实际填写 | 按保存的原图检查 | 按实际填写 | 看日志/session.json | 模型验证后填写 |

若需要同时观察定位，可另开只读终端：

```bash
source ~/gcs_ws/devel/setup.bash
rostopic echo /ekf/ekf_odom/pose/pose/position
```

其中 z 是当前 EKF/world 坐标，不天然是离地高度。平地静态测试可记录地面起点 z，再比较 `z - z_ground`；这仍不是遇到台阶、坡面或障碍时的真实 AGL。不要为了视觉试验重置 EKF/TF。

120 FPS 的单帧周期约 8.33 ms；长曝光、光线不足、自动曝光降速和运动模糊都需要实际检查。曝光控制名、单位和可用范围因设备而异，本文不提供未经相机验证的通用写入值。先查询驱动控制项，再按相机说明调整。

## 5. 十秒内连续拍照，保存 YOLO 原始素材

先关闭预览进程/ROS 相机节点以释放相机，再运行：

```bash
CAMERA_SCRIPT="$HOME/gcs_ws/src/Diff-Planner/src/diff_planner/plan_manage/scripts/camera_yolo_capture.py"
PYTHONNOUSERSITE=1 /usr/bin/python3 "$CAMERA_SCRIPT" \
  --mode capture \
  --device /dev/video0 \
  --width 1280 --height 720 \
  --fps 120 --fourcc MJPG \
  --duration 10 \
  --height-m 0.8 \
  --output "$HOME/vision_datasets/yolo_collect"
```

- 默认预热 1 秒，预热后开始最多 10 秒的接收采集窗口。`--duration` 可缩短，但不允许大于 10；只保存窗口内新读取的图像，不重复上一帧来补数。
- 打开相机、预热以及窗口结束后的队列落盘不包含在这 10 秒内，所以**进程总运行时间可以超过 10 秒**。相机驱动 `read()` 若阻塞，也可能推迟进程退出；阻塞后在窗口外才返回的图像不会保存。本工具不是硬实时计时设备。
- 默认 `--save-fps 0`，保存每个实际接收且成功入队的帧。真实持续 120 FPS 时，10 秒名义上约 1200 张，不能保证固定张数；请看 `saved`、读取 FPS 和丢帧统计。
- 若想边采边看，加 `--preview`；预览的文字只画在显示副本上，保存 JPG 保持原始尺寸，没有 FPS/高度文字污染训练图。
- 如只需要稀疏的训练照片，追加 `--save-fps 10`，表示在相机仍请求 120 FPS 的情况下最多每秒选 10 张新帧；不会补帧。这样能减少近重复图和磁盘负载，但不再是保存全部 120 FPS 帧。
- `--height-m` 可省略，不要填写虚构高度。采集位置变化时建议停止一组后换高度重新采，避免一组十秒内高度变化却都标成同一个人工高度。

每次命令新建独立目录，不覆盖前一组：

```text
~/vision_datasets/yolo_collect/<日期时间_唯一后缀>/
  images/
    frame_000000.jpg
    frame_000001.jpg
    ...
  frames.jsonl
  session.json
```

`frames.jsonl` 为每张成功保存的图片记录文件名、接收序号、窗口内时间、主机接收返回时间、原图尺寸及人工高度标签。该时间不是曝光时间，也未与 EKF 做时间同步。

`session.json` 记录请求/协商设置、实际读取 FPS、接收数 `received`、入队数 `submitted`、成功保存数 `saved`、队列丢帧 `queue_dropped`、主动抽帧 `rate_skipped`、读取失败数及结束原因。正常完整采集应为 `status: complete`；按 q/Ctrl+C 或异常退出的组不能当作完整十秒数据。

写盘使用有限后台队列，默认最多 32 帧；满时丢弃新帧并计数，而不是无限堆积内存。`--queue-size` 可调整，但加大队列不能解决持续的磁盘/编码吞吐不足。持续读取低于请求的 90% 或发生队列丢帧会给出警告；接收帧率达标也不等于全部图片成功写盘。

```bash
df -h "$HOME"
ls -td "$HOME/vision_datasets/yolo_collect/"*/ | head
```

先完成地面十秒测试并检查结果，再评估与定位/控制同时运行时的资源影响。120 FPS 解码、显示和 JPEG 编码会占用 CPU、内存及 USB/磁盘带宽；有限队列不能保证不影响飞控相关进程。不要在已有定位断更问题的情况下直接叠加高负载飞行试验。

## 6. 后续整理为 YOLO 数据集

这是采集脚本，不是检测或自动标注脚本；它不会生成伪造标签、空的全背景标签，也不会启动 YOLO。

1. 检查原图尺寸、目标可辨识度和模糊程度，挑选有代表性的正样本及真实负样本。
2. 用标注工具导入 `images/`，统一类别名称与编号，导出 YOLO detection 格式。
3. 每个有框图像的同名 `.txt` 每行通常为 `class_id x_center y_center width height`，坐标相对图像宽高归一化；按实际任务的 YOLO 版本/检测或分割格式核对，不要混用。
4. 按场次/高度/背景划分训练和验证集，避免把同一次十秒连拍的相邻近重复帧随机分到两边，造成验证结果虚高。
5. 保留 `session.json`、`frames.jsonl` 和原始图，标注后再整理 `images/train`、`images/val`、`labels/train`、`labels/val` 和数据集配置。文件名 `frame_000000.jpg` 会在每组重新编号，合并前加场次前缀，并保持图片与标签同名，避免覆盖。

参考接口说明：

- OpenCV VideoCapture 属性及后端差异：https://docs.opencv.org/4.x/d4/d15/group__videoio__flags__base.html
- Linux V4L2 帧间隔协商：https://docs.kernel.org/userspace-api/media/v4l/vidioc-g-parm.html
- Ultralytics YOLO detection 数据集格式：https://docs.ultralytics.com/datasets/detect/
