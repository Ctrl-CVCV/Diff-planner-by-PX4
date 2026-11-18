# Faster-LIO 2026-11-17 当前最佳地面测试检查点

本检查点保存用户当前选定版本，不表示已完成飞行、运动精度或全系统长期负载验收。

- 降采样 `filter_size_surf: 0.20`、`filter_size_map: 0.40`。
- `point_filter_num: 3`，没有 IVox 每格点数上限，在线外参估计关闭。
- 标定外参保留；实际运行的 `~/fasterlio_ws` 副本仍使用 `-0.07625571` 秒静态时间补偿。
- 90 秒地面测试输入 10.000 Hz、输出 10.001 Hz，里程计消息年龄中位数 22.9 ms、P95 33.5 ms，未观察到持续积压。
- 测试同时重启清空队列，不能将全部改善归因于降采样参数。

## 历史检查点与当前统一版本

**以下描述针对历史提交 `1dbfede`，不是当前整理后的目录状态。** 当时主仓库中的 `src/Diff-Planner/src/realflight_modules/faster-lio/faster-lio` 保留官方恢复版；实际通过测试的进程使用 `~/fasterlio_ws` 的程序和动态库。当前已将实际运行源码和实机 YAML 迁回 `gcs_ws`，启动和编译见 [当前指南](../../FASTER_LIO_GUIDE.md)。切换仍须地面停止旧链、编译后重新启动。

`runtime.patch` 保存 `1dbfede` 中主仓库 Faster-LIO 包到当时实际运行副本的全部六处文件差异，包括飞行 YAML、时间补偿实现、自定义 launch 和 RViz 配置。它是归档，不会自动修改主仓库源码或运行节点。其中示例 `mid360.yaml` 是外部副本的历史配置，**不作为当前实机入口**；当前使用 `mid360_px4.yaml`。

仅在单独的历史复现克隆、已经检出 `1dbfede` 且工作区干净时，可在仓库根目录检查并应用（**不要向当前已经整合的代码再次应用**）：

```bash
git apply --check --directory=src/Diff-Planner/src/realflight_modules/faster-lio/faster-lio docs/checkpoints/faster_lio_20261117/runtime.patch
git apply --directory=src/Diff-Planner/src/realflight_modules/faster-lio/faster-lio docs/checkpoints/faster_lio_20261117/runtime.patch
```

之后仍须在选定的工作区重新编译、正确加载 `config/mid360_px4.yaml`，并核对程序和动态库来源。不要在现有脏工作区盲目应用，也不要在飞行中切换定位或启动第二套定位源。

`verification.json` 保存上述六处文件的原始/目标 SHA256 和原始 90 秒测量结果；不收录编译产物、二进制、rosbag、点云、运行日志或凭据。归档的是源码和配置，不包含完整系统依赖及 EKF 的瞬时内部状态。

重启定位会重置原点；飞行前需要重新初始化并检查 EKF/MAVROS 链路。当前使用与验收边界见 [当前指南](../../FASTER_LIO_GUIDE.md)。
