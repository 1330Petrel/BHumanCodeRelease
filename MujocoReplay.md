# MuJoCo 原生球检测回放

该入口基于 PR #1（`6364143`）的 TSV/YUYV provider，增加真实 `RobotBallPerceptor`、544 ONNX 模型与原生 NMS/尺寸门控。它运行离线球感知，不是完整 K1/T2 机器人程序。

## 构建与静态检查

本机使用 Apple Clang、CMake Unix Makefiles、ONNX Runtime 1.22.0；沿用已经初始化的 SimRobot/Eigen 等子模块和 `Util/onnxruntime/lib/macOS`。已有环境配置见父目录 `.setup/README.md`。从 BHumanCodeRelease 根目录运行：

```sh
cmake -S Make/CMake -B Build/macOS/CMake/Develop -G 'Unix Makefiles' -DCMAKE_BUILD_TYPE=Develop
cmake --build Build/macOS/CMake/Develop --target MujocoReplay CheckThreads -j 6
DYLD_LIBRARY_PATH="$PWD/Util/onnxruntime/lib/macOS" Build/macOS/CheckThreads/Develop/CheckThreads "$PWD/Config/Scenarios/MujocoReplay/threads.cfg"
```

本次已在 macOS ARM64 构建与实跑。Linux 可使用仓库原 `Make/Linux/generate`，然后在对应 Linux CMake 目录构建 `MujocoReplay`；Linux 本轮未验收。Python 运行器可通过 `--binary` 指定其他平台的二进制路径。

## 一条命令导出并回放

从同级 robocup_mujoco 根目录执行（输出目录应为新的目录）：

```sh
/opt/miniconda3/envs/robocup_mujoco/bin/python -m mujoco_runtime.bhuman_replay --capture ../.setup/logs/week10-detector-dataset/capture --output ../.setup/logs/my-bhuman-replay --bhuman-root ../BHumanCodeRelease
```

输入是 native capture 的 `manifest.json`、`frames.jsonl` 和 RGB PNG。运行器用 B-Human 的整数全范围 RGB→YUV 系数，成对平均 U/V，输出 `{Y0,U,Y1,V}`。`CameraImage.width` 为像素对数272，`CameraInfo.width` 为真实像素数544。TSV 每行包含时间、宽高、fx/fy/cx/cy、3×3旋转、平移和球半径（mm）、YUYV相对路径。光学右/下/前转换为 B-Human 前/左/上；矩阵原点是机器人地面投影，保留真实相机高度。

跨 world reset 保留原采集间隔并映射为单调非零 FrameInfo；原始 `frame_id/reset_id/capture_time_ns` 保存在 `frame_keys.jsonl`。不依赖重新渲染或隐藏球真值来生成检测。

输出 `results.jsonl` 每输入帧一行：帧键、`available`、最终球观测 `detections`、post-NMS门控前 `candidates`、原生 BallPercept。框为 `xyxy`，最终球框由中心和半径构造；最终置信度为空，因为 BallPercept 不提供该字段。`latency_ms` 包含预处理、推理、解码/NMS与可选张量指纹，排除初始化、文件传输及最终尺寸门控。

每帧记录 YUYV SHA-256、输入 float 张量和原始 ONNX 输出 FNV-1a64；`run_info.json` 固定模型、二进制、源码、配置哈希。相同入口重复运行只验证确定性，不能称为与原 SimRobot 图的迁移等价。

## 近似与失败处理

当前 replay 明确使用空 T2 自身轮廓、无效 FieldBoundary（关闭该门控）、不分类球衣、1 mm²几何协方差；Settings 使用 K1，实际相机和100 mm球半径来自 T2 编译场景。这些近似不代表完整原生模块图已等价；球尺寸门控与原模型阈值实际执行。

缺输入、少输出或部分失败返回非零；正常结束必须输入数、提供帧数、输出数完全一致。缺第96帧的负例已验证返回1，保留前95帧且不重复旧观测。Headless Debug 有有效控制台接收端，错误不会访问 SimRobot 的空 GUI 接收端。
