# Step2 准备 Pi3 条件文件

对应代码：

```text
step2_prepare_pi3_conditions.py
```

## 作用

Pi3 不只需要图片，还需要每一帧对应的相机位姿和相机内参。本步骤会根据采集数据生成 Pi3 需要的：

```text
condition_robot_poses_intrinsics.npz
```

同时会把原始图片去畸变后保存到输出目录中的 `images` 文件夹。

## 输入

默认输入来自当前主线数据：

```text
data/liver_data/frames
data/liver_data/samples.jsonl
data/handeye_compare/calib_handeye_compare.npz
data/calib_result/camera_calibration.npz
```

也可以指定新的 session，例如：

```powershell
--source-dir "E:\research\FYP\2026FYP\src\tactile_pipeline\session10\frames"
--samples-jsonl "E:\research\FYP\2026FYP\src\tactile_pipeline\session10\samples.jsonl"
```

## 运行

```powershell
cd E:\research\FYP\2026FYP\src\tactile_pipeline

& E:\research\FYP\vggt_env\Scripts\python.exe step2_prepare_pi3_conditions.py
```

指定 session10：

```powershell
& E:\research\FYP\vggt_env\Scripts\python.exe step2_prepare_pi3_conditions.py `
  --source-dir "E:\research\FYP\2026FYP\src\tactile_pipeline\session10\frames" `
  --samples-jsonl "E:\research\FYP\2026FYP\src\tactile_pipeline\session10\samples.jsonl"
```

## 输出

输出目录默认在：

```text
E:\research\FYP\2026FYP\src\camera\runs\pi3_<session>_frames_n<数量>
```

主要文件：

```text
images
condition_robot_poses_intrinsics.npz
condition_manifest.json
```

## 说明

`poses` 是相机在 base 坐标系下的位姿，`intrinsics` 是每一帧对应的相机内参。后续 Pi3 会用这些信息把重建结果对齐到 robot/base 坐标系。
