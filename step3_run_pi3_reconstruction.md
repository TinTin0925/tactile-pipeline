# Step3 运行 Pi3 三维重建

对应代码：

```text
step3_run_pi3_reconstruction.py
```

## 作用

这一步调用同学提供的 Pi3 代码 `example_mm_aligned.py`，使用 Step2 生成的条件文件完成三维重建。

脚本会自动执行：

1. 如果 conditions 不存在，先运行 Step2 的准备逻辑
2. 调用 Pi3 重建
3. 使用 `--align_to_condition_pose` 对齐到 robot/base 坐标系
4. 将点云重新投影到图片上获得 RGB 颜色

## 运行

默认运行：

```powershell
cd E:\research\FYP\2026FYP\src\tactile_pipeline

& E:\research\FYP\vggt_env\Scripts\python.exe step3_run_pi3_reconstruction.py
```

运行 session10：

```powershell
& E:\research\FYP\vggt_env\Scripts\python.exe step3_run_pi3_reconstruction.py `
  --source-dir "E:\research\FYP\2026FYP\src\tactile_pipeline\session10\frames" `
  --samples-jsonl "E:\research\FYP\2026FYP\src\tactile_pipeline\session10\samples.jsonl" `
  --force-conditions
```

如果帧数较多、显存不足，可以使用：

```powershell
--interval 2
```

如果希望保留更多点：

```powershell
--max-points 1500000
```

## 输出

主要输出：

```text
pi3_<session>_frames_n<N>_i<interval>_aligned.ply
pi3_<session>_frames_n<N>_i<interval>_aligned_projected_rgb.ply
pi3_<session>_frames_n<N>_i<interval>_aligned_alignment.json
```

其中最常用的是：

```text
*_aligned_projected_rgb.ply
```

它是已经对齐到 base 坐标系并重新上色后的 Pi3 点云。
