# Step4 查看 Pi3 点云

对应代码：

```text
step4_view_pi3_pointcloud.py
```

## 作用

这一步用于快速打开最新生成的 Pi3 点云。脚本会自动在 `camera/runs` 中寻找最新的：

```text
pi3_*/*projected_rgb.ply
```

## 运行

```powershell
cd E:\research\FYP\2026FYP\src\tactile_pipeline

& E:\research\FYP\vggt_env\Scripts\python.exe step4_view_pi3_pointcloud.py
```

指定某个点云：

```powershell
& E:\research\FYP\vggt_env\Scripts\python.exe step4_view_pi3_pointcloud.py `
  "E:\research\FYP\2026FYP\src\camera\runs\pi3_session10_frames_n8\pi3_session10_frames_n8_i1_aligned_projected_rgb.ply"
```

## 可选参数

```powershell
--point-size 2
--background 255,255,255
--no-show-frame
```

## 说明

默认会显示坐标轴。这个坐标轴主要用于观察点云位置，不代表 CAD 原点一定正确。
