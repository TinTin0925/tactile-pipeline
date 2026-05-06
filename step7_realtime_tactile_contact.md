# Step7 实时触觉接触检测

对应代码：

```text
step7_realtime_tactile_contact.py
```

## 作用

这一步用于将触觉头的实时位姿叠加到 Pi3 点云中，并判断触觉头是否接触到点云表面。

脚本会读取：

```text
Pi3 点云
手眼标定
camera-to-tactile 标定
实时或回放的 T_base_tool
```

然后使用 KDTree 查询触觉头位置附近最近的点云点。

## 运行

默认运行：

```powershell
cd E:\research\FYP\2026FYP\src\tactile_pipeline

& E:\research\FYP\vggt_env\Scripts\python.exe step7_realtime_tactile_contact.py
```

指定点云：

```powershell
& E:\research\FYP\vggt_env\Scripts\python.exe step7_realtime_tactile_contact.py `
  --cloud "E:\research\FYP\2026FYP\src\camera\runs\pi3_session10_frames_n8\pi3_session10_frames_n8_i1_aligned_projected_rgb.ply"
```

## 输出

脚本会打开 Open3D 窗口，并在终端输出接触距离、最近点位置和状态。

## 注意

这一步依赖触觉头外参。如果后续触觉头安装位置改变，需要重新确认 camera-to-tactile 或 tool-to-tactile 标定。
