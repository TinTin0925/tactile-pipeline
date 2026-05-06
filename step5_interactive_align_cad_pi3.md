# Step5 交互式对齐 CAD 和 Pi3 点云

对应代码：

```text
step5_interactive_align_cad_pi3.py
```

## 作用

这一步用于手动对齐 STL/CAD 模型和 Pi3 点云。界面中会显示 Pi3 点云和红色 CAD 点云，可以通过滑条调节：

```text
RX / RY / RZ
Scale
TX / TY / TZ
```

## 运行

```powershell
cd E:\research\FYP\2026FYP\src\tactile_pipeline

& E:\research\FYP\vggt_env\Scripts\python.exe step5_interactive_align_cad_pi3.py
```

指定 session10 点云：

```powershell
& E:\research\FYP\vggt_env\Scripts\python.exe step5_interactive_align_cad_pi3.py `
  --cloud "E:\research\FYP\2026FYP\src\camera\runs\pi3_session10_frames_n8\pi3_session10_frames_n8_i1_aligned_projected_rgb.ply" `
  --translation-range-mm 150
```

## 输出

点击 `Save Transform` 后会保存：

```text
E:\research\FYP\2026FYP\src\camera\runs\cad_pi3_interactive_align\T_cad_to_pi3_manual.json
```

点击 `Export Combined PLY` 后会保存：

```text
interactive_combined_cloud_cad.ply
```

## 后续

保存的 `T_cad_to_pi3_manual.json` 会在 Step6 中用于正式误差计算。
