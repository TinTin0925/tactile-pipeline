# Step6 比较 CAD 和 Pi3 点云误差

对应代码：

```text
step6_compare_cad_pointcloud.py
```

## 作用

这一步用于计算 Pi3 点云和 STL/CAD 模型之间的几何误差。它会输出：

```text
cloud 到 CAD 的距离
CAD 到 cloud 的距离
覆盖率
彩色误差点云
报告 JSON
```

## 推荐流程

先在 Step5 中手动对齐 CAD，保存：

```text
T_cad_to_pi3_manual.json
```

然后在 Step6 中使用这个 transform 进行正式比较。

## 运行

```powershell
cd E:\research\FYP\2026FYP\src\tactile_pipeline

& E:\research\FYP\vggt_env\Scripts\python.exe step6_compare_cad_pointcloud.py `
  --cad "E:\research\FYP\2026FYP\src\tactile_pipeline\phantom vs VGGT.stl" `
  --cad-scale 0.001 `
  --cad-transform "E:\research\FYP\2026FYP\src\camera\runs\cad_pi3_interactive_align\T_cad_to_pi3_manual.json" `
  --threshold 0.003 `
  --out-dir "E:\research\FYP\2026FYP\src\camera\runs\cad_pi3_comparison"
```

## 输出

```text
cad_compare_report.json
cloud_colored_by_cad_distance.ply
cad_colored_by_cloud_coverage.ply
combined_cloud_cad_comparison.ply
cloud_aligned_to_cad.ply
```

## 查看结果

```powershell
& E:\research\FYP\vggt_env\Scripts\python.exe view_pointcloud.py `
  "E:\research\FYP\2026FYP\src\camera\runs\cad_pi3_comparison\combined_cloud_cad_comparison.ply" `
  --point-size 2 `
  --background 255,255,255 `
  --show-frame
```

## 说明

如果没有提供 `--cloud`，脚本会默认使用最新的 Pi3 点云。`--cad-scale 0.001` 表示 STL 单位是毫米，需要转换到米。
