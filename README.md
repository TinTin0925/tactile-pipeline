# Tactile Pipeline

这是用于 FYP 触觉头与三维重建实验的主线代码。当前主流程已经从 VGGT/COLMAP/SuperPoint 迁移到 Pi3：使用采集图片、机器人位姿、手眼标定和相机内参，生成对齐到 robot/base 坐标系的彩色点云，并与 STL/CAD 模型进行对齐和误差比较。

## 主流程

```text
Step1  启动相机服务和 UR3 上位机
Step2  准备 Pi3 条件文件：去畸变图片 + camera poses + intrinsics
Step3  运行 Pi3 重建并对齐到 robot/base 坐标系
Step4  查看 Pi3 彩色点云
Step5  交互式对齐 STL/CAD 和 Pi3 点云
Step6  计算 CAD 与 Pi3 点云误差
Step7  实时触觉接触检测
```

## 文件说明

| Step | 代码 | 说明 |
| --- | --- | --- |
| 1 | `step1_start_camera_and_ui.py` | 一键启动 `camera_service` 和 `UR3Demo.UI` |
| 2 | `step2_prepare_pi3_conditions.py` | 根据图片、`samples.jsonl`、手眼和内参生成 Pi3 conditions |
| 3 | `step3_run_pi3_reconstruction.py` | 调用 Pi3 的 `example_mm_aligned.py` 生成 aligned 彩色点云 |
| 4 | `step4_view_pi3_pointcloud.py` | 查看最新 Pi3 点云 |
| 5 | `step5_interactive_align_cad_pi3.py` | 手动调节 CAD 的平移、旋转和等比例缩放 |
| 6 | `step6_compare_cad_pointcloud.py` | 用保存的 CAD transform 计算误差和覆盖率 |
| 7 | `step7_realtime_tactile_contact.py` | 实时显示触觉头与点云接触关系 |

每个 Step 都有对应的中文说明 Markdown，例如：

```text
step3_run_pi3_reconstruction.md
```

## 环境

推荐使用已有 Pi3/VGGT 环境：

```powershell
E:\research\FYP\vggt_env\Scripts\python.exe
```

主要依赖：

```text
torch
opencv-python
open3d
numpy
plyfile
huggingface_hub
```

Pi3 代码默认路径：

```text
E:\research\FYP\5.4\Pi3
```

如果路径不同，可以在 Step3 中用参数覆盖：

```powershell
--pi3-root <Pi3目录>
--pi3-python <Python解释器>
```

## 快速运行

### Step1 启动采集系统

```powershell
cd E:\research\FYP\2026FYP\src\tactile_pipeline

& C:\Users\lenovo\AppData\Local\Programs\Python\Python310\python.exe step1_start_camera_and_ui.py
```

### Step3 直接运行 Pi3 重建

如果 Step2 的 conditions 不存在，Step3 会自动先生成。

```powershell
& E:\research\FYP\vggt_env\Scripts\python.exe step3_run_pi3_reconstruction.py
```

运行某个新 session：

```powershell
& E:\research\FYP\vggt_env\Scripts\python.exe step3_run_pi3_reconstruction.py `
  --source-dir "E:\research\FYP\2026FYP\src\tactile_pipeline\session10\frames" `
  --samples-jsonl "E:\research\FYP\2026FYP\src\tactile_pipeline\session10\samples.jsonl" `
  --force-conditions
```

8GB 显存下，如果帧数较多建议使用：

```powershell
--interval 2
```

### Step4 查看点云

```powershell
& E:\research\FYP\vggt_env\Scripts\python.exe step4_view_pi3_pointcloud.py
```

### Step5 手动对齐 STL/CAD

```powershell
& E:\research\FYP\vggt_env\Scripts\python.exe step5_interactive_align_cad_pi3.py `
  --translation-range-mm 150
```

点击 `Save Transform` 后会生成：

```text
E:\research\FYP\2026FYP\src\camera\runs\cad_pi3_interactive_align\T_cad_to_pi3_manual.json
```

### Step6 计算误差

```powershell
& E:\research\FYP\vggt_env\Scripts\python.exe step6_compare_cad_pointcloud.py `
  --cad "E:\research\FYP\2026FYP\src\tactile_pipeline\phantom vs VGGT.stl" `
  --cad-scale 0.001 `
  --cad-transform "E:\research\FYP\2026FYP\src\camera\runs\cad_pi3_interactive_align\T_cad_to_pi3_manual.json" `
  --threshold 0.003 `
  --out-dir "E:\research\FYP\2026FYP\src\camera\runs\cad_pi3_comparison"
```

## 数据与大文件

仓库中保留代码和轻量配置。采集图片、点云结果、`camera/runs`、Python 缓存、C# `bin/obj` 等生成文件不应提交到 git。

建议本地数据结构：

```text
tactile_pipeline/
  data/
  session10/
  picture/
```

这些目录可在本地使用，但默认不作为仓库主内容管理。

## 当前说明

旧的 VGGT、COLMAP、SuperPoint/SuperGlue 路线已经不是主流程。相关代码如果需要可以作为备选实验保留在历史版本中；当前 README 和 Step 文档以 Pi3 主线为准。
