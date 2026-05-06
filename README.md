# Tactile Pipeline

用于 FYP 触觉头与三维重建实验的主线代码。流程以 Pi3 为核心：从棋盘格标定出相机内参和手眼关系，采集图片和机器人位姿后，生成对齐到 robot/base 坐标系的彩色点云，并与 STL/CAD 模型进行对齐和误差比较。

## 路径配置（唯一入口）

**每次换 session 只需修改 `run_pi3_session.py` 中的路径区，所有脚本自动跟随。**

```python
# run_pi3_session.py 关键路径
SOURCE_DIR    = Path(r"...\sessionXX\frames")       # 图片帧目录
SAMPLES_JSONL = Path(r"...\sessionXX\samples.jsonl") # 机器人位姿记录
HAND_EYE      = Path(r"...\sessionXX\handeye_compare\calib_handeye_compare.npz")
CALIB         = Path(r"...\sessionXX\calib_result\camera_calibration.npz")
```

各脚本与 `run_pi3_session.py` 的路径依赖关系：

| 脚本 | 使用变量 | 自动推导 |
|---|---|---|
| `0.1` | `SOURCE_DIR` | 图片目录 / 输出目录 |
| `0.3` | `SOURCE_DIR`, `SAMPLES_JSONL`, `CALIB`, `HAND_EYE` | session 目录、相机 JSON、输出目录 |
| `step2–7` | 全部 | — |

## 主流程

```text
Step 0.1  相机内参标定：棋盘格图片 → camera_calibration.npz / .json
Step 0.3  手眼标定：samples.jsonl + 内参 → calib_handeye_compare.npz
Step 1    启动相机服务和 UR3 上位机，采集图片 + samples.jsonl
Step 2    准备 Pi3 conditions：去畸变图片 + camera poses + intrinsics
Step 3    运行 Pi3 重建，输出对齐到 base 坐标系的彩色点云
Step 4    查看 Pi3 彩色点云
Step 5    交互式对齐 STL/CAD 与 Pi3 点云（正式位姿 + 调试）
Step 6    计算 CAD 与 Pi3 点云误差
Step 7    实时触觉接触检测
```

## 文件说明

| 文件 | 说明 |
|---|---|
| `run_pi3_session.py` | **路径配置总入口**，所有 step 从此 import 路径 |
| `0.1_capture_checkerboard_polaris_iteration_new.py` | 相机内参标定（棋盘格图片 → npz/json） |
| `0.3_camera_handeye_calib_session02_fixed_metrics.py` | 手眼标定（5 种方法对比，自动选最优） |
| `step1_start_camera_and_ui.py` | 一键启动 `camera_service` 和 `UR3Demo.UI` |
| `step2_prepare_pi3_conditions.py` | 生成 Pi3 conditions（去畸变图 + poses + intrinsics） |
| `step3_run_pi3_reconstruction.py` | 调用 Pi3 `example_mm_aligned.py`，生成 aligned 彩色点云 |
| `step4_view_pi3_pointcloud.py` | 查看最新 Pi3 点云 |
| `step5_interactive_align_cad_pi3.py` | 交互式调节 CAD，用于调试偏差 |
| `step5_6_align_cad_and_visualize.py` | 正式 CAD 位姿计算 + 可视化 |
| `step6_compare_cad_pointcloud.py` | 计算 CAD 与点云误差和覆盖率 |
| `step7_realtime_tactile_contact.py` | 实时显示触觉头与点云接触关系 |

## 环境

推荐 Python 解释器（含 Pi3/VGGT 依赖）：

```text
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

Pi3 代码默认路径（可在 `run_pi3_session.py` 中修改）：

```text
E:\research\FYP\5.4\Pi3
```

## 快速运行

### Step 0.1 相机内参标定

```powershell
cd E:\research\FYP\2026FYP\src\tactile_pipeline

& E:\research\FYP\vggt_env\Scripts\python.exe 0.1_capture_checkerboard_polaris_iteration_new.py
```

路径从 `run_pi3_session.py` 自动读取（`SOURCE_DIR` → 图片目录，输出到 `sessionXX\calib_result\`）。

### Step 0.3 手眼标定

```powershell
& E:\research\FYP\vggt_env\Scripts\python.exe 0.3_camera_handeye_calib_session02_fixed_metrics.py
```

自动对比 TSAI / PARK / HORAUD / ANDREFF / DANIILIDIS 五种方法，按固定棋盘一致性选最优，输出到 `sessionXX\handeye_compare\`。

### Step 1 启动采集系统

```powershell
& C:\Users\lenovo\AppData\Local\Programs\Python\Python310\python.exe step1_start_camera_and_ui.py
```

### Step 2–3 准备 conditions 并运行 Pi3 重建

```powershell
& E:\research\FYP\vggt_env\Scripts\python.exe step3_run_pi3_reconstruction.py
```

Step2 conditions 不存在时会自动先生成。8GB 显存帧数较多时可加 `--interval 2`。

### Step 4 查看点云

```powershell
& E:\research\FYP\vggt_env\Scripts\python.exe step4_view_pi3_pointcloud.py
```

### Step 5 对齐 CAD

```powershell
# 正式位姿计算 + 可视化
& E:\research\FYP\vggt_env\Scripts\python.exe step5_6_align_cad_and_visualize.py

# 交互式调试（仅调试用）
& E:\research\FYP\vggt_env\Scripts\python.exe step5_interactive_align_cad_pi3.py
```

### Step 6 计算误差

```powershell
& E:\research\FYP\vggt_env\Scripts\python.exe step6_compare_cad_pointcloud.py `
  --cad "E:\research\FYP\2026FYP\src\tactile_pipeline\phantom vs VGGT.stl" `
  --cad-scale 0.001 `
  --threshold 0.003
```

### Step 7 实时触觉接触检测

```powershell
& E:\research\FYP\vggt_env\Scripts\python.exe step7_realtime_tactile_contact.py
```

机器人 IP 在 `run_pi3_session.py` 中配置（`ROBOT_IP`）。

## 数据与大文件

仓库只保留代码和轻量配置，以下内容不提交 git：

```text
session*/          # 采集数据（图片、点云、标定结果）
camera/runs/       # 生成文件
__pycache__/
```
