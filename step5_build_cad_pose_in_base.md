# Step5 正式计算 CAD/STL 在 base 坐标系下的位姿

对应代码：

```text
step5_build_cad_pose_in_base.py
```

## 作用

这一步用于正式实验中的 STL/CAD 对齐。它不是手动拖动模型，而是根据棋盘格/夹具在 robot/base 坐标系下的位姿，以及 STL 原点相对于棋盘格中心的固定几何关系，计算：

```text
T_cad_to_base
```

这样每次真实肝脏模型放到同一个棋盘格/夹具关系下时，STL 都会被放到 base 坐标系中的正确位置。

## 默认假设

默认使用：

```text
data/handeye_compare/summary.json
```

默认 STL 原点相对于棋盘格中心的位置为：

```text
(200, 0.5, -20) mm
```

其中坐标定义为：

```text
X: 平行棋盘格边，朝向肝脏模型方向
Y: 平行棋盘格边，远离机械臂方向
Z: 垂直向上
```

默认轴映射为：

```text
user X = checkerboard +X
user Y = checkerboard +Y
user Z = X cross Y
```

如果实际方向相反，可以通过参数修改，例如：

```powershell
--user-x-from-board=-x
--user-y-from-board=-y
```

PowerShell 中负号参数建议使用等号写法。

## 运行

```powershell
cd E:\research\FYP\2026FYP\src\tactile_pipeline

& E:\research\FYP\vggt_env\Scripts\python.exe step5_build_cad_pose_in_base.py
```

指定轴方向：

```powershell
& E:\research\FYP\vggt_env\Scripts\python.exe step5_build_cad_pose_in_base.py `
  --user-x-from-board=-x `
  --user-y-from-board=-y
```

## 输出

默认输出：

```text
E:\research\FYP\2026FYP\src\camera\runs\cad_pose_in_base\T_cad_to_base.json
```

这个 JSON 可以直接给 Step6 使用。

## Step6 使用

```powershell
& E:\research\FYP\vggt_env\Scripts\python.exe step6_compare_cad_pointcloud.py `
  --cad "E:\research\FYP\2026FYP\src\tactile_pipeline\phantom vs VGGT.stl" `
  --cad-scale 0.001 `
  --cad-transform "E:\research\FYP\2026FYP\src\camera\runs\cad_pose_in_base\T_cad_to_base.json" `
  --threshold 0.003
```

## 注意

这一步是正式实验用的几何对齐方式。`step5_interactive_align_cad_pi3.py` 只建议作为调试工具，用来观察偏差来源，不建议作为最终误差评估的真值对齐方式。
