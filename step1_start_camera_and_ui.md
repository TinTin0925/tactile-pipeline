# Step1 启动相机服务和上位机

对应代码：

```text
step1_start_camera_and_ui.py
```

## 作用

这一步用于一键启动采集系统。脚本会先启动 XIMEA 相机的 FastAPI 服务，然后启动 UR3 上位机界面。

启动顺序是：

1. 检查 `http://127.0.0.1:8001/health`
2. 如果相机服务没有运行，启动：

```powershell
uvicorn camera_service:app --host 127.0.0.1 --port 8001
```

3. 等待相机服务可用
4. 启动上位机：

```powershell
dotnet run --project .\UR3Demo.UI\UR3Demo.UI.csproj
```

## 运行

```powershell
cd E:\research\FYP\2026FYP\src\tactile_pipeline

& C:\Users\lenovo\AppData\Local\Programs\Python\Python310\python.exe step1_start_camera_and_ui.py
```

VSCode 中也可以直接打开 `step1_start_camera_and_ui.py`，点击运行。

## 输出

运行后会出现两个 PowerShell 窗口：

```text
camera_service / uvicorn
UR3Demo.UI 上位机
```

## 注意

如果 8001 端口已经有相机服务在运行，脚本不会重复启动相机服务，只会继续启动上位机。
