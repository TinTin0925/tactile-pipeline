using System.Globalization;
using System.IO;
using System.Windows;
using System.Windows.Media.Imaging;
using Microsoft.Win32;
using UR3Demo.Core;
using UR3Demo.Robot.Logging;
using UR3Demo.Robot.Services;

namespace UR3Demo.UI;

/// <summary>
/// 职责：hand-eye 采集窗口。
/// 当前负责：
/// 1. 配置输出路径
/// 2. 显示相机预览
/// 3. 显示当前 TCP pose
/// 4. 执行“采集当前样本”
///
/// 新增（V1 自动采集预留）：
/// 5. 提供 RunAutoCalibrationCaptureAsync 入口，便于后端先跑通自动采集
/// 
/// 注意：
/// 当前先不强行改 XAML，避免 UI 改动过大。
/// 后面测试通过后，再把这个入口绑到按钮和文件选择框上。
/// </summary>
public partial class HandEyeCaptureWindow : Window
{
    private readonly IRobotService _robotService;
    private readonly IHandEyeCaptureService _captureService;
    private readonly ICameraService _cameraService;
    private readonly PythonCameraServiceClient _pythonCameraClient;
    private readonly ILogger _logger;

    // --- V1 自动采集新增依赖 ---
    private readonly ICapturePlanLoader _capturePlanLoader;
    private readonly IOutputSessionGuard _outputSessionGuard;
    private readonly IVisionHook _visionHook;
    private readonly IHandEyeSummaryProvider _handEyeSummaryProvider;
    private readonly IRobotReachWatcher _robotReachWatcher;
    private readonly IAutoCaptureOrchestrator _autoCaptureOrchestrator;
    private CancellationTokenSource? _autoCaptureCts;

    public HandEyeCaptureWindow(IRobotService robotService)
    {
        InitializeComponent();

        _robotService = robotService;
        _logger = new FileLogger();
        _captureService = new HandEyeCaptureService(_logger);
        _pythonCameraClient = new PythonCameraServiceClient("http://127.0.0.1:8001");
        _cameraService = _pythonCameraClient;

        // --- V1 自动采集新增服务实例 ---
        _capturePlanLoader = new CapturePlanLoader(_logger);
        _outputSessionGuard = new OutputSessionGuard();
        _visionHook = new NullVisionHook();
        _handEyeSummaryProvider = new HandEyeSummaryProvider(_logger);
        _robotReachWatcher = new RobotReachWatcher(_robotService, _logger);
        _autoCaptureOrchestrator = new AutoCaptureOrchestrator(
            _robotService,
            new MotionService(_robotService, _logger),
            _cameraService,
            _captureService,
            _outputSessionGuard,
            _visionHook,
            _handEyeSummaryProvider,
            _robotReachWatcher,
            _logger);

        _cameraService.PreviewFrameArrived += OnPreviewFrameArrived;
        Loaded += HandEyeCaptureWindow_Loaded;
        Closed += HandEyeCaptureWindow_Closed;
    }

    private async void HandEyeCaptureWindow_Loaded(object sender, RoutedEventArgs e)
    {
        try
        {
            await _cameraService.OpenAsync();
            AppendInfo("Camera preview started.");
        }
        catch (Exception ex)
        {
            AppendInfo($"Failed to open camera: {ex.Message}");
        }
    }

    private async void HandEyeCaptureWindow_Closed(object? sender, EventArgs e)
    {
        await _cameraService.CloseAsync();
    }

    private void BrowseButton_Click(object sender, RoutedEventArgs e)
    {
        var dlg = new OpenFolderDialog
        {
            Title = "Choose hand-eye output folder"
        };

        if (dlg.ShowDialog() == true)
        {
            OutputPathTextBox.Text = dlg.FolderName;
        }
    }

    private async void CaptureButton_Click(object sender, RoutedEventArgs e)
    {
        try
        {
            _captureService.ConfigureOutput(OutputPathTextBox.Text);
            RefreshSessionInfo();

            var session = _captureService.GetSessionInfo();
            var pose = _robotService.GetSnapshot().ActualTcpPose.ToArray();

            if (pose.Length != 6)
                throw new InvalidOperationException("Current TCP pose is invalid.");

            var imageBytes = _cameraService.CaptureImageBytes();
            if (imageBytes == null || imageBytes.Length == 0)
                throw new InvalidOperationException("No image captured from camera.");

            var result = await _captureService.CaptureSampleAsync(pose, imageBytes);
            LastCaptureText.Text = $"Last Capture: #{result.Index} -> {result.ImagePath}";
            CurrentPoseText.Text = "Current TCP Pose: " + FormatPose(pose);

            RefreshSessionInfo();
            AppendInfo(result.Message);
        }
        catch (Exception ex)
        {
            AppendInfo($"Capture failed: {ex.Message}");
        }
    }

    private void CameraSettingsButton_Click(object sender, RoutedEventArgs e)
    {
        try
        {
            var win = new CameraSettingsWindow(_pythonCameraClient)
            {
                Owner = this
            };
            win.ShowDialog();
        }
        catch (Exception ex)
        {
            AppendInfo($"Open camera settings failed: {ex.Message}");
        }
    }

    private void CloseButtonEx_Click(object sender, RoutedEventArgs e)
    {
        Close();
    }

    /// <summary>
    /// V1 自动采集入口。
    /// 
    /// 使用方式：
    /// 1. 在调试阶段，可从任意按钮事件或临时测试代码里直接调用
    /// 2. 支持输入 waypoint json / samples.jsonl
    /// 3. 输出目录仍然沿用现有 hand-eye 结构
    /// 
    /// 例子：
    /// await RunAutoCalibrationCaptureAsync(
    ///     waypointFilePath: @"D:\UCL\FYP\project\data\plans\calib_plan.json",
    ///     outputPath: @"D:\UCL\FYP\project\data\handeye\runs\session_04",
    ///     handEyeSummaryPath: @"D:\UCL\FYP\project\data\handeye\runs\session_02\handeye_result\summary.json");
    /// </summary>
    public async Task RunAutoCalibrationCaptureAsync(
        string waypointFilePath,
        string outputPath,
        string? handEyeSummaryPath = null,
        CancellationToken ct = default)
    {
        try
        {
            AppendInfo($"Auto capture loading plan from: {waypointFilePath}");

            AutoCapturePlan plan = LoadPlanByExtension(waypointFilePath);
            plan.TaskType = "calibration";
            plan.OutputPath = outputPath;
            plan.HandEyeSummaryPath = handEyeSummaryPath;

            var check = _outputSessionGuard.Check(outputPath);
            if (!check.Ok)
            {
                AppendInfo(check.Message);
                MessageBox.Show(check.Message, "Output Path Warning");
                return;
            }

            var runResult = await _autoCaptureOrchestrator.RunAsync(plan, ct);

            RefreshSessionInfo();
            AppendInfo($"Auto capture finished. Success={runResult.SuccessCount}, Fail={runResult.FailCount}");
        }
        catch (Exception ex)
        {
            AppendInfo($"Auto capture failed: {ex.Message}");
            MessageBox.Show(ex.Message, "Auto Capture Error");
        }
    }

    private AutoCapturePlan LoadPlanByExtension(string path)
    {
        string ext = Path.GetExtension(path).Trim().ToLowerInvariant();

        return ext switch
        {
            ".json" => _capturePlanLoader.LoadFromJson(path),
            ".jsonl" => _capturePlanLoader.LoadFromSamplesJsonl(path),
            _ => throw new InvalidOperationException($"Unsupported waypoint file type: {ext}")
        };
    }

    private void RefreshSessionInfo()
    {
        var s = _captureService.GetSessionInfo();

        SessionModeText.Text = $"Mode: {(s.IsAppendMode ? "Append" : "New")}";
        SessionRootText.Text = $"Root: {s.RootDirectory}";
        SessionJsonlText.Text = $"JSONL: {s.SamplesJsonlPath}";
        SampleCountText.Text = $"Next Index: {s.NextIndex}";
    }

    private void OnPreviewFrameArrived(byte[]? bytes)
    {
        if (bytes == null || bytes.Length == 0) return;

        Dispatcher.Invoke(() =>
        {
            try
            {
                using var ms = new MemoryStream(bytes);
                var bmp = new BitmapImage();
                bmp.BeginInit();
                bmp.CacheOption = BitmapCacheOption.OnLoad;
                bmp.StreamSource = ms;
                bmp.EndInit();
                bmp.Freeze();

                PreviewImage.Source = bmp;
            }
            catch
            {
                // 预览失败先静默，不影响主流程
            }
        });
    }

    private void AppendInfo(string text)
    {
        Dispatcher.Invoke(() =>
        {
            InfoTextBox.AppendText($"[{DateTime.Now:HH:mm:ss}] {text}{Environment.NewLine}");
            InfoTextBox.ScrollToEnd();
        });
    }

    private static string FormatPose(double[] pose)
    {
        return string.Format(
            CultureInfo.InvariantCulture,
            "[{0:F4}, {1:F4}, {2:F4}, {3:F4}, {4:F4}, {5:F4}]",
            pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
    }

    private void BrowseWaypointButton_Click(object sender, RoutedEventArgs e)
    {
        var dlg = new OpenFileDialog
        {
            Title = "Choose waypoint file",
            Filter = "Waypoint Files (*.json;*.jsonl)|*.json;*.jsonl|JSON Files (*.json)|*.json|JSONL Files (*.jsonl)|*.jsonl|All Files (*.*)|*.*"
        };

        if (dlg.ShowDialog() == true)
        {
            WaypointFileTextBox.Text = dlg.FileName;
            AutoCapturePlanText.Text = $"Plan: {dlg.FileName}";
        }
    }

    private void BrowseSummaryButton_Click(object sender, RoutedEventArgs e)
    {
        var dlg = new OpenFileDialog
        {
            Title = "Choose hand-eye summary.json",
            Filter = "JSON Files (*.json)|*.json|All Files (*.*)|*.*"
        };

        if (dlg.ShowDialog() == true)
        {
            HandEyeSummaryTextBox.Text = dlg.FileName;
        }
    }

    private async void RunAutoCaptureButton_Click(object sender, RoutedEventArgs e)
    {
        try
        {
            string waypointPath = WaypointFileTextBox.Text.Trim();
            string outputPath = OutputPathTextBox.Text.Trim();
            string summaryPath = HandEyeSummaryTextBox.Text.Trim();

            if (string.IsNullOrWhiteSpace(waypointPath))
                throw new InvalidOperationException("Waypoint file path cannot be empty.");

            if (string.IsNullOrWhiteSpace(outputPath))
                throw new InvalidOperationException("Output path cannot be empty.");

            _autoCaptureCts?.Cancel();
            _autoCaptureCts = new CancellationTokenSource();

            AutoCaptureStateText.Text = "Auto State: Running";
            AutoCapturePlanText.Text = $"Plan: {waypointPath}";
            AppendInfo("Auto capture started.");

            AutoCapturePlan plan = LoadPlanByExtension(waypointPath);
            plan.TaskType = "calibration";
            plan.OutputPath = outputPath;
            plan.HandEyeSummaryPath = string.IsNullOrWhiteSpace(summaryPath) ? null : summaryPath;
            plan.MoveHomeBeforeStart = MoveHomeBeforeStartCheckBox.IsChecked == true;
            plan.MoveHomeAfterFinish = MoveHomeAfterFinishCheckBox.IsChecked == true;
            plan.StopOnFailure = StopOnFailureCheckBox.IsChecked == true;
            plan.EnableVisionHook = EnableVisionHookCheckBox.IsChecked == true;

            var check = _outputSessionGuard.Check(outputPath);
            if (!check.Ok)
            {
                AutoCaptureStateText.Text = "Auto State: Blocked";
                AppendInfo(check.Message);
                MessageBox.Show(check.Message, "Output Path Warning");
                return;
            }

            var runResult = await _autoCaptureOrchestrator.RunAsync(plan, _autoCaptureCts.Token);

            RefreshSessionInfo();
            AutoCaptureStateText.Text = $"Auto State: Finished (OK={runResult.SuccessCount}, Fail={runResult.FailCount})";
            AppendInfo($"Auto capture finished. Success={runResult.SuccessCount}, Fail={runResult.FailCount}");
        }
        catch (OperationCanceledException)
        {
            AutoCaptureStateText.Text = "Auto State: Cancelled";
            AppendInfo("Auto capture cancelled.");
        }
        catch (Exception ex)
        {
            AutoCaptureStateText.Text = "Auto State: Error";
            AppendInfo($"Auto capture failed: {ex.Message}");
            MessageBox.Show(ex.Message, "Auto Capture Error");
        }
    }
}