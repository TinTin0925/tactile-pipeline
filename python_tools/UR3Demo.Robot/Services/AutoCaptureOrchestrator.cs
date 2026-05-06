using UR3Demo.Core;

namespace UR3Demo.Robot.Services;

/// <summary>
/// 脚本级说明：
/// 自动采集主编排器。
/// 
/// 当前 V1 负责：
/// 1. 检查输出目录
/// 2. 自动移动到目标点
/// 3. 等待机器人到位并稳定
/// 4. 抓拍图像
/// 5. （可选）调用视觉 Hook
/// 6. 保存图像与实际 TCP 位姿
/// 
/// 未来扩展点：
/// - 根据视觉误差做姿态修正
/// - 沿相机视线轴向移动
/// - 多轮修正后再拍照
/// </summary>
public class AutoCaptureOrchestrator : IAutoCaptureOrchestrator
{
    private readonly IRobotService _robot;
    private readonly IMotionService _motion;
    private readonly ICameraService _camera;
    private readonly IHandEyeCaptureService _captureService;
    private readonly IOutputSessionGuard _outputGuard;
    private readonly IVisionHook _visionHook;
    private readonly IHandEyeSummaryProvider _handEyeSummaryProvider;
    private readonly IRobotReachWatcher _reachWatcher;
    private readonly ILogger _logger;

    public AutoCaptureOrchestrator(
        IRobotService robot,
        IMotionService motion,
        ICameraService camera,
        IHandEyeCaptureService captureService,
        IOutputSessionGuard outputGuard,
        IVisionHook visionHook,
        IHandEyeSummaryProvider handEyeSummaryProvider,
        IRobotReachWatcher reachWatcher,
        ILogger logger)
    {
        _robot = robot;
        _motion = motion;
        _camera = camera;
        _captureService = captureService;
        _outputGuard = outputGuard;
        _visionHook = visionHook;
        _handEyeSummaryProvider = handEyeSummaryProvider;
        _reachWatcher = reachWatcher;
        _logger = logger;
    }

    public async Task<AutoCaptureRunResult> RunAsync(AutoCapturePlan plan, CancellationToken ct = default)
    {
        if (plan == null)
            throw new ArgumentNullException(nameof(plan));

        if (plan.Waypoints == null || plan.Waypoints.Count == 0)
            throw new InvalidOperationException("Auto capture plan contains no waypoints.");

        var enabledWaypoints = plan.Waypoints.Where(w => w.Enabled).ToList();
        if (enabledWaypoints.Count == 0)
            throw new InvalidOperationException("No enabled waypoints found in plan.");

        var check = _outputGuard.Check(plan.OutputPath);
        if (!check.Ok)
            throw new InvalidOperationException(check.Message);

        _captureService.ConfigureOutput(plan.OutputPath);

        var result = new AutoCaptureRunResult
        {
            TotalCount = enabledWaypoints.Count
        };

        if (!_camera.IsOpened)
            await _camera.OpenAsync();

        if (plan.MoveHomeBeforeStart)
        {
            _logger.Info("Auto capture: move home before start.");
            await _motion.MoveHomeAsync();
            await Task.Delay(1200, ct);
        }

        foreach (var wp in enabledWaypoints)
        {
            ct.ThrowIfCancellationRequested();

            try
            {
                await ExecuteSingleWaypointAsync(plan, wp, ct);
                var session = _captureService.GetSessionInfo();

                result.SuccessCount++;
                result.Items.Add(new AutoCaptureItemResult
                {
                    WaypointIndex = wp.Index,
                    Success = true,
                    Message = "Captured successfully.",
                    ImagePath = Path.Combine(session.FramesDirectory, $"{session.NextIndex - 1:D4}.png")
                });
            }
            catch (Exception ex)
            {
                result.FailCount++;
                result.Items.Add(new AutoCaptureItemResult
                {
                    WaypointIndex = wp.Index,
                    Success = false,
                    Message = ex.Message
                });

                _logger.Error($"Auto capture failed at waypoint #{wp.Index}", ex);

                if (plan.StopOnFailure)
                    throw;
            }
        }

        if (plan.MoveHomeAfterFinish)
        {
            _logger.Info("Auto capture: move home after finish.");
            await _motion.MoveHomeAsync();
        }

        return result;
    }

    private async Task ExecuteSingleWaypointAsync(AutoCapturePlan plan, CaptureWaypoint wp, CancellationToken ct)
    {
        _logger.Info($"Auto capture start waypoint #{wp.Index}");

        if (wp.TcpPose is { Length: 6 })
        {
            await _motion.MoveToPoseAsync(wp.TcpPose, plan.Speed, plan.Acc);

            await _reachWatcher.WaitUntilPoseReachedAsync(
                wp.TcpPose,
                plan.PositionToleranceM,
                plan.RotationToleranceRad,
                plan.StableSampleCount,
                plan.ReachTimeoutMs,
                ct);
        }
        else if (wp.JointPose is { Length: 6 })
        {
            await _motion.MoveToJointAsync(wp.JointPose, plan.Speed, plan.Acc);
            await Task.Delay(plan.ReachTimeoutMs / 3, ct); // V1：关节目标暂用固定等待
        }
        else
        {
            throw new InvalidOperationException($"Waypoint #{wp.Index} does not contain a valid pose.");
        }

        int dwellMs = wp.DwellMs ?? plan.SettleDelayMs;
        if (dwellMs > 0)
            await Task.Delay(dwellMs, ct);

        for (int iteration = 0; iteration <= plan.MaxCorrectionIterations; iteration++)
        {
            ct.ThrowIfCancellationRequested();

            byte[] imageBytes = _camera.CaptureImageBytes()
                ?? throw new InvalidOperationException("Failed to capture image from camera.");

            var actualPose = _robot.GetSnapshot().ActualTcpPose.ToArray();
            if (actualPose.Length != 6)
                throw new InvalidOperationException("Current TCP pose is invalid.");

            VisionHookResult hookResult = new()
            {
                Success = true,
                AllowCapture = true,
                Message = "Vision hook skipped."
            };

            if (plan.EnableVisionHook)
            {
                var ctx = new VisionHookContext
                {
                    TaskType = plan.TaskType,
                    WaypointIndex = wp.Index,
                    TargetTcpPose = wp.TcpPose,
                    ActualTcpPose = actualPose,
                    ImageBytes = imageBytes,
                    HandEyeSummaryPath = plan.HandEyeSummaryPath
                };

                hookResult = await _visionHook.ProcessAsync(ctx, ct);
                if (!hookResult.Success)
                    throw new InvalidOperationException($"Vision hook failed: {hookResult.Message}");
            }

            // V1 正常路径：直接允许保存
            if (hookResult.AllowCapture && !hookResult.NeedPoseCorrection && !hookResult.NeedAxisMove)
            {
                await _captureService.CaptureSampleAsync(actualPose, imageBytes);
                _logger.Info($"Auto capture saved waypoint #{wp.Index}");
                return;
            }

            // 未来扩展：根据视觉反馈做修正
            if (iteration >= plan.MaxCorrectionIterations)
                throw new InvalidOperationException(
                    $"Waypoint #{wp.Index} still requires correction, but max correction iterations reached.");

            if (hookResult.NeedPoseCorrection && hookResult.SuggestedRotationDeltaRad is { Length: 3 })
            {
                await ApplyRotationCorrectionAsync(hookResult.SuggestedRotationDeltaRad);
            }

            if (hookResult.NeedAxisMove)
            {
                if (string.IsNullOrWhiteSpace(plan.HandEyeSummaryPath))
                    throw new InvalidOperationException("Axis move requested but HandEyeSummaryPath is not set.");

                await ApplyAxisMoveAsync(
                    hookResult.SuggestedAxisOffsetM,
                    plan.HandEyeSummaryPath!,
                    plan.CameraAxisConvention);
            }

            await Task.Delay(plan.SettleDelayMs, ct);
        }

        throw new InvalidOperationException($"Waypoint #{wp.Index} failed to produce a capturable frame.");
    }

    /// <summary>
    /// 未来重建场景的预留：
    /// 根据视觉模块返回的旋转增量，微调工具姿态。
    /// V1 先直接调用现有 IMotionService.RotateToolAsync。
    /// </summary>
    private async Task ApplyRotationCorrectionAsync(double[] dRot)
    {
        if (dRot.Length != 3)
            throw new ArgumentException("Rotation correction must contain 3 values.", nameof(dRot));

        _logger.Info($"Apply rotation correction: [{dRot[0]:F6}, {dRot[1]:F6}, {dRot[2]:F6}]");
        await _motion.RotateToolAsync(dRot[0], dRot[1], dRot[2]);
    }

    /// <summary>
    /// 未来重建场景的预留：
    /// 沿相机视线轴向移动。
    /// 
    /// 计算逻辑：
    /// 1. 从 summary.json 读取 camera forward in tool
    /// 2. 结合当前 T_base_tool 的旋转，变换到 base 坐标
    /// 3. 在 base 中做平移增量
    /// </summary>
    private async Task ApplyAxisMoveAsync(
        double offsetM,
        string handEyeSummaryPath,
        CameraAxisConvention axisConvention)
    {
        var forwardInTool = _handEyeSummaryProvider.GetCameraForwardInTool(handEyeSummaryPath, axisConvention);
        var actualPose = _robot.GetSnapshot().ActualTcpPose.ToArray();

        if (actualPose.Length != 6)
            throw new InvalidOperationException("Current TCP pose is invalid.");

        var rBaseTool = PoseAxisAngleToRotationMatrix(actualPose[3], actualPose[4], actualPose[5]);
        var forwardInBase = Multiply3x3Vec(rBaseTool, forwardInTool);

        double dx = forwardInBase[0] * offsetM;
        double dy = forwardInBase[1] * offsetM;
        double dz = forwardInBase[2] * offsetM;

        _logger.Info($"Apply axis move: offset={offsetM:F6} m, dBase=({dx:F6}, {dy:F6}, {dz:F6})");
        await _motion.MoveLinearOffsetAsync(dx, dy, dz);
    }

    private static double[,] PoseAxisAngleToRotationMatrix(double rx, double ry, double rz)
    {
        double theta = Math.Sqrt(rx * rx + ry * ry + rz * rz);

        double[,] R = new double[3, 3];
        if (theta < 1e-12)
        {
            R[0, 0] = 1; R[0, 1] = 0; R[0, 2] = 0;
            R[1, 0] = 0; R[1, 1] = 1; R[1, 2] = 0;
            R[2, 0] = 0; R[2, 1] = 0; R[2, 2] = 1;
            return R;
        }

        double kx = rx / theta;
        double ky = ry / theta;
        double kz = rz / theta;
        double c = Math.Cos(theta);
        double s = Math.Sin(theta);
        double v = 1 - c;

        R[0, 0] = kx * kx * v + c;
        R[0, 1] = kx * ky * v - kz * s;
        R[0, 2] = kx * kz * v + ky * s;

        R[1, 0] = ky * kx * v + kz * s;
        R[1, 1] = ky * ky * v + c;
        R[1, 2] = ky * kz * v - kx * s;

        R[2, 0] = kz * kx * v - ky * s;
        R[2, 1] = kz * ky * v + kx * s;
        R[2, 2] = kz * kz * v + c;

        return R;
    }

    private static double[] Multiply3x3Vec(double[,] R, double[] v)
    {
        return
        [
            R[0,0] * v[0] + R[0,1] * v[1] + R[0,2] * v[2],
            R[1,0] * v[0] + R[1,1] * v[1] + R[1,2] * v[2],
            R[2,0] * v[0] + R[2,1] * v[1] + R[2,2] * v[2]
        ];
    }
}