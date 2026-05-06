using System.Text.Json.Serialization;

namespace UR3Demo.Core;

/// <summary>
/// 脚本级说明：
/// 表示一次“自动采集任务”的完整配置。
/// 
/// V1 主要用于自动标定数据采集：
/// 1. 从 waypoint 文件读取目标点
/// 2. 自动移动到目标点
/// 3. 抓拍并保存图像 + 实际 TCP 位姿
/// 
/// 未来扩展：
/// - 可接入视觉反馈
/// - 可接入姿态修正
/// - 可接入沿相机视线轴向移动
/// </summary>
public class AutoCapturePlan
{
    /// <summary>
    /// 任务类型。当前建议：
    /// "calibration" / "reconstruction"
    /// </summary>
    [JsonPropertyName("taskType")]
    public string TaskType { get; set; } = "calibration";

    /// <summary>
    /// 目标点来源文件路径（json 或 samples.jsonl）
    /// </summary>
    [JsonPropertyName("inputWaypointsPath")]
    public string? InputWaypointsPath { get; set; }

    /// <summary>
    /// 输出目录路径。最终会生成：
    /// outputPath/frames/*.png
    /// outputPath/samples.jsonl
    /// </summary>
    [JsonPropertyName("outputPath")]
    public string OutputPath { get; set; } = string.Empty;

    /// <summary>
    /// hand-eye summary.json 路径。
    /// V1 可以不填；未来做相机视线轴向移动时使用。
    /// </summary>
    [JsonPropertyName("handEyeSummaryPath")]
    public string? HandEyeSummaryPath { get; set; }

    [JsonPropertyName("moveHomeBeforeStart")]
    public bool MoveHomeBeforeStart { get; set; } = true;

    [JsonPropertyName("moveHomeAfterFinish")]
    public bool MoveHomeAfterFinish { get; set; } = false;

    [JsonPropertyName("speed")]
    public double Speed { get; set; } = 0.08;

    [JsonPropertyName("acc")]
    public double Acc { get; set; } = 0.08;

    /// <summary>
    /// 机器人到位后，再额外等待一段时间再拍照，减小残余振动影响。
    /// </summary>
    [JsonPropertyName("settleDelayMs")]
    public int SettleDelayMs { get; set; } = 1200;

    [JsonPropertyName("reachTimeoutMs")]
    public int ReachTimeoutMs { get; set; } = 15000;

    [JsonPropertyName("positionToleranceM")]
    public double PositionToleranceM { get; set; } = 0.002;

    [JsonPropertyName("rotationToleranceRad")]
    public double RotationToleranceRad { get; set; } = 0.03;

    /// <summary>
    /// 连续多少次满足误差阈值才认为“真的稳定到位”。
    /// </summary>
    [JsonPropertyName("stableSampleCount")]
    public int StableSampleCount { get; set; } = 5;

    [JsonPropertyName("stopOnFailure")]
    public bool StopOnFailure { get; set; } = true;

    /// <summary>
    /// 是否启用视觉 Hook。V1 默认 false。
    /// </summary>
    [JsonPropertyName("enableVisionHook")]
    public bool EnableVisionHook { get; set; } = false;

    /// <summary>
    /// 未来用于“单点多轮修正”的最大迭代次数。
    /// V1 默认 0，表示不做修正。
    /// </summary>
    [JsonPropertyName("maxCorrectionIterations")]
    public int MaxCorrectionIterations { get; set; } = 0;

    /// <summary>
    /// 相机前向轴约定。通常 PositiveZ 即相机 +Z。
    /// 未来沿相机视线轴向移动时使用。
    /// </summary>
    [JsonPropertyName("cameraAxisConvention")]
    public CameraAxisConvention CameraAxisConvention { get; set; } = CameraAxisConvention.PositiveZ;

    [JsonPropertyName("waypoints")]
    public List<CaptureWaypoint> Waypoints { get; set; } = [];
}

/// <summary>
/// 相机前向轴约定。
/// </summary>
public enum CameraAxisConvention
{
    PositiveZ,
    NegativeZ
}