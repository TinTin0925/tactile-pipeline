namespace UR3Demo.Core;

/// <summary>
/// 脚本级说明：
/// 这是“视觉 Hook”收到的上下文数据。
/// 
/// V1 中 Python 视觉还不是主流程，
/// 但这个上下文对象已经把后续可能需要的信息都预留出来：
/// - 当前图像
/// - 当前实际位姿
/// - 当前目标位姿
/// - 任务类型
/// - hand-eye summary 路径
/// </summary>
public class VisionHookContext
{
    public string TaskType { get; set; } = "calibration";

    public int WaypointIndex { get; set; }

    public double[]? TargetTcpPose { get; set; }

    public double[]? ActualTcpPose { get; set; }

    public byte[] ImageBytes { get; set; } = Array.Empty<byte>();

    public string? HandEyeSummaryPath { get; set; }
}