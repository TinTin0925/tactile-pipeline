namespace UR3Demo.Core;

/// <summary>
/// 职责：承载机器人当前状态快照。
/// 这是 UI 显示层、运动逻辑层读取机器人状态的统一数据结构。
/// </summary>
public class RobotSnapshot
{
    public DateTime Timestamp { get; set; } = DateTime.Now;
    public bool IsConnected { get; set; }

    public double[] ActualQ { get; set; } = new double[6];
    public double[] ActualTcpPose { get; set; } = new double[6];

    public int RobotMode { get; set; }
    public int SafetyMode { get; set; }

    public ulong DigitalInputs { get; set; }
    public ulong DigitalOutputs { get; set; }

    public string StatusText { get; set; } = "Disconnected";
}