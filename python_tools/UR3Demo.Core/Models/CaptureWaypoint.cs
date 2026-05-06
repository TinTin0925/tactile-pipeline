using System.Text.Json.Serialization;

namespace UR3Demo.Core;

/// <summary>
/// 脚本级说明：
/// 表示一个自动采集目标点。
/// 
/// V1 推荐直接使用 TcpPose：
/// [x, y, z, rx, ry, rz]
/// 
/// 为兼容将来扩展，也保留 JointPose 字段。
/// </summary>
public class CaptureWaypoint
{
    [JsonPropertyName("index")]
    public int Index { get; set; }

    [JsonPropertyName("name")]
    public string? Name { get; set; }

    /// <summary>
    /// 目标 TCP 位姿：
    /// [x, y, z, rx, ry, rz]
    /// 位置单位 m，姿态为 UR 轴角弧度
    /// </summary>
    [JsonPropertyName("tcpPose")]
    public double[]? TcpPose { get; set; }

    /// <summary>
    /// 可选：目标关节角（弧度）
    /// </summary>
    [JsonPropertyName("jointPose")]
    public double[]? JointPose { get; set; }

    [JsonPropertyName("enabled")]
    public bool Enabled { get; set; } = true;

    [JsonPropertyName("dwellMs")]
    public int? DwellMs { get; set; }
}