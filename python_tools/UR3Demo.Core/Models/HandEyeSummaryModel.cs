using System.Text.Json.Serialization;

namespace UR3Demo.Core;

/// <summary>
/// 脚本级说明：
/// 这个模型用于读取 hand-eye 的 summary.json。
/// 
/// 当前我们只关心：
/// - T_tool_camera
/// - T_camera_tool
/// 
/// 因为未来“沿相机视线轴向移动”要依赖这些外参。
/// 你提供的 summary.json 已经包含这两个矩阵。
/// </summary>
public class HandEyeSummaryModel
{
    [JsonPropertyName("method_selected")]
    public string? MethodSelected { get; set; }

    [JsonPropertyName("robot_pose_convention")]
    public string? RobotPoseConvention { get; set; }

    [JsonPropertyName("T_tool_camera")]
    public double[][] TToolCamera { get; set; } = Array.Empty<double[]>();

    [JsonPropertyName("T_camera_tool")]
    public double[][] TCameraTool { get; set; } = Array.Empty<double[]>();
}