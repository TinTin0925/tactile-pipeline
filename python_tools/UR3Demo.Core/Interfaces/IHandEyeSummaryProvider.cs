namespace UR3Demo.Core;

/// <summary>
/// 脚本级说明：
/// 读取 hand-eye summary.json，并提供相机前向轴方向。
/// </summary>
public interface IHandEyeSummaryProvider
{
    HandEyeSummaryModel Load(string path);

    /// <summary>
    /// 返回“相机前向轴”在 tool 坐标系中的方向向量。
    /// 通常基于 T_tool_camera 的旋转部分计算。
    /// </summary>
    double[] GetCameraForwardInTool(string path, CameraAxisConvention axisConvention);
}