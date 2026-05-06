namespace UR3Demo.Core;

/// <summary>
/// 脚本级说明：
/// 统一的视觉 Hook 返回结果。
/// 
/// V1 中默认只需要：
/// - Success
/// - AllowCapture
/// 
/// 未来扩展到重建扫描时，可直接用下面这些字段：
/// - NeedPoseCorrection
/// - NeedAxisMove
/// - PixelErrorUv
/// - TargetFullyVisible
/// - SuggestedAxisOffsetM
/// - SuggestedRotationDeltaRad
/// </summary>
public class VisionHookResult
{
    public bool Success { get; set; }

    /// <summary>
    /// 当前图像是否允许作为最终结果保存。
    /// 对 calibration 场景，未来可用来表达：
    /// - 标定板可见
    /// - 角点完整
    /// - 图像有效
    /// </summary>
    public bool AllowCapture { get; set; } = true;

    public bool NeedPoseCorrection { get; set; }

    public bool NeedAxisMove { get; set; }

    public double[]? PixelErrorUv { get; set; }

    public bool? TargetFullyVisible { get; set; }

    /// <summary>
    /// 未来用于沿相机视线轴向移动的建议位移，单位 m。
    /// 正负方向由上层配合 CameraAxisConvention 解释。
    /// </summary>
    public double SuggestedAxisOffsetM { get; set; }

    /// <summary>
    /// 未来用于姿态修正的工具增量旋转：
    /// [dRx, dRy, dRz]
    /// </summary>
    public double[]? SuggestedRotationDeltaRad { get; set; }

    public string Message { get; set; } = string.Empty;
}