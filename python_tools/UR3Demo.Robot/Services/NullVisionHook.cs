using UR3Demo.Core;

namespace UR3Demo.Robot.Services;

/// <summary>
/// 脚本级说明：
/// V1 默认使用的视觉 Hook 空实现。
/// 
/// 作用：
/// - 保持自动采集主流程完整
/// - 暂时不依赖 Python 视觉算法
/// - 未来只需要替换掉这个类，即可接入真正的 Python 视觉模块
/// </summary>
public class NullVisionHook : IVisionHook
{
    public Task<VisionHookResult> ProcessAsync(VisionHookContext context, CancellationToken ct = default)
    {
        return Task.FromResult(new VisionHookResult
        {
            Success = true,
            AllowCapture = true,
            Message = "Vision hook bypassed."
        });
    }
}