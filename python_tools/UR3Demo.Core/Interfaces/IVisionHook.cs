namespace UR3Demo.Core;

/// <summary>
/// 脚本级说明：
/// 预留给 Python 视觉处理模块的统一接入点。
/// 
/// V1 中可以先使用 NullVisionHook，表示不做任何视觉判断，
/// 直接允许保存。
/// </summary>
public interface IVisionHook
{
    Task<VisionHookResult> ProcessAsync(VisionHookContext context, CancellationToken ct = default);
}