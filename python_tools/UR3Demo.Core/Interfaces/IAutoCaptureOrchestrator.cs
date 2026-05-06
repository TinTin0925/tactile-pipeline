namespace UR3Demo.Core;

/// <summary>
/// 脚本级说明：
/// 自动采集的主编排器。
/// 
/// 它负责：
/// - 读取任务计划
/// - 控制机器人移动
/// - 等待稳定
/// - 抓拍
/// - 调用视觉 Hook（预留）
/// - 保存最终样本
/// </summary>
public interface IAutoCaptureOrchestrator
{
    Task<AutoCaptureRunResult> RunAsync(AutoCapturePlan plan, CancellationToken ct = default);
}