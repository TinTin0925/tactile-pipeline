namespace UR3Demo.Core;

/// <summary>
/// 脚本级说明：
/// 负责读取自动采集的目标点配置。
/// 
/// 支持两种来源：
/// 1. 新的任务 JSON
/// 2. 旧的 samples.jsonl
/// </summary>
public interface ICapturePlanLoader
{
    AutoCapturePlan LoadFromJson(string path);

    AutoCapturePlan LoadFromSamplesJsonl(string path);
}