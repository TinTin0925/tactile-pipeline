namespace UR3Demo.Core;

/// <summary>
/// 脚本级说明：
/// 用于在真正开始采集前，对输出目录进行风险检查。
/// 
/// 重点需求：
/// 如果 outputPath/frames 已存在且非空，应提醒或阻止继续，
/// 避免把新任务数据混进旧任务中。
/// </summary>
public class OutputPathCheckResult
{
    public bool Ok { get; set; }

    public bool FramesFolderExists { get; set; }

    public bool FramesFolderHasFiles { get; set; }

    public bool SamplesJsonlExists { get; set; }

    public string RootDirectory { get; set; } = string.Empty;

    public string FramesDirectory { get; set; } = string.Empty;

    public string SamplesJsonlPath { get; set; } = string.Empty;

    public string Message { get; set; } = string.Empty;
}