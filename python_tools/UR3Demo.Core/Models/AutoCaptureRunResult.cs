namespace UR3Demo.Core;

/// <summary>
/// 脚本级说明：
/// 表示一次自动采集任务的最终运行结果。
/// </summary>
public class AutoCaptureRunResult
{
    public bool Success => FailCount == 0;

    public int TotalCount { get; set; }

    public int SuccessCount { get; set; }

    public int FailCount { get; set; }

    public List<AutoCaptureItemResult> Items { get; set; } = [];
}

public class AutoCaptureItemResult
{
    public int WaypointIndex { get; set; }

    public bool Success { get; set; }

    public string Message { get; set; } = string.Empty;

    public string? ImagePath { get; set; }
}