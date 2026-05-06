namespace UR3Demo.Core;

/// <summary>
/// 职责：描述当前采集 session 的状态。
/// </summary>
public class HandEyeSessionInfo
{
    public string RootDirectory { get; set; } = string.Empty;
    public string SamplesJsonlPath { get; set; } = string.Empty;
    public string FramesDirectory { get; set; } = string.Empty;
    public bool IsAppendMode { get; set; }
    public int NextIndex { get; set; }
    public int ExistingSampleCount { get; set; }
}