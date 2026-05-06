namespace UR3Demo.Core;

/// <summary>
/// 职责：表示一次采样动作的结果。
/// </summary>
public class HandEyeCaptureResult
{
    public bool Success { get; set; }
    public string Message { get; set; } = string.Empty;
    public int Index { get; set; }
    public string ImagePath { get; set; } = string.Empty;
    public string SamplesJsonlPath { get; set; } = string.Empty;
}