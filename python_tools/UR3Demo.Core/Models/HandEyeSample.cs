namespace UR3Demo.Core;

/// <summary>
/// 职责：表示一条 hand-eye 采样记录。
/// 一条样本 = 一张图片 + 一个时间戳 + 一个机器人末端位姿。
/// </summary>
public class HandEyeSample
{
    public int Index { get; set; }
    public double Timestamp { get; set; }
    public string ImagePath { get; set; } = string.Empty;
    public double[][] TBaseTool { get; set; } = Array.Empty<double[]>();
}