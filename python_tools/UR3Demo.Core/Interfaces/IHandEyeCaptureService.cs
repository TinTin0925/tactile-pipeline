namespace UR3Demo.Core;

/// <summary>
/// 职责：管理 hand-eye 样本采集会话。
/// </summary>
public interface IHandEyeCaptureService
{
    void ConfigureOutput(string pathInput);
    HandEyeSessionInfo GetSessionInfo();

    Task<HandEyeCaptureResult> CaptureSampleAsync(double[] currentTcpPose, byte[] imageBytes);
}