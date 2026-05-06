namespace UR3Demo.Core;

/// <summary>
/// 脚本级说明：
/// 因为当前 MoveL / MoveJ 是“发送命令”，并不会阻塞到机器人实际停稳，
/// 所以自动采集必须额外有一个“到位与稳定判定器”。
/// </summary>
public interface IRobotReachWatcher
{
    Task WaitUntilPoseReachedAsync(
        double[] targetPose,
        double positionToleranceM,
        double rotationToleranceRad,
        int stableSampleCount,
        int timeoutMs,
        CancellationToken ct = default);
}