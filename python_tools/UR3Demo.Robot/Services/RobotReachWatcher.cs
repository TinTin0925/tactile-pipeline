using UR3Demo.Core;

namespace UR3Demo.Robot.Services;

/// <summary>
/// 脚本级说明：
/// 负责判定机器人是否“真正到位并稳定”。
/// 
/// 为什么需要它：
/// 当前 MoveL/MoveJ 只是把 URScript 发给机器人，
/// 并不是阻塞到机器人物理运动完全结束。
/// 所以自动拍照前必须基于 RTDE 实际位姿再做一次闭环判定。
/// </summary>
public class RobotReachWatcher : IRobotReachWatcher
{
    private readonly IRobotService _robot;
    private readonly ILogger _logger;

    public RobotReachWatcher(IRobotService robot, ILogger logger)
    {
        _robot = robot;
        _logger = logger;
    }

    public async Task WaitUntilPoseReachedAsync(
        double[] targetPose,
        double positionToleranceM,
        double rotationToleranceRad,
        int stableSampleCount,
        int timeoutMs,
        CancellationToken ct = default)
    {
        if (targetPose == null || targetPose.Length != 6)
            throw new ArgumentException("targetPose must contain 6 values.", nameof(targetPose));

        int stableCount = 0;
        var started = DateTime.UtcNow;

        while ((DateTime.UtcNow - started).TotalMilliseconds < timeoutMs)
        {
            ct.ThrowIfCancellationRequested();

            var actual = _robot.GetSnapshot().ActualTcpPose;

            if (actual == null || actual.Length != 6)
            {
                await Task.Delay(100, ct);
                continue;
            }

            double posErr = Math.Sqrt(
                Math.Pow(actual[0] - targetPose[0], 2) +
                Math.Pow(actual[1] - targetPose[1], 2) +
                Math.Pow(actual[2] - targetPose[2], 2));

            double rotErr = Math.Sqrt(
                Math.Pow(actual[3] - targetPose[3], 2) +
                Math.Pow(actual[4] - targetPose[4], 2) +
                Math.Pow(actual[5] - targetPose[5], 2));

            bool reached = posErr <= positionToleranceM && rotErr <= rotationToleranceRad;

            if (reached)
            {
                stableCount++;
                if (stableCount >= stableSampleCount)
                {
                    _logger.Info($"Robot reached target pose. posErr={posErr:F6} m, rotErr={rotErr:F6} rad");
                    return;
                }
            }
            else
            {
                stableCount = 0;
            }

            await Task.Delay(100, ct);
        }

        throw new TimeoutException("Robot did not reach target pose within timeout.");
    }
}