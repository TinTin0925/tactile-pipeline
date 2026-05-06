using UR3Demo.Core;

namespace UR3Demo.Robot.Services;

/// <summary>
/// 职责：高层运动逻辑封装。
/// 
/// 负责：
/// 1. Home 点运动
/// 2. Joint jog
/// 3. TCP 增量 jog
/// 4. 按目标关节 / 目标位姿移动
/// 5. 简单按点模板
/// 6. 标定点序列执行
/// 
/// 不负责：
/// 1. 底层 RTDE / Socket 通信
/// 2. UI 控件处理
/// 3. Python 算法调用
/// </summary>
public class MotionService : IMotionService
{
    private readonly IRobotService _robot;
    private readonly ILogger _logger;

    // 你当前组员给的 HOME 姿态角度，可以先放在这里。
    // 后续也可以改成从配置文件加载。
    private static readonly double[] HomeDeg = { 45.76, -90.18, 90.08, -180.43, -90.09, 45.06 };

    public MotionService(IRobotService robot, ILogger logger)
    {
        _robot = robot;
        _logger = logger;
    }

    public async Task MoveHomeAsync()
    {
        var homeRad = HomeDeg.Select(DegToRad).ToArray();
        _logger.Info("Move to Home pose.");
        await _robot.MoveJAsync(homeRad, speed: 0.15, acc: 0.15);
    }

    public async Task JogJointAsync(int jointIndex, double deltaRad)
    {
        _logger.Info($"High-level joint jog: J{jointIndex}, delta={deltaRad:F6} rad");
        await _robot.JogJointAsync(jointIndex, deltaRad);
    }

    public async Task JogTcpAsync(double dx, double dy, double dz, double dRx, double dRy, double dRz)
    {
        var snapshot = _robot.GetSnapshot();
        var current = snapshot.ActualTcpPose.ToArray();

        if (current.Length != 6)
            throw new InvalidOperationException("Current TCP pose is invalid.");

        var target = new double[]
        {
            current[0] + dx,
            current[1] + dy,
            current[2] + dz,
            current[3] + dRx,
            current[4] + dRy,
            current[5] + dRz
        };

        _logger.Info($"TCP jog: dPos=({dx:F4},{dy:F4},{dz:F4}), dRot=({dRx:F4},{dRy:F4},{dRz:F4})");
        await _robot.MoveLAsync(target, speed: 0.05, acc: 0.05);
    }

    public async Task MoveToJointAsync(double[] jointsRad, double speed = 0.2, double acc = 0.2)
    {
        _logger.Info("Move to target joints.");
        await _robot.MoveJAsync(jointsRad, speed, acc);
    }

    public async Task MoveToPoseAsync(double[] tcpPose, double speed = 0.1, double acc = 0.1)
    {
        _logger.Info("Move to target TCP pose.");
        await _robot.MoveLAsync(tcpPose, speed, acc);
    }

    public async Task MoveLinearOffsetAsync(double dx, double dy, double dz)
    {
        await JogTcpAsync(dx, dy, dz, 0, 0, 0);
    }

    public async Task RotateToolAsync(double dRx, double dRy, double dRz)
    {
        await JogTcpAsync(0, 0, 0, dRx, dRy, dRz);
    }

    public async Task PressPointAsync(double[] approachPose, double[] targetPose, int dwellMs = 300)
    {
        _logger.Info("PressPoint start: approach -> target -> dwell -> retreat");

        await _robot.MoveLAsync(approachPose, speed: 0.08, acc: 0.08);
        await _robot.MoveLAsync(targetPose, speed: 0.03, acc: 0.03);
        await Task.Delay(dwellMs);
        await _robot.MoveLAsync(approachPose, speed: 0.08, acc: 0.08);

        _logger.Info("PressPoint finished.");
    }

    public async Task RunCalibrationJointSequenceAsync(IEnumerable<double[]> jointPoints, int dwellMs = 1500)
    {
        int index = 0;
        foreach (var q in jointPoints)
        {
            index++;
            _logger.Info($"Calibration sequence move to point {index}");

            await _robot.MoveJAsync(q, speed: 0.15, acc: 0.15);
            await Task.Delay(dwellMs);
        }

        _logger.Info("Calibration joint sequence finished.");
    }

    private static double DegToRad(double deg) => deg * Math.PI / 180.0;
}