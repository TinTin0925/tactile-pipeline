namespace UR3Demo.Core;

/// <summary>
/// 职责：提供“运动逻辑层”接口。
/// 这里不直接处理 Socket/RTDE 细节，而是在底层 IRobotService 之上，
/// 封装更适合 UI 和 demo 流程调用的高层运动能力。
/// </summary>
public interface IMotionService
{
    /// <summary>
    /// 运动到预设 Home 位（关节空间）。
    /// </summary>
    Task MoveHomeAsync();

    /// <summary>
    /// 单个关节相对 jog，单位是弧度。
    /// </summary>
    Task JogJointAsync(int jointIndex, double deltaRad);

    /// <summary>
    /// TCP 增量 jog。
    /// dx/dy/dz: 米
    /// dRx/dRy/dRz: 轴角增量，单位弧度
    /// </summary>
    Task JogTcpAsync(double dx, double dy, double dz, double dRx, double dRy, double dRz);

    /// <summary>
    /// 按关节目标移动。
    /// </summary>
    Task MoveToJointAsync(double[] jointsRad, double speed = 0.2, double acc = 0.2);

    /// <summary>
    /// 按 TCP 目标位姿移动。
    /// tcpPose = [x, y, z, rx, ry, rz]
    /// </summary>
    Task MoveToPoseAsync(double[] tcpPose, double speed = 0.1, double acc = 0.1);

    /// <summary>
    /// 只做位置直线增量移动。
    /// </summary>
    Task MoveLinearOffsetAsync(double dx, double dy, double dz);

    /// <summary>
    /// 只做工具姿态增量旋转。
    /// </summary>
    Task RotateToolAsync(double dRx, double dRy, double dRz);

    /// <summary>
    /// 一个简单的“预接近 -> 目标 -> 停留 -> 退出”模板动作。
    /// 适合按点、接触、抓取前的最后一段。
    /// </summary>
    Task PressPointAsync(double[] approachPose, double[] targetPose, int dwellMs = 300);

    /// <summary>
    /// 执行一组关节姿态序列。
    /// 适合标定点采样。
    /// </summary>
    Task RunCalibrationJointSequenceAsync(IEnumerable<double[]> jointPoints, int dwellMs = 1500);
}