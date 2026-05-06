namespace UR3Demo.Core;

/// <summary>
/// 职责：底层机器人驱动接口。
/// 只负责“连接、状态、原始运动命令、停止”等基础能力。
/// 不负责 UI 逻辑，不负责复杂流程，不负责标定编排。
/// </summary>
public interface IRobotService
{
    Task ConnectAsync(string ip);
    Task DisconnectAsync();

    /// <summary>
    /// 关节空间运动。
    /// jointsRad 长度必须为 6，单位为弧度。
    /// </summary>
    Task MoveJAsync(double[] jointsRad, double speed, double acc);

    /// <summary>
    /// TCP 线性运动。
    /// tcpPose = [x, y, z, rx, ry, rz]
    /// 位置单位：米；姿态单位：轴角弧度。
    /// </summary>
    Task MoveLAsync(double[] tcpPose, double speed, double acc);

    /// <summary>
    /// 单个关节相对 jog。
    /// </summary>
    Task JogJointAsync(int jointIndex, double deltaRadians);

    Task StopAsync();

    RobotSnapshot GetSnapshot();

    event Action<RobotSnapshot>? SnapshotUpdated;
}