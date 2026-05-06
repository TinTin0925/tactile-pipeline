using System.Reflection;
using UR3Demo.Core;
using UR3Demo.Robot.Logging;
using UnderAutomation.UniversalRobots;
using UnderAutomation.UniversalRobots.Common;
using UnderAutomation.UniversalRobots.Rtde;

namespace UR3Demo.Robot.Services;

/// <summary>
/// 职责：UR 机器人底层驱动实现。
/// 
/// 负责：
/// 1. 建立与 UR 的连接（Dashboard / Primary / RTDE）
/// 2. 订阅 RTDE 状态并更新 RobotSnapshot
/// 3. 提供底层 MoveJ / MoveL / Stop / Joint Jog 能力
/// 
/// 不负责：
/// 1. UI 交互逻辑
/// 2. 标定流程编排
/// 3. 复杂路径模板
/// 4. Python 通信
/// </summary>
public class UrRobotService : IRobotService
{
    private static readonly object LicenseLock = new();
    private static bool _licenseRegistered;

    private UR? _robot;
    private readonly RobotSnapshot _snapshot = new();
    private readonly ILogger _logger;

    public event Action<RobotSnapshot>? SnapshotUpdated;

    public UrRobotService()
        : this(new FileLogger())
    {
    }

    public UrRobotService(ILogger logger)
    {
        _logger = logger;
    }

    public async Task ConnectAsync(string ip)
    {
        await Task.Run(() =>
        {
            try
            {
                _logger.Info($"Connecting to robot: {ip}");

                RegisterUnderAutomationLicenseFromEnvironment();

                _robot = new UR();

                var param = new ConnectParameters(ip);

                // Dashboard：用于 stop、状态控制等
                param.Dashboard.Enable = true;

                // Primary Interface：用于发送 URScript
                param.PrimaryInterface.Enable = true;

                // RTDE：用于实时状态读取
                param.Rtde.Enable = true;
                param.Rtde.Frequency = 10; // demo 先保守设置
                param.Rtde.Version = RtdeVersions.V2;

                // 订阅本 demo 需要的输出字段
                param.Rtde.OutputSetup.Add(RtdeOutputData.ActualQ);
                param.Rtde.OutputSetup.Add(RtdeOutputData.ActualTcpPose);
                param.Rtde.OutputSetup.Add(RtdeOutputData.RobotMode);
                param.Rtde.OutputSetup.Add(RtdeOutputData.SafetyMode);
                param.Rtde.OutputSetup.Add(RtdeOutputData.ActualDigitalInputBits);
                param.Rtde.OutputSetup.Add(RtdeOutputData.ActualDigitalOutputBits);

                RegisterRtdeCallbacks();

                _robot.Connect(param);

                _snapshot.IsConnected = true;
                _snapshot.StatusText = "Connected";
                _snapshot.Timestamp = DateTime.Now;

                _logger.Info("Robot connected successfully.");
                RaiseSnapshot();
            }
            catch (Exception ex)
            {
                _logger.Error("Failed to connect robot.", ex);
                throw;
            }
        });
    }

    private static void RegisterUnderAutomationLicenseFromEnvironment()
    {
        if (_licenseRegistered)
            return;

        lock (LicenseLock)
        {
            if (_licenseRegistered)
                return;

            string? licenseName = Environment.GetEnvironmentVariable("UNDERAUTOMATION_LICENSE_NAME");
            string? licenseKey = Environment.GetEnvironmentVariable("UNDERAUTOMATION_LICENSE_KEY");

            if (!string.IsNullOrWhiteSpace(licenseName) && !string.IsNullOrWhiteSpace(licenseKey))
            {
                UR.RegisterLicense(licenseName, licenseKey);
            }

            _licenseRegistered = true;
        }
    }

    public async Task DisconnectAsync()
    {
        await Task.Run(() =>
        {
            try
            {
                if (_robot != null)
                {
                    _robot.Disconnect();
                    _robot = null;
                }

                _snapshot.IsConnected = false;
                _snapshot.StatusText = "Disconnected";
                _snapshot.Timestamp = DateTime.Now;

                _logger.Info("Robot disconnected.");
                RaiseSnapshot();
            }
            catch (Exception ex)
            {
                _logger.Error("Failed to disconnect robot.", ex);
                throw;
            }
        });
    }

    public async Task MoveJAsync(double[] jointsRad, double speed, double acc)
    {
        if (_robot == null) throw new InvalidOperationException("Robot is not connected.");
        ValidateJointArray(jointsRad);

        await Task.Run(() =>
        {
            try
            {
                string script =
                    $"movej([{FormatJointArray(jointsRad)}], a={acc:F6}, v={speed:F6})";

                _logger.Info($"Send MoveJ: [{FormatJointArray(jointsRad)}], a={acc:F4}, v={speed:F4}");
                _robot.PrimaryInterface.Script.Send(script);
            }
            catch (Exception ex)
            {
                _logger.Error("Failed to execute MoveJ.", ex);
                throw;
            }
        });
    }

    public async Task MoveLAsync(double[] tcpPose, double speed, double acc)
    {
        if (_robot == null) throw new InvalidOperationException("Robot is not connected.");
        ValidateTcpPoseArray(tcpPose);

        await Task.Run(() =>
        {
            try
            {
                string script =
                    $"movel(p[{FormatPoseArray(tcpPose)}], a={acc:F6}, v={speed:F6})";

                _logger.Info($"Send MoveL: [{FormatPoseArray(tcpPose)}], a={acc:F4}, v={speed:F4}");
                _robot.PrimaryInterface.Script.Send(script);
            }
            catch (Exception ex)
            {
                _logger.Error("Failed to execute MoveL.", ex);
                throw;
            }
        });
    }

    public async Task JogJointAsync(int jointIndex, double deltaRadians)
    {
        if (_robot == null) throw new InvalidOperationException("Robot is not connected.");
        if (jointIndex < 0 || jointIndex > 5)
            throw new ArgumentOutOfRangeException(nameof(jointIndex), "jointIndex must be between 0 and 5.");

        await Task.Run(() =>
        {
            try
            {
                var q = _snapshot.ActualQ.ToArray();
                q[jointIndex] += deltaRadians;

                string script =
                    $"movej([{FormatJointArray(q)}], a=0.300000, v=0.200000)";

                _logger.Info($"Jog joint J{jointIndex}, delta={deltaRadians:F6} rad");
                _robot.PrimaryInterface.Script.Send(script);
            }
            catch (Exception ex)
            {
                _logger.Error("Failed to jog joint.", ex);
                throw;
            }
        });
    }

    public async Task StopAsync()
    {
        if (_robot == null) return;

        await Task.Run(() =>
        {
            try
            {
                _logger.Warn("Stop requested.");
                _robot.Dashboard.Stop();
            }
            catch (Exception ex)
            {
                _logger.Error("Failed to stop robot.", ex);
                throw;
            }
        });
    }

    public RobotSnapshot GetSnapshot()
    {
        return new RobotSnapshot
        {
            Timestamp = _snapshot.Timestamp,
            IsConnected = _snapshot.IsConnected,
            ActualQ = _snapshot.ActualQ.ToArray(),
            ActualTcpPose = _snapshot.ActualTcpPose.ToArray(),
            RobotMode = _snapshot.RobotMode,
            SafetyMode = _snapshot.SafetyMode,
            DigitalInputs = _snapshot.DigitalInputs,
            DigitalOutputs = _snapshot.DigitalOutputs,
            StatusText = _snapshot.StatusText
        };
    }

    private void RegisterRtdeCallbacks()
    {
        if (_robot == null) return;

        _robot.Rtde.OutputDataReceived += (s, e) =>
        {
            try
            {
                var values = e.OutputDataValues;

                _snapshot.Timestamp = DateTime.Now;
                _snapshot.ActualQ = ReadJointValues(values.ActualQ);
                _snapshot.ActualTcpPose = new double[]
                {
                    values.ActualTcpPose.X,
                    values.ActualTcpPose.Y,
                    values.ActualTcpPose.Z,
                    values.ActualTcpPose.Rx,
                    values.ActualTcpPose.Ry,
                    values.ActualTcpPose.Rz
                };
                _snapshot.RobotMode = (int)values.RobotMode;
                _snapshot.SafetyMode = (int)values.SafetyMode;
                _snapshot.DigitalInputs = values.ActualDigitalInputBits;
                _snapshot.DigitalOutputs = values.ActualDigitalOutputBits;
                _snapshot.StatusText = "RTDE Running";

                RaiseSnapshot();
            }
            catch (Exception ex)
            {
                _logger.Error("Failed to process RTDE output packet.", ex);
            }
        };
    }

    private void RaiseSnapshot()
    {
        SnapshotUpdated?.Invoke(GetSnapshot());
    }

    private static double[] ReadJointValues(object joints)
    {
        if (joints == null) return new double[6];

        var t = joints.GetType();
        string[] names = { "Base", "Shoulder", "Elbow", "Wrist1", "Wrist2", "Wrist3" };
        var values = new double[6];

        for (int i = 0; i < 6; i++)
        {
            var p = t.GetProperty(names[i], BindingFlags.Public | BindingFlags.Instance);
            if (p == null)
                throw new InvalidOperationException($"JointsDoubleValues 缺少属性: {names[i]}");

            var v = p.GetValue(joints);
            if (v == null)
                throw new InvalidOperationException($"JointsDoubleValues 属性 {names[i]} 的值为空");

            values[i] = Convert.ToDouble(v);
        }

        return values;
    }

    private static void ValidateJointArray(double[] jointsRad)
    {
        if (jointsRad == null || jointsRad.Length != 6)
            throw new ArgumentException("Joint target must contain exactly 6 values.", nameof(jointsRad));
    }

    private static void ValidateTcpPoseArray(double[] tcpPose)
    {
        if (tcpPose == null || tcpPose.Length != 6)
            throw new ArgumentException("TCP pose must contain exactly 6 values.", nameof(tcpPose));
    }

    private static string FormatJointArray(double[] joints)
    {
        return string.Join(",", joints.Select(v => v.ToString("F6", System.Globalization.CultureInfo.InvariantCulture)));
    }

    private static string FormatPoseArray(double[] pose)
    {
        return string.Join(",", pose.Select(v => v.ToString("F6", System.Globalization.CultureInfo.InvariantCulture)));
    }
}
