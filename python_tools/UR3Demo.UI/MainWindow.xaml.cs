using System.Globalization;
using System.Windows;
using UR3Demo.Core;
using UR3Demo.Robot.Logging;
using UR3Demo.Robot.Services;

namespace UR3Demo.UI;

/// <summary>
/// 职责：主控制面板 UI。
/// 
/// 当前负责：
/// 1. 连接 / 断开机器人
/// 2. 显示 TCP / Joint / RobotMode / SafetyMode
/// 3. Joint Jog
/// 4. TCP 位置 Jog
/// 5. TCP 姿态 Jog
/// 6. MoveJ Home
/// 
/// 当前不负责：
/// 1. 日志文件读取与实时绑定
/// 2. Python 服务通信展示
/// 3. 标定流程编排
/// </summary>
public partial class MainWindow : Window
{
    private readonly IRobotService _robotService;
    private readonly IMotionService _motionService;
    private readonly ILogger _logger;

    public MainWindow()
    {
        InitializeComponent();

        _logger = new FileLogger();
        _robotService = new UrRobotService(_logger);
        _motionService = new MotionService(_robotService, _logger);

        _robotService.SnapshotUpdated += OnSnapshotUpdated;
    }

    private async void ConnectButton_Click(object sender, RoutedEventArgs e)
    {
        try
        {
            await _robotService.ConnectAsync(IpTextBox.Text.Trim());
        }
        catch (Exception ex)
        {
            MessageBox.Show(ex.Message, "Connect Error");
        }
    }

    private async void DisconnectButton_Click(object sender, RoutedEventArgs e)
    {
        try
        {
            await _robotService.DisconnectAsync();
        }
        catch (Exception ex)
        {
            MessageBox.Show(ex.Message, "Disconnect Error");
        }
    }

    private async void MoveJButton_Click(object sender, RoutedEventArgs e)
    {
        try
        {
            await _motionService.MoveHomeAsync();
        }
        catch (Exception ex)
        {
            MessageBox.Show(ex.Message, "MoveJ Home Error");
        }
    }

    private async void StopButton_Click(object sender, RoutedEventArgs e)
    {
        try
        {
            await _robotService.StopAsync();
        }
        catch (Exception ex)
        {
            MessageBox.Show(ex.Message, "Stop Error");
        }
    }

    private void OpenHandEyeCapture_Click(object sender, RoutedEventArgs e)
    {
        var win = new HandEyeCaptureWindow(_robotService);
        win.Show();
    }

    #region Joint Jog

    private async Task JogJointAsync(int jointIndex, double sign)
    {
        try
        {
            double stepDeg = ReadDoubleFromTextBox(JointStepTextBox, 5.0);
            double deltaRad = sign * stepDeg * Math.PI / 180.0;
            await _motionService.JogJointAsync(jointIndex, deltaRad);
        }
        catch (Exception ex)
        {
            MessageBox.Show(ex.Message, "Joint Jog Error");
        }
    }

    private async void J0Minus_Click(object sender, RoutedEventArgs e) => await JogJointAsync(0, -1);
    private async void J0Plus_Click(object sender, RoutedEventArgs e) => await JogJointAsync(0, +1);

    private async void J1Minus_Click(object sender, RoutedEventArgs e) => await JogJointAsync(1, -1);
    private async void J1Plus_Click(object sender, RoutedEventArgs e) => await JogJointAsync(1, +1);

    private async void J2Minus_Click(object sender, RoutedEventArgs e) => await JogJointAsync(2, -1);
    private async void J2Plus_Click(object sender, RoutedEventArgs e) => await JogJointAsync(2, +1);

    private async void J3Minus_Click(object sender, RoutedEventArgs e) => await JogJointAsync(3, -1);
    private async void J3Plus_Click(object sender, RoutedEventArgs e) => await JogJointAsync(3, +1);

    private async void J4Minus_Click(object sender, RoutedEventArgs e) => await JogJointAsync(4, -1);
    private async void J4Plus_Click(object sender, RoutedEventArgs e) => await JogJointAsync(4, +1);

    private async void J5Minus_Click(object sender, RoutedEventArgs e) => await JogJointAsync(5, -1);
    private async void J5Plus_Click(object sender, RoutedEventArgs e) => await JogJointAsync(5, +1);

    #endregion

    #region TCP Position Jog

    private async Task JogTcpPositionAsync(double dx, double dy, double dz)
    {
        try
        {
            // UI 输入是 mm，这里转成 m
            double stepMm = ReadDoubleFromTextBox(TcpStepTextBox, 5.0);
            double stepM = stepMm / 1000.0;

            await _motionService.JogTcpAsync(
                dx * stepM,
                dy * stepM,
                dz * stepM,
                0, 0, 0);
        }
        catch (Exception ex)
        {
            MessageBox.Show(ex.Message, "TCP Position Jog Error");
        }
    }

    private async void MoveForward_Click(object sender, RoutedEventArgs e) => await JogTcpPositionAsync(+1, 0, 0);
    private async void MoveBack_Click(object sender, RoutedEventArgs e) => await JogTcpPositionAsync(-1, 0, 0);
    private async void MoveLeft_Click(object sender, RoutedEventArgs e) => await JogTcpPositionAsync(0, +1, 0);
    private async void MoveRight_Click(object sender, RoutedEventArgs e) => await JogTcpPositionAsync(0, -1, 0);
    private async void MoveUp_Click(object sender, RoutedEventArgs e) => await JogTcpPositionAsync(0, 0, +1);
    private async void MoveDown_Click(object sender, RoutedEventArgs e) => await JogTcpPositionAsync(0, 0, -1);

    #endregion

    #region TCP Rotation Jog

    private async Task JogTcpRotationAsync(double dRxSign, double dRySign, double dRzSign)
    {
        try
        {
            double stepDeg = ReadDoubleFromTextBox(RotStepTextBox, 5.0);
            double stepRad = stepDeg * Math.PI / 180.0;

            await _motionService.JogTcpAsync(
                0, 0, 0,
                dRxSign * stepRad,
                dRySign * stepRad,
                dRzSign * stepRad);
        }
        catch (Exception ex)
        {
            MessageBox.Show(ex.Message, "TCP Rotation Jog Error");
        }
    }

    private async void RxMinus_Click(object sender, RoutedEventArgs e) => await JogTcpRotationAsync(-1, 0, 0);
    private async void RxPlus_Click(object sender, RoutedEventArgs e) => await JogTcpRotationAsync(+1, 0, 0);

    private async void RyMinus_Click(object sender, RoutedEventArgs e) => await JogTcpRotationAsync(0, -1, 0);
    private async void RyPlus_Click(object sender, RoutedEventArgs e) => await JogTcpRotationAsync(0, +1, 0);

    private async void RzMinus_Click(object sender, RoutedEventArgs e) => await JogTcpRotationAsync(0, 0, -1);
    private async void RzPlus_Click(object sender, RoutedEventArgs e) => await JogTcpRotationAsync(0, 0, +1);

    #endregion

    #region Target Pose Move
    private void UseCurrentPose_Click(object sender, RoutedEventArgs e)
    {
        try
        {
            var s = _robotService.GetSnapshot();
            if (s.ActualTcpPose.Length < 6)
            {
                MessageBox.Show("Current TCP pose is invalid.", "Pose Error");
                return;
            }

            TargetXTextBox.Text = s.ActualTcpPose[0].ToString("F6", CultureInfo.InvariantCulture);
            TargetYTextBox.Text = s.ActualTcpPose[1].ToString("F6", CultureInfo.InvariantCulture);
            TargetZTextBox.Text = s.ActualTcpPose[2].ToString("F6", CultureInfo.InvariantCulture);
            TargetRxTextBox.Text = s.ActualTcpPose[3].ToString("F6", CultureInfo.InvariantCulture);
            TargetRyTextBox.Text = s.ActualTcpPose[4].ToString("F6", CultureInfo.InvariantCulture);
            TargetRzTextBox.Text = s.ActualTcpPose[5].ToString("F6", CultureInfo.InvariantCulture);
        }
        catch (Exception ex)
        {
            MessageBox.Show(ex.Message, "Use Current Pose Error");
        }
    }

    private async void MoveToTargetPose_Click(object sender, RoutedEventArgs e)
    {
        try
        {
            double[] target =
            [
                ReadDoubleFromTextBox(TargetXTextBox, 0.0),
                ReadDoubleFromTextBox(TargetYTextBox, 0.0),
                ReadDoubleFromTextBox(TargetZTextBox, 0.0),
                ReadDoubleFromTextBox(TargetRxTextBox, 0.0),
                ReadDoubleFromTextBox(TargetRyTextBox, 0.0),
                ReadDoubleFromTextBox(TargetRzTextBox, 0.0)
            ];

            await _motionService.MoveToPoseAsync(target, speed: 0.05, acc: 0.05);
        }
        catch (Exception ex)
        {
            MessageBox.Show(ex.Message, "Move To Target Error");
        }
    }
    
    #endregion

    private void OnSnapshotUpdated(RobotSnapshot s)
    {
        Dispatcher.Invoke(() =>
        {
            StatusTextBlock.Text = $"{s.StatusText}  |  {s.Timestamp:HH:mm:ss.fff}";

            if (s.ActualTcpPose.Length >= 6)
            {
                Tcp0.Text = $"X  = {s.ActualTcpPose[0]:F4}";
                Tcp1.Text = $"Y  = {s.ActualTcpPose[1]:F4}";
                Tcp2.Text = $"Z  = {s.ActualTcpPose[2]:F4}";
                Tcp3.Text = $"Rx = {s.ActualTcpPose[3]:F4}";
                Tcp4.Text = $"Ry = {s.ActualTcpPose[4]:F4}";
                Tcp5.Text = $"Rz = {s.ActualTcpPose[5]:F4}";
            }

            if (s.ActualQ.Length >= 6)
            {
                J0.Text = $"J0 = {s.ActualQ[0]:F4}";
                J1.Text = $"J1 = {s.ActualQ[1]:F4}";
                J2.Text = $"J2 = {s.ActualQ[2]:F4}";
                J3.Text = $"J3 = {s.ActualQ[3]:F4}";
                J4.Text = $"J4 = {s.ActualQ[4]:F4}";
                J5.Text = $"J5 = {s.ActualQ[5]:F4}";
            }

            RobotModeText.Text = $"RobotMode: {s.RobotMode}";
            SafetyModeText.Text = $"SafetyMode: {s.SafetyMode}";
        });
    }

    /// <summary>
    /// 从 TextBox 读取浮点数。
    /// 失败时返回默认值，并把文本框重置成默认值，避免后续重复报错。
    /// </summary>
    private static double ReadDoubleFromTextBox(System.Windows.Controls.TextBox textBox, double defaultValue)
    {
        var text = textBox.Text?.Trim();
        if (double.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out double v))
            return v;
        if (double.TryParse(text, NumberStyles.Float, CultureInfo.CurrentCulture, out v))
            return v;

        textBox.Text = defaultValue.ToString(CultureInfo.InvariantCulture);
        return defaultValue;
    }
}