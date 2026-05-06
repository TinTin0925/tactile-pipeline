using System.Globalization;
using System.Windows;
using UR3Demo.Core;
using UR3Demo.Robot.Services;

namespace UR3Demo.UI;

public partial class CameraSettingsWindow : Window
{
    private readonly PythonCameraServiceClient _cameraClient;

    public CameraSettingsWindow(PythonCameraServiceClient cameraClient)
    {
        InitializeComponent();
        _cameraClient = cameraClient;
        Loaded += CameraSettingsWindow_Loaded;
    }

    private async void CameraSettingsWindow_Loaded(object sender, RoutedEventArgs e)
    {
        await RefreshStatusInternalAsync();
    }

    private async Task RefreshStatusInternalAsync()
    {
        try
        {
            CameraStatusDto s = await _cameraClient.GetStatusAsync();

            StatusExposureText.Text = FormatNullable(s.ExposureMs);
            StatusGainText.Text = FormatNullable(s.GainDb);
            StatusFramerateText.Text = FormatNullable(s.FramerateHz);
            StatusMeasuredFpsText.Text = FormatNullable(s.MeasuredFps);
            StatusAutoWbText.Text = s.AutoWb.HasValue ? (s.AutoWb.Value ? "开启" : "关闭") : "--";
            StatusWbText.Text = $"{FormatNullable(s.WbR)}, {FormatNullable(s.WbG)}, {FormatNullable(s.WbB)}";
            StatusErrorText.Text = string.IsNullOrWhiteSpace(s.LastError) ? "--" : s.LastError;

            if (s.ExposureMs.HasValue) ExposureTextBox.Text = s.ExposureMs.Value.ToString("F2", CultureInfo.InvariantCulture);
            if (s.GainDb.HasValue) GainTextBox.Text = s.GainDb.Value.ToString("F2", CultureInfo.InvariantCulture);
            if (s.FramerateHz.HasValue) FramerateTextBox.Text = s.FramerateHz.Value.ToString("F2", CultureInfo.InvariantCulture);

            if (s.WbR.HasValue) WbRTextBox.Text = s.WbR.Value.ToString("F3", CultureInfo.InvariantCulture);
            if (s.WbG.HasValue) WbGTextBox.Text = s.WbG.Value.ToString("F3", CultureInfo.InvariantCulture);
            if (s.WbB.HasValue) WbBTextBox.Text = s.WbB.Value.ToString("F3", CultureInfo.InvariantCulture);
        }
        catch (Exception ex)
        {
            MessageBox.Show(ex.Message, "Refresh Status Error");
        }
    }

    private async void RefreshStatus_Click(object sender, RoutedEventArgs e)
    {
        await RefreshStatusInternalAsync();
    }

    private async void ApplyImageParams_Click(object sender, RoutedEventArgs e)
    {
        try
        {
            double? exposure = ReadNullableDouble(ExposureTextBox.Text);
            double? gain = ReadNullableDouble(GainTextBox.Text);
            double? framerate = ReadNullableDouble(FramerateTextBox.Text);

            await _cameraClient.ApplyManualImageAsync(exposure, gain, framerate);
            await RefreshStatusInternalAsync();
            MessageBox.Show("曝光 / 增益 / 帧率已应用。", "Success");
        }
        catch (Exception ex)
        {
            MessageBox.Show(ex.Message, "Apply Image Params Error");
        }
    }

    private async void EnableAwb_Click(object sender, RoutedEventArgs e)
    {
        try
        {
            await _cameraClient.EnableAutoWhiteBalanceAsync();
            await RefreshStatusInternalAsync();
            MessageBox.Show("自动白平衡已开启。", "Success");
        }
        catch (Exception ex)
        {
            MessageBox.Show(ex.Message, "Enable AWB Error");
        }
    }

    private async void DisableAwb_Click(object sender, RoutedEventArgs e)
    {
        try
        {
            await _cameraClient.DisableAutoWhiteBalanceAsync();
            await RefreshStatusInternalAsync();
            MessageBox.Show("自动白平衡已关闭。", "Success");
        }
        catch (Exception ex)
        {
            MessageBox.Show(ex.Message, "Disable AWB Error");
        }
    }

    private async void FreezeAwb_Click(object sender, RoutedEventArgs e)
    {
        try
        {
            await _cameraClient.FreezeAutoWhiteBalanceAsync();
            await RefreshStatusInternalAsync();
            MessageBox.Show("当前自动白平衡已冻结并保存。", "Success");
        }
        catch (Exception ex)
        {
            MessageBox.Show(ex.Message, "Freeze AWB Error");
        }
    }

    private async void ApplyManualWb_Click(object sender, RoutedEventArgs e)
    {
        try
        {
            double wbR = ReadRequiredDouble(WbRTextBox.Text, "WB R");
            double wbG = ReadRequiredDouble(WbGTextBox.Text, "WB G");
            double wbB = ReadRequiredDouble(WbBTextBox.Text, "WB B");

            await _cameraClient.ApplyManualWhiteBalanceAsync(wbR, wbG, wbB);
            await RefreshStatusInternalAsync();
            MessageBox.Show("手动白平衡已写入。", "Success");
        }
        catch (Exception ex)
        {
            MessageBox.Show(ex.Message, "Apply Manual WB Error");
        }
    }

    private async void OpenPythonPanel_Click(object sender, RoutedEventArgs e)
    {
        try
        {
            await _cameraClient.OpenParameterPanelAsync();
            MessageBox.Show("本地参数面板已打开。", "Success");
        }
        catch (Exception ex)
        {
            MessageBox.Show(ex.Message, "Open Python Panel Error");
        }
    }

    private async void SaveSettings_Click(object sender, RoutedEventArgs e)
    {
        try
        {
            await _cameraClient.SaveCurrentSettingsAsync();
            await RefreshStatusInternalAsync();
            MessageBox.Show("当前相机参数已保存。", "Success");
        }
        catch (Exception ex)
        {
            MessageBox.Show(ex.Message, "Save Settings Error");
        }
    }

    private void Close_Click(object sender, RoutedEventArgs e)
    {
        Close();
    }

    private static string FormatNullable(double? value)
    {
        return value.HasValue ? value.Value.ToString("F3", CultureInfo.InvariantCulture) : "--";
    }

    private static double? ReadNullableDouble(string? text)
    {
        text = text?.Trim();
        if (string.IsNullOrWhiteSpace(text))
            return null;

        if (double.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out var v))
            return v;
        if (double.TryParse(text, NumberStyles.Float, CultureInfo.CurrentCulture, out v))
            return v;

        throw new InvalidOperationException($"无效数字：{text}");
    }

    private static double ReadRequiredDouble(string? text, string fieldName)
    {
        var value = ReadNullableDouble(text);
        if (!value.HasValue)
            throw new InvalidOperationException($"{fieldName} 不能为空。");
        return value.Value;
    }
}