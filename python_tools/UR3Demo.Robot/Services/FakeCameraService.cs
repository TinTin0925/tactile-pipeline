using UR3Demo.Core;

namespace UR3Demo.Robot.Services;

/// <summary>
/// 职责：占位相机服务。
/// 在真实 XIMEA 接入前，用固定 PNG 字节模拟预览与抓拍。
/// 不依赖 WPF 类型，保证 Robot 层保持纯类库。
/// </summary>
public class FakeCameraService : ICameraService
{
    private Timer? _timer;

    public bool IsOpened { get; private set; }

    public event Action<byte[]?>? PreviewFrameArrived;

    public Task OpenAsync()
    {
        IsOpened = true;

        _timer = new Timer(_ =>
        {
            PreviewFrameArrived?.Invoke(CaptureImageBytes());
        }, null, 0, 500);

        return Task.CompletedTask;
    }

    public Task CloseAsync()
    {
        _timer?.Dispose();
        _timer = null;
        IsOpened = false;
        return Task.CompletedTask;
    }

    public byte[]? CaptureImageBytes()
    {
        // 一个极小的内置 PNG（1x1 像素），先用于打通流程。
        // 后续替换成真实 XIMEA 抓拍即可。
        const string base64Png =
            "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAusB9Wn8nS8AAAAASUVORK5CYII=";

        return Convert.FromBase64String(base64Png);
    }
}