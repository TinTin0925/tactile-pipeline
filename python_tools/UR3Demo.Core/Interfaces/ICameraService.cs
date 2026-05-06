namespace UR3Demo.Core;

/// <summary>
/// 职责：抽象相机服务。
/// 当前只要求：打开、关闭、取预览帧、抓拍单帧。
/// </summary>
public interface ICameraService
{
    bool IsOpened { get; }

    Task OpenAsync();
    Task CloseAsync();

    /// <summary>
    /// 返回一张用于保存的图像字节，例如 PNG/JPEG。
    /// </summary>
    byte[]? CaptureImageBytes();

    /// <summary>
    /// 预览帧到达事件。参数是图像字节流，可后续改成 BitmapSource。
    /// </summary>
    event Action<byte[]?>? PreviewFrameArrived;
}