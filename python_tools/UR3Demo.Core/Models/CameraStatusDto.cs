namespace UR3Demo.Core;

public sealed class CameraStatusDto
{
    public bool IsOpen { get; set; }
    public bool PreviewRunning { get; set; }
    public double? ExposureMs { get; set; }
    public double? GainDb { get; set; }
    public double? FramerateHz { get; set; }
    public double? MeasuredFps { get; set; }
    public bool? AutoWb { get; set; }
    public double? WbR { get; set; }
    public double? WbG { get; set; }
    public double? WbB { get; set; }
    public string? LastError { get; set; }
}