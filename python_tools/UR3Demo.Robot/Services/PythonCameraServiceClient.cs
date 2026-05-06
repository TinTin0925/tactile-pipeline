using System.Text.Json;
using System.Net.Http.Json;
using UR3Demo.Core;

namespace UR3Demo.Robot.Services;

/// <summary>
/// 职责：通过 HTTP 访问 Python Camera Service。
/// 当前提供：
///    — 打开/关闭相机
///    — 获取最新预览帧
///    — 单张抓拍
///    — 查询状态
///    — 调整曝光/增益/帧率
///    — 自动白平衡 / 冻结白平衡 / 手动白平衡
///    — 打开本地参数面板
///    — 保存当前配置
/// </summary>
public class PythonCameraServiceClient : ICameraService, IDisposable
{
    private readonly HttpClient _http;
    private Timer? _previewTimer;

    public bool IsOpened { get; private set; }

    public event Action<byte[]?>? PreviewFrameArrived;

    public PythonCameraServiceClient(string baseUrl = "http://127.0.0.1:8001")
    {
        _http = new HttpClient
        {
            BaseAddress = new Uri(baseUrl),
            Timeout = TimeSpan.FromSeconds(5)
        };
    }

    public async Task OpenAsync()
    {
        var resp = await _http.PostAsync("/camera/open", null);
        resp.EnsureSuccessStatusCode();

        IsOpened = true;

        _previewTimer?.Dispose();
        _previewTimer = new Timer(async _ =>
        {
            try
            {
                var bytes = await _http.GetByteArrayAsync("/camera/frame");
                PreviewFrameArrived?.Invoke(bytes);
            }
            catch
            {
                // 预览失败时静默，避免打断 UI
            }
        }, null, 0, 300);
    }

    public async Task CloseAsync()
    {
        _previewTimer?.Dispose();
        _previewTimer = null;

        try
        {
            var resp = await _http.PostAsync("/camera/close", null);
            resp.EnsureSuccessStatusCode();
        }
        finally
        {
            IsOpened = false;
        }
    }

    public byte[]? CaptureImageBytes()
    {
        try
        {
            return _http.PostAsync("/camera/capture", null)
                        .GetAwaiter()
                        .GetResult()
                        .Content
                        .ReadAsByteArrayAsync()
                        .GetAwaiter()
                        .GetResult();
        }
        catch
        {
            return null;
        }
    }

    public async Task<CameraStatusDto> GetStatusAsync()
    {
        var resp = await _http.GetAsync("/camera/status");
        resp.EnsureSuccessStatusCode();

        var json = await resp.Content.ReadAsStringAsync();
        using var doc = JsonDocument.Parse(json);
        var root = doc.RootElement;

        return new CameraStatusDto
        {
            IsOpen = TryGetBool(root, "is_open") ?? false,
            PreviewRunning = TryGetBool(root, "preview_running") ?? false,
            ExposureMs = TryGetDouble(root, "exposure_ms"),
            GainDb = TryGetDouble(root, "gain_db"),
            FramerateHz = TryGetDouble(root, "framerate_hz"),
            MeasuredFps = TryGetDouble(root, "measured_fps"),
            AutoWb = TryGetBool(root, "auto_wb"),
            WbR = TryGetDouble(root, "wb_r"),
            WbG = TryGetDouble(root, "wb_g"),
            WbB = TryGetDouble(root, "wb_b"),
            LastError = root.TryGetProperty("last_error", out var err) ? err.ToString() : null
        };
    }

    public async Task ApplyManualImageAsync(double? exposureMs, double? gainDb, double? framerateHz)
    {
        var payload = new Dictionary<string, object?>();

        if (exposureMs.HasValue) payload["exposure_ms"] = exposureMs.Value;
        if (gainDb.HasValue) payload["gain_db"] = gainDb.Value;
        if (framerateHz.HasValue) payload["framerate_hz"] = framerateHz.Value;

        var resp = await _http.PostAsJsonAsync("/camera/manual_image", payload);
        resp.EnsureSuccessStatusCode();
    }

    public async Task EnableAutoWhiteBalanceAsync()
    {
        var resp = await _http.PostAsync("/camera/awb/enable", null);
        resp.EnsureSuccessStatusCode();
    }

    public async Task DisableAutoWhiteBalanceAsync()
    {
        var resp = await _http.PostAsync("/camera/awb/disable", null);
        resp.EnsureSuccessStatusCode();
    }

    public async Task FreezeAutoWhiteBalanceAsync()
    {
        var resp = await _http.PostAsync("/camera/awb/freeze", null);
        resp.EnsureSuccessStatusCode();
    }

    public async Task ApplyManualWhiteBalanceAsync(double wbR, double wbG, double wbB)
    {
        var payload = new
        {
            wb_r = wbR,
            wb_g = wbG,
            wb_b = wbB
        };

        var resp = await _http.PostAsJsonAsync("/camera/manual_wb", payload);
        resp.EnsureSuccessStatusCode();
    }

    public async Task SaveCurrentSettingsAsync()
    {
        var resp = await _http.PostAsync("/camera/settings/save", null);
        resp.EnsureSuccessStatusCode();
    }

    public async Task OpenParameterPanelAsync()
    {
        var resp = await _http.PostAsync("/camera/panel/open", null);
        resp.EnsureSuccessStatusCode();
    }

    public async Task CloseParameterPanelAsync()
    {
        var resp = await _http.PostAsync("/camera/panel/close", null);
        resp.EnsureSuccessStatusCode();
    }

    private static double? TryGetDouble(JsonElement root, string name)
    {
        if (!root.TryGetProperty(name, out var p)) return null;
        if (p.ValueKind == JsonValueKind.Number && p.TryGetDouble(out var v)) return v;
        return null;
    }

    private static bool? TryGetBool(JsonElement root, string name)
    {
        if (!root.TryGetProperty(name, out var p)) return null;
        if (p.ValueKind == JsonValueKind.True || p.ValueKind == JsonValueKind.False)
            return p.GetBoolean();
        return null;
    }

    public void Dispose()
    {
        _previewTimer?.Dispose();
        _http.Dispose();
    }
}