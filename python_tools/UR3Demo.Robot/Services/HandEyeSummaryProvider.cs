using System.Text.Json;
using UR3Demo.Core;

namespace UR3Demo.Robot.Services;

/// <summary>
/// 脚本级说明：
/// 读取 hand-eye summary.json，并提供相机前向轴（在 tool 坐标系下）。
/// 
/// 约定：
/// - 使用 T_tool_camera 的旋转部分
/// - camera +Z 视为默认前向轴
/// - 若使用 NegativeZ，则取相反方向
/// 
/// 你给的 summary.json 已经包含 T_tool_camera / T_camera_tool，
/// 所以后续“沿相机视线轴向移动”可以直接依赖本类。
/// </summary>
public class HandEyeSummaryProvider : IHandEyeSummaryProvider
{
    private readonly ILogger _logger;
    private readonly Dictionary<string, HandEyeSummaryModel> _cache = new(StringComparer.OrdinalIgnoreCase);

    public HandEyeSummaryProvider(ILogger logger)
    {
        _logger = logger;
    }

    public HandEyeSummaryModel Load(string path)
    {
        if (string.IsNullOrWhiteSpace(path))
            throw new ArgumentException("summary.json path cannot be empty.", nameof(path));

        string fullPath = Path.GetFullPath(path);

        if (_cache.TryGetValue(fullPath, out var cached))
            return cached;

        if (!File.Exists(fullPath))
            throw new FileNotFoundException("summary.json not found.", fullPath);

        var json = File.ReadAllText(fullPath);
        var summary = JsonSerializer.Deserialize<HandEyeSummaryModel>(json, new JsonSerializerOptions
        {
            PropertyNameCaseInsensitive = true
        }) ?? throw new InvalidOperationException("Failed to parse hand-eye summary.");

        if (summary.TToolCamera == null || summary.TToolCamera.Length < 4)
            throw new InvalidOperationException("summary.json does not contain valid T_tool_camera.");

        _cache[fullPath] = summary;
        _logger.Info($"Loaded hand-eye summary: {fullPath}");
        return summary;
    }

    public double[] GetCameraForwardInTool(string path, CameraAxisConvention axisConvention)
    {
        var summary = Load(path);
        var ttc = summary.TToolCamera;

        // R_tool_camera 的第三列，就是 camera +Z 在 tool 坐标系中的方向
        double[] zAxisInTool =
        [
            ttc[0][2],
            ttc[1][2],
            ttc[2][2]
        ];

        if (axisConvention == CameraAxisConvention.NegativeZ)
        {
            zAxisInTool[0] *= -1.0;
            zAxisInTool[1] *= -1.0;
            zAxisInTool[2] *= -1.0;
        }

        return Normalize(zAxisInTool);
    }

    private static double[] Normalize(double[] v)
    {
        double norm = Math.Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        if (norm < 1e-12) return [0, 0, 1];
        return [v[0] / norm, v[1] / norm, v[2] / norm];
    }
}