using System.Globalization;
using System.Text.Json;
using UR3Demo.Core;

namespace UR3Demo.Robot.Services;

/// <summary>
/// 脚本级说明：
/// 负责把 waypoint 文件解析成 AutoCapturePlan。
/// 
/// 支持：
/// 1. 新的任务 JSON
/// 2. 旧的 hand-eye samples.jsonl
/// 
/// 其中 samples.jsonl 兼容现有手眼标定格式：
/// 每一行包含 ImagePath + TBaseTool。
/// 我们会把 TBaseTool 反解成 [x,y,z,rx,ry,rz]，作为目标 TCP 位姿。
/// </summary>
public class CapturePlanLoader : ICapturePlanLoader
{
    private readonly ILogger _logger;

    public CapturePlanLoader(ILogger logger)
    {
        _logger = logger;
    }

    public AutoCapturePlan LoadFromJson(string path)
    {
        if (string.IsNullOrWhiteSpace(path))
            throw new ArgumentException("Plan json path cannot be empty.", nameof(path));

        if (!File.Exists(path))
            throw new FileNotFoundException("Plan json file not found.", path);

        var json = File.ReadAllText(path);
        var plan = JsonSerializer.Deserialize<AutoCapturePlan>(json, new JsonSerializerOptions
        {
            PropertyNameCaseInsensitive = true
        }) ?? throw new InvalidOperationException("Failed to parse plan json.");

        plan.InputWaypointsPath = Path.GetFullPath(path);

        if (plan.Waypoints == null || plan.Waypoints.Count == 0)
            throw new InvalidOperationException("Plan json contains no waypoints.");

        ValidateWaypoints(plan.Waypoints);
        _logger.Info($"Loaded auto capture plan json: {path}, waypoints={plan.Waypoints.Count}");
        return plan;
    }

    public AutoCapturePlan LoadFromSamplesJsonl(string path)
    {
        if (string.IsNullOrWhiteSpace(path))
            throw new ArgumentException("samples.jsonl path cannot be empty.", nameof(path));

        if (!File.Exists(path))
            throw new FileNotFoundException("samples.jsonl not found.", path);

        var lines = File.ReadLines(path)
            .Where(line => !string.IsNullOrWhiteSpace(line))
            .ToList();

        if (lines.Count == 0)
            throw new InvalidOperationException("samples.jsonl is empty.");

        var waypoints = new List<CaptureWaypoint>();

        foreach (var line in lines)
        {
            var sample = JsonSerializer.Deserialize<HandEyeSample>(line, new JsonSerializerOptions
            {
                PropertyNameCaseInsensitive = true
            });

            if (sample == null)
                continue;

            if (sample.TBaseTool == null || sample.TBaseTool.Length < 4)
                throw new InvalidOperationException($"Sample #{sample.Index} does not contain valid TBaseTool.");

            var tcpPose = MatrixToPose6D(sample.TBaseTool);

            waypoints.Add(new CaptureWaypoint
            {
                Index = sample.Index,
                Name = $"from_jsonl_{sample.Index:D4}",
                TcpPose = tcpPose,
                Enabled = true
            });
        }

        if (waypoints.Count == 0)
            throw new InvalidOperationException("No valid waypoints found in samples.jsonl.");

        var plan = new AutoCapturePlan
        {
            TaskType = "calibration",
            InputWaypointsPath = Path.GetFullPath(path),
            Waypoints = waypoints
        };

        _logger.Info($"Loaded waypoints from samples.jsonl: {path}, waypoints={waypoints.Count}");
        return plan;
    }

    private static void ValidateWaypoints(IEnumerable<CaptureWaypoint> waypoints)
    {
        foreach (var wp in waypoints)
        {
            if (!wp.Enabled) continue;

            bool hasTcp = wp.TcpPose is { Length: 6 };
            bool hasJoint = wp.JointPose is { Length: 6 };

            if (!hasTcp && !hasJoint)
                throw new InvalidOperationException(
                    $"Waypoint #{wp.Index} must provide either TcpPose[6] or JointPose[6].");
        }
    }

    /// <summary>
    /// 把 4x4 齐次矩阵转为 UR 风格 pose:
    /// [x, y, z, rx, ry, rz]
    /// 
    /// 旋转部分使用 Rodrigues 轴角向量表示。
    /// </summary>
    private static double[] MatrixToPose6D(double[][] m)
    {
        double x = m[0][3];
        double y = m[1][3];
        double z = m[2][3];

        double r00 = m[0][0], r01 = m[0][1], r02 = m[0][2];
        double r10 = m[1][0], r11 = m[1][1], r12 = m[1][2];
        double r20 = m[2][0], r21 = m[2][1], r22 = m[2][2];

        double trace = r00 + r11 + r22;
        double cosTheta = Math.Clamp((trace - 1.0) / 2.0, -1.0, 1.0);
        double theta = Math.Acos(cosTheta);

        double rx, ry, rz;

        if (theta < 1e-12)
        {
            rx = ry = rz = 0.0;
        }
        else
        {
            double denom = 2.0 * Math.Sin(theta);
            double kx = (r21 - r12) / denom;
            double ky = (r02 - r20) / denom;
            double kz = (r10 - r01) / denom;

            rx = kx * theta;
            ry = ky * theta;
            rz = kz * theta;
        }

        return [x, y, z, rx, ry, rz];
    }
}