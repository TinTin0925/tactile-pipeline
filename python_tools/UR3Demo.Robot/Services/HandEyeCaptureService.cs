using System.Globalization;
using System.Text.Json;
using UR3Demo.Core;
using UR3Demo.Robot.Logging;

namespace UR3Demo.Robot.Services;

/// <summary>
/// 职责：管理 hand-eye 采集数据的输出目录、图片保存和 samples.jsonl 续写/新建。
/// 不负责相机预览，不负责机器人移动，不负责 UI。
/// </summary>
public class HandEyeCaptureService : IHandEyeCaptureService
{
    private readonly ILogger _logger;
    private HandEyeSessionInfo _session = new();

    public HandEyeCaptureService()
        : this(new FileLogger())
    {
    }

    public HandEyeCaptureService(ILogger logger)
    {
        _logger = logger;
    }

    public void ConfigureOutput(string pathInput)
    {
        if (string.IsNullOrWhiteSpace(pathInput))
            throw new ArgumentException("Output path cannot be empty.", nameof(pathInput));

        var normalized = Environment.ExpandEnvironmentVariables(pathInput.Trim());

        if (normalized.IndexOfAny(Path.GetInvalidPathChars()) >= 0)
            throw new ArgumentException("Output path contains invalid characters.", nameof(pathInput));

        string rootDir;
        string samplesJsonlPath;
        bool appendMode = false;
        int existingCount = 0;

        // 1) 输入是目录 or 目录格式
        if (Directory.Exists(normalized))
        {
            rootDir = normalized;
            samplesJsonlPath = Path.Combine(rootDir, "samples.jsonl");
        }
        else
        {
            var ext = Path.GetExtension(normalized);

            // 2) 输入是 .jsonl 文件
            if (ext.Equals(".jsonl", StringComparison.OrdinalIgnoreCase))
            {
                rootDir = Path.GetDirectoryName(Path.GetFullPath(normalized))
                          ?? throw new InvalidOperationException("Cannot resolve output directory.");
                samplesJsonlPath = Path.GetFullPath(normalized);
            }
            else
            {
                // 3) 输入是不存在的目录路径
                rootDir = Path.GetFullPath(normalized);
                samplesJsonlPath = Path.Combine(rootDir, "samples.jsonl");
            }
        }

        Directory.CreateDirectory(rootDir);
        var framesDir = Path.Combine(rootDir, "frames");
        Directory.CreateDirectory(framesDir);

        if (File.Exists(samplesJsonlPath))
        {
            appendMode = true;
            existingCount = File.ReadLines(samplesJsonlPath)
                .Count(line => !string.IsNullOrWhiteSpace(line));
        }

        _session = new HandEyeSessionInfo
        {
            RootDirectory = rootDir,
            SamplesJsonlPath = samplesJsonlPath,
            FramesDirectory = framesDir,
            IsAppendMode = appendMode,
            ExistingSampleCount = existingCount,
            NextIndex = existingCount
        };

        _logger.Info($"Hand-eye output configured. Root={rootDir}, Append={appendMode}, NextIndex={_session.NextIndex}");
    }

    public HandEyeSessionInfo GetSessionInfo() => _session;

    public async Task<HandEyeCaptureResult> CaptureSampleAsync(double[] currentTcpPose, byte[] imageBytes)
    {
        if (string.IsNullOrWhiteSpace(_session.RootDirectory))
            throw new InvalidOperationException("Capture session is not configured.");

        if (currentTcpPose == null || currentTcpPose.Length != 6)
            throw new ArgumentException("Current TCP pose must contain 6 values.", nameof(currentTcpPose));

        if (imageBytes == null || imageBytes.Length == 0)
            throw new ArgumentException("Image bytes are empty.", nameof(imageBytes));

        return await Task.Run(() =>
        {
            int index = _session.NextIndex;
            double timestamp = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() / 1000.0;

            string imagePath = Path.Combine(_session.FramesDirectory, $"{index:D4}.png");
            File.WriteAllBytes(imagePath, imageBytes);

            double[][] tBaseTool = Pose6DToMatrix(currentTcpPose);

            var sample = new HandEyeSample
            {
                Index = index,
                Timestamp = timestamp,
                ImagePath = imagePath,
                TBaseTool = tBaseTool
            };

            string jsonLine = JsonSerializer.Serialize(sample);
            File.AppendAllText(_session.SamplesJsonlPath, jsonLine + Environment.NewLine);

            _session.NextIndex++;

            _logger.Info($"Captured sample #{index}, image={imagePath}");

            return new HandEyeCaptureResult
            {
                Success = true,
                Message = "Sample captured successfully.",
                Index = index,
                ImagePath = imagePath,
                SamplesJsonlPath = _session.SamplesJsonlPath
            };
        });
    }

    /// <summary>
    /// 把 UR 风格 pose [x,y,z,rx,ry,rz] 转成 4x4，再转成 jagged array，便于 JSON 序列化。
    /// 这里的旋转按 Rodrigues 轴角向量处理。
    /// </summary>
    private static double[][] Pose6DToMatrix(double[] pose)
    {
        double x = pose[0], y = pose[1], z = pose[2];
        double rx = pose[3], ry = pose[4], rz = pose[5];

        // Rodrigues 轴角 -> 旋转矩阵
        double theta = Math.Sqrt(rx * rx + ry * ry + rz * rz);

        double[,] R = new double[3, 3];
        if (theta < 1e-12)
        {
            R[0, 0] = 1; R[0, 1] = 0; R[0, 2] = 0;
            R[1, 0] = 0; R[1, 1] = 1; R[1, 2] = 0;
            R[2, 0] = 0; R[2, 1] = 0; R[2, 2] = 1;
        }
        else
        {
            double kx = rx / theta;
            double ky = ry / theta;
            double kz = rz / theta;

            double c = Math.Cos(theta);
            double s = Math.Sin(theta);
            double v = 1 - c;

            R[0, 0] = kx * kx * v + c;
            R[0, 1] = kx * ky * v - kz * s;
            R[0, 2] = kx * kz * v + ky * s;

            R[1, 0] = ky * kx * v + kz * s;
            R[1, 1] = ky * ky * v + c;
            R[1, 2] = ky * kz * v - kx * s;

            R[2, 0] = kz * kx * v - ky * s;
            R[2, 1] = kz * ky * v + kx * s;
            R[2, 2] = kz * kz * v + c;
        }

        return
        [
            [R[0,0], R[0,1], R[0,2], x],
            [R[1,0], R[1,1], R[1,2], y],
            [R[2,0], R[2,1], R[2,2], z],
            [0.0,    0.0,    0.0,    1.0]
        ];
    }
}