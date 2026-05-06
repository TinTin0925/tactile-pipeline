using UR3Demo.Core;

namespace UR3Demo.Robot.Services;

/// <summary>
/// 脚本级说明：
/// 在开始采集前，检查输出目录是否安全。
/// 
/// 当前重点检查：
/// - output/frames 是否存在
/// - output/frames 是否非空
/// - output/samples.jsonl 是否存在
/// 
/// 默认策略：
/// 如果 frames 已存在且非空，则返回 Ok=false，
/// 由上层提醒用户，避免新旧任务数据混写。
/// </summary>
public class OutputSessionGuard : IOutputSessionGuard
{
    public OutputPathCheckResult Check(string outputPath)
    {
        if (string.IsNullOrWhiteSpace(outputPath))
            throw new ArgumentException("Output path cannot be empty.", nameof(outputPath));

        string rootDir;
        if (Directory.Exists(outputPath))
        {
            rootDir = outputPath;
        }
        else if (Path.GetExtension(outputPath).Equals(".jsonl", StringComparison.OrdinalIgnoreCase))
        {
            rootDir = Path.GetDirectoryName(Path.GetFullPath(outputPath))
                      ?? throw new InvalidOperationException("Cannot resolve output directory.");
        }
        else
        {
            rootDir = Path.GetFullPath(outputPath);
        }

        string framesDir = Path.Combine(rootDir, "frames");
        string samplesJsonl = Path.Combine(rootDir, "samples.jsonl");

        bool framesExists = Directory.Exists(framesDir);
        bool framesHasFiles = framesExists && Directory.EnumerateFiles(framesDir).Any();
        bool jsonlExists = File.Exists(samplesJsonl);

        var result = new OutputPathCheckResult
        {
            RootDirectory = rootDir,
            FramesDirectory = framesDir,
            SamplesJsonlPath = samplesJsonl,
            FramesFolderExists = framesExists,
            FramesFolderHasFiles = framesHasFiles,
            SamplesJsonlExists = jsonlExists,
            Ok = !framesHasFiles
        };

        result.Message = framesHasFiles
            ? $"Output frames folder is not empty: {framesDir}"
            : $"Output path check passed: {rootDir}";

        return result;
    }
}