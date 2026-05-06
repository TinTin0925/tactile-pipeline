using UR3Demo.Core;

namespace UR3Demo.Robot.Logging;

/// <summary>
/// 职责：最小文件日志实现。
/// 当前只负责把日志写入本地文件，便于 demo 调试。
/// 后续如果需要，也可以扩展为同时输出到 UI 面板或控制台。
/// </summary>
public class FileLogger : ILogger
{
    private readonly string _logPath;
    private readonly object _lock = new();

    public FileLogger(string? logPath = null)
    {
        _logPath = logPath ?? Path.Combine(AppContext.BaseDirectory, "ur3demo.log");
    }

    public void Info(string message) => Write("INFO", message);

    public void Warn(string message) => Write("WARN", message);

    public void Error(string message, Exception? ex = null)
    {
        var fullMessage = ex == null
            ? message
            : $"{message}{Environment.NewLine}{ex}";
        Write("ERROR", fullMessage);
    }

    private void Write(string level, string message)
    {
        lock (_lock)
        {
            File.AppendAllText(
                _logPath,
                $"[{DateTime.Now:yyyy-MM-dd HH:mm:ss.fff}] [{level}] {message}{Environment.NewLine}");
        }
    }
}