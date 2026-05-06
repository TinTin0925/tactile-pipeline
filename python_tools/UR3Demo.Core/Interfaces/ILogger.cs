namespace UR3Demo.Core;

/// <summary>
/// 职责：统一日志接口。
/// 当前先支持文件日志；后续可以扩展到 UI 日志面板、控制台、网络日志等。
/// </summary>
public interface ILogger
{
    void Info(string message);
    void Warn(string message);
    void Error(string message, Exception? ex = null);
}