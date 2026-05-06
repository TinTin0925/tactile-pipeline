namespace UR3Demo.Core;

/// <summary>
/// 脚本级说明：
/// 在真正执行采集前，对输出目录做安全检查。
/// </summary>
public interface IOutputSessionGuard
{
    OutputPathCheckResult Check(string outputPath);
}