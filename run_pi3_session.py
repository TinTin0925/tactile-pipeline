"""
Session 路径配置文件。
只需修改这里的路径，0.1 / 0.3 / step2 / step3 / step4 / step7 启动时会自动读取这些值。
本文件不执行任何操作，直接运行没有效果。
"""
from __future__ import annotations

from pathlib import Path

from vggt_step_common import DEFAULT_RUNS_DIR, count_image_files


# ===========================================================================
# 【路径配置区】每次换数据集只需改这里
# ===========================================================================

# 本次采集的原始图片帧目录（Step1 拍摄后保存图片的文件夹）
SOURCE_DIR = Path(r"E:\research\FYP\2026FYP\src\tactile_pipeline\session11\frames")

# 机器人末端位姿记录文件（.jsonl 格式，每行对应一张图片的 T_base_tool 4×4 矩阵）
# 通常由 Polaris / Step1 的 UI 同步生成，与图片帧一一对应
SAMPLES_JSONL = Path(r"E:\research\FYP\2026FYP\src\tactile_pipeline\session11\samples.jsonl")

# 手眼标定结果文件（.npz），包含相机相对于机器人末端的变换矩阵 T_tool_camera
# 由手眼标定流程（handeye calibration）生成，不同采集 session 共用同一份
HAND_EYE = Path(r"E:\research\FYP\2026FYP\src\tactile_pipeline\session11\handeye_compare\calib_handeye_compare.npz")

# 相机内参标定结果文件（.npz），包含内参矩阵 K 和畸变系数 dist
# 由相机标定流程（camera calibration）生成，镜头不换则共用同一份
CALIB = Path(r"E:\research\FYP\2026FYP\src\tactile_pipeline\session11\calib_result\camera_calibration.npz")

# ===========================================================================
# 【Pi3 环境配置区】第一次配好后一般不用改
# ===========================================================================

# Pi3 代码库的根目录
PI3_ROOT = Path(r"E:\research\FYP\5.4\Pi3")

# Pi3 使用的 Python 解释器（含 torch、pi3 依赖的虚拟环境）
PI3_PYTHON = Path(r"E:\research\FYP\vggt_env\Scripts\python.exe")

# 运行设备：'cuda'（GPU）或 'cpu'
DEVICE: str = "cuda"

# ===========================================================================
# 【采集参数】一般不需要改动
# ===========================================================================

# 使用的图片帧数；None 表示自动读取 SOURCE_DIR 内全部图片
NUM_FRAMES: int | None = None

# 从第几张图片开始（0 = 第一张，按文件名数字排序）
START_INDEX: int = 0

# 帧间隔（1 = 每帧都用；2 = 每隔一帧取一张，以此类推）
FRAME_STRIDE: int = 1

# 是否对图片做去畸变处理（推荐保持 True）
UNDISTORT: bool = True

# 去畸变 alpha 参数（0.0 = 裁掉黑边；1.0 = 保留所有像素含黑边）
UNDISTORT_ALPHA: float = 0.1

# ===========================================================================
# 【Pi3 重建参数】显存不足时可调 INTERVAL，其余保持默认
# ===========================================================================

# Pi3 推理帧间隔；None = 自动（≤20 帧用 1，否则用 2 节省显存）
INTERVAL: int | None = None

# 置信度阈值，低于此值的点会被过滤（越高点越少但越准）
CONF_MIN: float = 0.60

# 输出点云最大点数
MAX_POINTS: int = 500_000

# 深度过滤范围（单位：米），超出范围的点被丢弃
DEPTH_MIN: float = 0.05
DEPTH_MAX: float = 0.30

# 距离过滤上限（单位：米），距相机原点过远的点被丢弃
DISTANCE_MAX: float = 0.35

# 是否在重建后用原始图片给点云上色（生成 *_projected_rgb.ply）
PROJECT_RGB: bool = True

# ===========================================================================
# 【机器人连接配置】step7 实时触觉接触检测使用
# ===========================================================================

# UR 机器人 IP 地址（与上位机连接的 IP 一致）
# step7 会通过 RTDE 直连机器人读取实时 TCP 位姿，无需上位机转发
# 设为空字符串 "" 则退回到 --pose-json 文件轮询模式（离线测试用）
ROBOT_IP: str = "192.168.1.102"

# ===========================================================================
# 以下是供其他 step 脚本 import 使用的辅助函数，无需修改
# ===========================================================================


def session_output_path() -> Path:
    """返回本 session 最终输出的点云路径（供 step3 / step4 import 使用）。"""
    source_dir = SOURCE_DIR.resolve()
    frame_stride = max(1, FRAME_STRIDE)
    num_frames = NUM_FRAMES if NUM_FRAMES is not None else count_image_files(source_dir)
    parent_tag = source_dir.parent.name or "images"
    stride_tag = "" if frame_stride <= 1 else f"_s{frame_stride}"
    out_dir = DEFAULT_RUNS_DIR / f"pi3_{parent_tag}_{source_dir.name}_n{num_frames}{stride_tag}"
    interval = INTERVAL if INTERVAL is not None else (1 if num_frames <= 20 else 2)
    save_path = out_dir / f"{out_dir.name}_i{interval}_aligned.ply"
    if PROJECT_RGB:
        return save_path.with_name(f"{save_path.stem}_projected_rgb.ply")
    return save_path
