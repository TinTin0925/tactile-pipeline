from __future__ import annotations

import argparse
import json
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Iterable

import numpy as np
import open3d as o3d

from vggt_step_common import DEFAULT_RUNS_DIR
import run_pi3_session as _cfg


DEFAULT_CAMERA_TO_TACTILE = (
    DEFAULT_RUNS_DIR
    / "session_20260504_170543"
    / "camera_to_tactile_result_square_1p5mm"
    / "camera_to_tactile_calibration.json"
)

DEFAULT_TACTILE_STL = Path(r"E:\research\FYP\Tactile.STL")


def newest_file(patterns: Iterable[str]) -> Path | None:
    candidates: list[Path] = []
    for pattern in patterns:
        candidates.extend(DEFAULT_RUNS_DIR.glob(pattern))
    candidates = [p for p in candidates if p.is_file()]
    if not candidates:
        return None
    return max(candidates, key=lambda p: p.stat().st_mtime)


def default_cloud_path() -> Path:
    found = newest_file(
        [
            "pi3_*/*projected_rgb.ply",
            "pi3_*/*aligned.ply",
            "vggt_viser_export_liver_data*_aligned/points.ply",
            "refined*liver*/points_refined.ply",
            "vggt_viser_export_*_aligned/points.ply",
        ]
    )
    if found is None:
        raise FileNotFoundError("No aligned/refined point cloud found under camera/runs.")
    return found


def default_pose_json() -> Path:
    return _cfg.SOURCE_DIR.parent / "current_T_base_tool.json"


def ensure_transform(value: Any, name: str) -> np.ndarray:
    T = np.asarray(value, dtype=np.float64)
    if T.shape != (4, 4):
        raise ValueError(f"{name} must be 4x4, got {T.shape}")
    if not np.all(np.isfinite(T)):
        raise ValueError(f"{name} contains non-finite values")
    if not np.allclose(T[3], [0.0, 0.0, 0.0, 1.0], atol=1e-8):
        raise ValueError(f"{name} last row must be [0, 0, 0, 1]")
    return T


def axis_angle_to_matrix(rotvec: np.ndarray) -> np.ndarray:
    theta = float(np.linalg.norm(rotvec))
    if theta < 1e-12:
        return np.eye(3)
    axis = rotvec / theta
    x, y, z = axis
    K = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]], dtype=np.float64)
    return np.eye(3) + np.sin(theta) * K + (1.0 - np.cos(theta)) * (K @ K)


def pose6d_to_matrix(pose: Iterable[float]) -> np.ndarray:
    arr = np.asarray(list(pose), dtype=np.float64)
    if arr.shape != (6,):
        raise ValueError(f"UR pose must have 6 values, got {arr.shape}")
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = axis_angle_to_matrix(arr[3:6])
    T[:3, 3] = arr[:3]
    return T


def load_transform_from_json(path: Path, keys: Iterable[str]) -> np.ndarray:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if isinstance(payload, list):
        return ensure_transform(payload, str(path))
    for key in keys:
        if key in payload:
            return ensure_transform(payload[key], key)
    raise KeyError(f"No transform key {list(keys)} found in {path}")


def load_transform_from_npz(path: Path, keys: Iterable[str]) -> np.ndarray:
    data = np.load(str(path), allow_pickle=True)
    for key in keys:
        if key in data:
            return ensure_transform(data[key], key)
    raise KeyError(f"No transform key {list(keys)} found in {path}. Available: {list(data.keys())}")


def load_hand_eye(path: Path, key: str) -> np.ndarray:
    if path.suffix.lower() == ".npz":
        return load_transform_from_npz(path, [key])
    return load_transform_from_json(path, [key])


def load_camera_to_tactile(path: Path, translation_scale: float) -> np.ndarray:
    if path.suffix.lower() == ".npz":
        data = np.load(str(path), allow_pickle=True)
        if "T_camera_tactile_m" in data:
            return ensure_transform(data["T_camera_tactile_m"], "T_camera_tactile_m")
        if "T_camera_tactile" in data:
            T = ensure_transform(data["T_camera_tactile"], "T_camera_tactile")
        else:
            T = load_transform_from_npz(path, ["T_camera_tactile_mm"])
            translation_scale = 0.001
    else:
        T = load_transform_from_json(path, ["T_camera_tactile"])

    T = T.copy()
    T[:3, 3] *= translation_scale
    return T


def load_tool_tactile(path: Path) -> np.ndarray:
    if path.suffix.lower() == ".npz":
        return load_transform_from_npz(path, ["T_tool_tactile_m", "T_tool_tactile"])
    return load_transform_from_json(path, ["T_tool_tactile_m", "T_tool_tactile"])


def load_live_pose(path: Path) -> np.ndarray:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if isinstance(payload, list):
        return ensure_transform(payload, str(path))

    matrix_keys = ["T_base_tool", "TBaseTool", "T_base_tcp", "TBaseTcp"]
    for key in matrix_keys:
        if key in payload:
            return ensure_transform(payload[key], key)

    pose_keys = ["ActualTcpPose", "actualTcpPose", "tcpPose", "pose", "ur_pose"]
    for key in pose_keys:
        if key in payload:
            return pose6d_to_matrix(payload[key])

    raise KeyError(f"No T_base_tool/TBaseTool or UR 6D pose found in {path}")


def iter_replay_poses(jsonl_path: Path) -> Iterable[tuple[float, np.ndarray]]:
    with jsonl_path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            row = json.loads(line)
            if "TBaseTool" in row:
                yield float(row.get("Timestamp", time.time())), ensure_transform(row["TBaseTool"], "TBaseTool")
            elif "T_base_tool" in row:
                yield float(row.get("timestamp", time.time())), ensure_transform(row["T_base_tool"], "T_base_tool")


def query_contact(
    kdtree: o3d.geometry.KDTreeFlann,
    points: np.ndarray,
    p_tip: np.ndarray,
    k: int,
) -> tuple[np.ndarray, np.ndarray]:
    _, idx, d2 = kdtree.search_knn_vector_3d(p_tip.astype(np.float64), k)
    distances = np.sqrt(np.asarray(d2, dtype=np.float64))
    indices = np.asarray(idx, dtype=np.int64)
    return distances, indices


def write_event(log_path: Path, payload: dict[str, Any]) -> None:
    with log_path.open("a", encoding="utf-8") as f:
        f.write(json.dumps(payload, ensure_ascii=False) + "\n")


def make_sphere(radius: float, color: list[float]) -> o3d.geometry.TriangleMesh:
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
    sphere.compute_vertex_normals()
    sphere.paint_uniform_color(color)
    return sphere


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Step 5: real-time tactile contact detection on aligned point cloud.")
    parser.add_argument("--cloud", type=Path, default=None, help="Aligned/refined point cloud in base coordinates.")
    parser.add_argument("--pose-json", type=Path, default=default_pose_json(),
                        help="Live pose file. Accepts T_base_tool/TBaseTool 4x4 or ActualTcpPose/tcpPose [x,y,z,rx,ry,rz].")
    parser.add_argument("--replay-jsonl", type=Path, default=None,
                        help="Replay TBaseTool from samples.jsonl instead of watching a live pose file.")
    parser.add_argument("--tool-tactile", type=Path, default=None,
                        help="Optional pre-composed T_tool_tactile calibration npz/json.")
    parser.add_argument("--hand-eye", type=Path, default=_cfg.HAND_EYE)
    parser.add_argument("--hand-eye-key", default="T_tool_camera",
                        help="Key in hand-eye npz for T_tool_camera.")
    parser.add_argument("--camera-to-tactile", type=Path, default=DEFAULT_CAMERA_TO_TACTILE)
    parser.add_argument("--camera-to-tactile-translation-scale", type=float, default=0.001,
                        help="Use 0.001 when T_camera_tactile translation is in mm; use 1.0 if already meters.")
    parser.add_argument("--tactile-stl", type=Path, default=DEFAULT_TACTILE_STL,
                        help="STL model of the tactile head in tactile frame coordinates.")
    parser.add_argument("--tactile-stl-scale", type=float, default=0.001,
                        help="Scale factor for tactile STL (0.001 if STL is in mm, 1.0 if meters).")
    parser.add_argument("--tactile-stl-rot-deg", nargs=3, type=float, default=[90.0, 0, 0.0],
                        metavar=("RX", "RY", "RZ"),
                        help="Pre-rotation applied to STL in tactile frame (XYZ Euler, degrees). "
                             "Use to align the cylinder axis with the optical axis.")
    parser.add_argument("--tip-offset-tactile", nargs=3, type=float, default=[0.0, 0.0, 0.0],
                        metavar=("X", "Y", "Z"), help="Tip offset in tactile frame, meters.")
    parser.add_argument("--threshold", type=float, default=0.003, help="Contact threshold in meters.")
    parser.add_argument("--k", type=int, default=5)
    parser.add_argument("--hz", type=float, default=30.0)
    parser.add_argument("--point-size", type=float, default=2.0)
    parser.add_argument("--sphere-radius", type=float, default=0.004)
    parser.add_argument("--robot-ip", type=str, default=_cfg.ROBOT_IP,
                        help="Robot IP for direct RTDE live pose. Empty string = use --pose-json file polling.")
    parser.add_argument("--rtde-frequency", type=float, default=10.0,
                        help="RTDE read frequency in Hz (upper computer default is 10 Hz).")
    parser.add_argument("--no-visualize", action="store_true")
    parser.add_argument("--one-shot", action="store_true")
    parser.add_argument("--log", type=Path, default=None)
    parser.add_argument("--log-all", action="store_true", help="Log every pose, not only contact frames.")
    return parser.parse_args()


def euler_xyz_deg_to_R(rx_deg: float, ry_deg: float, rz_deg: float) -> np.ndarray:
    rx, ry, rz = np.deg2rad([rx_deg, ry_deg, rz_deg])
    Rx = np.array([[1, 0, 0], [0, np.cos(rx), -np.sin(rx)], [0, np.sin(rx), np.cos(rx)]])
    Ry = np.array([[np.cos(ry), 0, np.sin(ry)], [0, 1, 0], [-np.sin(ry), 0, np.cos(ry)]])
    Rz = np.array([[np.cos(rz), -np.sin(rz), 0], [np.sin(rz), np.cos(rz), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def load_tactile_stl(
    stl_path: Path,
    scale: float,
    rot_deg: list[float] | None = None,
) -> o3d.geometry.TriangleMesh:
    mesh = o3d.io.read_triangle_mesh(str(stl_path))
    if mesh.is_empty():
        raise RuntimeError(f"Tactile STL is empty or unreadable: {stl_path}")
    if abs(scale - 1.0) > 1e-9:
        mesh.scale(scale, center=np.zeros(3))
    if rot_deg is not None and any(abs(v) > 1e-9 for v in rot_deg):
        R = euler_xyz_deg_to_R(*rot_deg)
        T_pre = np.eye(4)
        T_pre[:3, :3] = R
        mesh.transform(T_pre)
    mesh.compute_vertex_normals()
    mesh.paint_uniform_color([0.9, 0.45, 0.1])
    return mesh


def main() -> None:
    args = parse_args()
    cloud_path = args.cloud.resolve() if args.cloud else default_cloud_path()
    if not cloud_path.is_file():
        raise FileNotFoundError(f"Point cloud not found: {cloud_path}")

    pcd = o3d.io.read_point_cloud(str(cloud_path))
    points = np.asarray(pcd.points, dtype=np.float64)
    if len(points) < 1:
        raise ValueError(f"Point cloud has no points: {cloud_path}")
    kdtree = o3d.geometry.KDTreeFlann(pcd)

    if args.tool_tactile:
        T_tool_tactile = load_tool_tactile(args.tool_tactile.resolve())
        tool_tactile_source = str(args.tool_tactile.resolve())
    else:
        T_tool_camera = load_hand_eye(args.hand_eye.resolve(), args.hand_eye_key)
        T_camera_tactile = load_camera_to_tactile(
            args.camera_to_tactile.resolve(),
            args.camera_to_tactile_translation_scale,
        )
        # T_base_tactile = T_base_tool @ T_tool_camera @ T_camera_tactile
        T_tool_tactile = T_tool_camera @ T_camera_tactile
        tool_tactile_source = (
            f"{args.hand_eye.resolve()}::{args.hand_eye_key}"
            f" @ {args.camera_to_tactile.resolve()}"
        )

    tip_offset = np.array([*args.tip_offset_tactile, 1.0], dtype=np.float64)
    loop_dt = 1.0 / max(args.hz, 1e-6)
    log_path = args.log
    if log_path is None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path = DEFAULT_RUNS_DIR / "step7_tactile_contact" / f"contact_log_{stamp}.jsonl"
    log_path.parent.mkdir(parents=True, exist_ok=True)

    # ── RTDE 实时连接（直接读 UR 机器人，无需上位机转发）────────────────────
    rtde_r = None
    use_rtde = bool(args.robot_ip) and not args.replay_jsonl
    if use_rtde:
        try:
            import rtde_receive  # pip install ur-rtde
            rtde_r = rtde_receive.RTDEReceiveInterface(args.robot_ip, args.rtde_frequency)
            print(f"[Step7] RTDE connected: {args.robot_ip} @ {args.rtde_frequency} Hz")
        except ImportError:
            print("[Step7] WARNING: ur-rtde not installed (pip install ur-rtde).")
            print("[Step7] Falling back to --pose-json file polling.")
            use_rtde = False
        except Exception as e:
            print(f"[Step7] WARNING: RTDE connect failed ({e}), falling back to --pose-json.")
            use_rtde = False

    print("[Step7] cloud:", cloud_path)
    print("[Step7] points:", len(points))
    print("[Step7] T_tool_tactile source:", tool_tactile_source)
    print("[Step7] threshold:", args.threshold, "m")
    print("[Step7] log:", log_path)
    if args.replay_jsonl:
        print("[Step7] replay:", args.replay_jsonl.resolve())
    elif use_rtde:
        print(f"[Step7] live pose: RTDE @ {args.robot_ip}")
    else:
        print("[Step7] live pose json:", args.pose_json.resolve())

    # ── 初始化可视化 ───────────────────────────────────────────────────────
    vis = None
    tactile_mesh: o3d.geometry.TriangleMesh | None = None
    near_sphere: o3d.geometry.TriangleMesh | None = None
    T_tactile_prev = np.eye(4, dtype=np.float64)

    if not args.no_visualize:
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name="Step7 Tactile Contact", width=1280, height=820)
        vis.add_geometry(pcd)
        render = vis.get_render_option()
        render.point_size = args.point_size
        render.background_color = np.array([0.05, 0.05, 0.05])

        # 加载触觉头 STL 模型
        stl_path = args.tactile_stl.resolve() if args.tactile_stl else None
        if stl_path and stl_path.exists():
            tactile_mesh = load_tactile_stl(stl_path, args.tactile_stl_scale, args.tactile_stl_rot_deg)
            vis.add_geometry(tactile_mesh)
            print("[Step7] tactile STL:", stl_path)
            print("[Step7] STL pre-rotation (XYZ deg):", args.tactile_stl_rot_deg)
        else:
            # 无 STL 时退回到球形标记
            tactile_mesh = make_sphere(args.sphere_radius, [1.0, 0.6, 0.0])
            vis.add_geometry(tactile_mesh)
            if stl_path:
                print(f"[Step7] warning: STL not found {stl_path}, using sphere")

        near_sphere = make_sphere(args.sphere_radius * 0.6, [0.1, 0.45, 1.0])
        vis.add_geometry(near_sphere)

    last_mtime = None
    last_near = np.zeros(3)
    last_state: bool | None = None

    replay_iter = iter_replay_poses(args.replay_jsonl.resolve()) if args.replay_jsonl else None

    try:
        while True:
            if replay_iter is not None:
                try:
                    timestamp, T_base_tool = next(replay_iter)
                except StopIteration:
                    break
            elif rtde_r is not None:
                # 直接从 UR 机器人 RTDE 读取实时 TCP 位姿 [x,y,z,rx,ry,rz]（单位：米/弧度）
                tcp_pose = rtde_r.getActualTCPPose()
                T_base_tool = pose6d_to_matrix(tcp_pose)
                timestamp = time.time()
            else:
                pose_path = args.pose_json.resolve()
                if not pose_path.is_file():
                    print(f"[Step7] waiting for pose file: {pose_path}")
                    time.sleep(max(0.5, loop_dt))
                    continue
                mtime = pose_path.stat().st_mtime
                if mtime == last_mtime and not args.one_shot:
                    if vis:
                        vis.poll_events()
                        vis.update_renderer()
                    time.sleep(loop_dt)
                    continue
                last_mtime = mtime
                timestamp = time.time()
                T_base_tool = load_live_pose(pose_path)

            # 计算触觉头在 base 坐标系中的位姿
            T_base_tactile = T_base_tool @ T_tool_tactile
            p_tip = (T_base_tactile @ tip_offset)[:3]

            distances, indices = query_contact(kdtree, points, p_tip, max(1, args.k))
            nearest = points[indices[0]]
            in_contact = bool(distances[0] <= args.threshold)

            if last_state is None or in_contact != last_state or args.one_shot:
                state = "CONTACT" if in_contact else "free"
                print(f"[Step7] {state}: dist={distances[0]*1000:.2f} mm  tip={np.round(p_tip*1000,1)} mm")
                last_state = in_contact

            if args.log_all or in_contact:
                write_event(log_path, {
                    "timestamp": timestamp,
                    "in_contact": in_contact,
                    "distance_m": float(distances[0]),
                    "threshold_m": float(args.threshold),
                    "p_tip_base_m": p_tip.tolist(),
                    "nearest_cloud_point_m": nearest.tolist(),
                    "nearest_indices": indices.tolist(),
                    "nearest_distances_m": distances.tolist(),
                    "T_base_tool": T_base_tool.tolist(),
                    "T_base_tactile": T_base_tactile.tolist(),
                })

            if vis and tactile_mesh is not None:
                # 用增量变换更新 STL 位姿（避免累计浮点误差）
                T_delta = T_base_tactile @ np.linalg.inv(T_tactile_prev)
                tactile_mesh.transform(T_delta)
                T_tactile_prev = T_base_tactile.copy()

                if in_contact:
                    tactile_mesh.paint_uniform_color([1.0, 0.1, 0.1])
                else:
                    tactile_mesh.paint_uniform_color([0.9, 0.45, 0.1])

                near_sphere.translate(nearest - last_near, relative=True)
                last_near = nearest.copy()

                vis.update_geometry(tactile_mesh)
                vis.update_geometry(near_sphere)
                vis.poll_events()
                vis.update_renderer()

            if args.one_shot:
                break
            time.sleep(loop_dt)

    except KeyboardInterrupt:
        print("\n[Step7] stopped")
    finally:
        if rtde_r is not None:
            rtde_r.disconnect()
        if vis:
            vis.destroy_window()


if __name__ == "__main__":
    main()
