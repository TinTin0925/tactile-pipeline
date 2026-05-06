import numpy as np
from typing import Tuple


def split_rt(T: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    职责：把 4x4 齐次矩阵拆成 (R, t)。
    返回:
      R: 3x3
      t: 3x1
    """
    T = np.asarray(T, dtype=np.float64)
    if T.shape != (4, 4):
        raise ValueError(f"T must be 4x4, got {T.shape}")
    R = T[:3, :3].copy()
    t = T[:3, 3:4].copy()
    return R, t


def make_T(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """
    职责：由 (R, t) 组装 4x4 齐次矩阵。
    """
    R = np.asarray(R, dtype=np.float64)
    t = np.asarray(t, dtype=np.float64).reshape(3, 1)

    if R.shape != (3, 3):
        raise ValueError(f"R must be 3x3, got {R.shape}")

    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3:4] = t
    return T


def inv_T(T: np.ndarray) -> np.ndarray:
    """
    职责：SE(3) 逆变换。
    """
    T = np.asarray(T, dtype=np.float64)
    if T.shape != (4, 4):
        raise ValueError(f"T must be 4x4, got {T.shape}")

    R = T[:3, :3]
    t = T[:3, 3:4]

    Tinv = np.eye(4, dtype=np.float64)
    Tinv[:3, :3] = R.T
    Tinv[:3, 3:4] = -R.T @ t
    return Tinv


def pose6d_to_T(pose6d) -> np.ndarray:
    """
    职责：把 UR 风格 pose [x,y,z,rx,ry,rz] 转成 4x4。
    旋转部分按 Rodrigues 轴角向量解释。
    """
    import cv2

    pose6d = np.asarray(pose6d, dtype=np.float64).reshape(-1)
    if pose6d.size != 6:
        raise ValueError("pose6d must contain 6 values")

    x, y, z, rx, ry, rz = pose6d
    rvec = np.array([[rx], [ry], [rz]], dtype=np.float64)
    R, _ = cv2.Rodrigues(rvec)
    t = np.array([[x], [y], [z]], dtype=np.float64)
    return make_T(R, t)


def T_to_pose6d(T: np.ndarray) -> np.ndarray:
    """
    职责：把 4x4 转成 UR 风格 pose [x,y,z,rx,ry,rz]。
    """
    import cv2

    R, t = split_rt(T)
    rvec, _ = cv2.Rodrigues(R)
    return np.array(
        [t[0, 0], t[1, 0], t[2, 0], rvec[0, 0], rvec[1, 0], rvec[2, 0]],
        dtype=np.float64,
    )