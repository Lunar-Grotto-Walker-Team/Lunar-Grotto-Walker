"""3 维静态卡尔曼滤波器。

state x = [px, py, pz] (odom 系,米)
A = I3    (静态目标 x_pred = x)
Q = sigma_p^2 * I3    (per-tick process noise)
H = I3    (直接观测位置)
R = sigma_obs^2 * I3 (observation noise)

为什么 dt = 1 / tick:静态模型 A = I 下,x_pred = x 与 dt 无关;
P_pred = P + Q,Q 必须是"每 tick 增量"才稳定。
"""
from dataclasses import dataclass

import numpy as np


@dataclass
class KalmanState3D:
    """3 维 KF 状态。"""
    x: np.ndarray  # (3,) 位置向量
    P: np.ndarray  # (3, 3) 协方差


class KalmanFilter3D:
    """3 维静态 KF,3×3 矩阵,纯 NumPy。

    Args:
        sigma_p: per-tick 过程噪声标准差 (m/tick),典型 0.05。
        sigma_obs: 观测噪声标准差 (m),典型 0.03(D435i @ ~3m)。
    """

    def __init__(self, sigma_p: float, sigma_obs: float):
        if sigma_p <= 0 or sigma_obs <= 0:
            raise ValueError("sigma_p 和 sigma_obs 必须 > 0")
        self._Q = (sigma_p ** 2) * np.eye(3)
        self._R = (sigma_obs ** 2) * np.eye(3)

    def predict(self, state: KalmanState3D) -> KalmanState3D:
        """静态 x_pred = x;P_pred = P + Q。

        Returns:
          新的 KalmanState(不修改入参)。
        """
        return KalmanState3D(x=state.x.copy(), P=state.P + self._Q)

    def update(self, state: KalmanState3D, z: np.ndarray) -> KalmanState3D:
        """KF update,对角 P/R 简化代数。

        由于本实现里 P 和 R 始终是对角矩阵(`sigma^2 * np.eye(3)`),
        把 K 显式算成 (3,) 的对角向量,保证 `new_x` 满足 ``np.ndarray (3,)``
        的 ``KalmanState3D.x`` 形状契约;同时 ``new_P`` 重新包成 (3,3) 对角矩阵
        以匹配 ``KalmanState3D.P`` 的 (3,3) 契约。
        直觉上:K_diag[i] = P[i,i] / (P[i,i] + R[i,i]) ∈ [0, 1],即每维独立的
        scalar KF gain。

        Args:
          state: 当前状态
          z: (3,) 观测向量

        Returns:
          更新后的新 KalmanState。
        """
        if z.shape != (3,):
            raise ValueError(f"z must be shape (3,), got {z.shape}")

        y = z - state.x
        P_diag = np.diag(state.P)              # (3,)
        R_diag = np.diag(self._R)              # (3,)
        S_diag = P_diag + R_diag               # (3,)
        K_diag = P_diag / S_diag               # (3,) scalar KF gain
        new_x = state.x + K_diag * y           # (3,)  形状契约保留
        new_P_diag = (1.0 - K_diag) * P_diag   # (3,)
        new_P = np.diag(new_P_diag)            # (3, 3) 对角
        return KalmanState3D(x=new_x, P=new_P)

    def maha_distance(self, state: KalmanState3D, z: np.ndarray) -> float:
        """3 维 Mahalanobis 距离(对角 S 简化)。

        用于关联门控,数值 = sqrt((z-x)^T S^{-1} (z-x))。
        """
        if z.shape != (3,):
            raise ValueError(f"z must be shape (3,), got {z.shape}")
        y = z - state.x
        S_diag = np.diag(state.P + self._R)  # (3,)
        # 避免除零
        if np.any(S_diag <= 0):
            return float("inf")
        return float(np.sqrt(np.sum(y ** 2 / S_diag)))
