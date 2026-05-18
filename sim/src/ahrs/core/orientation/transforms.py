"""
File Name: ./src/ahrs/core/orientation/transforms.py
Author: Beomjun Chung
Updated: 2026-05-18

Description:
  순수 변환 함수 모음 (static methods only)

    purpose:
        자세 표현(Quaternion, DCM, Euler) 간 상호 변환 및 기본 quaternion 연산.
        외부 의존 없는 순수 함수 집합 — 모든 레이어가 여기에 의존함.

    Notes:
        - Quaternion convention: scalar-last [x, y, z, w]
        - Identity: [0, 0, 0, 1]
        - Euler order: ZYX (yaw → pitch → roll), 항공 표준
        - 내부 단위: rad
        - v_world = R · v_body  (body → world)
        - gimbal lock: pitch = ±90° 근처 → euler 출력만, 필터 내부 사용 금지
"""

from __future__ import annotations

import numpy as np


class Transforms:
    # =========================================================================
    # Quaternion 기본 연산  (scalar-last [x, y, z, w])
    # =========================================================================

    @staticmethod
    def quat_mult(q: np.ndarray, r: np.ndarray) -> np.ndarray:
        """Hamilton product p = q ⊗ r, scalar-last [x, y, z, w]."""
        x0, y0, z0, w0 = q
        x1, y1, z1, w1 = r
        return np.array([
            w0*x1 + x0*w1 + y0*z1 - z0*y1,  # x
            w0*y1 - x0*z1 + y0*w1 + z0*x1,  # y
            w0*z1 + x0*y1 - y0*x1 + z0*w1,  # z
            w0*w1 - x0*x1 - y0*y1 - z0*z1,  # w
        ], dtype=float)

    @staticmethod
    def quat_conjugate(q: np.ndarray) -> np.ndarray:
        """켤레: q* = [-x, -y, -z, w]"""
        return np.array([-q[0], -q[1], -q[2], q[3]], dtype=float)

    @staticmethod
    def quat_normalize(q: np.ndarray) -> np.ndarray:
        """단위 쿼터니언 정규화. norm=0이면 항등 쿼터니언 반환."""
        n = np.linalg.norm(q)
        if n < 1e-10:
            return np.array([0.0, 0.0, 0.0, 1.0])
        return q / n

    # =========================================================================
    # DCM → 다른 표현
    # =========================================================================

    @staticmethod
    def dcm_to_euler(R: np.ndarray) -> np.ndarray:
        """
        DCM → Euler [roll, pitch, yaw] [rad], ZYX 순서.
        pitch = ±90° 근처에서 짐벌락 발생.
        """
        roll  = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], np.sqrt(R[0, 0]**2 + R[1, 0]**2))
        yaw   = np.arctan2(R[1, 0], R[0, 0])
        return np.array([roll, pitch, yaw], dtype=float)

    @staticmethod
    def dcm_to_quat(R: np.ndarray) -> np.ndarray:
        """
        DCM → 쿼터니언 [x, y, z, w], scalar-last.
        Shepperd 방법: 가장 큰 성분 먼저 계산해 수치 안정성 확보.
        """
        m00, m01, m02 = R[0]
        m10, m11, m12 = R[1]
        m20, m21, m22 = R[2]
        tr = m00 + m11 + m22

        if tr > 0:
            S = np.sqrt(tr + 1.0) * 2   # S = 4w
            w = 0.25 * S
            x = (m21 - m12) / S
            y = (m02 - m20) / S
            z = (m10 - m01) / S
        elif (m00 > m11) and (m00 > m22):
            S = np.sqrt(1.0 + m00 - m11 - m22) * 2   # S = 4x
            x = 0.25 * S
            w = (m21 - m12) / S
            y = (m01 + m10) / S
            z = (m02 + m20) / S
        elif m11 > m22:
            S = np.sqrt(1.0 + m11 - m00 - m22) * 2   # S = 4y
            y = 0.25 * S
            w = (m02 - m20) / S
            x = (m01 + m10) / S
            z = (m12 + m21) / S
        else:
            S = np.sqrt(1.0 + m22 - m00 - m11) * 2   # S = 4z
            z = 0.25 * S
            w = (m10 - m01) / S
            x = (m02 + m20) / S
            y = (m12 + m21) / S

        q = np.array([x, y, z, w], dtype=float)
        return q / np.linalg.norm(q)

    # =========================================================================
    # Euler → 다른 표현
    # =========================================================================

    @staticmethod
    def euler_to_dcm(roll: float, pitch: float, yaw: float) -> np.ndarray:
        """
        Euler [rad] → DCM, ZYX 순서 (R = Rz·Ry·Rx).
        v_world = R · v_body
        """
        cy, sy = np.cos(yaw),   np.sin(yaw)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cr, sr = np.cos(roll),  np.sin(roll)

        return np.array([
            [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
            [sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
            [  -sp,             cp*sr,             cp*cr ],
        ], dtype=float)

    @staticmethod
    def euler_to_quat(roll: float, pitch: float, yaw: float) -> np.ndarray:
        """
        Euler [rad] → 쿼터니언 [x, y, z, w], scalar-last, ZYX 순서.
        q = qz ⊗ qy ⊗ qx
        """
        cy, sy = np.cos(yaw   * 0.5), np.sin(yaw   * 0.5)
        cp, sp = np.cos(pitch * 0.5), np.sin(pitch * 0.5)
        cr, sr = np.cos(roll  * 0.5), np.sin(roll  * 0.5)

        x = sr*cp*cy - cr*sp*sy
        y = cr*sp*cy + sr*cp*sy
        z = cr*cp*sy - sr*sp*cy
        w = cr*cp*cy + sr*sp*sy

        q = np.array([x, y, z, w], dtype=float)
        return q / np.linalg.norm(q)

    # =========================================================================
    # Quaternion → 다른 표현
    # =========================================================================

    @staticmethod
    def quat_to_dcm(q: np.ndarray) -> np.ndarray:
        """
        쿼터니언 [x, y, z, w] → DCM.
        v_world = R · v_body
        """
        x, y, z, w = q
        return np.array([
            [1 - 2*(y**2 + z**2),     2*(x*y - z*w),     2*(x*z + y*w)],
            [    2*(x*y + z*w),    1 - 2*(x**2 + z**2),   2*(y*z - x*w)],
            [    2*(x*z - y*w),        2*(y*z + x*w),  1 - 2*(x**2 + y**2)],
        ], dtype=float)

    @staticmethod
    def quat_to_euler(q: np.ndarray) -> np.ndarray:
        """
        쿼터니언 [x, y, z, w] → Euler [roll, pitch, yaw] [rad], ZYX.
        출력 전용. 필터 내부 사용 금지.
        """
        x, y, z, w = q
        roll  = np.arctan2(2*(w*x + y*z), 1 - 2*(x**2 + y**2))
        pitch = np.arcsin(np.clip(2*(w*y - z*x), -1.0, 1.0))
        yaw   = np.arctan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2))
        return np.array([roll, pitch, yaw], dtype=float)

    # =========================================================================
    # 오차 계산
    # =========================================================================

    @staticmethod
    def quat_error(q_est: np.ndarray, q_true: np.ndarray) -> np.ndarray:
        """
        오차 쿼터니언: q_err = q_true* ⊗ q_est.
        벡터 부분 [x,y,z]의 2배 ≈ 소각도 오차 벡터 [rad].
        """
        return Transforms.quat_mult(Transforms.quat_conjugate(q_true), q_est)

    @staticmethod
    def geodesic_error(q_est: np.ndarray, q_true: np.ndarray) -> float:
        """
        두 쿼터니언 사이의 각도 오차 [rad].
        d = 2·arccos(|q_err_w|)  (scalar component = index 3)
        """
        q_err = Transforms.quat_error(q_est, q_true)
        w = np.clip(np.abs(q_err[3]), 0.0, 1.0)
        return float(2.0 * np.arccos(w))
