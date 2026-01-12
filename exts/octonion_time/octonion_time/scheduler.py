from .octonion import Octonion
from .delta_q import compute_delta_q
import numpy as np


class OctonionScheduler:
    """Temporal semantics scheduler based on octonion update"""

    def __init__(self):
        self.q = Octonion()

        # 语义层最大子步数（防止计算爆炸）（Maximum number of sub-steps in the semantic layer (to prevent computational explosion)
        self.max_substeps = 8
        self.omega_threshold = 5.0

    def on_physics_step(self, dt):
        omega_norm = self._read_angular_velocity_norm()
        control = self._read_control()

        # 根据动态强度决定语义子步数（Determine the number of semantic sub-steps based on dynamic intensity）
        substeps = self._compute_substeps(omega_norm)

        sub_dt = dt / substeps

        for _ in range(substeps):
            dq = compute_delta_q(
                sub_dt,
                control=control,
                omega_norm=omega_norm,
            )

            # 基本一致性护栏（Basic Consistency Guardrail）
            if abs(dq.norm() - 1.0) > 1e-3:
                dq.normalize()

            # 非交换语义更新（Non-commutative semantic update）
            self.q = self.q * dq

        # 全局稳定化（数值护栏，不是物理守恒）（Global stabilization (numerical barrier, not physical conservation)）
        self.q.normalize()

        if substeps > 1:
            print(
                f"[Octonion] Adaptive semantic sub-stepping: {substeps} steps"
            )

    def _compute_substeps(self, omega_norm):
        if omega_norm < self.omega_threshold:
            return 1

        # 简单线性映射（第一版）（Simple Linear Mapping (First Edition)）
        scale = min(
            int(np.ceil(omega_norm / self.omega_threshold)),
            self.max_substeps,
        )
        return max(1, scale)

    # ===== placeholder hooks =====

    def _read_control(self):
        return 0.0

    def _read_angular_velocity_norm(self):
        return 1.0
