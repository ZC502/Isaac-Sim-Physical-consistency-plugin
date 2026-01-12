from .octonion import Octonion
from .delta_q import compute_delta_q

class OctonionScheduler:

    def __init__(self):
        self.q = Octonion()

    def on_physics_step(self, dt):
        # 从 PhysX / articulation 读角速度(Read angular velocity)
        omega_norm = self._read_angular_velocity_norm()

        # 生成 Δq(Generate Δq)
        dq = compute_delta_q(dt, control=self._read_control(), omega_norm=omega_norm)

        # 非交换更新(Non-commutative update）
        self.q = self.q * dq
        self.q.normalize()

        # 自适应 sub-stepping（关键）（Adaptive sub-stepping (key)）
        if omega_norm > 5.0:
            self._request_additional_substep()

    def _read_control(self):
        return 0.0

    def _read_angular_velocity_norm(self):
        return 1.0

    def _request_additional_substep(self):
        # 第一版：只打印（First version: only print）
        print("[Octonion] High compute density → local refinement")
