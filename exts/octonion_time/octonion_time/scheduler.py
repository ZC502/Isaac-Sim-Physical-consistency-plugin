from .octonion import Octonion
from .delta_q import compute_delta_q
import numpy as np
import carb


class OctonionScheduler:
    """
    Temporal semantics scheduler based on octonion update.

    v0.4:
    - Pure observer
    - Drift defined ONLY by non-associativity (associator)
    - No control authority, no gain scheduling
    """

    def __init__(self):
        self.q = Octonion()

        # External physical signal
        self._external_omega = None

        # v0.4: execution order tag (+1 / -1)
        self._order_signal = 1.0

        # Last measured associator
        self._last_associator = np.zeros(8, dtype=float)

        self.max_substeps = 8
        self.omega_threshold = 5.0

    # ---------------------------------------------------------
    # Main update
    # ---------------------------------------------------------
    def on_physics_step(self, dt: float):
        omega_norm = self._read_angular_velocity_norm()
        control = self._read_control()

        substeps = self._compute_substeps(omega_norm)
        sub_dt = dt / substeps

        for _ in range(substeps):
            dq = compute_delta_q(
                sub_dt,
                control=control,
                omega_norm=omega_norm,
            )

            if abs(dq.norm() - 1.0) > 1e-3:
                dq.normalize()

            # Path-dependent comparison (core audit)
            q_forward = self.q * dq
            q_permuted = dq * self.q

            assoc = q_forward - q_permuted
            self._last_associator = assoc.i.copy()

            # Observer chooses canonical update (no control)
            self.q = q_forward

        self.q.normalize()

        if substeps > 1:
            carb.log_warn(
                f"[Octonion-v0.4] High dynamics: {substeps} substeps "
                f"(Omega={omega_norm:.2f}, OrderTag={self._order_signal:+.0f})"
            )

    # ---------------------------------------------------------
    # Diagnostic output
    # ---------------------------------------------------------
    def get_associator_magnitude(self) -> float:
        """
        Drift = ||associator||
        Captures order-dependence in numerical integration.
        """
        return float(np.linalg.norm(self._last_associator))

    # ---------------------------------------------------------
    # External bridges
    # ---------------------------------------------------------
    def set_external_omega(self, omega_norm: float):
        self._external_omega = float(omega_norm)

    def set_order_signal(self, signal: float):
        """
        signal ∈ {+1, -1}
        Used for audit alignment & logging only.
        """
        self._order_signal = float(signal)

    def _read_angular_velocity_norm(self) -> float:
        if self._external_omega is not None:
            return self._external_omega
        return 0.0

    # ---------------------------------------------------------
    # Substep logic
    # ---------------------------------------------------------
    def _compute_substeps(self, omega_norm: float) -> int:
        if omega_norm < self.omega_threshold:
            return 1

        scale = min(
            int(np.ceil(omega_norm / self.omega_threshold)),
            self.max_substeps,
        )
        return max(1, scale)

    # ---------------------------------------------------------
    # Placeholder control hook
    # ---------------------------------------------------------
    def _read_control(self):
        return 0.0
