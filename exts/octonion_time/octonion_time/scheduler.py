from .octonion import Octonion
from .delta_q import compute_delta_q
import numpy as np
import carb  


class OctonionScheduler:
    """
    Temporal semantics scheduler based on octonion update.

    v0.4:
    - Acts as a pure observer
    - Drift is defined ONLY by non-associativity (associator)
    - No control authority, no hidden gain scheduling
    """

    def __init__(self):
        self.q = Octonion()

        # External physical signal (fed by extension.py)
        self._external_omega = None

        # v0.4: execution order signal (+1 or -1)
        self._order_signal = 1.0

        # Last measured associator (for diagnostics)
        self._last_associator = np.zeros(8, dtype=float)

        # Maximum number of sub-steps
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

            # -------------------------------------------------
            # v0.4 Core: non-associative comparison
            # -------------------------------------------------

            # Path 1: standard update
            q_forward = self.q * dq

            # Path 2: permuted update (same energy, different order)
            q_permuted = dq * self.q

            # Associator: order-dependent discrepancy
            assoc = q_forward - q_permuted
            self._last_associator = assoc.i.copy()

            # Choose canonical update (observer does NOT control)
            self.q = q_forward

        self.q.normalize()

        if substeps > 1:
            carb.log_warn(
                f"[Octonion-v0.4] High dynamics: {substeps} substeps "
                f"(Omega={omega_norm:.2f})"
            )

    # ---------------------------------------------------------
    # Diagnostic outputs (Observer ONLY)
    # ---------------------------------------------------------
    def get_associator_magnitude(self) -> float:
        """
        v0.4:
        Drift is defined as the magnitude of the associator.
        This captures order-dependence in numerical integration.
        """
        return float(np.linalg.norm(self._last_associator))

    # ---------------------------------------------------------
    # External signal bridges
    # ---------------------------------------------------------
    def set_external_omega(self, omega_norm: float):
        self._external_omega = float(omega_norm)

    def set_order_signal(self, signal: float):
        """
        signal ∈ {+1, -1}
        Encodes execution order without modifying physics.
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
    # Placeholder control hook (unused)
    # ---------------------------------------------------------
    def _read_control(self):
        return 0.0
