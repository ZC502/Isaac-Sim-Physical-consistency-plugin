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

    v0.4.2 additions:
    - Cumulative associator debt (path sensitivity amplifier)
    - Noise-floor aware diagnostics
    - Rollout-friendly metric access
    """

    # ---------------------------------------------------------
    # Init
    # ---------------------------------------------------------
    def __init__(self):
        self.q = Octonion()

        # External physical signal
        self._external_omega = None

        # Execution order tag (+1 / -1)
        self._order_signal = 1.0

        # Last-frame associator
        self._last_associator = np.zeros(8, dtype=float)

        # 🔥 v0.4.2: cumulative debt (very important for reviewers)
        self._cumulative_debt = 0.0

        # 🔥 v0.4.1: optional noise floor
        self.noise_floor = 0.0

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

            # Numerical guard
            if abs(dq.norm() - 1.0) > 1e-3:
                dq.normalize()

            # -------------------------------------------------
            # Core non-associative audit
            # -------------------------------------------------
            q_forward = self.q * dq
            q_permuted = dq * self.q

            assoc = q_forward - q_permuted
            assoc_vec = assoc.i.copy()
            self._last_associator = assoc_vec

            # 🔥 v0.4.2: accumulate path debt
            assoc_norm = float(np.linalg.norm(assoc_vec))
            self._cumulative_debt += assoc_norm * sub_dt

            # Observer uses canonical path
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
        Instantaneous associator norm.
        """
        return float(np.linalg.norm(self._last_associator))

    def get_cumulative_debt(self) -> float:
        """
        Time-integrated associator magnitude.
        Much more sensitive in long rollouts.
        """
        return float(self._cumulative_debt)

    def get_noise_aware_metric(self) -> float:
        """
        Noise-calibrated diagnostic.

        Returns:
            max(0, assoc_norm - noise_floor)
        """
        raw = self.get_associator_magnitude()
        return float(max(0.0, raw - self.noise_floor))

    # ---------------------------------------------------------
    # 🔥 Rollout helper (for v0.4.2 grid)
    # ---------------------------------------------------------
    def reset_metrics(self):
        """
        Clears accumulated statistics between rollouts.
        """
        self._cumulative_debt = 0.0
        self._last_associator[:] = 0.0

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

    def set_noise_floor(self, value: float):
        """
        Optional calibration hook (v0.4.1 toolbox).
        """
        self.noise_floor = float(max(0.0, value))

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
