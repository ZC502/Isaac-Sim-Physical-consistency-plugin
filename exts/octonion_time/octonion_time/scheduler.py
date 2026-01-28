# get_drift_magnitude() is the ONLY signal exposed to the PhysX feedback loop.
# It defines when and how the solver intervention is triggered.

from .octonion import Octonion
from .delta_q import compute_delta_q
import numpy as np
import carb  

class OctonionScheduler:
    """Temporal semantics scheduler based on octonion update"""

    def __init__(self):
        self.q = Octonion()

        # Maximum number of sub-steps (to prevent computational explosion)
        self.max_substeps = 8
        self.omega_threshold = 5.0

    def on_physics_step(self, dt):
        """PhysX step callback for temporal semantics"""
        omega_norm = self._read_angular_velocity_norm()
        control = self._read_control()

        # Determine semantic sub-steps based on dynamic intensity
        substeps = self._compute_substeps(omega_norm)
        sub_dt = dt / substeps

        for _ in range(substeps):
            dq = compute_delta_q(
                sub_dt,
                control=control,
                omega_norm=omega_norm,
            )

            # Basic Consistency Guardrail
            if abs(dq.norm() - 1.0) > 1e-3:
                dq.normalize()

            # Non-commutative semantic update (The core of temporal audit)
            self.q = self.q * dq

        # Global stabilization (Numerical barrier)
        self.q.normalize()

        
        if substeps > 1:
            carb.log_warn(
                f"[Octonion-Causality] Dynamic Spike! Scaling to {substeps} "
                f"sub-steps to preserve temporal continuity. (Omega: {omega_norm:.2f})"
            )

    def _compute_substeps(self, omega_norm):
        if omega_norm < self.omega_threshold:
            return 1

        # Simple Linear Mapping (First Edition)
        scale = min(
            int(np.ceil(omega_norm / self.omega_threshold)),
            self.max_substeps,
        )
        return max(1, scale)

    def get_drift_magnitude(self) -> float:
        """
        Returns a scalar representing temporal inconsistency 
        detected by octonion evolution (i6 component).
        """
        # Return the non-associative perturbation component unique to octonions
        return float(abs(self.q.i[6]))

    # ===== placeholder hooks =====
    # Read the real-time angular velocity of the physics engine through the interface in the Demo

    def _read_control(self):
        return 0.0

    def _read_angular_velocity_norm(self):
        # Initially set to 1.0, it should be changed to read the measured value when testing the Demo.
        return 1.0
