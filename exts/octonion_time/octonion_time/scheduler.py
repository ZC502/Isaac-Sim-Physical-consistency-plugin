from .octonion import Octonion
from .delta_q import compute_delta_q
import numpy as np
import carb  

class OctonionScheduler:
    """
    Temporal semantics scheduler based on octonion update.

    get_drift_magnitude() is the ONLY signal exposed to the PhysX feedback loop.
    It defines when and how the solver intervention is triggered.
    """

    def __init__(self):
        self.q = Octonion()

        # External physical signal (fed by extension.py)
        self._external_omega = None
        
        # Maximum number of sub-steps (to prevent computational explosion)
        self.max_substeps = 8
        self.omega_threshold = 5.0

    # ---------------------------------------------------------
    # Main update
    # ---------------------------------------------------------
    def on_physics_step(self, dt: float):
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

            # Basic consistency guardrail
            if abs(dq.norm() - 1.0) > 1e-3:
                dq.normalize()

            # Non-commutative semantic update
            self.q = self.q * dq

        # Global stabilization
        self.q.normalize()

        if substeps > 1:
            carb.log_warn(
                f"[Octonion-Causality] Dynamic Spike! "
                f"Scaling to {substeps} sub-steps "
                f"(Omega={omega_norm:.2f})"
            )

    # ---------------------------------------------------------
    # Semantic diagnostics
    # ---------------------------------------------------------
    def get_drift_magnitude(self) -> float:
        """
        Returns a scalar representing temporal inconsistency
        detected by octonion evolution (i6 component).
        """
        return float(abs(self.q.i[6]))

    # ---------------------------------------------------------
    # External physical signal bridge
    # ---------------------------------------------------------
    def set_external_omega(self, omega_norm: float):
        """
        Inject real-time angular velocity magnitude
        from the physics engine.
        """
        self._external_omega = float(omega_norm)

    def _read_angular_velocity_norm(self) -> float:
        """
        Prefer external physical signal if provided.
        """
        if self._external_omega is not None:
            return self._external_omega
        return 0.0

    # ---------------------------------------------------------
    # Placeholder control hook
    # ---------------------------------------------------------
    def _read_control(self):
        return 0.0
