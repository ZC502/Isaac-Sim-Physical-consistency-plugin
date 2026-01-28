import omni.ext
import omni.physx
import carb

from .scheduler import OctonionScheduler


class OctonionTimeExtension(omni.ext.IExt):
    """
    Octonion-based temporal semantics extension.

    v0.2:
    - Implements real-time PhysX solver intervention
    - Closed-loop feedback: drift -> solver iterations
    """

    def on_startup(self, ext_id):
        carb.log_info("[Octonion] Extension startup")

        self._physx = omni.physx.get_physx_interface()
        if self._physx is None:
            carb.log_warn("[Octonion] PhysX interface not available")
            return

        # --- Intervention config ---
        self.enable_intervention = True
        self.base_iters = 4
        self.max_iters = 24
        self.drift_min = 0.001
        self.drift_max = 0.01
        self._last_iter = None

        # Scheduler
        self.scheduler = OctonionScheduler()

        # Subscribe to PhysX step
        self._subscription = self._physx.subscribe_physics_step_events(
            self._on_physics_step
        )

        carb.log_info(
            "[Octonion] PhysX step hook registered (Active Intervention enabled)"
        )

    def _on_physics_step(self, step_event):
        """PhysX step callback with feedback loop"""
        dt = step_event.dt

        # 1. Update octonion temporal state
        self.scheduler.on_physics_step(dt)

        # 2. Retrieve drift magnitude
        if hasattr(self.scheduler, "get_drift_magnitude"):
            drift = float(self.scheduler.get_drift_magnitude())
        else:
            # Explicit placeholder (visible & honest)
            drift = 0.0
            carb.log_warn_once(
                "[Octonion] get_drift_magnitude() not implemented in scheduler "
                "(using 0.0 placeholder)"
            )

        # 3. Closed-loop intervention
        if self.enable_intervention:
            self._apply_physx_intervention(drift)

    def _apply_physx_intervention(self, drift_magnitude: float):
        """Real-time PhysX solver intervention"""

        # Clamp & normalize drift
        drift_clamped = max(
            self.drift_min, min(drift_magnitude, self.drift_max)
        )
        alpha = (
            (drift_clamped - self.drift_min)
            / (self.drift_max - self.drift_min)
        )

        new_iters = int(
            self.base_iters
            + alpha * (self.max_iters - self.base_iters)
        )

        try:
            self._physx.set_solver_position_iteration_count(new_iters)
        except Exception as e:
            carb.log_error(
                f"[Octonion] Failed to set solver iterations: {e}"
            )
            return

        # Log only on change
        if new_iters != self._last_iter:
            carb.log_info(
                f"[Octonion-Feedback] "
                f"Drift={drift_magnitude:.5f} | "
                f"SolverIters -> {new_iters}"
            )
            self._last_iter = new_iters

    def on_shutdown(self):
        carb.log_info("[Octonion] Extension shutdown")

        self._subscription = None
        self.scheduler = None
        self._physx = None
