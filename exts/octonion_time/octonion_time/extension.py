import numpy as np
import carb
import omni.ext
import omni.physx
import omni.usd

from pxr import UsdPhysics
from .scheduler import OctonionScheduler


class OctonionTimeExtension(omni.ext.IExt):
    """
    Octonion-based temporal semantics extension.
    v0.4-C: Order-permutation observer (no black-box control)
    """

    def on_startup(self, ext_id):
        carb.log_info("[Octonion] Extension startup (v0.4-C)")

        self._physx = omni.physx.get_physx_interface()
        if self._physx is None:
            carb.log_warn("[Octonion] PhysX interface not available")
            return

        # ---------------------------
        # Observer config (v0.4)
        # ---------------------------
        self.enable_intervention = False  # v0.4-C: observer only

        self.base_iters = 4
        self.max_iters = 24
        self.drift_min = 0.001
        self.drift_max = 0.01
        self._last_iter = None

        self.base_damping = 0.0
        self.max_damping = 50.0
        self._managed_joints = []
        self._demo_bound = False

        self._articulation = None

        # v0.4: order permutation signal
        self._order_flip = 1.0

        self.scheduler = OctonionScheduler()

        self._subscription = self._physx.subscribe_physics_step_events(
            self._on_physics_step
        )

        carb.log_info("[Octonion] PhysX step hook registered (Observer mode)")

    # ---------------------------------------------------------
    # External order signal (called by demo script)
    # ---------------------------------------------------------
    def set_order_flip(self, flip: bool):
        self._order_flip = bool(flip)


    # ---------------------------------------------------------
    # PhysX step callback
    # ---------------------------------------------------------
    def _on_physics_step(self, step_event):
        dt = step_event.dt

        self._update_scheduler_inputs()
        self.scheduler.on_physics_step(dt)

        if not self._demo_bound:
            self._try_bind_demo_joints()

        # v0.4-C: pure associator drift
        drift = self.scheduler.get_associator_magnitude()

        if self.enable_intervention:
            self._apply_physx_intervention(drift)
            self._apply_joint_damping_feedback(drift)

    # ---------------------------------------------------------
    # Scheduler input bridge
    # ---------------------------------------------------------
    def _update_scheduler_inputs(self):
        omega_norm = self._read_angular_velocity_norm()
        self.scheduler.set_external_omega(omega_norm)

        # v0.4-C: expose order only (no physics modification)
        self.scheduler.set_order_signal(self._order_flip)

    # ---------------------------------------------------------
    # Articulation bridge
    # ---------------------------------------------------------
    def _bind_articulation_if_needed(self):
        if self._articulation is not None:
            return

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return

        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                try:
                    self._articulation = self._physx.get_articulation(prim.GetPath())
                    carb.log_info(f"[Octonion] Articulation bound at {prim.GetPath()}")
                    return
                except Exception as e:
                    carb.log_warn(f"[Octonion][WARN] Articulation bind failed: {e}")

    def _read_angular_velocity_norm(self) -> float:
        self._bind_articulation_if_needed()
        if self._articulation is None:
            return 0.0

        try:
            vels = self._articulation.get_joint_velocities()
            if vels is None or len(vels) == 0:
                return 0.0
            return float(np.linalg.norm(vels))
        except Exception as e:
            carb.log_warn(f"[Octonion][WARN] Velocity read failed: {e}")
            return 0.0

    # ---------------------------------------------------------
    # Optional feedback paths (disabled by default)
    # ---------------------------------------------------------
    def _apply_physx_intervention(self, drift_magnitude):
        drift = max(self.drift_min, min(drift_magnitude, self.drift_max))
        alpha = (drift - self.drift_min) / (self.drift_max - self.drift_min)

        new_iters = int(self.base_iters + alpha * (self.max_iters - self.base_iters))
        self._physx.set_solver_position_iteration_count(new_iters)

        if new_iters != self._last_iter:
            carb.log_info(
                f"[Octonion-Feedback] Drift={drift_magnitude:.5f} → SolverIters={new_iters}"
            )
            self._last_iter = new_iters

    def _apply_joint_damping_feedback(self, drift_magnitude):
        if not self._managed_joints:
            return

        alpha = min(drift_magnitude / self.drift_max, 1.0)
        damping = self.base_damping + (alpha ** 2) * self.max_damping

        for drive in self._managed_joints:
            drive.GetDampingAttr().Set(damping)

    def on_shutdown(self):
        carb.log_info("[Octonion] Extension shutdown")
        self._subscription = None
        self.scheduler = None
        self._managed_joints.clear()
        self._articulation = None
