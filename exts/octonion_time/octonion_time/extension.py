import omni.ext
import omni.physx
import omni.usd
import carb

from pxr import UsdPhysics, PhysxSchema
from .scheduler import OctonionScheduler


class OctonionTimeExtension(omni.ext.IExt):
    """
    Octonion-based temporal semantics extension.
    v0.3: Active solver + joint damping intervention for demo scenes.
    """

    def on_startup(self, ext_id):
        carb.log_info("[Octonion] Extension startup")

        self._physx = omni.physx.get_physx_interface()
        if self._physx is None:
            carb.log_warn("[Octonion] PhysX interface not available")
            return

        # ---------------------------
        # Intervention config
        # ---------------------------
        self.enable_intervention = True

        # Solver iteration control
        self.base_iters = 4
        self.max_iters = 24
        self.drift_min = 0.001
        self.drift_max = 0.01
        self._last_iter = None

        # Joint damping control (Demo only)
        self.base_damping = 0.0
        self.max_damping = 50.0
        self._managed_joints = []
        self._demo_bound = False

        # Scheduler
        self.scheduler = OctonionScheduler()

        # Subscribe PhysX step
        self._subscription = self._physx.subscribe_physics_step_events(
            self._on_physics_step
        )

        carb.log_info("[Octonion] PhysX step hook registered (Active Intervention enabled)")

    # ---------------------------------------------------------
    # PhysX step callback
    # ---------------------------------------------------------
    def _on_physics_step(self, step_event):
        dt = step_event.dt

        # 1. Update octonion temporal semantics
        self.scheduler.on_physics_step(dt)

        # 2. Lazy bind demo joints (only once)
        if not self._demo_bound:
            self._try_bind_demo_joints()

        # 3. Compute drift
        drift = self.scheduler.get_drift_magnitude()

        # 4. Apply closed-loop intervention
        if self.enable_intervention:
            self._apply_physx_intervention(drift)
            self._apply_joint_damping_feedback(drift)

    # ---------------------------------------------------------
    # Demo detection & joint binding
    # ---------------------------------------------------------
    def _try_bind_demo_joints(self):
        stage = omni.usd.get_context().get_stage()
        demo_root = "/World/CantileverArm"
        prim = stage.GetPrimAtPath(demo_root)

        if not prim.IsValid():
            return

        carb.log_info(f"[Octonion] Demo scene detected at {demo_root}")

        # Find all revolute joints under demo root
        self._managed_joints.clear()

        for p in stage.Traverse():
            if not p.GetPath().pathString.startswith(demo_root):
                continue

            if p.IsA(UsdPhysics.RevoluteJoint):
                drive = UsdPhysics.DriveAPI.Apply(p, "angular")
                if drive:
                    # Initialize damping to zero
                    drive.CreateDampingAttr().Set(self.base_damping)
                    self._managed_joints.append(drive)

        carb.log_info(
            f"[Octonion] Bound {len(self._managed_joints)} joints "
            f"for adaptive damping feedback"
        )

        self._demo_bound = True

    # ---------------------------------------------------------
    # Solver iteration feedback
    # ---------------------------------------------------------
    def _apply_physx_intervention(self, drift_magnitude):
        drift_clamped = max(self.drift_min, min(drift_magnitude, self.drift_max))
        alpha = (drift_clamped - self.drift_min) / (self.drift_max - self.drift_min)

        new_iters = int(self.base_iters + alpha * (self.max_iters - self.base_iters))
        self._physx.set_solver_position_iteration_count(new_iters)

        if new_iters != self._last_iter:
            carb.log_info(
                f"[Octonion-Feedback] Drift={drift_magnitude:.5f} "
                f"→ SolverIters={new_iters}"
            )
            self._last_iter = new_iters

    # ---------------------------------------------------------
    # Joint damping feedback (visual!)
    # ---------------------------------------------------------
    def _apply_joint_damping_feedback(self, drift_magnitude):
        if not self._managed_joints:
            return

        # Map drift to damping (quadratic = stronger visual effect)
        alpha = min(drift_magnitude / self.drift_max, 1.0)
        damping = self.base_damping + (alpha ** 2) * self.max_damping

        for drive in self._managed_joints:
            drive.GetDampingAttr().Set(damping)

        # Only log when effect is visible
        if damping > 1.0:
            carb.log_info(
                f"[Octonion-Damping] Adaptive damping applied: {damping:.2f}"
            )

    def on_shutdown(self):
        carb.log_info("[Octonion] Extension shutdown")
        self._subscription = None
        self.scheduler = None
        self._managed_joints.clear()
