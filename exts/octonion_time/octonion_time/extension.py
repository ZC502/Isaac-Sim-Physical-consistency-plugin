import omni.ext
import omni.physx

from .scheduler import OctonionScheduler


class OctonionTimeExtension(omni.ext.IExt):
    """Octonion-based temporal semantics extension"""

    def on_startup(self, ext_id):
        print("[Octonion] Extension startup")

        self._physx = omni.physx.get_physx_interface()
        if self._physx is None:
            print("[Octonion][WARN] PhysX interface not available")
            return

        # 初始化调度器（承载八元数时间语义）(Adaptive sub-stepping (key))
        self.scheduler = OctonionScheduler()

        # 订阅 PhysX step（时间语义唯一入口）(Subscribe to PhysX step (the only entry for time semantics))
        self._subscription = self._physx.subscribe_physics_step_events(
            self._on_physics_step
        )

        print("[Octonion] PhysX step hook registered")

    def _on_physics_step(self, step_event):
        """PhysX step callback"""
        dt = step_event.dt

        # 将离散 dt 转化为八元数语义子步(Convert discrete dt into octonion semantic substeps)
        self.scheduler.on_physics_step(dt)

    def on_shutdown(self):
        print("[Octonion] Extension shutdown")

        # 正确注销 PhysX 回调(Properly unregister PhysX callbacks)
        if self._subscription is not None:
            self._subscription = None

        self.scheduler = None
        self._physx = None
