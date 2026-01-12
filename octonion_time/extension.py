import omni.ext
import omni.physx as physx
from .scheduler import OctonionScheduler

class OctonionTimeExtension(omni.ext.IExt):

    def on_startup(self, ext_id):
        print("[Octonion] Extension startup")

        self.scheduler = OctonionScheduler()

        self._physx = physx.get_physx_interface()
        self._subscription = self._physx.subscribe_physics_step_events(
            self._on_physics_step
        )

    def _on_physics_step(self, step_event):
        dt = step_event.dt
        self.scheduler.on_physics_step(dt)

    def on_shutdown(self):
        print("[Octonion] Extension shutdown")
        self._subscription = None
