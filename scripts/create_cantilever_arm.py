# =====================================================
# v0.4 Cantilever Arm – Order-Permutation Stress Test
# =====================================================

from omni.isaac.kit import SimulationApp
from omni.isaac.core import World
from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.core.utils.prims import define_prim
from omni.isaac.core.articulations import Articulation
from pxr import UsdPhysics, PhysxSchema, Gf
import numpy as np

# =====================================================
# Experimental Modes (v0.4)
# =====================================================

MODE_BASELINE_PHYSX = 0
MODE_GAIN_SCHEDULER = 1
MODE_OCTONION_OBSERVER = 2

EXPERIMENT_MODE = MODE_OCTONION_OBSERVER

# =====================================================
# Order Permutation Injector
# =====================================================

class OrderPermutationInjector:
    """
    Applies identical torques with permuted order.
    Breaks numerical associativity without changing energy.
    """
    def __init__(self, articulation: Articulation):
        self.art = articulation
        self.flip = False

    def apply(self):
        if self.flip:
            self._A_then_B()
        else:
            self._B_then_A()
        self.flip = not self.flip

    def _A_then_B(self):
        self.art.apply_action(
            {"joint1": 10.0, "joint2": -10.0}
        )

    def _B_then_A(self):
        self.art.apply_action(
            {"joint2": -10.0, "joint1": 10.0}
        )

# =====================================================
# Scene Construction
# =====================================================

world = World(stage_units_in_meters=1.0)
stage = get_current_stage()

ROOT = "/World/CantileverArm"

root = define_prim(ROOT, "Xform")
UsdPhysics.ArticulationRootAPI.Apply(root)

# Base
base = define_prim(f"{ROOT}/Base", "Cube")
base.GetAttribute("xformOp:scale").Set(Gf.Vec3f(0.2))
UsdPhysics.RigidBodyAPI.Apply(base)
UsdPhysics.CollisionAPI.Apply(base)
PhysxSchema.PhysxRigidBodyAPI.Apply(base).CreateKinematicEnabledAttr().Set(True)

# Link 1
link1 = define_prim(f"{ROOT}/Link1", "Capsule")
link1.GetAttribute("radius").Set(0.05)
link1.GetAttribute("height").Set(1.0)
link1.GetAttribute("xformOp:translate").Set(Gf.Vec3f(0, 0, -0.5))
UsdPhysics.RigidBodyAPI.Apply(link1)
UsdPhysics.CollisionAPI.Apply(link1)
UsdPhysics.MassAPI.Apply(link1).CreateMassAttr().Set(2.0)

joint1 = UsdPhysics.RevoluteJoint.Define(stage, f"{ROOT}/Joint1")
joint1.CreateAxisAttr("X")
joint1.CreateBody0Rel().SetTargets([f"{ROOT}/Base"])
joint1.CreateBody1Rel().SetTargets([f"{ROOT}/Link1"])
joint1.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0.5))
PhysxSchema.PhysxJointAPI.Apply(joint1.GetPrim()).CreateJointFrictionAttr().Set(0.0)

# Link 2
link2 = define_prim(f"{ROOT}/Link2", "Capsule")
link2.GetAttribute("radius").Set(0.04)
link2.GetAttribute("height").Set(1.5)
link2.GetAttribute("xformOp:translate").Set(Gf.Vec3f(0, 0, -1.5))
UsdPhysics.RigidBodyAPI.Apply(link2)
UsdPhysics.CollisionAPI.Apply(link2)
UsdPhysics.MassAPI.Apply(link2).CreateMassAttr().Set(5.0)

joint2 = UsdPhysics.RevoluteJoint.Define(stage, f"{ROOT}/Joint2")
joint2.CreateAxisAttr("X")
joint2.CreateBody0Rel().SetTargets([f"{ROOT}/Link1"])
joint2.CreateBody1Rel().SetTargets([f"{ROOT}/Link2"])
joint2.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0, -1.0))
joint2.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0.75))
PhysxSchema.PhysxJointAPI.Apply(joint2.GetPrim()).CreateJointFrictionAttr().Set(0.0)

# =====================================================
# Runtime Hook
# =====================================================

world.reset()
articulation = Articulation(ROOT)
world.scene.add(articulation)
injector = OrderPermutationInjector(articulation)

print("[v0.4] Cantilever arm ready. Running experiment mode:", EXPERIMENT_MODE)

# =====================================================
# Simulation Loop
# =====================================================

for _ in range(6000):
    world.step(render=True)

    if EXPERIMENT_MODE == MODE_BASELINE_PHYSX:
        articulation.apply_action({"joint1": 10.0, "joint2": -10.0})

    elif EXPERIMENT_MODE == MODE_GAIN_SCHEDULER:
        v = np.linalg.norm(articulation.get_joint_velocities())
        gain = min(v * 2.0, 20.0)
        articulation.apply_action({"joint1": gain, "joint2": -gain})

    elif EXPERIMENT_MODE == MODE_OCTONION_OBSERVER:
        injector.apply()
