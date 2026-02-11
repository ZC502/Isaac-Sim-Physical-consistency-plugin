# =====================================================
# v0.4 Cantilever Arm – Order-Permutation Audit Benchmark
# =====================================================
# This script implements the "Rational Physics" regime:
# - Reasonable Damping (1.0) and Friction (0.05)
# - Order-Permutation Perturbation (A->B vs B->A)
# - Mode C: Observer-only link to Octonion Extension
# =====================================================

import numpy as np
import omni.kit.app
from omni.isaac.kit import SimulationApp
from omni.isaac.core import World
from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.core.utils.prims import define_prim
from omni.isaac.core.articulations import Articulation
from pxr import UsdPhysics, PhysxSchema, Gf

# =====================================================
# 1. Experimental Modes
# =====================================================
MODE_BASELINE_PHYSX = 0      # No adaptation
MODE_GAIN_SCHEDULER = 1      # Velocity-based scalar gain
MODE_OCTONION_OBSERVER = 2   # Non-associative audit (Octonion)

# SET YOUR MODE HERE
EXPERIMENT_MODE = MODE_OCTONION_OBSERVER

# =====================================================
# 2. Order Permutation Injector
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
        # We switch the order of torque application every step
        if self.flip:
            self._A_then_B()
        else:
            self._B_then_A()
        self.flip = not self.flip

    def _A_then_B(self):
        # Sequence: Joint 1 then Joint 2
        self.art.apply_action({"joint1": 10.0, "joint2": -10.0})

    def _B_then_A(self):
        # Sequence: Joint 2 then Joint 1
        self.art.apply_action({"joint2": -10.0, "joint1": 10.0})

# =====================================================
# 3. Scene Construction
# =====================================================
world = World(stage_units_in_meters=1.0)
stage = get_current_stage()
ROOT = "/World/CantileverArm"

root = define_prim(ROOT, "Xform")
UsdPhysics.ArticulationRootAPI.Apply(root)

# --- Base (Kinematic) ---
base = define_prim(f"{ROOT}/Base", "Cube")
base.GetAttribute("xformOp:scale").Set(Gf.Vec3f(0.2))
UsdPhysics.RigidBodyAPI.Apply(base)
PhysxSchema.PhysxRigidBodyAPI.Apply(base).CreateKinematicEnabledAttr().Set(True)

# --- Link 1 ---
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

# Rational Physics: Friction 0.05, Damping 1.0
PhysxSchema.PhysxJointAPI.Apply(joint1.GetPrim()).CreateJointFrictionAttr().Set(0.05)
drive1 = PhysxSchema.PhysxJointDriveAPI.Apply(joint1.GetPrim(), "angular")
drive1.CreateDampingAttr().Set(1.0)

# --- Link 2 ---
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

# Rational Physics: Friction 0.05, Damping 1.0
PhysxSchema.PhysxJointAPI.Apply(joint2.GetPrim()).CreateJointFrictionAttr().Set(0.05)
drive2 = PhysxSchema.PhysxJointDriveAPI.Apply(joint2.GetPrim(), "angular")
drive2.CreateDampingAttr().Set(1.0)

# =====================================================
# 4. Runtime & Extension Binding
# =====================================================
world.reset()
articulation = Articulation(ROOT)
world.scene.add(articulation)
injector = OrderPermutationInjector(articulation)

# Attempt to link with Octonion Extension
ext_manager = omni.kit.app.get_app().get_extension_manager()
oct_ext = ext_manager.get_extension_instance("OctonionTimeExtension")

if oct_ext:
    print("[v0.4] Successfully linked to OctonionTimeExtension.")
else:
    print("[v0.4] WARNING: OctonionTimeExtension not found. Mode C will run without audit logging.")

print(f"[v0.4] Experiment ready. Mode: {EXPERIMENT_MODE}")

# =====================================================
# 5. Main Simulation Loop
# =====================================================
for i in range(10000):
    world.step(render=True)

    if EXPERIMENT_MODE == MODE_BASELINE_PHYSX:
        # Standard torque application (Fixed order)
        articulation.apply_action({"joint1": 10.0, "joint2": -10.0})

    elif EXPERIMENT_MODE == MODE_GAIN_SCHEDULER:
        # Simple velocity-based heuristic
        v = np.linalg.norm(articulation.get_joint_velocities())
        gain = min(v * 2.0, 20.0)
        articulation.apply_action({"joint1": gain, "joint2": -gain})

    elif EXPERIMENT_MODE == MODE_OCTONION_OBSERVER:
        # Inject Order-Permutation Perturbation
        injector.apply()
        
        # Bridge the execution order signal to the Octonion Auditor
        if oct_ext:
            oct_ext.set_order_flip(injector.flip)

    if i % 500 == 0:
        print(f"Step {i}...")
