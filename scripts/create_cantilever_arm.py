from omni.isaac.kit import SimulationApp
from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.core.utils.prims import define_prim
from pxr import UsdPhysics, PhysxSchema, Gf

# You only need this line if you are running it in an independent script (not in the Script Editor).
# simulation_app = SimulationApp({"headless": False})

stage = get_current_stage()

ROOT = "/World/CantileverArm"

# -----------------------------
# Root
# -----------------------------
root = define_prim(ROOT, "Xform")
UsdPhysics.ArticulationRootAPI.Apply(root)

# -----------------------------
# Base (fixed)
# -----------------------------
base = define_prim(f"{ROOT}/Base", "Cube")
base.GetAttribute("xformOp:scale").Set(Gf.Vec3f(0.2, 0.2, 0.2))
UsdPhysics.RigidBodyAPI.Apply(base)
UsdPhysics.CollisionAPI.Apply(base)

PhysxSchema.PhysxRigidBodyAPI.Apply(base).CreateKinematicEnabledAttr().Set(True)

# -----------------------------
# Link 1
# -----------------------------
link1 = define_prim(f"{ROOT}/Link1", "Capsule")
link1.GetAttribute("radius").Set(0.05)
link1.GetAttribute("height").Set(1.0)
link1.GetAttribute("xformOp:translate").Set(Gf.Vec3f(0, 0, -0.5))

UsdPhysics.RigidBodyAPI.Apply(link1)
UsdPhysics.CollisionAPI.Apply(link1)
UsdPhysics.MassAPI.Apply(link1).CreateMassAttr().Set(2.0)

# Joint 1
joint1 = UsdPhysics.RevoluteJoint.Define(stage, f"{ROOT}/Joint1")
joint1.CreateAxisAttr("X")
joint1.CreateBody0Rel().SetTargets([f"{ROOT}/Base"])
joint1.CreateBody1Rel().SetTargets([f"{ROOT}/Link1"])
joint1.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0, 0))
joint1.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0.5))

PhysxSchema.PhysxJointAPI.Apply(joint1.GetPrim()).CreateJointFrictionAttr().Set(0.0)

# -----------------------------
# Link 2 (long + heavy → jitter amplifier)
# -----------------------------
link2 = define_prim(f"{ROOT}/Link2", "Capsule")
link2.GetAttribute("radius").Set(0.04)
link2.GetAttribute("height").Set(1.5)
link2.GetAttribute("xformOp:translate").Set(Gf.Vec3f(0, 0, -1.5))

UsdPhysics.RigidBodyAPI.Apply(link2)
UsdPhysics.CollisionAPI.Apply(link2)
UsdPhysics.MassAPI.Apply(link2).CreateMassAttr().Set(5.0)

# Joint 2
joint2 = UsdPhysics.RevoluteJoint.Define(stage, f"{ROOT}/Joint2")
joint2.CreateAxisAttr("X")
joint2.CreateBody0Rel().SetTargets([f"{ROOT}/Link1"])
joint2.CreateBody1Rel().SetTargets([f"{ROOT}/Link2"])
joint2.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0, -1.0))
joint2.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0.75))

PhysxSchema.PhysxJointAPI.Apply(joint2.GetPrim()).CreateJointFrictionAttr().Set(0.0)

print("[Demo] Cantilever arm demo created at /World/CantileverArm")

# simulation_app.close()
