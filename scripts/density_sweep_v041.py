# =========================================================
# v0.4.1 Density Scaling Auto Sweep (Minimal Version)
# =========================================================

import numpy as np
from omni.isaac.core import World
from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.core.utils.prims import define_prim
from omni.isaac.core.articulations import Articulation
from pxr import UsdPhysics, PhysxSchema, Gf

# ---------------------------------------------------------
# Config
# ---------------------------------------------------------

DENSITY_LEVELS = [1, 10, 100]   # sweep targets
SIM_STEPS = 2000
ROOT_TEMPLATE = "/World/CantileverArm_{}"

# ---------------------------------------------------------
# Cantilever builder (minimal reusable)
# ---------------------------------------------------------

def build_cantilever(stage, root_path):
    root = define_prim(root_path, "Xform")
    UsdPhysics.ArticulationRootAPI.Apply(root)

    # Base
    base = define_prim(f"{root_path}/Base", "Cube")
    base.GetAttribute("xformOp:scale").Set(Gf.Vec3f(0.2))
    UsdPhysics.RigidBodyAPI.Apply(base)
    PhysxSchema.PhysxRigidBodyAPI.Apply(base).CreateKinematicEnabledAttr().Set(True)

    # Link1
    link1 = define_prim(f"{root_path}/Link1", "Capsule")
    link1.GetAttribute("radius").Set(0.05)
    link1.GetAttribute("height").Set(1.0)
    link1.GetAttribute("xformOp:translate").Set(Gf.Vec3f(0, 0, -0.5))
    UsdPhysics.RigidBodyAPI.Apply(link1)
    UsdPhysics.CollisionAPI.Apply(link1)
    UsdPhysics.MassAPI.Apply(link1).CreateMassAttr().Set(2.0)

    joint1 = UsdPhysics.RevoluteJoint.Define(stage, f"{root_path}/Joint1")
    joint1.CreateAxisAttr("X")
    joint1.CreateBody0Rel().SetTargets([f"{root_path}/Base"])
    joint1.CreateBody1Rel().SetTargets([f"{root_path}/Link1"])
    joint1.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0.5))

    PhysxSchema.PhysxJointAPI.Apply(joint1.GetPrim()).CreateJointFrictionAttr().Set(0.05)
    drive1 = PhysxSchema.PhysxJointDriveAPI.Apply(joint1.GetPrim(), "angular")
    drive1.CreateDampingAttr().Set(1.0)

    # Link2
    link2 = define_prim(f"{root_path}/Link2", "Capsule")
    link2.GetAttribute("radius").Set(0.04)
    link2.GetAttribute("height").Set(1.5)
    link2.GetAttribute("xformOp:translate").Set(Gf.Vec3f(0, 0, -1.5))
    UsdPhysics.RigidBodyAPI.Apply(link2)
    UsdPhysics.CollisionAPI.Apply(link2)
    UsdPhysics.MassAPI.Apply(link2).CreateMassAttr().Set(5.0)

    joint2 = UsdPhysics.RevoluteJoint.Define(stage, f"{root_path}/Joint2")
    joint2.CreateAxisAttr("X")
    joint2.CreateBody0Rel().SetTargets([f"{root_path}/Link1"])
    joint2.CreateBody1Rel().SetTargets([f"{root_path}/Link2"])
    joint2.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0, -1.0))
    joint2.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0.75))

    PhysxSchema.PhysxJointAPI.Apply(joint2.GetPrim()).CreateJointFrictionAttr().Set(0.05)
    drive2 = PhysxSchema.PhysxJointDriveAPI.Apply(joint2.GetPrim(), "angular")
    drive2.CreateDampingAttr().Set(1.0)


# ---------------------------------------------------------
# Main sweep
# ---------------------------------------------------------

world = World(stage_units_in_meters=1.0)
stage = get_current_stage()

results = {}

for N in DENSITY_LEVELS:
    print(f"\n[v0.4.1] Running density level N={N}")

    world.reset()

    articulations = []

    # Build scene
    for i in range(N):
        root_path = ROOT_TEMPLATE.format(i)
        build_cantilever(stage, root_path)

        art = Articulation(root_path)
        world.scene.add(art)
        articulations.append(art)

    # Run simulation
    vel_accum = []

    for _ in range(SIM_STEPS):
        world.step(render=False)

        frame_vel = 0.0
        for art in articulations:
            v = art.get_joint_velocities()
            if v is not None and len(v) > 0:
                frame_vel += np.linalg.norm(v)

        vel_accum.append(frame_vel)

    mean_vel = float(np.mean(vel_accum))
    results[N] = mean_vel

    print(f"[v0.4.1] N={N} → mean velocity proxy = {mean_vel:.6f}")

print("\n========== Density Sweep Summary ==========")
for k, v in results.items():
    print(f"N={k:4d} | proxy={v:.6f}")
