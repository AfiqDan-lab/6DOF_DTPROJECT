#!/usr/bin/env python3
"""
make_usd.py  -  export a planned arm motion to an animated USD scene for Omniverse.

It plans a real collision-free motion with your twin (home -> right bin ->
left bin -> home, dodging the box), then writes a USD file where the arm is
built from cylinders whose transforms are keyframed across the trajectory.
Open the file in your Omniverse app and press play to see it render.

    python -m pip install usd-core        # only needed to RE-generate the file
    python scripts/make_usd.py            # writes arm_motion.usda

Requires usd-core, which needs Python 3.11 (it may not have a 3.14 wheel yet).
If pip refuses it on 3.14, just use the arm_motion.usda file already provided.
"""
import sys

import numpy as np
from pxr import Usd, UsdGeom, UsdLux, Gf

import arm_lib as A
from nlp_command import NAMED_LOCATIONS

FPS = 30
OBSTACLE = A.Box([0.33, 0.0, 0.11], [0.09, 0.11, 0.11])
LINK_COLOR = Gf.Vec3f(0.15, 0.45, 0.85)


def build_trajectory(chain):
    """A tour that includes an obstacle-avoiding move (right bin -> left bin)."""
    L = NAMED_LOCATIONS
    waypoints = []
    for a, b in [("home", "right bin"), ("right bin", "left bin"), ("left bin", "home")]:
        path, _ = A.plan_safe(chain, L[a], L[b], [OBSTACLE])
        waypoints += path
    return waypoints


def segment_matrix(a, b):
    """4x4 placing a cylinder's local +Z axis along segment a->b, centered."""
    a, b = np.asarray(a), np.asarray(b)
    mid = (a + b) / 2.0
    d = b - a
    length = float(np.linalg.norm(d))
    if length < 1e-6:
        return None, 0.0
    dirv = d / length
    m = Gf.Matrix4d().SetRotate(
        Gf.Rotation(Gf.Vec3d(0, 0, 1), Gf.Vec3d(float(dirv[0]), float(dirv[1]), float(dirv[2]))))
    m.SetTranslateOnly(Gf.Vec3d(float(mid[0]), float(mid[1]), float(mid[2])))
    return m, length


def box_matrix(center, half):
    m = Gf.Matrix4d().SetScale(Gf.Vec3d(2 * float(half[0]), 2 * float(half[1]), 2 * float(half[2])))
    m.SetTranslateOnly(Gf.Vec3d(float(center[0]), float(center[1]), float(center[2])))
    return m


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "arm_motion.usda"
    chain = A.load_arm()
    traj = build_trajectory(chain)
    skels = [np.array(A.skeleton(chain, q)) for q in traj]
    n_frames = len(traj)
    n_seg = len(skels[0]) - 1
    seg_len = [float(np.linalg.norm(skels[0][i + 1] - skels[0][i])) for i in range(n_seg)]

    stage = Usd.Stage.CreateNew(out)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetStartTimeCode(0)
    stage.SetEndTimeCode(n_frames - 1)
    stage.SetTimeCodesPerSecond(FPS)

    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    # lighting
    UsdLux.DomeLight.Define(stage, "/World/DomeLight").CreateIntensityAttr(800)
    sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
    sun.CreateIntensityAttr(1800)
    sun.CreateAngleAttr(1.0)

    # ground
    ground = UsdGeom.Cube.Define(stage, "/World/Ground")
    ground.CreateSizeAttr(1.0)
    ground.CreateDisplayColorAttr([Gf.Vec3f(0.72, 0.72, 0.75)])
    gm = Gf.Matrix4d().SetScale(Gf.Vec3d(2.0, 2.0, 0.01))
    gm.SetTranslateOnly(Gf.Vec3d(0, 0, -0.005))
    ground.AddTransformOp().Set(gm)

    # obstacle box (static)
    box = UsdGeom.Cube.Define(stage, "/World/Obstacle")
    box.CreateSizeAttr(1.0)
    box.CreateDisplayColorAttr([Gf.Vec3f(0.95, 0.55, 0.15)])
    box.AddTransformOp().Set(box_matrix(OBSTACLE.c, OBSTACLE.h))

    # target marker (static, at the left bin)
    tgt = UsdGeom.Sphere.Define(stage, "/World/Target")
    tgt.CreateRadiusAttr(0.02)
    tgt.CreateDisplayColorAttr([Gf.Vec3f(0.1, 0.8, 0.2)])
    tmx = Gf.Matrix4d()
    tmx.SetTranslateOnly(Gf.Vec3d(*[float(v) for v in NAMED_LOCATIONS["left bin"]]))
    tgt.AddTransformOp().Set(tmx)

    # the arm: one cylinder per link segment, transform keyframed over the trajectory
    UsdGeom.Xform.Define(stage, "/World/Arm")
    ops = []
    for i in range(n_seg):
        cyl = UsdGeom.Cylinder.Define(stage, f"/World/Arm/link_{i}")
        cyl.CreateAxisAttr("Z")
        cyl.CreateRadiusAttr(0.035 if i < 3 else 0.028)
        cyl.CreateHeightAttr(max(seg_len[i], 1e-3))
        cyl.CreateDisplayColorAttr([LINK_COLOR])
        ops.append(cyl.AddTransformOp())

    for f in range(n_frames):
        sk = skels[f]
        for i, op in enumerate(ops):
            m, _ = segment_matrix(sk[i], sk[i + 1])
            if m is not None:
                op.Set(m, Usd.TimeCode(f))

    stage.GetRootLayer().Save()
    print(f"Wrote {out}: {n_frames} frames @ {FPS} fps, {n_seg} arm segments.")


if __name__ == "__main__":
    main()
