#!/usr/bin/env python3
"""
make_eval_usd.py  -  visual evaluation tour for Omniverse.

The numeric companion, evaluate.py, reports the metrics. This writes a USD
animation where the arm visits a spread of test targets across its workspace
(each shown as a marker it should touch) and avoids the obstacle on the way,
so you can WATCH the system pass its tests in Omniverse before trusting it on
real hardware.

    python -m pip install usd-core     # needs Python 3.11 (may not install on 3.14)
    python scripts/make_eval_usd.py    # writes eval_showcase.usda

If usd-core won't install on your Python, just open the eval_showcase.usda
file already provided.
"""
import sys

import numpy as np
from pxr import Usd, UsdGeom, UsdLux, Gf

import arm_lib as A
from nlp_command import NAMED_LOCATIONS

FPS = 30
HOLD_FRAMES = 15                       # pause at each target so you can see it reach
OBSTACLE = A.Box([0.33, 0.0, 0.11], [0.09, 0.11, 0.11])
LINK_COLOR = Gf.Vec3f(0.15, 0.45, 0.85)

# representative test targets spanning the workspace (all reachable);
# several crossings pass over the box so avoidance is visible.
TOUR = [
    NAMED_LOCATIONS["home"],
    NAMED_LOCATIONS["left bin"],
    NAMED_LOCATIONS["right bin"],      # left -> right crosses the box
    (0.30, 0.00, 0.65),                # high center
    NAMED_LOCATIONS["drop zone"],
    (0.45, 0.00, 0.40),                # far reach
    (0.30, -0.25, 0.25),               # low right
    NAMED_LOCATIONS["home"],
]


def segment_matrix(a, b):
    a, b = np.asarray(a), np.asarray(b)
    mid = (a + b) / 2.0
    d = b - a
    length = float(np.linalg.norm(d))
    if length < 1e-6:
        return None
    dirv = d / length
    m = Gf.Matrix4d().SetRotate(
        Gf.Rotation(Gf.Vec3d(0, 0, 1), Gf.Vec3d(float(dirv[0]), float(dirv[1]), float(dirv[2]))))
    m.SetTranslateOnly(Gf.Vec3d(float(mid[0]), float(mid[1]), float(mid[2])))
    return m


def translate_scale(center, scale):
    m = Gf.Matrix4d().SetScale(Gf.Vec3d(*[float(s) for s in scale]))
    m.SetTranslateOnly(Gf.Vec3d(*[float(c) for c in center]))
    return m


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "eval_showcase.usda"
    chain = A.load_arm()

    # plan the whole tour, holding briefly at each target
    traj, reach_errors = [], []
    for a, b in zip(TOUR[:-1], TOUR[1:]):
        path, _ = A.plan_safe(chain, a, b, [OBSTACLE])
        if path is None:
            print(f"  (skipped unreachable/blocked leg {a}->{b})")
            continue
        traj += path + [path[-1]] * HOLD_FRAMES
        reach_errors.append(np.linalg.norm(A.tcp_position(chain, path[-1]) - np.array(b)))

    skels = [np.array(A.skeleton(chain, q)) for q in traj]
    n_frames = len(traj)
    n_seg = len(skels[0]) - 1
    seg_len = [float(np.linalg.norm(skels[0][i + 1] - skels[0][i])) for i in range(n_seg)]
    print(f"tour legs planned, worst reach error {max(reach_errors)*1000:.2f} mm")

    stage = Usd.Stage.CreateNew(out)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetStartTimeCode(0)
    stage.SetEndTimeCode(n_frames - 1)
    stage.SetTimeCodesPerSecond(FPS)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    UsdLux.DomeLight.Define(stage, "/World/DomeLight").CreateIntensityAttr(800)
    sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
    sun.CreateIntensityAttr(1800)
    sun.CreateAngleAttr(1.0)

    ground = UsdGeom.Cube.Define(stage, "/World/Ground")
    ground.CreateSizeAttr(1.0)
    ground.CreateDisplayColorAttr([Gf.Vec3f(0.72, 0.72, 0.75)])
    ground.AddTransformOp().Set(translate_scale((0, 0, -0.005), (2.0, 2.0, 0.01)))

    box = UsdGeom.Cube.Define(stage, "/World/Obstacle")
    box.CreateSizeAttr(1.0)
    box.CreateDisplayColorAttr([Gf.Vec3f(0.95, 0.55, 0.15)])
    box.AddTransformOp().Set(translate_scale(OBSTACLE.c, 2 * OBSTACLE.h))

    # a marker sphere at every test target (the points the arm should touch)
    markers = UsdGeom.Xform.Define(stage, "/World/Targets")
    seen = set()
    for k, t in enumerate(TOUR):
        key = tuple(round(v, 3) for v in t)
        if key in seen:
            continue
        seen.add(key)
        s = UsdGeom.Sphere.Define(stage, f"/World/Targets/t_{k}")
        s.CreateRadiusAttr(0.018)
        s.CreateDisplayColorAttr([Gf.Vec3f(0.1, 0.8, 0.2)])
        m = Gf.Matrix4d()
        m.SetTranslateOnly(Gf.Vec3d(*[float(v) for v in t]))
        s.AddTransformOp().Set(m)

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
            m = segment_matrix(sk[i], sk[i + 1])
            if m is not None:
                op.Set(m, Usd.TimeCode(f))

    stage.GetRootLayer().Save()
    print(f"Wrote {out}: {n_frames} frames @ {FPS} fps, {len(seen)} target markers.")


if __name__ == "__main__":
    main()
