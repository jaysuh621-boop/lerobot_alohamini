"""Shared URDF helpers: forward kinematics (FK) and mesh collision checks.

Uses common.URDF_PATH and the SIGN/OFFSET mapping in common.py. Never connects to the arm.
Needs trimesh and python-fcl (installed in the lerobot_alohamini env on 2026-10-19).
"""

import itertools
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import fcl
import numpy as np
import trimesh

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import common  # noqa: E402

URDF = common.URDF_PATH
_root = ET.parse(URDF).getroot()


def origin_mat(el) -> np.ndarray:
    """4x4 transform from a URDF <origin xyz rpy> element."""
    if el is None:
        return np.eye(4)
    xyz = list(map(float, (el.get("xyz") or "0 0 0").split()))
    rpy = list(map(float, (el.get("rpy") or "0 0 0").split()))
    m = trimesh.transformations.euler_matrix(*rpy, "sxyz")
    m[:3, 3] = xyz
    return m


# Visual mesh of each link, with its scale and visual origin applied.
meshes: dict[str, trimesh.Trimesh] = {}
for _link in _root.findall("link"):
    _vis = _link.find("visual")
    if _vis is None:
        continue
    _me = _vis.find("geometry/mesh")
    _m = trimesh.load(URDF.parent / _me.get("filename"), force="mesh")
    if _me.get("scale"):
        _m.apply_scale(list(map(float, _me.get("scale").split())))
    _m.apply_transform(origin_mat(_vis.find("origin")))
    meshes[_link.get("name")] = _m

# (name, type, parent link, child link, origin transform, axis)
joints = []
for _j in _root.findall("joint"):
    _ax = _j.find("axis")
    joints.append((_j.get("name"), _j.get("type"), _j.find("parent").get("link"), _j.find("child").get("link"),
                   origin_mat(_j.find("origin")),
                   np.array(list(map(float, _ax.get("xyz").split()))) if _ax is not None else None))
child_to_parent = {c: p for _, _, p, c, _, _ in joints}
ROOT_LINK = (set(meshes) - set(child_to_parent)).pop()

# fcl collision geometry (BVH) of each mesh. Built once; trimesh's own CollisionManager needs scipy.
_bvh = {}
for _n, _m in meshes.items():
    _b = fcl.BVHModel()
    _b.beginModel(len(_m.vertices), len(_m.faces))
    _b.addSubModel(_m.vertices.astype(float), _m.faces.astype(np.int32))
    _b.endModel()
    _bvh[_n] = _b


def fk(q_rad: dict[str, float]) -> dict[str, np.ndarray]:
    """World 4x4 transform of each link for URDF joint angles in radians (missing joints = 0)."""
    T = {ROOT_LINK: np.eye(4)}
    pending = list(joints)
    while pending:
        for jt in list(pending):
            name, typ, parent, child, O, ax = jt
            if parent in T:
                R = np.eye(4)
                if typ == "revolute":
                    R = trimesh.transformations.rotation_matrix(q_rad.get(name, 0.0), ax)
                T[child] = T[parent] @ O @ R
                pending.remove(jt)
    return T


def objects(T: dict[str, np.ndarray]) -> dict[str, fcl.CollisionObject]:
    return {n: fcl.CollisionObject(_bvh[n], fcl.Transform(T[n][:3, :3], T[n][:3, 3])) for n in _bvh}


def depth_mm(o, a: str, b: str) -> float:
    """Overlap depth in mm if links a and b overlap (positive), else minus the gap in mm."""
    res = fcl.CollisionResult()
    if fcl.collide(o[a], o[b], fcl.CollisionRequest(num_max_contacts=100000, enable_contact=True), res):
        return max(c.penetration_depth for c in res.contacts) * 1000
    return -fcl.distance(o[a], o[b], fcl.DistanceRequest(), fcl.DistanceResult()) * 1000


def colliding_pairs(T: dict[str, np.ndarray]) -> set[tuple[str, str]]:
    o = objects(T)
    return {(a, b) for a, b in itertools.combinations(sorted(o), 2)
            if fcl.collide(o[a], o[b], fcl.CollisionRequest(), fcl.CollisionResult())}


def adjacent(a: str, b: str) -> bool:
    return child_to_parent.get(a) == b or child_to_parent.get(b) == a


def parse_reading(text: str) -> dict[str, float]:
    """Joint readings from text such as 'shoulder_pan=-12.8 shoulder_lift=-103.3 ...' (view_3d.py output)."""
    return {k: float(v) for k, v in re.findall(r"(\w+)=\s*(-?[\d.]+)", text)}
