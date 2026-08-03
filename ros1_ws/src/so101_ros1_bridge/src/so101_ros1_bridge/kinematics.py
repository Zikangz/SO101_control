import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass

import numpy as np


def _vec(text, default):
    if text is None:
        return np.array(default, dtype=float)
    return np.array([float(x) for x in text.split()], dtype=float)


def _translation(xyz):
    mat = np.eye(4)
    mat[:3, 3] = xyz
    return mat


def _rpy_matrix(rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    rot = rz @ ry @ rx
    mat = np.eye(4)
    mat[:3, :3] = rot
    return mat


def _origin_matrix(xyz, rpy):
    return _translation(xyz) @ _rpy_matrix(rpy)


def _axis_angle(axis, angle):
    axis = np.array(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm <= 0.0:
        return np.eye(4)
    x, y, z = axis / norm
    c = math.cos(angle)
    s = math.sin(angle)
    one_c = 1.0 - c
    rot = np.array(
        [
            [c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s],
            [y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s],
            [z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c],
        ],
        dtype=float,
    )
    mat = np.eye(4)
    mat[:3, :3] = rot
    return mat


@dataclass
class JointDef:
    name: str
    joint_type: str
    parent: str
    child: str
    origin: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float


class SO101Kinematics:
    def __init__(self, joints, base_link="base_link", tip_link="gripper_frame_link", limits_override=None):
        self.joints = {joint.name: joint for joint in joints}
        self.base_link = base_link
        self.tip_link = tip_link
        self.chain = self._build_chain(joints, base_link, tip_link)
        self.chain_joint_names = [joint.name for joint in self.chain if joint.joint_type != "fixed"]
        self.limits = {joint.name: (joint.lower, joint.upper) for joint in self.chain}
        for name, limit in (limits_override or {}).items():
            if name in self.limits and isinstance(limit, (list, tuple)) and len(limit) == 2:
                self.limits[name] = (float(limit[0]), float(limit[1]))

    @classmethod
    def from_urdf(cls, urdf_xml, base_link="base_link", tip_link="gripper_frame_link", limits_override=None):
        root = ET.fromstring(urdf_xml)
        joints = []
        for elem in root.findall("joint"):
            name = elem.attrib["name"]
            joint_type = elem.attrib.get("type", "fixed")
            parent = elem.find("parent").attrib["link"]
            child = elem.find("child").attrib["link"]
            origin_elem = elem.find("origin")
            xyz = _vec(origin_elem.attrib.get("xyz") if origin_elem is not None else None, [0.0, 0.0, 0.0])
            rpy = _vec(origin_elem.attrib.get("rpy") if origin_elem is not None else None, [0.0, 0.0, 0.0])
            axis_elem = elem.find("axis")
            axis = _vec(axis_elem.attrib.get("xyz") if axis_elem is not None else None, [0.0, 0.0, 1.0])
            limit_elem = elem.find("limit")
            lower = float(limit_elem.attrib.get("lower", "-3.141592653589793")) if limit_elem is not None else -math.inf
            upper = float(limit_elem.attrib.get("upper", "3.141592653589793")) if limit_elem is not None else math.inf
            joints.append(
                JointDef(
                    name=name,
                    joint_type=joint_type,
                    parent=parent,
                    child=child,
                    origin=_origin_matrix(xyz, rpy),
                    axis=axis,
                    lower=lower,
                    upper=upper,
                )
            )
        return cls(joints, base_link=base_link, tip_link=tip_link, limits_override=limits_override)

    def _build_chain(self, joints, base_link, tip_link):
        child_to_joint = {joint.child: joint for joint in joints}
        chain = []
        link = tip_link
        while link != base_link:
            if link not in child_to_joint:
                raise ValueError("No URDF chain from %s to %s; stopped at %s" % (base_link, tip_link, link))
            joint = child_to_joint[link]
            chain.append(joint)
            link = joint.parent
        chain.reverse()
        return chain

    def clip_joint(self, name, value):
        lower, upper = self.limits.get(name, (-math.inf, math.inf))
        return max(lower, min(upper, float(value)))

    def complete_positions(self, positions):
        return {name: float(positions.get(name, 0.0)) for name in self.chain_joint_names}

    def fk(self, positions):
        q = self.complete_positions(positions)
        transform = np.eye(4)
        for joint in self.chain:
            transform = transform @ joint.origin
            if joint.joint_type in ("revolute", "continuous"):
                transform = transform @ _axis_angle(joint.axis, q.get(joint.name, 0.0))
            elif joint.joint_type == "prismatic":
                transform = transform @ _translation(joint.axis * q.get(joint.name, 0.0))
        return transform

    def _active_ik_joints(self, active_joints, locked_joints):
        return [
            name
            for name in active_joints
            if name in self.chain_joint_names and name not in locked_joints
        ]

    def _limit_margin(self, name, value):
        lower, upper = self.limits.get(name, (-math.inf, math.inf))
        if not math.isfinite(lower) or not math.isfinite(upper):
            return math.inf
        return min(float(value) - lower, upper - float(value))

    def _joint_midpoint(self, name):
        lower, upper = self.limits.get(name, (-math.inf, math.inf))
        if math.isfinite(lower) and math.isfinite(upper):
            return 0.5 * (lower + upper)
        return 0.0

    def _candidate_seed_positions(self, seed_positions, active, locked_joints):
        base = self.complete_positions(seed_positions)
        for name, value in locked_joints.items():
            if name in base:
                base[name] = self.clip_joint(name, value)

        candidates = [dict(base)]

        neutral = dict(base)
        for name in active:
            neutral[name] = self.clip_joint(name, self._joint_midpoint(name))
        candidates.append(neutral)

        zero = dict(base)
        for name in active:
            zero[name] = self.clip_joint(name, 0.0)
        candidates.append(zero)

        planar_templates = [
            {"shoulder_lift": -0.45, "elbow_flex": 0.55, "wrist_flex": 0.45},
            {"shoulder_lift": -0.35, "elbow_flex": 0.30, "wrist_flex": 0.30},
            {"shoulder_lift": -0.10, "elbow_flex": 0.10, "wrist_flex": 0.45},
            {"shoulder_lift": 0.20, "elbow_flex": -0.20, "wrist_flex": 0.20},
            {"shoulder_lift": 0.35, "elbow_flex": -0.35, "wrist_flex": 0.10},
        ]
        for template in planar_templates:
            candidate = dict(base)
            used = False
            for name, value in template.items():
                if name in active:
                    candidate[name] = self.clip_joint(name, value)
                    used = True
            if used:
                candidates.append(candidate)

        unique = []
        seen = set()
        for candidate in candidates:
            key = tuple(round(float(candidate.get(name, 0.0)), 5) for name in self.chain_joint_names)
            if key in seen:
                continue
            seen.add(key)
            unique.append(candidate)
        return unique

    def _ik_score(self, solution, seed_positions, active, err_norm):
        continuity = 0.0
        limit_penalty = 0.0
        for name in active:
            value = float(solution.get(name, 0.0))
            continuity += (value - float(seed_positions.get(name, 0.0))) ** 2
            margin = self._limit_margin(name, value)
            if math.isfinite(margin):
                limit_penalty += max(0.0, 0.05 - margin) ** 2
        return float(err_norm) + 0.0005 * math.sqrt(continuity) + 0.08 * limit_penalty

    def _solve_ik_position_once(
        self,
        target_xyz,
        seed_positions,
        active_joints,
        locked_joints=None,
        max_iters=120,
        tolerance=0.006,
        damping=0.035,
        max_step=0.08,
    ):
        target = np.array(target_xyz, dtype=float)
        locked_joints = dict(locked_joints or {})
        active = self._active_ik_joints(active_joints, locked_joints)
        if not active:
            return False, dict(seed_positions), float("inf"), 0

        q = self.complete_positions(seed_positions)
        for name, value in locked_joints.items():
            if name in q:
                q[name] = self.clip_joint(name, value)

        eps = 1e-4
        last_err = float("inf")
        for iteration in range(max_iters):
            current = self.fk(q)[:3, 3]
            err = target - current
            err_norm = float(np.linalg.norm(err))
            last_err = err_norm
            if err_norm <= tolerance:
                return True, dict(q), err_norm, iteration

            jac = np.zeros((3, len(active)), dtype=float)
            for col, name in enumerate(active):
                q_eps = dict(q)
                q_eps[name] = self.clip_joint(name, q_eps[name] + eps)
                jac[:, col] = (self.fk(q_eps)[:3, 3] - current) / eps

            lhs = jac @ jac.T + (damping ** 2) * np.eye(3)
            try:
                step = jac.T @ np.linalg.solve(lhs, err)
            except np.linalg.LinAlgError:
                step = np.linalg.pinv(jac) @ err

            step_norm = float(np.linalg.norm(step))
            if step_norm > max_step:
                step = step * (max_step / step_norm)

            for idx, name in enumerate(active):
                q[name] = self.clip_joint(name, q[name] + step[idx])

        return False, dict(q), last_err, max_iters

    def solve_ik_position(
        self,
        target_xyz,
        seed_positions,
        active_joints,
        locked_joints=None,
        max_iters=120,
        tolerance=0.006,
        damping=0.035,
        max_step=0.08,
        multi_start=True,
    ):
        locked_joints = dict(locked_joints or {})
        active = self._active_ik_joints(active_joints, locked_joints)
        if not active:
            return False, dict(seed_positions), float("inf"), 0

        if not multi_start:
            return self._solve_ik_position_once(
                target_xyz,
                seed_positions,
                active,
                locked_joints=locked_joints,
                max_iters=max_iters,
                tolerance=tolerance,
                damping=damping,
                max_step=max_step,
            )

        best = None
        for candidate_seed in self._candidate_seed_positions(seed_positions, active, locked_joints):
            ok, solution, err, iters = self._solve_ik_position_once(
                target_xyz,
                candidate_seed,
                active,
                locked_joints=locked_joints,
                max_iters=max_iters,
                tolerance=tolerance,
                damping=damping,
                max_step=max_step,
            )
            score = self._ik_score(solution, seed_positions, active, err)
            candidate = (score, float(err), -int(bool(ok)), ok, solution, err, iters)
            if best is None or candidate < best:
                best = candidate

        _, _, _, ok, solution, err, iters = best
        return bool(ok), dict(solution), float(err), int(iters)
