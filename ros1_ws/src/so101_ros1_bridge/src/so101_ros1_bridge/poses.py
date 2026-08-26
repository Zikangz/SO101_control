"""Shared conservative SO101 follower poses for first-phase desk testing."""

JOINT_ORDER = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

ACTIVE_JOINTS_4DOF = ["shoulder_lift", "elbow_flex", "wrist_flex", "gripper"]

# These poses match the default 4-DOF safety mode:
# shoulder_pan and wrist_roll remain locked by the bridge config.
# Revolute joints are radians. The gripper command is normalized:
# 0.0 closed, 1.0 open.
SAFE_POSES = {
    "stow": {
        "shoulder_lift": -0.45,
        "elbow_flex": 0.85,
        "wrist_flex": -0.35,
        "gripper": 0.5,
    },
    "ready": {
        "shoulder_lift": 0.0,
        "elbow_flex": 0.0,
        "wrist_flex": 0.0,
        "gripper": 0.5,
    },
    "reach": {
        "shoulder_lift": 0.25,
        "elbow_flex": -0.35,
        "wrist_flex": 0.15,
        "gripper": 0.75,
    },
    "grasp": {
        "shoulder_lift": 0.25,
        "elbow_flex": -0.35,
        "wrist_flex": 0.15,
        "gripper": 0.15,
    },
    "release": {
        "shoulder_lift": 0.25,
        "elbow_flex": -0.35,
        "wrist_flex": 0.15,
        "gripper": 0.9,
    },
    "return": {
        "shoulder_lift": 0.0,
        "elbow_flex": 0.0,
        "wrist_flex": 0.0,
        "gripper": 0.5,
    },
}

SAFE_SEQUENCES = {
    "stow": ["stow"],
    "ready": ["ready"],
    "grasp": ["ready", "reach", "grasp"],
    "release": ["release"],
    "return": ["return", "stow"],
    "cycle": ["stow", "ready", "reach", "grasp", "release", "return", "stow"],
}
