# Kinematics & Dynamics

![Robots Simulation](images/robot.gif)

This library provides a unified framework for **robot kinematics and dynamics** that supports both the **Denavit–Hartenberg (DH)** and **URDF (Unified Robot Description Format)** representations.

The library implements the following features:

- **Forward kinematics**: compute the end-effector pose from joint positions  
- **Inverse kinematics**: via Damped Least Squares (DLS) inverse Jacobian  
- **Pose interpolation**: smooth Cartesian motion using SLERP (Spherical Linear Interpolation)  
- **Inverse dynamics**: via Recursive Newton–Euler algorithm (RNEA)  

An example setup is provided for the **SO101** robotic arm from [LeRobot](https://github.com/huggingface/lerobot), while the models (`.xml`, `.urdf`) are obtained from [TheRobotStudio](https://github.com/TheRobotStudio/SO-ARM100/tree/385e8d7c68e24945df6c60d9bd68837a4b7411ae/Simulation/SO101). The modular structure allows you to easily replace the SO101 with any custom N-DOF manipulator.

## 1. Project structure

### 1.1 Folder structure

```bash
controls/
│
├─ environment.yml              ← Conda environment and dependencies
│
├─ models/                      ← Robot models and assets
│   ├─ so101/                   ← .xml and .urdf files 
│
├─ scripts/                     ← Core libraries (URDF + DH utilities)
│   ├─ kinematics.py            ← Forward and inverse kinematics functions
│   ├─ dynamics.py              ← Robot dynamics
│   ├─ model.py                 ← URDF/DH loading utilities
│   ├─ utils.py                 ← General-purpose robot math utilities
│
├─ simulator/                   ← MuJoCo-based simulation tools
│   ├─ main_so101_mj.py         ← MuJoCo simulation for SO100 robot
│   ├─ main_fp3_mj.py           ← MuJoCo simulation for Fanka Emika Panda robot
│   ├─ main_g1_mj.py            ← MuJoCo simulation for G1 robot
│   
├─ tests/                       ← Testing 
│   ├─ unit_tests.py            ← Test DH methods via unit testing
│
├─ main_dh.py                   ← Main script for DH-based kinematics/dynamics
├─ main_urdf.py                 ← Main script for URDF-based kinematics
│
└─ README.md                    ← Project documentation 
```

### 1.2 Setup conda environment and run the code

Clone repo and go inside folder:

```bash
git clone https://github.com/Argo-Robot/controls
cd controls
```

Download necessary libs by using conda environment:

```
conda env create -f environment.yml
conda activate controls_env
pip install --upgrade --force-reinstall networkx==2.8.8
```

To run `main_dh.py` or `main_urdf.py` just type:

```bash
cd controls
python main_dh.py
python main_urdf.py
```

To run `main_so101_mj.py` just type:

```bash
cd controls
python -m simulator.main_so101_mj
```

### 1.3 Example in `main_dh.py` 

- Initialize the `so100` robot model
- Transform mechanical angles in DH angles
- Define a goal pose `T_goal`
- Trajectory interpolation from start to goal pose.
- Solves IK with only position tracking
- Transform DH angles in mechanical angles
- Check mechanical angles are within their physical limits
- Initialize $q$, $\dot q$, $\ddot q$, $F_{ext}$
- Compute corresponding joint torques
- Compute B, C, g matrices for the dynamic model
- Transform a force from one frame to another

### 1.4 Example in `main_so101_mj.py`

- Load the `so101` robot model and load it also in Mujoco
- Define an initial joint configuration `q_init` (current joint state from Mujoco) and a goal pose `T_goal`
- Trajectory interpolation from start to goal pose.
- Solves IK step and simulate current step update in Mujoco
- Check joint angles are within their physical limits
- Visualize entire trajectory simulation through Mujoco

---

## 2. Modules

### 2.1 `Model`

This module defines a unified interface to load and describe a robot model, either from manual DH parameters or from a URDF file.

#### Main classes

- **RobotLoader**: Base class defining shared attributes and methods for any robot description (DH or URDF).
- **DH_loader**: Defines a robot model manually via Denavit–Hartenberg parameters.
- **URDF_loader**: Loads a robot from a .urdf file using urdfpy.
- **RobotModel**: High-level wrapper that unifies DH- and URDF-based models.

#### Parameters

- `link_mass`: link mass.
- `link_com`: link CoM wrt frame $i$.
- `link_inertia`: link inertia wrt frame $i$.
- `mech_joint_limits_low`: mechanical joint position limits lower bound
- `mech_joint_limits_up`: mechanical joint position limits upper bound
- `worldTbase`: 4x4 homogeneous transform.
- `nTtool`: 4x4 homogeneous transform.
- `dh_table`: DH table as a list of $[ \theta, d, a, \alpha ]$ entries.
- `from_dh_to_mech()`: DH angles to mechanical angles conversion.
- `from_mech_to_dh()`: mechanical angles to DH angles conversion.
- `root_link`: root link in the chain based on URDF description.

#### Note

`from_dh_to_mech()`, `from_mech_to_dh()`, `nTtool`, `worldTbase` may need to be modified based on your SO100 robot assembly setup.

---

### 2.2 `Utils`

Collection of static methods:

- `inv_homog_mat(T)`: efficiently inverts a 4x4 transformation.
- `calc_lin_err(T1, T2)`: linear position error.
- `calc_ang_err(T1, T2)`: angular error.
- `calc_dh_matrix(dh, θ)`: returns the homogeneous transform using standard DH convention.
- `calc_urdf_joint_transform(joint, q)`: compose parentTjoint + jointTchild(q).
- `calc_urdf_joint_transform_motion_only(joint, q)`: compose jointTchild(q) only.
- `calc_geom_jac(...)`: compute geometrical Jacobian wrt base-frame.
- `calc_geom_jac_n(...)`: compute geometrical jacobian wrt n-frame.
- `dls_right_pseudoinv(...)`: Damped Least Squares pseudoinverse.
- `dls_right_pseudoinv_weighted(J, W, lambda_val=0.001)`: compute Damped Least Squares Right-Pseudo-Inverse weighted.

---

### 2.3 `kinematics`

Main class for computing kinematics:

---

#### `forward_kinematics(...)`

Returns the tool pose in the world frame:

$$
^{world}T_{tool} = ^{world}T_{base} \cdot ^{base}T_n(q) \cdot ^nT_{tool}
$$

---

#### `inverse_kinematics(...)`

Computes inverse kinematics using pose interpolation and inverse Jacobian method. Optional orientation tracking.

$$
q_{k+1} = q_k + J^{\dagger} \cdot K \cdot e
$$

Where:

- $q_k$: joint positions
- $K$: scalar gain
- $J^{\dagger}$: right pseudo-inverse of the Jacobian
- $e$: Cartesian error

---

#### Internal helpers:

- `_forward_kinematics_baseTn`: computes fkine from base-frame to n-frame.
- `_inverse_kinematics_step_baseTn`: performs one step of iterative IK.
- `_interp_init`, `_interp_execute`: pose interpolation (position + orientation via SLERP).
- `calc_geom_jacobian()`: compute geometric Jacobian.
- `check_joint_limits()`: check joint limits.

---

### 2.4 `dynamics`

Main class (**available for DH only**) for computing inverse dynamics and retrieving dynamic model components of a serial-link manipulator:

---

#### `inverse_dynamics(...)`

Computes joint torques using the **Newton-Euler recursive algorithm**:

$$
B(q)\ddot{q} + C(q, \dot{q})\dot{q} + G(q) = \tau + J^{T}F_{ext}
$$

- Performs forward and backward recursion over links
- Returns torques at each joint
- Supports external wrench $F_{ext}$ expressed in the last link frame

---

#### `get_B(...)`

Computes the **inertia matrix** $B(q) \in \mathbb{R}^{n \times n}$

---

#### `get_G(...)`

Returns the **gravity torque vector** $G(q)$. It captures torques at each joint due to the weight of the links.

---

#### `get_Cqdot(...)`

Returns the **Coriolis and centrifugal forces** contribution $C(q, \dot{q})\dot{q}$

---

#### `get_robot_model(...)`

Returns the full dynamic model components:

- $B(q)$: Inertia matrix  
- $C(q, \dot{q})\dot{q}$: Coriolis and centrifugal term  
- $G(q)$: Gravity vector

---

#### `transform_force(...)`

Express a Cartesian force $F_{ext} \in \mathbb{R}^6$ from a source frame to a target frame.

---

## 3. DH representation

DH (Denavit–Hartenberg) frames provide a systematic and compact way to model the kinematics of robotic arms. Each joint and link transformation is represented by a standard set of parameters, allowing for consistent and scalable computation of **forward kinematics, inverse kinematics and Jacobians**.

Each robot uses the **standard DH convention**:

- $\theta$: variable joint angle (actuated)
- $d, a, \alpha$: constant link parameters defined by the mechanical structure

Once the DH table is ready, the homogeneous transformation from frame( i-1 ) to frame( i ) is:

$$
^{i-1}A_{i} =
\begin{bmatrix}
\cos\theta & -\sin\theta\cos\alpha & \sin\theta\sin\alpha & a\cos\theta \\
\sin\theta & \cos\theta\cos\alpha & -\cos\theta\sin\alpha & a\sin\theta \\
0 & \sin\alpha & \cos\alpha & d \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

Finally, the forward kinematics is computed as:

$$
^{base}T_{n}(q) = A_1(\theta_1) \cdot A_2(\theta_2) \cdot \dots \cdot A_n(\theta_n)
$$

<p align="center">
  <img src="./images/dh1.png" alt="DH"/><br>
  <em>Figure: DH frames and DH table computed for the SO100 robotic arm</em>
</p>

<p align="center">
  <img src="./images/dh2.png" alt="DH"/><br>
  <em>Figure: DH angles Vs mechanical angles</em>
</p>

---

## 4. URDF representation

URDF (Unified Robot Description Format) provides a structured way to describe the **geometry and kinematics** of a robot. Each joint defines a rigid-body transformation between a **parent link** and a **child link**, capturing both fixed offsets and variable motion.

Each `<joint>` element includes:

- `<origin xyz="x y z" rpy="r p y">`: specifies the **fixed transformation** from the parent link to the joint frame.  
  - `xyz` defines the translation vector `[x, y, z]`  
  - `rpy` defines the rotation about the roll–pitch–yaw axes `[r, p, y]` (in radians)
- `<axis xyz="a_x a_y a_z">`: specifies the **motion axis** (rotation or translation) expressed in the joint frame.
- `<type>`: determines the motion type:
  - `revolute`: rotation around the axis  
  - `prismatic`: translation along the axis  
  - `fixed`: no motion  

### 4.1 Transformation Composition

The homogeneous transformation from the **parent link** frame to the **child link** frame is defined as:

$$
^{parent}T_{child}(q) = ^{parent}T_{joint} \cdot ^{joint}T_{child}(q)
$$

where:

$$
^{parent}T_{joint} =
\begin{bmatrix}
R_{rpy} & p_{xyz} \\
0 & 1
\end{bmatrix}
$$

- $R_{rpy}$ is the rotation matrix obtained from the roll–pitch–yaw angles.  
- $p_{xyz} = [x, y, z]^T$ is the translation vector.

### 4.2 Revolute Joint

For a **revolute** joint, the relative motion between the joint and the child link is a rotation of angle $q$ about the given axis $a = [a_x, a_y, a_z]^T$:

$$
^{joint}T_{child}(q) =
\begin{bmatrix}
\exp([\hat{a}]\,q) & 0 \\
0 & 1
\end{bmatrix}
$$

where $\exp([\hat{a}]\,q)$ represents the rotation matrix using the exponential map (Rodrigues’ formula).

### 4.3 Prismatic Joint

For a **prismatic** joint, the motion corresponds to a translation of magnitude $q$ along the joint axis $a = [a_x, a_y, a_z]^T$:

$$
^{joint}T_{child}(q) =
\begin{bmatrix}
I_{3} & a\,q \\
0 & 1
\end{bmatrix}
$$

### 4.4 Fixed Joint

For a **fixed** joint (no motion), the transformation is purely static:

$$
^{joint}T_{child}(q) = I_{4}
$$

### 4.5 Summary

The complete forward kinematic chain in URDF can thus be expressed as the product of all joint transforms from the base to the end-effector:

$$
^{base}T_{ee}(q) = \prod_{i=1}^{N} \ ^{parent_i}T_{child_i}(q_i)
$$

where each transformation includes both the fixed origin offset and the variable motion due to the joint’s type.

---

## 5. Full contributions

### 5.1 Kinematics:

- Forward kinematics using DH/URDF
- Jacobian computation using DH/URDF
- Inverse kinematics using Jacobian and dump-least square method to avoid singularities
- Pose interpolation: linear (position) + SLERP (orientation)
- Out of Bound joint position limits checker

### 5.2 Dynamics (DH representation only):

- Inverse Dynamics via Recursive Newton-Euler equations
- Estimate M, C, g of the full robot dynamic model
- Transform forces between frames

---

## 6. Known issues

- In `main_dh.py`, the methods `convert_dh_to_mech()` and `convert_mech_to_dh()` may need to be adjusted depending on your current SO100 assembly setup.
- MuJoCo and the URDF loader may load joint names in a different order. Joint ordering must therefore be aligned manually. You can verify this by printing the joint names from both sources and comparing them visually. See `simulator/main_so101_mj.py` for a reference example.

---

## 7. References

- **B. Siciliano, L. Sciavicco, L. Villani, G. Oriolo (2009)**: *Robotics: Modelling, Planning and Control*
- **P.I. Corke (2017)**: [*A Robotics Toolbox for MATLAB*](https://petercorke.com/toolboxes/robotics-toolbox/)
- **Samuel R. Buss (2009)**: [*Inverse Kinematics with Pseudoinverse and Damped Least Squares methods*](https://www.cs.cmu.edu/~15464-s13/lectures/lecture6/iksurvey.pdf)
- **Jin S. Ahn, Won J. Chung**: [SLERP Interpolation for Orientation](https://worldcomp-proceedings.com/proc/p2011/CSC3066.pdf)

---

## License

This project is licensed under the MIT License – see the [LICENSE](https://github.com/Argo-Robot/kinematics/blob/main/LICENSE) file for details.
