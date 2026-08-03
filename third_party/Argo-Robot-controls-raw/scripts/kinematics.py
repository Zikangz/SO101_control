import copy
import numpy as np
from enum import Enum
from scipy.spatial.transform import Rotation as R

from scripts.utils import *



class ReferenceFrame(Enum):
    BASE = 0
    LOCAL = 1


class RobotKinematics:
    
    """Parent class to define methods common to both DH and URDF representations"""

    def forward_kinematics(self, robot, q, target_link_name=None):
        
        """compute forward kinematics (worldTtool)"""

        baseTn = self._forward_kinematics_baseTn(robot, q, target_link_name)

        return robot.worldTbase @ baseTn @ robot.nTtool

    def _forward_kinematics_baseTn(self, robot, q, target_link_name):
        
        """compute forward kinematics (baseTn)"""
        
        raise NotImplementedError

    def calc_geom_jacobian(self, robot, q, target=None, reference_frame=ReferenceFrame.BASE):
        
        """Compute geometric Jacobian for target (link or frame) wrt BASE or LOCAL frame"""

        raise NotImplementedError

    def inverse_kinematics(self, robot, q_start, desired_worldTtool, target_link_name=None, use_orientation=True, k=0.8, n_iter=50):
        
        """compute inverse kinematics (T_desired must be expressed in worldTtool)
        It is performed an interpolation both for linear and angular components"""

        # I compute ikine with baseTn
        desired_baseTn = (RobotUtils.inv_homog_mat(robot.worldTbase)
                          @ desired_worldTtool
                          @ RobotUtils.inv_homog_mat(robot.nTtool))

        # don't override current joint positions
        q = copy.deepcopy(q_start)

        # init interpolator
        n_steps = self._interp_init(self._forward_kinematics_baseTn(robot, q, target_link_name), desired_baseTn)

        for i in range(0, n_steps + 1):

            # current setpoint as baseTn
            T_desired_interp = self._interp_execute(i)

            # get updated joint positions
            q = self._inverse_kinematics_step_baseTn(robot, q, T_desired_interp, target_link_name, use_orientation, k, n_iter)
        
        # check final error
        current_worldTtool = self.forward_kinematics(robot, q, target_link_name)
        err_lin = RobotUtils.calc_lin_err(current_worldTtool, desired_worldTtool)
        lin_error_norm = np.linalg.norm(err_lin)
        assert lin_error_norm < 1e-2, (f"[ERROR] Large position error ({lin_error_norm:.4f}). Check target reachability (position/orientation)")

        return q
    
    def _inverse_kinematics_step_baseTn(self, robot, q_start, T_desired, target_link_name=None, use_orientation=True, k=0.8, n_iter=50):
        
        """compute inverse kinematics (T_desired must be expressed in baseTn)"""

        # don't override current joint positions
        q = copy.deepcopy(q_start)

        for _ in range(n_iter):
            # compute current pose baseTn
            T_current = self._forward_kinematics_baseTn(robot, q, target_link_name)

            # compute linear error
            err_lin = RobotUtils.calc_lin_err(T_current, T_desired)

            # decide whether to use full jacobian or not
            if use_orientation:
                err_ang = RobotUtils.calc_ang_err(T_current, T_desired)  # compute angular error
                error = np.concatenate((err_lin, err_ang))  # total error
                J_geom = self.calc_geom_jacobian(robot, q, target_link_name)  # full jacobian
            else:
                error = err_lin  # total error
                J_geom = self.calc_geom_jacobian(robot, q, target_link_name)[:3, :]  # take only the position part

            # stop if error is minimum
            if np.linalg.norm(error) < 1e-5:
                break

            # Damped Least Squares Right-Pseudo-Inverse
            J_pinv = RobotUtils.dls_right_pseudoinv(J_geom)

            # keep integrating resulting joint positions
            q += k * (J_pinv @ error)

        return q

    def check_joint_limits(self, robot, q_vec): 
    
        """raise an error in case mechanical joint limits are exceeded"""

        for i, q in enumerate(q_vec):
            assert robot.mech_joint_limits_low[i] <= q <= robot.mech_joint_limits_up[i], (f"[ERROR] Joint limits out of bound. J{i + 1} = {q}, but limits are ({robot.mech_joint_limits_low[i]}, {robot.mech_joint_limits_up[i]})")

    def _interp_init(self, T_start, T_final, freq = 100, trans_speed = 0.1, rot_speed = 0.5):
        
        """Initialize interpolation parameters"""

        self.t_start = T_start[:3, 3]
        self.t_final = T_final[:3, 3]
        R_start = T_start[:3, :3]
        R_final = T_final[:3, :3]
        self.q_start = R.from_matrix(R_start).as_quat() 
        self.q_final = R.from_matrix(R_final).as_quat()  

        # linear distance
        trans_dist = RobotUtils.calc_distance(self.t_final, self.t_start)
        t_trans = trans_dist / trans_speed

        # angular distance
        rotvec = R.from_matrix(R_final @ R_start.T).as_rotvec()
        ang_dist = np.linalg.norm(rotvec)
        t_rot = ang_dist / rot_speed

        # total time & corresponding n_steps
        total_time = max(t_trans, t_rot)
        self.n_steps = int(np.ceil(freq*total_time))

        # normalize quaternions
        self.q_start = self.q_start / np.linalg.norm(self.q_start)
        self.q_final = self.q_final / np.linalg.norm(self.q_final)

        # check angle between the two quaternions
        dot = np.dot(self.q_start, self.q_final)

        # enforce shortest path for SLERP
        if np.dot(self.q_start, self.q_final) < 0.0:
            self.q_final = -self.q_final  
            dot = -dot

        # clamp to avoid NaNs
        dot = np.clip(dot, -1.0, 1.0)

        # compute angle
        self.theta = np.arccos(dot)
        self.sin_theta = np.sin(self.theta)

        return self.n_steps
    
    def _interp_execute(self, i):
        
        """Compute Cartesian pose setpoint for the current step"""

        # n_steps == 0 means Tgoal == Tinit
        # In this way I also avoid division by zero
        if self.n_steps == 0:
            s = 1.0
        else:
            s = i / self.n_steps  # compute current step

        # interpolate position
        t_interp = (1 - s) * self.t_start + s * self.t_final
        
        # interpolate orientation with SLERP (handle near-zero rotation difference where sin_theta ≈ 0)
        if np.isclose(self.sin_theta, 0.0):
            q_interp = self.q_final
        else:
            theta_t = self.theta * s
            w0 = np.sin(self.theta - theta_t) / self.sin_theta
            w1 = np.sin(theta_t) / self.sin_theta
            q_interp = w0 * self.q_start + w1 * self.q_final

        # normalize
        q_interp = q_interp / np.linalg.norm(q_interp)
        R_interp = R.from_quat(q_interp).as_matrix()

        # compute current setpoint
        T_interp = np.eye(4)
        T_interp[:3, :3] = R_interp
        T_interp[:3, 3] = t_interp

        return T_interp
    



class DH_Kinematics(RobotKinematics):

    """Child class in charge to override methods specific for DH representation"""

    def __init__(self):

        super().__init__()  # Initialize RobotKinematics

    def _forward_kinematics_baseTn(self, robot, q, target_link_name=None):
        
        """compute forward kinematics (baseTn)"""

        T = np.eye(4)
        DOF = len(q)

        # loop over DH frames
        for i in range(DOF):
            dh = robot.dh_table[i]
            T_link = RobotUtils.calc_dh_matrix(dh, q[i])
            T = T @ T_link

        return T
    
    def calc_geom_jac_0(self, robot, q, target_link_name=None):
        
        """compute geometrical jacobian for base-frame wrt base-frame"""

        DOF = len(q)
        J = np.zeros((6, DOF))
        P = np.zeros((3, DOF+1))
        z = np.zeros((3, DOF+1))
        base_P_i = np.eye(4)
        
        # define z0
        z[:,0] = np.array([0,0,1])
        
        for i in range(DOF):
            
            # get Pi and zi
            i_T_ip1 = RobotUtils.calc_dh_matrix(robot.dh_table[i], q[i])
            base_P_i = base_P_i @ i_T_ip1
            P[:,i+1] = base_P_i[:3,3]
            z[:,i+1] = base_P_i[:3,2]
            
        for i in range(DOF):
            
            # compose jacobian matrix
            J[:3, i] = np.cross(z[:,i] , P[:,DOF] - P[:,i])
            J[3:, i] = z[:,i] 
            
        return J
    
    def calc_geom_jac_n(self, robot, q, base_T_n):
        
        """compute geometrical jacobian for n-frame wrt n-frame"""
        
        DOF = len(q)
        Jn = np.zeros((6, DOF))
        T = np.zeros((6,6))
        R = base_T_n[:3,:3]
        
        # get jacobian in base frame
        J0 = self.calc_geom_jac_0(robot, q)
        
        # compose mapping matrix
        T[:3,:3] = R.T
        T[3:,3:] = R.T
        
        # compute jacobian in n-frame
        Jn = T @ J0
        
        return Jn
                          
    def calc_geom_jacobian(self, robot, q, target=None, reference_frame=ReferenceFrame.BASE):
        
        """Compute geometric Jacobian for n-frame (target) wrt BASE or LOCAL frame"""
        
        if (reference_frame == ReferenceFrame.LOCAL):
            J = self.calc_geom_jac_n(robot, q, target)
        elif (reference_frame == ReferenceFrame.BASE):
            J = self.calc_geom_jac_0(robot, q)

        return J
    



class URDF_Kinematics(RobotKinematics):

    """Child class in charge to override methods specific for URDF representation"""

    def __init__(self):

        super().__init__()  # Initialize RobotKinematics


    def _forward_kinematics_baseTn(self, robot, q, target_link_name="base_link"):
        
        """
        Compute forward kinematics (baseTn).
        q is the FULL configuration vector (size = n_movable_joints of robot).
        """

        # get full joint chain from base to target (movables + fixed)
        chain = self.get_joint_chain(robot, robot.root_link, target_link_name)

        # build mapping: joint name -> index in q_full
        movable_joints = [j for j in robot.loader.robot.joints if j.joint_type != "fixed"]
        joint_index_map = {j.name: idx for idx, j in enumerate(movable_joints)}

        T = np.eye(4)

        # go through joint chain
        for joint in chain:
            if joint.joint_type == "fixed":
                # just apply fixed joint transform (no q)
                T = T @ RobotUtils.calc_urdf_joint_transform(joint, 0.0)
            else:
                # apply transform with corresponding joint variable
                idx = joint_index_map[joint.name]   # pick correct entry from q_full
                T = T @ RobotUtils.calc_urdf_joint_transform(joint, q[idx])

        return T

    def get_joint_chain(self, robot, base_link_name, target_link_name):

        """Returns the list of joints (movables + fixed) connecting base_link_name to target_link_name"""

        # build link to joint mapping
        link_to_joint = {joint.child: joint for joint in robot.loader.robot.joints}

        # init
        chain = []
        current_link = target_link_name

        # go through the chain (include both movable and not movable joints)
        while (current_link != base_link_name):
            joint = link_to_joint[current_link]
            chain.append(joint)
            current_link = joint.parent  

        chain.reverse()       
        return chain

    def calc_geom_jacobian(self, robot, q, target=None, reference_frame=ReferenceFrame.BASE, full=True):
        
        """
        Compute geometric Jacobian for target link wrt BASE or LOCAL frame.
        q is the FULL configuration vector (size = n_movable_joints of robot).
        It return the full Jacobian by default (6 x n_movable_joints)
        """

        # get chain joints
        chain = self.get_joint_chain(robot, robot.root_link, target)
       
        # mapping q_full to q_chain
        movable_joints_full = [j for j in robot.loader.robot.joints if j.joint_type != "fixed"]
        joint_index_map = {j.name: idx for idx, j in enumerate(movable_joints_full)}
        
        # init
        movables_joints_chain = [j for j in chain if j.joint_type != "fixed"]
        n_dof = len(movable_joints_full) if full == True else len(movables_joints_chain)
        n_chain = len(chain)  # number of joints in the chain (including fixed)

        J = np.zeros((6, n_dof))
        P = np.zeros((3, n_chain + 1))  # origins of joints
        z = np.zeros((3, n_chain + 1))  # axes of joints in base frame
        T = np.eye(4)

        # z0 (z-axis of base)
        z[:, 0] = np.array([0, 0, 1])

        # precompute P and z
        for i, joint in enumerate(chain):

            # move to joint frame (origin only)
            T = T @ joint.origin

            if joint.joint_type != "fixed":
                # joint position in base
                P[:, i + 1] = T[:3, 3].copy()

                # joint axis in base frame
                axis = np.array(joint.axis, dtype=float)
                nrm = np.linalg.norm(axis)
                axis = axis / nrm if nrm > 1e-12 else np.array([0., 0., 1.])
                z[:, i + 1] = T[:3, :3] @ axis

                # apply joint motion
                idx = joint_index_map[joint.name]   # pick correct entry from q_full
                T = T @ RobotUtils.calc_urdf_joint_transform_motion_only(joint, q[idx])
            else:
                # fixed joint: just apply identity motion (no contribution)
                P[:, i + 1] = T[:3, 3].copy()
                z[:, i + 1] = T[:3, 2].copy()

        # end-effector position
        p_end = T[:3, 3].copy()

        # fill Jacobian
        idx = -1

        for i, joint in enumerate(chain):
            
            if joint.joint_type == "fixed":
                continue
            
            # pick correct entry for composing Jacobian columns
            idx = joint_index_map[joint.name] if full==True else idx+1

            if joint.joint_type in ("revolute", "continuous"):
                J[:3, idx] = np.cross(z[:, i + 1], p_end - P[:, i + 1])
                J[3:, idx] = z[:, i + 1]
            elif joint.joint_type == "prismatic":
                J[:3, idx] = z[:, i + 1]
                J[3:, idx] = np.zeros(3)

        # map to LOCAL frame if requested
        if reference_frame == ReferenceFrame.LOCAL:
            R = T[:3, :3]
            T_map = np.zeros((6, 6))
            T_map[:3, :3] = R.T
            T_map[3:, 3:] = R.T
            J = T_map @ J

        return J