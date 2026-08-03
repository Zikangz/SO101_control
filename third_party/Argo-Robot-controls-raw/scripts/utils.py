import numpy as np
from scipy.spatial.transform import Rotation as R


class RobotUtils:
    
    @staticmethod
    def calc_distance(p1, p2):
        
        """compute distance between two 3D vectors"""

        return np.linalg.norm(p2 - p1)

    @staticmethod
    def inv_homog_mat(T):
        
        """invert homogenous transformation matrix"""

        R = T[:3, :3]
        t = T[:3, 3]
        T_inv = np.eye(4)
        T_inv[:3, :3] = R.T
        T_inv[:3, 3] = -R.T @ t
        return T_inv

    @staticmethod
    def calc_lin_err(T_current, T_desired):
        
        """compute linear error between 2 homogenous transformations"""

        return T_desired[:3, 3] - T_current[:3, 3]

    @staticmethod
    def calc_ang_err(T_current, T_desired):
        
        """compute angular error between two homogenous transformations (axis-angle notation)"""

        R_current = T_current[:3, :3]
        R_desired = T_desired[:3, :3]
        return 0.5 * (np.cross(R_current[:, 0], R_desired[:, 0])
                + np.cross(R_current[:, 1], R_desired[:, 1])
                + np.cross(R_current[:, 2], R_desired[:, 2]))
    
    @staticmethod
    def calc_dh_matrix(dh, theta):
    
        """compute dh matrix"""

        _, d, a, alpha = dh
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)

        return np.array([[ct, -st * ca, st * sa, a * ct], 
                         [st, ct * ca, -ct * sa, a * st], 
                         [0, sa, ca, d], 
                         [0, 0, 0, 1]]) 

    @staticmethod
    def calc_urdf_joint_transform(joint, q):
        
        """compose parentTjoint + jointTchild(q)"""

        # extract origin (translation + rotation)
        T_origin = joint.origin
            
        # compose fixed origin transform and motion transform
        return T_origin @ RobotUtils.calc_urdf_joint_transform_motion_only(joint, q)
    
    @staticmethod
    def calc_urdf_joint_transform_motion_only(joint, q):
        
        """compose jointTchild(q) only"""

        # start from identity matrix
        T_motion = np.eye(4)
        
        if joint.joint_type == "revolute" or joint.joint_type == "continuous":
            axis = np.array(joint.axis)  # axis direction: [x, y, z]
            T_motion[:3, :3] = R.from_rotvec(axis * q).as_matrix() # axis-angle representation
        elif joint.joint_type == "prismatic":
            axis = np.array(joint.axis)
            T_motion[:3, 3] = axis * q
            
        # for fixed joints, T_motion stays identity
        return T_motion
    
    @staticmethod
    def dls_right_pseudoinv(J, lambda_val=0.001):
        
        """compute Damped Least Squares Right-Pseudo-Inverse"""
               
        JT = J.T
        JTJ = JT @ J
        J_pinv = np.linalg.inv(JTJ + lambda_val * np.eye(JTJ.shape[0])) @ JT  
        
        return J_pinv

    @staticmethod
    def dls_right_pseudoinv_weighted(J, W, lambda_val=0.001):
        
        """compute Damped Least Squares Right-Pseudo-Inverse weighted"""
               
        JT = J.T
        JTJ = JT @ J
        J_pinv = np.linalg.inv(JTJ + lambda_val * W) @ JT 
        
        return J_pinv