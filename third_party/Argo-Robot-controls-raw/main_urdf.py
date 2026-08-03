import os

from scripts.model import *
from scripts.kinematics import *
from scripts.dynamics import *


## URDF Loader ##

print("\n\nLOADING ROBOT MODEL:\n\n")

# Get directory where this script is located
script_dir = os.path.dirname(os.path.abspath(__file__))

# Construct relative path to URDF
urdf_path = os.path.join(script_dir, "models", "so101", "so101.urdf") 

# Instantiate URDF loader
urdf_loader = URDF_loader()
urdf_loader.load(urdf_path)

# Wrap in RobotModel
robot_model = RobotModel(urdf_loader)

# Info
robot_model.print_model_properties()
print("Number of joints: ", robot_model.get_n_joints())
print("Number of links: ", robot_model.get_n_links())

## kinematics ##

print("\n\nKINEMATICS EXAMPLE:\n\n")

# init
kin = URDF_Kinematics()

# define ee target link name
ee_name = "gripper_link"

# get current joint positions (only movable joints, no fixed joints and no floating base pose) 
q_init = np.zeros(6) 
print("q_init: ", np.rad2deg(q_init))

# compute start pose
T_start = kin.forward_kinematics(robot_model, q_init, target_link_name=ee_name) 
print("\nT_start = \n", T_start)

# Define relative goal pose
T_goal = T_start.copy()
T_goal[:3, 3] += np.array([-0.1, 0.0, 0.1])
print("\nT_goal = \n", T_goal)

# # Define absolute goal pose
# T_goal = np.array([
#     [1.0, 0.0, 0.0,  0.0],
#     [0.0, 1.0, 0.0, -0.17],
#     [0.0, 0.0, 1.0,  0.37],
#     [0.0, 0.0, 0.0,  1.0]
# ])
# print("\nT_goal = \n", T_goal)

# IK with internal interpolation
q_final = kin.inverse_kinematics(robot_model, q_init, T_goal, target_link_name=ee_name, use_orientation=True, k=0.8, n_iter=1) 
print("\nFinal joint angles = ", q_final)

T_final = kin.forward_kinematics(robot_model, q_final, target_link_name=ee_name)  
print("\nFinal pose direct kinematics = \n", T_final)

print("\nerr_lin = ", RobotUtils.calc_lin_err(T_goal, T_final))
print("err_ang = ", RobotUtils.calc_ang_err(T_goal, T_final))

# raise an error in case joint limits are exceeded
kin.check_joint_limits(robot_model, q_final)

print("\n\nJACOBIANS EXAMPLE:\n\n")

# Print chain list from base_link to flange
chain = kin.get_joint_chain(robot_model, robot_model.root_link, ee_name)
for idx, joint in enumerate(chain):
    print(idx, joint.name, joint.parent, joint.child)

# jacobians
q = np.array([0.1, 0.2, 0.1, 0.2, 0.1, 0.2]) 
base_T_n = kin._forward_kinematics_baseTn(robot_model, q, target_link_name=ee_name)
J0 = kin.calc_geom_jacobian(robot_model, q, target=ee_name)
Jn = kin.calc_geom_jacobian(robot_model, q, target=ee_name, reference_frame=ReferenceFrame.LOCAL)
print("\ngeometric jacobian in base-frame: \n", J0)
print("\ngeometric jacobian in n-frame: \n", Jn)

