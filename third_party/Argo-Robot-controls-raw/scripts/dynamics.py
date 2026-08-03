import copy
import numpy as np

from scripts.kinematics import *    


class RobotDynamics:
    
    def inverse_dynamics(self, robot, q, qdot, qddot, Fext = np.zeros(6), gravity_on = True):
        
        """compute joint torques taking into account full dynamic model: B*qddot + C*qdot + G = tau
        NOTE: Fext must be expressed in n-frame"""
        
        # init
        N_DOF = len(q)
        
        # precompute i_A_ip1 matrices
        A = np.zeros((N_DOF,4,4))
        for i in range(0, N_DOF):
            A[i] = RobotUtils.calc_dh_matrix(robot.dh_table[i], q[i])
        
        ## forward recursion ##
        
        # init
        w = np.zeros((N_DOF+1,3)) # angular velocity link i
        wdot = np.zeros((N_DOF+1,3)) # angular acceleration link i
        a = np.zeros((N_DOF+1,3)) # acceleration origin of frame i
        a_com  = np.zeros((N_DOF+1,3)) # acceleration com link i
        
        # init values
        w[0] = np.zeros(3)
        wdot[0] = np.zeros(3)
        a[0] = np.array([0.0, 0.0, -9.81]) if gravity_on else np.array([0.0, 0.0, 0.0])
        z0 = np.array([0.0, 0.0, 1.0]) 
        
        # loop over links
        for i in range(1, N_DOF+1):
            
            # extract useful quantities            
            ri = robot.link_com[i] # com link i wrt frame i
            im1_A_i = A[i-1] 
            Ri = im1_A_i[:3,:3] # taken from i-1_A_i matrix
            Rim1 = Ri.T # i_R_i-1
            t_wrt_i = Rim1 @ im1_A_i[:3,3] # vector from O_i-1 to Oi, expressed wrt Oi
    
            # compute velocities for each link
            w[i] = Rim1 @ (w[i-1] + z0 * qdot[i-1])
            wdot[i] = Rim1 @ (wdot[i-1] + z0 * qddot[i-1] + np.cross(w[i-1], (z0 * qdot[i-1])))
            a[i] = Rim1 @ a[i-1] + np.cross(wdot[i], t_wrt_i) + np.cross(w[i], np.cross(w[i], t_wrt_i))  
            a_com[i] = a[i] + np.cross(wdot[i], ri) + np.cross(w[i], np.cross(w[i], ri))
        
        ## backward recursion ##
        
        # init
        f = np.zeros((N_DOF+1,3)) # force on link i at its center of mass, in frame i
        t = np.zeros((N_DOF+1,3)) # torque on link i about its COM, in frame i
        F = np.zeros((N_DOF+2,3)) # total force from link i+1 acting on link i, at origin of frame i
        T = np.zeros((N_DOF+2,3)) # total torque from link i+1 acting on link i, about origin of frame i
        torques = np.zeros(N_DOF)
        
        # set external force
        F[N_DOF+1] = Fext[:3]
        T[N_DOF+1] = Fext[3:]
                
        # loop over links
        for i in range(N_DOF, 0, -1): 
            
            # extract useful quantities
            mi = robot.link_mass[i] # mass link i
            Ii = robot.link_inertia[i].reshape(3,3) # inertia link i defined wrt frame i
            ri = robot.link_com[i] # com link i wrt frame i
           
            # extract Ri, Pi
            i_A_ip1 = A[i-1]
            Rip1 = i_A_ip1[:3, :3]  # rotation from frame i to i+1
            Pip1 = i_A_ip1[:3, 3]   # translation from frame i to i+1
            
            # compute forces and torques for each link
            t[i] = Ii @ wdot[i] + np.cross(w[i], Ii @ w[i]) # Euler’s equation for torque about the COM
            f[i] = mi * a_com[i] # Newton second law
            F[i] = Rip1 @ f[i] + Rip1 @ F[i+1]  # total force acting on origin i of link i
            T[i] = Rip1 @ t[i] + Rip1 @ T[i+1] + np.cross(Pip1, Rip1 @ F[i+1]) + np.cross(Rip1 @ ri + Pip1, Rip1 @ f[i])  # total torque acting on origin i of link i     
        
        # extract joint torques 
        torques[:] = T[1:N_DOF+1,2]
        
        return torques           
    
    def get_B(self, robot, q):
        
        """compute B matrix (inertia terms)"""
        
        N_DOF = len(q)
        B = np.zeros((N_DOF, N_DOF))
        
        # build B matrix by columns
        for i in range(0,N_DOF):
            vddot = np.zeros(N_DOF)
            vddot[i] = 1.0
            torques = self.inverse_dynamics(robot, q, np.zeros(N_DOF), vddot, gravity_on = False)
            B[:,i] = torques
        
        return B
    
    def get_G(self, robot, q):
        
        """compute G vector (gravity terms)"""
        
        N_DOF = len(q)
        
        # torques = G
        torques = self.inverse_dynamics(robot, q, np.zeros(N_DOF), np.zeros(N_DOF))
        
        return torques.reshape(-1,1)
    
    def get_Cqdot(self, robot, q, qdot):
        
        """compute C*qdot matrix (coriolis and damping terms)"""
        
        N_DOF = len(q)
        
        # torques = C*qdot + G
        torques = self.inverse_dynamics(robot, q, qdot, np.zeros(N_DOF))
        Cqdot = torques.reshape(-1,1) - self.get_G(robot, q)
        
        return Cqdot
    
    def get_robot_model(self, robot, q, qdot):
        
        """compute full robot model (B, C*qdot, G)"""
        
        N_DOF = len(q)
        
        # get B
        B = self.get_B(robot, q)
        
        # get G
        G = self.get_G(robot, q)
        
        # get C*qdot
        torques = self.inverse_dynamics(robot, q, qdot, np.zeros(N_DOF))
        Cqdot = torques.reshape(-1,1) - G
        
        return B, Cqdot, G
    
    def transform_force(self, f_ext, from_T_to):
        
        """transform force in target frame
        NOTE: f_ext must be expressed in "from" frame"""
        
        f_ext_new = np.zeros(6)
        R = from_T_to[:3,:3]
        t = from_T_to[:3,3]
        
        # express force in new frame
        f_ext_new[:3] = R @ f_ext[:3]
        f_ext_new[3:] = R @ f_ext[3:] + np.cross(t, f_ext_new[:3])
        
        return f_ext_new   

