import casadi as ca
import numpy as np
import matplotlib.pyplot as plt

class Omni:
    def __init__(self, dt: float = 0.1):
        """
        Initialize the 6D omni-directional kinematic model.
        Args:
            dt: Time step for discrete simulation
        """
        # State and control dimensions
        self.state_dim = 7
        self.control_dim = 6
        self.dt = dt
        
        
        self.quaternion_rotation()
        self.quat_derivative()
        # import_rot = ca.Importer('rotation.c', 'shell')
        # self.rotation = ca.external('rotation', import_rot)
        
    def kinematics(self, state: ca.SX, controls: ca.SX) -> ca.SX:
        """
        Define the kinematic model of the omni-directional robot.
        Args:
            p: Current position as [x, y, z]
            q: Current orientation as [q0, q1, q2, q3], where q0 is the scalar part
                and q1, q2, q3 are the vector part.
            v_b: Linear velocity in body frame
            omega_b: Angular velocity in body frame
        Returns:
            State derivative as [vx, vy, qx_dot, qy_dot, qz_dot, qw_dot]
        """
        # Extract position and orientation
        x, y, z, q0, q1, q2, q3 = ca.vertsplit(state)
        p = ca.vertcat(x, y, z)
        q = ca.vertcat(q0, q1, q2, q3)
        
        # Extract control inputs
        v_x, v_y, v_z, w_x, w_y, w_z = ca.vertsplit(controls)
        omega_b = ca.vertcat(w_x, w_y, w_z)
        
        # Compute the state derivative
        p_dot = self.rotation(q) @ ca.vertcat(v_x, v_y, v_z)
        q_dot = self.quat_der(q, omega_b)
        
        return ca.vertcat(p_dot, q_dot)
      
    def quat_derivative(self) -> None: 
        """
        Compute the derivative of the quaternion.
        Args:
            q: Current quaternion
            omega: Angular velocity in body frame
        Returns:
            Quaternion derivative
        """
        q = ca.SX.sym('q', 4)  # Quaternion
        omega = ca.SX.sym('omega', 3)  # Angular velocity
        
        # Extract components
        q0, q1, q2, q3 = ca.vertsplit(q)
        wx, wy, wz = ca.vertsplit(omega)
        
        # Quaternion derivative formula
        # q_dot = 0.5 * Omega(omega) * q
        # where Omega(omega) is an operator matrix that represents the angular velocity
        # in quaternion space.
        q_dot = 0.5 * ca.vertcat(
            -q1 * wx - q2 * wy - q3 * wz,
            q0 * wx + q2 * wz - q3 * wy,
            q0 * wy - q1 * wz + q3 * wx,
            q0 * wz + q1 * wy - q2 * wx
        )
        
        self.quat_der = ca.Function('quat_der', [q, omega], [q_dot])
    
    def quaternion_rotation(self) -> None:
        """
        Compute the rotation matrix from body frame to world frame using quaternion.
        Generate the C file that computes the rotation matrix.
        """
        p = ca.SX.sym('p', 3)  # Position vector
        q = ca.SX.sym('q', 4)  # Quaternion
        
        q0, q1, q2, q3 = ca.vertsplit(q)
        
        # Rotation matrix from quaternion
        R = ca.SX.zeros(3, 3)
        R[0, 0] = 1 - 2 * (q2**2 + q3**2)
        R[0, 1] = 2 * (q1 * q2 - q0 * q3)
        R[0, 2] = 2 * (q1 * q3 + q0 * q2)
        R[1, 0] = 2 * (q1 * q2 + q0 * q3)
        R[1, 1] = 1 - 2 * (q1**2 + q3**2)
        R[1, 2] = 2 * (q2 * q3 - q0 * q1)
        R[2, 0] = 2 * (q1 * q3 - q0 * q2)
        R[2, 1] = 2 * (q2 * q3 + q0 * q1)
        R[2, 2] = 1 - 2 * (q1**2 + q2**2)
        
        self.rotation = ca.Function('rotation', [q], [R])
        # rotation.generate('rotation.c')
        
    def plot_trajectory(self, trajectory):
        """
        Plot the trajectory in 3D space.
        Args:
            trajectory: Array of shape (N, 3) representing the trajectory
        """
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        
        # Extract x, y, z coordinates
        x = trajectory[:, 0]
        y = trajectory[:, 1]
        z = trajectory[:, 2]
        
        # Plot the trajectory
        ax.plot(x, y, z)
        
        # Set labels
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        
        plt.show()
    
    
        
        
        
        