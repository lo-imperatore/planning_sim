import casadi as ca
import numpy as np
import matplotlib.pyplot as plt

class Omni:
    def __init__(self, position=None, orientation=None):
        """
        Initialize the 6D omni-directional kinematic model.
        Args:
            dt: Time step for discrete simulation
        """
        # State and control dimensions
        self.state_dim = 7
        self.control_dim = 6
        
        self.quaternion_rotation()
        self.quat_derivative()
        
        self.omni_kinematics()
        
        if position is not None and orientation is not None:
            # Initial state
            self.position = position
            self.orientation = orientation
        
    def omni_kinematics(self) -> None:
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
        state = ca.SX.sym('state', self.state_dim)
        controls = ca.SX.sym('controls', self.control_dim)
        
        # Extract position and orientation
        x, y, z, q0, q1, q2, q3 = ca.vertsplit(state)
        p = ca.vertcat(x, y, z)
        q = ca.vertcat(q0, q1, q2, q3)
        
        # Extract control inputs
        v_x, v_y, v_z, w_x, w_y, w_z = ca.vertsplit(controls)
        omega_b = ca.vertcat(w_x, w_y, w_z)

        v_y_zero = ca.SX(0)
        
        # Compute the state derivative
        p_dot = self.rotation(q) @ ca.vertcat(v_x, v_y, v_z)
        # p_dot = ca.vertcat(v_x, v_y, v_z)  # Linear velocity in world frame
        q_dot = self.quat_der(q, omega_b)
        
        # Build the kinematic model
        self.update_kinematics = ca.Function('kinematics', [state, controls], [ca.vertcat(p_dot, q_dot)])
        
        # Propagate the state using Runge-Kutta 4th order method
        X0 = ca.SX.sym('X0', self.state_dim)
        U = ca.SX.sym('U', self.control_dim)
        dT = ca.SX.sym('dT')
        M = 4
        X = X0
        Q = 0
        h = dT / M
        for j in range(M):
            k1 = self.update_kinematics (X, U)
            k2 = self.update_kinematics (X + h/2 * k1, U)
            k3 = self.update_kinematics (X + h/2 * k2, U)
            k4 = self.update_kinematics (X + h * k3, U)
            X = X+h/6*(k1 + 2*k2 + 2*k3 + k4)
            
        self.propagate_state = ca.Function('propagate', [X0, U, dT], [X])
      
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

class Unicycle():
    def __init__(self, position=None, orientation=None):
        """
        Initialize the 3D unicycle kinematic model.
        Args:
            position: Initial position [x, y, z]
            orientation: Initial orientation [q0, q1, q2, q3] (in radians)
        """
        # State and control dimensions
        self.state_dim = 7  # [x, y, z, q0, q1, q2, q3]
        self.control_dim = 6  # [v_x, v_y, v_z, w_x, w_y, w_z]
        
        self.quaternion_rotation()
        
        # Initialize kinematics
        self.uni_kinematics()
        
        if position is not None and orientation is not None:
            # Initial state
            self.position = position
            self.orientation = orientation
        
    def uni_kinematics(self) -> None:
        """
        Define the kinematic model of the unicycle robot.
        State: [x, y, z, q0, q1, q2, q3] - position and orientation
        Controls: [v_x, v_y, v_z, w_x, w_y, w_z] - linear velocity and angular velocity, with only v_x and w_z used
        """
        state = ca.SX.sym('state', self.state_dim)
        controls = ca.SX.sym('controls', self.control_dim)
        
        # Extract position and orientation
        x, y, z, q0, q1, q2, q3 = ca.vertsplit(state)
        p = ca.vertcat(x, y, z)
        q = ca.vertcat(q0, q1, q2, q3)
        
        # Extract control inputs
        v_x, v_y, v_z, w_x, w_y, w_z = ca.vertsplit(controls)
        
        # Compute the state derivative
        p_dot = self.rotation(q) @ ca.vertcat(v_x, 0, 0)
        q_dot = w_z / 2 * ca.vertcat(-q3, q2, -q1, q0)  # Quaternion derivative for unicycle
        
        # Build the kinematic model
        self.update_kinematics = ca.Function('kinematics', [state, controls], [ca.vertcat(p_dot, q_dot)])
        
        # Propagate the state using Runge-Kutta 4th order method
        X0 = ca.SX.sym('X0', self.state_dim)
        U = ca.SX.sym('U', self.control_dim)
        dT = ca.SX.sym('dT')
        M = 4
        X = X0
        h = dT / M
        for j in range(M):
            k1 = self.update_kinematics(X, U)
            k2 = self.update_kinematics(X + h/2 * k1, U)
            k3 = self.update_kinematics(X + h/2 * k2, U)
            k4 = self.update_kinematics(X + h * k3, U)
            X = X + h/6 * (k1 + 2*k2 + 2*k3 + k4)
            
        self.propagate_state = ca.Function('propagate', [X0, U, dT], [X])
    
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
    
    def plot_trajectory(self, trajectory, arrow_spacing=10):
        """
        Plot the 2D trajectory with orientation arrows.
        Args:
            trajectory: Array of shape (N, 3) representing the trajectory [x, y, theta]
            arrow_spacing: Display orientation arrow every N steps
        """
        fig, ax = plt.subplots(figsize=(10, 8))
        
        # Extract x, y coordinates and orientation
        x = trajectory[:, 0]
        y = trajectory[:, 1]
        theta = trajectory[:, 2]
        
        # Plot the trajectory
        ax.plot(x, y, 'b-', label='Path')
        
        # Plot orientation arrows
        for i in range(0, len(x), arrow_spacing):
            dx = 0.5 * np.cos(theta[i])
            dy = 0.5 * np.sin(theta[i])
            ax.arrow(x[i], y[i], dx, dy, head_width=0.2, head_length=0.3,
                     fc='red', ec='red')
        
        # Set labels and legend
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_title('Unicycle Model Trajectory')
        ax.grid(True)
        ax.axis('equal')
        
        plt.legend()
        plt.show()
              
class HybridKinematics:
    def __init__(self, position=None, orientation=None, model_type: str = 'unicycle'):
        """
        Initialize a hybrid kinematics model that can switch between omnidirectional and unicycle models.
        
        Args:
            position: Initial position [x, y, z]
            orientation: Initial orientation as [q0, q1, q2, q3]
            model_type: Type of model to use initially ('unicycle' or 'omni')
        """
        self.model_type = model_type.lower()
        
        if position is not None and orientation is not None:
            # Initial state
            self.position = position
            self.orientation = orientation
        
        # Set current state based on model type
        self._update_current_state()
        
    def _euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion."""
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        
        q0 = cy * cp * cr + sy * sp * sr
        q1 = cy * cp * sr - sy * sp * cr
        q2 = cy * sp * cr + sy * cp * sr
        q3 = sy * cp * cr - cy * sp * sr
        
        return [q0, q1, q2, q3]
    
    def _quaternion_to_euler(self, q):
        """Convert quaternion to Euler angles."""
        q0, q1, q2, q3 = q
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (q0 * q1 + q2 * q3)
        cosr_cosp = 1 - 2 * (q1 * q1 + q2 * q2)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (q0 * q2 - q3 * q1)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (q0 * q3 + q1 * q2)
        cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def _update_current_state(self):
        """Update internal state representation based on active model."""
        if self.model_type == 'unicycle':
            self.state_dim = self.unicycle.state_dim
            self.control_dim = self.unicycle.control_dim
        else:  # 'omni'
            self.state_dim = self.omni.state_dim
            self.control_dim = self.omni.control_dim

    @staticmethod
    def _quat_multiply(q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array([
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ])

    @staticmethod
    def _quat_rotate(q, v):
        w, x, y, z = q
        vx, vy, vz = v
        return np.array([
            (1 - 2*y*y - 2*z*z)*vx + (2*x*y - 2*z*w)*vy + (2*x*z + 2*y*w)*vz,
            (2*x*y + 2*z*w)*vx + (1 - 2*x*x - 2*z*z)*vy + (2*y*z - 2*x*w)*vz,
            (2*x*z - 2*y*w)*vx + (2*y*z + 2*x*w)*vy + (1 - 2*x*x - 2*y*y)*vz,
        ])

    @staticmethod
    def integrate_state(state, linear_vel_body, angular_vel_body, dt):
        """Integrate state forward by dt using body-frame velocities."""
        linear_vel_body = np.asarray(linear_vel_body, dtype=float)
        angular_vel_body = np.asarray(angular_vel_body, dtype=float)

        # Update position in world frame
        v_world = HybridKinematics._quat_rotate(state.orientation, linear_vel_body)
        state.position = state.position + v_world * dt

        # Update orientation using small-angle quaternion integration
        omega_norm = np.linalg.norm(angular_vel_body)
        if omega_norm > 1e-8:
            axis = angular_vel_body / omega_norm
            theta = omega_norm * dt
            dq = np.array([np.cos(theta / 2), *(np.sin(theta / 2) * axis)])
            state.orientation = HybridKinematics._quat_multiply(state.orientation, dq)
            state.orientation = state.orientation / np.linalg.norm(state.orientation)

        return state
    
    def switch_model(self, model_type: str):
        """
        Switch between unicycle and omnidirectional models.
        
        Args:
            model_type: Type of model to switch to ('unicycle' or 'omni')
        """
        if model_type.lower() == self.model_type:
            return  # Already using this model
        
        self.model_type = model_type.lower()
        self._update_current_state()
    
    def propagate(self, state, controls, dt):
        """
        Propagate state according to the active kinematic model.
        
        Args:
            state: Current state
            controls: Control inputs
            dt: Time step
        
        Returns:
            New state after applying controls for dt time
        """
        if self.model_type == 'unicycle':
            return self.unicycle.propagate_state(state, controls, dt).full()
        else:  # 'omni'
            return self.omni.propagate_state(state, controls, dt).full()
    
    def plot_trajectory(self, trajectory):
        """
        Plot the trajectory according to the active model.
        
        Args:
            trajectory: Array representing the trajectory
        """
        if self.model_type == 'unicycle':
            self.unicycle.plot_trajectory(trajectory)
        else:  # 'omni'
            self.omni.plot_trajectory(trajectory)

    
    
        
        
        
        