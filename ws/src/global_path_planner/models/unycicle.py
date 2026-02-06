import casadi as ca
import numpy as np
import matplotlib.pyplot as plt

class Unicycle:  # Fixed the spelling
    def __init__(self, dt: float = 0.1):
        """
        Initialize the unicycle kinematic model.
        
        Args:
            dt: Time step for discrete simulation
        """
        # State and control dimensions
        self.state_dim = 3  # [x, y, theta]
        self.control_dim = 2  # [v, omega]
        self.dt = dt
        
        # Optional constraints
        self.max_v = float('inf')
        self.max_omega = float('inf')
        
    def dynamics(self, state: ca.SX, u: ca.SX) -> ca.SX:
        """
        Define the continuous dynamics of the unicycle model.
        
        Args:
            state: Current state as [x, y, theta]
            u: Control input as [v, omega]
        
        Returns:
            State derivative as [vx, vy, vtheta]
        """
        x, y, theta = ca.vertsplit(state)
        v, omega = ca.vertsplit(u)
        
        x_dot = v * ca.cos(theta)
        y_dot = v * ca.sin(theta)
        theta_dot = omega
        
        return ca.vertcat(x_dot, y_dot, theta_dot)
    
    def discrete_dynamics(self, state: ca.SX, u: ca.SX) -> ca.SX:
        """
        Euler discretization of the dynamics for simulation.
        
        Args:
            state: Current state
            u: Control input
        
        Returns:
            Next state
        """
        derivatives = self.dynamics(state, u)
        next_state = state + self.dt * derivatives
        
        # Normalize theta to [-π, π]
        x, y, theta = ca.vertsplit(next_state)
        theta = ca.fmod(theta + ca.pi, 2*ca.pi) - ca.pi
        
        return ca.vertcat(x, y, theta)