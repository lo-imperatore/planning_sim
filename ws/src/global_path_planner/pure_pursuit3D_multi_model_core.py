import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Slerp
from scipy.spatial.transform import Rotation as R
import csv
import os
import copy

from models.omni_kino import Omni, HybridKinematics


class Waypoint:
    def __init__(self, position, orientation):
        self.position = position
        self.orientation = orientation


class OmnidirectionalPurePursuit3D:
    def __init__(
        self,
        waypoints,
        params=None,
    ):
        self.waypoints = copy.deepcopy(waypoints)

        # Get parameters
        self.lookahead_dist = params.get('lookahead_dist', 0.5)
        self.k_v = params.get('k_v', 1.0)
        self.k_omega = params.get('k_omega', 1.0)
        self.max_lin_speed = params.get('v_max', 1.0)
        self.max_ang_speed = params.get('omega_max', 1.0)
        self.max_omni_speed = params.get('v_omni_max', 1.5)
        self.angle_thresh_deg = params.get('angle_thresh_deg', 30)
        self.z_threshold = params.get('z_threshold', 0.05)
        self.weight_time = params.get('weight_time', False)
        self.dt = params.get('dt', 0.1)
        self.max_time = params.get('max_time', 30.0)
        self.print_debug = True
        self.goal_threshold = params.get('goal_threshold', 0.05)
        self.alpha_v = params.get('alpha_v', 0.5)
        self.alpha_w = params.get('alpha_w', 0.5)

        self.i = 0
        self.prev_angle = 0.0

        # Set the mode flag
        self.mode_flag = None

        # Define previous velocities
        self.prev_linear_vel = np.zeros(3)
        self.prev_angular_vel = np.zeros(3)

        # Velocities
        self.initial_linear_vel_x_body = 0
        self.linear_vel_prev_prev = np.zeros(3)
        self.linear_vel_prev = np.zeros(3)
        self.linear_vel_cmd_prev_prev = np.zeros(3)
        self.linear_vel_cmd_prev = np.zeros(3)

        self.angular_vel_prev_prev = np.zeros(3)
        self.angular_vel_prev = np.zeros(3)
        self.angular_vel_cmd_prev_prev = np.zeros(3)
        self.angular_vel_cmd_prev = np.zeros(3)

        self.state = Omni(
            position=copy.deepcopy(self.waypoints[0].position),
            orientation=copy.deepcopy(self.waypoints[0].orientation),
        )

        # Convert initial orientation to euler angles
        psi0, phi0, theta0 = self.quaternion_to_euler(
            self.state.orientation[0],
            self.state.orientation[1],
            self.state.orientation[2],
            self.state.orientation[3],
        )

        # Store history
        self.history = {
            'time': [0.0],
            'x': [self.state.position[0]],
            'y': [self.state.position[1]],
            'z': [self.state.position[2]],
            'psi': [psi0],
            'phi': [phi0],
            'theta': [theta0],
            'targets': [np.zeros(3)],
            'linear_vel_x': [0.0],
            'linear_vel_y': [0.0],
            'linear_vel_z': [0.0],
            'angular_vel_x': [0.0],
            'angular_vel_y': [0.0],
            'angular_vel_z': [0.0],
            'global_vel_x': [0.0],
            'global_vel_y': [0.0],
            'global_vel_z': [0.0],
            'mode': ['unicycle'],
        }

    def quaternion_rotate(self, q, v):
        """Rotate a vector v by a quaternion q"""
        w, x, y, z = q
        vx, vy, vz = v
        return np.array([
            (1 - 2*y*y - 2*z*z)*vx + (2*x*y - 2*z*w)*vy + (2*x*z + 2*y*w)*vz,
            (2*x*y + 2*z*w)*vx + (1 - 2*x*x - 2*z*z)*vy + (2*y*z - 2*x*w)*vz,
            (2*x*z - 2*y*w)*vx + (2*y*z + 2*x*w)*vy + (1 - 2*x*x - 2*y*y)*vz
        ])

    def quaternion_error(self, current_q, desired_q):
        """Compute orientation error quaternion"""
        return self.quaternion_multiply(self.quaternion_conjugate(current_q), desired_q)

    def quaternion_to_axis_angle(self, q):
        """Convert quaternion to axis-angle for quaternion in [w,z,y,x] format"""
        w = q[0]
        x = q[1]
        y = q[2]
        z = q[3]

        vector_part = np.array([x, y, z])

        norm = np.linalg.norm(vector_part)
        if norm < 1e-10:
            return np.array([0.0, 0.0, 1.0]), 0.0

        axis = vector_part / norm
        angle = 2 * np.arctan2(norm, w)
        """Convert quaternion to axis-angle for quaternion in [w,z,y,x] format"""

        return axis, angle

    def quaternion_multiply(self, q1, q2):
        """Multiply two quaternions (w, x, y, z)"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
            ])

    def rotate_vec_by_quat(self, v, q):
        """Rotate vector v by unit quaternion q: q ⊗ [0, v] ⊗ q*."""
        q_v = np.concatenate([[0.], v])
        return self.quaternion_multiply(self.quaternion_multiply(q, q_v), self.quaternion_conjugate(q))[1:]

    def quaternion_conjugate(self, q):
        """Conjugate of a quaternion (w, x, y, z)"""
        return np.array([q[0], -q[1], -q[2], -q[3]])

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convert Euler angles (roll, pitch, yaw) to quaternion.
        """
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        q0 = cr * cp * cy + sr * sp * sy
        q1 = sr * cp * cy - cr * sp * sy
        q2 = cr * sp * cy + sr * cp * sy
        q3 = cr * cp * sy - sr * sp * cy

        return q0, q1, q2, q3

    def quaternion_to_euler(self, q0, q1, q2, q3):
        """
        Convert quaternion (q0, q1, q2, q3) to Euler angles (roll, pitch, yaw).
        Args:
            q0, q1, q2, q3: Components of the quaternion
        Returns:
            roll, pitch, yaw: Euler angles in radians
        """
        sinr_cosp = 2 * (q0 * q1 + q2 * q3)
        cosr_cosp = 1 - 2 * (q1**2 + q2**2)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (q0 * q2 - q3 * q1)
        if abs(sinp) >= 1:
            pitch = np.sign(sinp) * np.pi / 2
        else:
            pitch = np.arcsin(sinp)

        siny_cosp = 2 * (q0 * q3 + q1 * q2)
        cosy_cosp = 1 - 2 * (q2**2 + q3**2)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def _closest_point_on_segment(self, point, a, b):
        """Find closest point on segment a-b"""
        ap = point - a
        ab = b - a
        t = np.dot(ap, ab) / (np.dot(ab, ab) + 1e-6)
        t = np.clip(t, 0.0, 1.0)

        cl_p = a + t * ab
        return cl_p

    def _find_target_point_3d(self):
        min_dist = np.inf
        closest_point = None
        closest_segment_idx = 0
        t_closest = 0.0

        robot_pos = copy.deepcopy(self.state.position)

        for i in range(len(self.waypoints) - 1):
            p1 = copy.deepcopy(self.waypoints[i].position)
            p2 = copy.deepcopy(self.waypoints[i+1].position)
            cp = self._closest_point_on_segment(robot_pos, p1, p2)
            dist = math.sqrt((cp[0] - robot_pos[0])**2 +
                            (cp[1] - robot_pos[1])**2 +
                            (cp[2] - robot_pos[2])**2)

            if dist < min_dist:
                min_dist = dist
                closest_point = cp
                closest_segment_idx = i

        return closest_segment_idx, closest_point

    def get_lookahead_point(self, start_segment, start_point):
        """Find lookahead point along path from start_point"""
        current_point = start_point.copy()
        remaining = self.lookahead_dist
        segment = start_segment
        path = [copy.deepcopy(wp.position) for wp in self.waypoints]

        while remaining > 0 and segment < len(path)-1:
            a = current_point
            b = path[segment+1]
            segment_vec = b - a
            segment_len = np.linalg.norm(segment_vec)

            if segment_len <= remaining:
                current_point = b
                remaining -= segment_len
                segment += 1
            else:
                direction = segment_vec / segment_len
                current_point += direction * remaining
                remaining = 0

        return current_point

    def compute_unicycle_velocities(self, goal):
        """
        Compute unicycle velocities for a unicycle model.
        Args:
            goal: Target position as [x,y,z]
        Returns:
            Linear velocity (v) and angular velocity (omega)
        """
        e_w = goal - self.state.position
        dist = np.linalg.norm(e_w)
        if dist < 1e-6:
            return 0.0, np.zeros(3)

        e_b = self.rotate_vec_by_quat(e_w, self.quaternion_conjugate(self.state.orientation))

        e_horizontal = np.array([e_b[0], e_b[1], 0.0])
        e_vertical = np.array([0.0, 0.0, e_b[2]])

        dist_h = np.linalg.norm(e_horizontal)

        f = np.array([1.0, 0.0, 0.0])

        if dist_h > 1e-6:
            d_hat_h = e_horizontal / dist_h
        else:
            d_hat_h = f

        align = np.clip(np.dot(f, d_hat_h), 0.0, 1.0)

        if align > 0.98:
            v = np.clip(self.k_v * dist_h, -self.max_lin_speed, self.max_lin_speed)
            omega_scale = 0.1
        else:
            v = np.clip(self.k_v * dist_h * align, -self.max_lin_speed * 0.05, self.max_lin_speed * 0.05)
            omega_scale = 1.0

        linear_vel = v

        axis = np.cross(f, d_hat_h)

        if np.linalg.norm(axis) < 1e-6:
            omega = np.zeros(3)
        else:
            axis /= np.linalg.norm(axis)
            angle = np.arccos(align)
            omega = self.k_omega * angle * axis * omega_scale

        return linear_vel, omega

    def compute_omnidirectional_velocities(self, goal):
        """
        Omnidirectional controller that computes velocity directly toward goal.

        Args:
            goal: Target position as [x,y,z]

        Returns:
            3D velocity vector in robot's body frame [vx, vy, vz]
        """
        error_world = goal - self.state.position

        error_body = self.rotate_vec_by_quat(error_world, self.quaternion_conjugate(self.state.orientation))

        norm = np.linalg.norm(error_body)

        if norm < 1e-6:
            return np.zeros(3)

        scale = min(self.max_omni_speed / norm, 1.0)

        return error_body * scale

    def hybrid_control(self, goal):
        """
        Hybrid controller that switches between unicycle and omnidirectional control modes in 3D space.

        Args:
            goal: Target position as [x,y,z]

        Returns:
            Dictionary with control mode and associated control signals
        """
        if self.mode_flag is None:
            self.mode_flag = "unicycle"

        dist_to_goal = np.linalg.norm(goal - self.state.position)

        if dist_to_goal < self.goal_threshold:
            return {
                "mode": "stop",
                "linear_vel": 0.0,
                "angular_vel": np.zeros(3),
                "omni_vel": np.zeros(3),
            }

        if self.mode_flag == "unicycle":
            linear_vel, angular_vel = self.compute_unicycle_velocities(goal)

            if abs(goal[2] - self.state.position[2]) > self.z_threshold:
                self.mode_flag = "omni"

            return {
                "mode": "unicycle",
                "linear_vel": linear_vel,
                "angular_vel": angular_vel,
                "omni_vel": np.zeros(3),
            }

        if self.mode_flag == "omni":
            omni_vel = self.compute_omnidirectional_velocities(goal)

            if abs(goal[2] - self.state.position[2]) <= self.z_threshold:
                self.mode_flag = "unicycle"

            return {
                "mode": "omni",
                "linear_vel": 0.0,
                "angular_vel": np.zeros(3),
                "omni_vel": omni_vel,
            }

        return {
            "mode": "unicycle",
            "linear_vel": 0.0,
            "angular_vel": np.zeros(3),
            "omni_vel": np.zeros(3),
        }

    def weighted_average(self, new, old):
        if self.weight_time:
            return (self.alpha_v * new + (1 - self.alpha_v) * old) / 2
        return new

    def update_velocity_history(self, linear_vel, angular_vel, cmd_linear_vel, cmd_angular_vel):
        self.linear_vel_prev_prev = self.linear_vel_prev
        self.linear_vel_prev = linear_vel
        self.linear_vel_cmd_prev_prev = self.linear_vel_cmd_prev
        self.linear_vel_cmd_prev = cmd_linear_vel

        self.angular_vel_prev_prev = self.angular_vel_prev
        self.angular_vel_prev = angular_vel
        self.angular_vel_cmd_prev_prev = self.angular_vel_cmd_prev
        self.angular_vel_cmd_prev = cmd_angular_vel

    def update_mode(self, new_mode):
        if self.mode_flag != new_mode:
            self.mode_flag = new_mode

    def integrate_state(self, linear_vel_body, angular_vel_body):
        self.state = HybridKinematics.integrate_state(self.state, linear_vel_body, angular_vel_body, self.dt)

    def update_history(self, time, linear_vel_body, angular_vel_body, global_vel):
        self.history['time'].append(time)
        self.history['x'].append(self.state.position[0])
        self.history['y'].append(self.state.position[1])
        self.history['z'].append(self.state.position[2])

        psi, phi, theta = self.quaternion_to_euler(
            self.state.orientation[0],
            self.state.orientation[1],
            self.state.orientation[2],
            self.state.orientation[3],
        )

        self.history['psi'].append(psi)
        self.history['phi'].append(phi)
        self.history['theta'].append(theta)
        self.history['targets'].append(self.target.copy())

        self.history['linear_vel_x'].append(linear_vel_body[0])
        self.history['linear_vel_y'].append(linear_vel_body[1])
        self.history['linear_vel_z'].append(linear_vel_body[2])

        self.history['angular_vel_x'].append(angular_vel_body[0])
        self.history['angular_vel_y'].append(angular_vel_body[1])
        self.history['angular_vel_z'].append(angular_vel_body[2])

        self.history['global_vel_x'].append(global_vel[0])
        self.history['global_vel_y'].append(global_vel[1])
        self.history['global_vel_z'].append(global_vel[2])

        self.history['mode'].append(self.mode_flag)

    def run(self):
        total_time = 0.0
        self.i = 0

        while total_time < self.max_time:
            segment_idx, closest_point = self._find_target_point_3d()
            self.target = self.get_lookahead_point(segment_idx, closest_point)

            distance_to_goal = np.linalg.norm(self.target - self.state.position)
            if distance_to_goal < self.goal_threshold:
                if self.print_debug:
                    print("Goal reached")
                break

            control = self.hybrid_control(self.target)
            self.update_mode(control['mode'])

            linear_vel_body = control['linear_vel']
            angular_vel_body = control['angular_vel']
            omni_vel_body = control['omni_vel']

            if np.isscalar(linear_vel_body):
                linear_vel_body = np.array([float(linear_vel_body), 0.0, 0.0])

            if self.mode_flag == "omni":
                linear_vel_body = omni_vel_body

            global_vel = self.quaternion_rotate(self.state.orientation, linear_vel_body)

            if self.weight_time:
                linear_vel_body = self.weighted_average(linear_vel_body, self.linear_vel_prev)
                angular_vel_body = self.weighted_average(angular_vel_body, self.angular_vel_prev)

            self.update_velocity_history(linear_vel_body, angular_vel_body, linear_vel_body, angular_vel_body)

            self.integrate_state(linear_vel_body, angular_vel_body)

            total_time += self.dt
            self.update_history(total_time, linear_vel_body, angular_vel_body, global_vel)

            self.i += 1

        return self.history

    def plot_3d(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        ax.plot(self.history['x'], self.history['y'], self.history['z'], label='Trajectory')
        ax.scatter(
            [wp.position[0] for wp in self.waypoints],
            [wp.position[1] for wp in self.waypoints],
            [wp.position[2] for wp in self.waypoints],
            c='r',
            marker='o',
            label='Waypoints'
        )

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('3D Pure Pursuit Trajectory')
        ax.legend()
        plt.show()

    def plot_velocities(self):
        time = self.history['time']
        linear_vel_x = self.history['linear_vel_x']
        linear_vel_y = self.history['linear_vel_y']
        linear_vel_z = self.history['linear_vel_z']
        angular_vel_x = self.history['angular_vel_x']
        angular_vel_y = self.history['angular_vel_y']
        angular_vel_z = self.history['angular_vel_z']

        fig, axs = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

        axs[0].plot(time, linear_vel_x, label='Linear Vel X')
        axs[0].plot(time, linear_vel_y, label='Linear Vel Y')
        axs[0].plot(time, linear_vel_z, label='Linear Vel Z')
        axs[0].set_ylabel('Linear Velocity (m/s)')
        axs[0].legend()

        axs[1].plot(time, angular_vel_x, label='Angular Vel X')
        axs[1].plot(time, angular_vel_y, label='Angular Vel Y')
        axs[1].plot(time, angular_vel_z, label='Angular Vel Z')
        axs[1].set_xlabel('Time (s)')
        axs[1].set_ylabel('Angular Velocity (rad/s)')
        axs[1].legend()

        plt.tight_layout()
        plt.show()

    def plot_modes(self):
        time = self.history['time']
        modes = self.history['mode']
        mode_numeric = [1 if mode == 'omni' else 0 for mode in modes]

        plt.figure()
        plt.step(time, mode_numeric, where='post')
        plt.yticks([0, 1], ['Unicycle', 'Omni'])
        plt.xlabel('Time (s)')
        plt.ylabel('Mode')
        plt.title('Control Mode Switching')
        plt.grid(True)
        plt.show()

    def save_history_to_csv(self, name="traj_bimodal_time_2_vel_soft"):
        file_path = name + '.csv'
        fieldnames = list(self.history.keys())

        with open(file_path, mode='w', newline='') as file:
            writer = csv.DictWriter(file, fieldnames=fieldnames)
            writer.writeheader()
            for i in range(len(self.history['time'])):
                row = {key: self.history[key][i] for key in fieldnames}
                writer.writerow(row)

        print(f"History saved to {file_path}")

    def compute_trajectory_stats(self, history, time_array, label, yaw_ref):
        dt = np.diff(time_array)

        pos = np.column_stack((history['x'], history['y'], history['z']))
        vel = np.gradient(pos, axis=0) / dt.mean()
        acc = np.gradient(vel, axis=0) / dt.mean()
        jerk = np.gradient(acc, axis=0) / dt.mean()

        # Speed, acceleration magnitude, jerk magnitude
        speed = np.linalg.norm(vel[:-1], axis=1)
        acc_mag = np.linalg.norm(acc, axis=1)
        jerk_mag = np.linalg.norm(jerk, axis=1)

        # Total distance using integrated speed
        distance = np.sum(speed * dt)

        # Energy proxies
        vel_energy = np.sum(speed**2 * dt)
        acc_energy = np.sum(acc_mag[:-1]**2 * dt[1:])

        psi = np.array(history['psi'])

        # Yaw error wrapped to [-π, π]
        yaw_error = np.arctan2(np.sin(psi - yaw_ref), np.cos(psi - yaw_ref))

        mean_yaw_error = np.mean(np.abs(yaw_error))
        max_yaw_error = np.max(np.abs(yaw_error))
        rmse_yaw_error = np.sqrt(np.mean(yaw_error**2))
        return {
            'Label': label,
            'Total Time (s)': time_array[-1] - time_array[0],
            'Total Distance (m)': distance,
            'Mean Speed (m/s)': np.mean(speed),
            'Max Speed (m/s)': np.max(speed),
            'Mean Acceleration (m/s²)': np.mean(acc_mag),
            'Max Acceleration (m/s²)': np.max(acc_mag),
            'Mean Jerk (m/s³)': np.mean(jerk_mag),
            'Max Jerk (m/s³)': np.max(jerk_mag),
            'Velocity Energy Proxy': vel_energy,
            'Acceleration Energy Proxy': acc_energy,
            'Mean Yaw Error (rad)': mean_yaw_error,
            'Max Yaw Error (rad)': max_yaw_error,
            'RMSE Yaw Error (rad)': rmse_yaw_error,
        }
