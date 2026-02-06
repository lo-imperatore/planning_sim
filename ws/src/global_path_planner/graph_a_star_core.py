import heapq
import time
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from open3d.geometry import LineSet, TriangleMesh  # type: ignore
from open3d.utility import Vector3dVector, Vector2iVector  # type: ignore
import open3d.visualization.gui as gui  # type: ignore
from scipy.spatial import cKDTree

from timing import Timer
from pure_pursuit3D_multi_model_core import OmnidirectionalPurePursuit3D, Waypoint


class GlobalPlanner:
    def __init__(self, pcd_path: str, obs_path: str, voxel_size: float = 0.1) -> None:
        init_start_time = time.time()

        with Timer("Point cloud loading"):
            self.pcd = o3d.io.read_point_cloud(pcd_path)
            self.obs = o3d.io.read_point_cloud(obs_path)

        with Timer("Point cloud down-sampling"):
            self.pcd = self.pcd.voxel_down_sample(voxel_size)

        with Timer("Point cloud processing"):
            self.pcd_pts = np.asarray(self.pcd.points)
            self.obs_pts = np.asarray(self.obs.points)
            self.navigable = None
            self.obstacle = None
            self.nav_tree = None
            self.nav_pts = None
            self.radius = 1.5

        with Timer("Penalty function"):
            self.collision, self.penalties = self.penalty_function()

        self.slopes = np.zeros(len(self.pcd_pts))
        self.normals: List[np.ndarray] = []
        self.height_dev = np.zeros(len(self.pcd_pts))
        self.n = np.zeros((len(self.pcd_pts), 3))

        with Timer("Building graph"):
            self.build_graph()

        total_init_time = time.time() - init_start_time
        print(f"Total initialization time: {total_init_time:.4f} seconds")

    def find_nearest_point_index(self, query_coord_xy: Sequence[float]) -> int:
        points_xy = self.pcd_pts[:, :2]
        query_xy = np.array(query_coord_xy)[:2]
        squared_dists = np.sum((points_xy - query_xy) ** 2, axis=1)
        return int(np.argmin(squared_dists))

    def build_graph(self) -> None:
        tree = o3d.geometry.KDTreeFlann(self.pcd)
        self.graph: Dict[int, List[Tuple[int, float]]] = {i: [] for i in range(len(self.pcd_pts))}
        self.edges = set()
        w_coll = 2
        w_dist = 10
        w_slope = 10

        for i in range(len(self.pcd_pts)):
            theta_i = self.pcd_pts[i, -1]
            _, idxs, dists = tree.search_radius_vector_3d(self.pcd_pts[i], self.radius)
            self.slopes[i], normal = self.compute_slopes(i, idxs[1:])
            self.normals.append(normal)
            for j, dist in zip(idxs[1:], dists[1:]):
                theta_j = self.pcd_pts[j, -1]
                delta = theta_j - theta_i
                angular_diff = np.abs((delta + np.pi) % (2 * np.pi) - np.pi)
                total_cost = w_coll * self.collision[j] + w_dist * dist + w_slope * self.slopes[j]
                self.graph[i].append((j, total_cost))
                self.edges.add(tuple(sorted((i, j))))

    def penalty_function(self) -> Tuple[np.ndarray, np.ndarray]:
        obs_tree = cKDTree(self.obs_pts)

        rect_length, rect_width = 0.9, 0.6
        diag_radius = np.sqrt((rect_length / 2) ** 2 + (rect_width / 2) ** 2)

        candidates = obs_tree.query_ball_point(self.pcd_pts, diag_radius)

        collision = np.zeros(len(self.pcd_pts))
        sigma = 0.5

        dists, _ = obs_tree.query(self.pcd_pts, k=1)
        penalties = 50 * np.exp(-(dists / sigma) ** 2)

        for i, neighbors in enumerate(candidates):
            if neighbors:
                relative_positions = self.obs_pts[neighbors] - self.pcd_pts[i]
                within_rect = np.logical_and(
                    np.abs(relative_positions[:, 0]) <= rect_length / 2,
                    np.abs(relative_positions[:, 1]) <= rect_width / 2,
                )
                if np.any(within_rect):
                    collision[i] = 5000
                else:
                    collision[i] = penalties[i]
            else:
                collision[i] = penalties[i]

        return collision, penalties

    def compute_slopes(self, idx: int, neigh_idxs: Sequence[int]) -> Tuple[float, np.ndarray]:
        p0 = self.pcd_pts[idx, :]
        pts = self.pcd_pts[neigh_idxs, :]
        X = pts[:, :2] - p0[:2]
        Z = pts[:, 2] - p0[2]

        A = np.c_[X, np.ones(len(pts))]
        a, b, _ = np.linalg.lstsq(A, Z, rcond=None)[0]

        n = np.array([-a, -b, 1.0])
        n /= np.linalg.norm(n)

        slope_rad = np.arccos(n[2])
        return np.degrees(slope_rad), n

    def compute_height_distribution(self, idx: int, neigh_idxs: Sequence[int]) -> Tuple[float, float, float, float]:
        node_height = self.pcd_pts[idx, 2]
        neighbor_heights = self.pcd_pts[neigh_idxs, 2]

        height_differences = neighbor_heights - node_height

        mean_height = np.mean(height_differences)
        std_height = np.std(height_differences)
        min_height = np.min(height_differences)
        max_height = np.max(height_differences)

        return mean_height, std_height, min_height, max_height

    def astar(self, start: int, goal: int, start_yaw: float = 0.0, goal_yaw: float = 0.0):
        with Timer("A* search"):
            print("A* search...")
            self.yaw_weight = 10000
            self.slope_weight = 1000
            self.goal_yaw_weight = 500

            open_set = []
            heapq.heappush(open_set, (0.0, start, start_yaw))

            g_score: Dict[Tuple[int, float], float] = {(start, start_yaw): 0.0}
            came_from: Dict[Tuple[int, float], Tuple[int, float]] = {}
            nodes_expanded = 0

            while open_set:
                f_current, current_node, current_yaw = heapq.heappop(open_set)
                nodes_expanded += 1
                if current_node == goal:
                    if goal_yaw is not None:
                        final_yaw_cost = self.goal_yaw_weight * abs(goal_yaw - current_yaw)
                        g_score[(current_node, current_yaw)] += final_yaw_cost
                    print(f"A* search completed: {nodes_expanded} nodes expanded")
                    return self._reconstruct((current_node, current_yaw), came_from)

                for neighbor, base_cost in self.graph[current_node]:
                    vec = self.pcd_pts[neighbor, :3] - self.pcd_pts[current_node, :3]
                    movement_yaw = np.arctan2(vec[1], vec[0])

                    delta_yaw = movement_yaw - current_yaw
                    if delta_yaw > np.pi:
                        delta_yaw -= 2 * np.pi
                    elif delta_yaw < -np.pi:
                        delta_yaw += 2 * np.pi

                    yaw_cost = abs(delta_yaw)

                    normal = self.normals[current_node]
                    normalized_vec = vec / np.linalg.norm(vec)
                    cos_phi = np.dot(normalized_vec, normal)
                    theta_rad = np.deg2rad(self.slopes[current_node])
                    slope_factor = self.slope_weight * np.sin(theta_rad) * cos_phi

                    step_cost = base_cost + self.yaw_weight * yaw_cost + slope_factor
                    tentative_g = g_score[(current_node, current_yaw)] + step_cost

                    state = (neighbor, movement_yaw)
                    if tentative_g < g_score.get(state, float("inf")):
                        g_score[state] = tentative_g
                        came_from[state] = (current_node, current_yaw)

                        h = np.linalg.norm(self.pcd_pts[neighbor, :3] - self.pcd_pts[goal, :3])
                        if goal_yaw is not None:
                            goal_yaw_diff = abs(goal_yaw - movement_yaw)
                            h += self.goal_yaw_weight * goal_yaw_diff

                        f_score = tentative_g + h
                        heapq.heappush(open_set, (f_score, neighbor, movement_yaw))

            print(f"A* search failed: {nodes_expanded} nodes expanded")
            return None

    def _reconstruct(self, end_state: Tuple[int, float], came_from: Dict[Tuple[int, float], Tuple[int, float]]):
        node, yaw = end_state
        path = []

        while (node, yaw) in came_from:
            prev_node, prev_yaw = came_from[(node, yaw)]
            path.append((node, yaw))
            node, yaw = prev_node, prev_yaw

        path.append((node, yaw))
        path.reverse()
        return path

    def visualize_scene_with_orientation_robust(self, path, start_idx, goal_idx, display_pcd: bool = False):
        try:
            if not hasattr(o3d.visualization, "gui"):
                print("Open3D built without GUI support, using simple visualization...")
                return self.visualize_scene_simple(path, start_idx, goal_idx, display_pcd)

            app = gui.Application.instance
            app.initialize()

            geometries = {}

            if len(self.edges) > 0:
                graph_lines = LineSet()
                graph_lines.points = Vector3dVector(self.pcd_pts)
                graph_lines.lines = Vector2iVector(list(self.edges))
                graph_lines.paint_uniform_color([0.8, 0.8, 0.8])
                geometries["graph"] = graph_lines

            if path:
                indices = [int(node_idx) for node_idx, *_ in path]
                if len(indices) > 1:
                    path_lines = LineSet()
                    path_points = self.pcd_pts[indices, :]
                    path_lines.points = Vector3dVector(path_points)
                    path_lines.lines = Vector2iVector([[i, i + 1] for i in range(len(indices) - 1)])
                    path_lines.paint_uniform_color([1.0, 0.0, 0.0])
                    geometries["path"] = path_lines

            start_marker = TriangleMesh.create_sphere(radius=0.1)
            start_marker.translate(self.pcd_pts[start_idx])
            start_marker.paint_uniform_color([0.0, 1.0, 0.0])
            start_marker.compute_vertex_normals()
            geometries["start"] = start_marker

            goal_marker = TriangleMesh.create_sphere(radius=0.1)
            goal_marker.translate(self.pcd_pts[goal_idx])
            goal_marker.paint_uniform_color([1.0, 0.0, 0.0])
            goal_marker.compute_vertex_normals()
            geometries["goal"] = goal_marker

            if display_pcd:
                geometries["pcd"] = self.pcd

            vis = o3d.visualization.O3DVisualizer("Path Visualization", 1024, 768)

            for name, geom in geometries.items():
                mat = o3d.visualization.rendering.MaterialRecord()
                if isinstance(geom, LineSet):
                    mat.shader = "unlitLine"
                    mat.line_width = 5.0 if name == "path" else 1.0
                else:
                    mat.shader = "defaultLit"
                vis.add_geometry(name, geom, mat)

            if path:
                indices = [int(node_idx) for node_idx, *_ in path]
                path_points = self.pcd_pts[indices, :]
                center = path_points.mean(axis=0)
            else:
                center = self.pcd_pts.mean(axis=0)

            min_bound = self.pcd_pts.min(axis=0)
            max_bound = self.pcd_pts.max(axis=0)
            scene_size = np.linalg.norm(max_bound - min_bound)

            eye = center + np.array([0.0, -scene_size * 0.5, scene_size * 0.3])
            up = np.array([0.0, 0.0, 1.0])
            vis.setup_camera(60.0, center, eye, up)

            app.add_window(vis)
            app.run()

        except Exception as e:
            print(f"Advanced visualization failed: {e}")
            print("Falling back to simple Open3D visualization.")
            self.visualize_scene_simple(path, start_idx, goal_idx, display_pcd)

    def visualize_scene_simple(self, path, start_idx, goal_idx, display_pcd: bool = False):
        try:
            geometries = []

            if path:
                indices = [int(node_idx) for node_idx, *_ in path]
                if len(indices) > 1:
                    path_lines = o3d.geometry.LineSet()
                    path_points = self.pcd_pts[indices, :]
                    path_lines.points = o3d.utility.Vector3dVector(path_points)
                    path_lines.lines = o3d.utility.Vector2iVector([[i, i + 1] for i in range(len(indices) - 1)])
                    path_lines.paint_uniform_color([1, 0, 0])
                    geometries.append(path_lines)

            if display_pcd:
                sample_size = min(5000, len(self.pcd_pts))
                sample_indices = np.random.choice(len(self.pcd_pts), sample_size, replace=False)
                sampled_pcd = o3d.geometry.PointCloud()
                sampled_pcd.points = o3d.utility.Vector3dVector(self.pcd_pts[sample_indices])
                sampled_pcd.paint_uniform_color([0.7, 0.7, 0.7])
                geometries.append(sampled_pcd)

            start_marker = o3d.geometry.TriangleMesh.create_sphere(radius=0.15)
            start_marker.translate(self.pcd_pts[start_idx])
            start_marker.paint_uniform_color([0, 1, 0])
            geometries.append(start_marker)

            goal_marker = o3d.geometry.TriangleMesh.create_sphere(radius=0.15)
            goal_marker.translate(self.pcd_pts[goal_idx])
            goal_marker.paint_uniform_color([1, 0, 0])
            geometries.append(goal_marker)

            o3d.visualization.draw_geometries(
                geometries,
                window_name="Path Visualization",
                width=800,
                height=600,
                left=50,
                top=50,
            )

            return True

        except Exception as e:
            print(f"Open3D visualization failed: {e}")
            return False

    def plot_path(self, path, **kwargs):
        plot_slopes = kwargs.get("slope", False)
        plot_penalties = kwargs.get("penalty", False)
        plot_collision = kwargs.get("collision", False)
        plot_height = kwargs.get("height", False)
        plot_z_only = kwargs.get("z_only", False)

        path_arr = np.array([p[0] for p in path], dtype=float)
        idx_column = np.array([int(p[0]) for p in path], dtype=int)

        path_coords = self.pcd_pts[idx_column, :3]

        fig = plt.figure(figsize=(8, 6))
        ax = fig.add_subplot(111, projection="3d")
        ax.plot(
            path_coords[:, 0],
            path_coords[:, 1],
            path_coords[:, 2],
            color="red",
            linewidth=3,
            label="A* path",
        )
        ax.scatter(self.pcd_pts[:, 0], self.pcd_pts[:, 1], self.pcd_pts[:, 2], color="blue", s=10)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_title("A* Path Visualization")
        ax.set_xlim([self.pcd_pts[:, 0].min(), self.pcd_pts[:, 0].max()])
        ax.set_ylim([self.pcd_pts[:, 1].min(), self.pcd_pts[:, 1].max()])
        ax.set_zlim([self.pcd_pts[:, 2].min(), self.pcd_pts[:, 2].max()])

        if plot_slopes:
            fig2 = plt.figure(figsize=(8, 6))
            ax2 = fig2.add_subplot(111, projection="3d")
            ax2.scatter(
                self.pcd_pts[:, 0],
                self.pcd_pts[:, 1],
                self.pcd_pts[:, 2],
                c=self.slopes,
                cmap="viridis",
                marker="o",
            )
            ax2.set_xlabel("X")
            ax2.set_ylabel("Y")
            ax2.set_zlabel("Z")
            ax2.set_title("Slope Visualization")
            ax2.set_xlim([self.pcd_pts[:, 0].min(), self.pcd_pts[:, 0].max()])
            ax2.set_ylim([self.pcd_pts[:, 1].min(), self.pcd_pts[:, 1].max()])
            ax2.set_zlim([self.pcd_pts[:, 2].min(), self.pcd_pts[:, 2].max()])
            plt.colorbar(ax2.collections[0], label="Slope Value")
            plt.tight_layout()

        if plot_penalties:
            fig3 = plt.figure(figsize=(8, 6))
            ax3 = fig3.add_subplot(111, projection="3d")
            ax3.scatter(
                self.pcd_pts[:, 0],
                self.pcd_pts[:, 1],
                self.pcd_pts[:, 2],
                c=self.penalties,
                cmap="viridis",
                marker="o",
            )
            ax3.set_xlabel("X")
            ax3.set_ylabel("Y")
            ax3.set_zlabel("Z")
            ax3.set_title("Penalty Visualization")
            ax3.set_xlim([self.pcd_pts[:, 0].min(), self.pcd_pts[:, 0].max()])
            ax3.set_ylim([self.pcd_pts[:, 1].min(), self.pcd_pts[:, 1].max()])
            ax3.set_zlim([self.pcd_pts[:, 2].min(), self.pcd_pts[:, 2].max()])
            plt.colorbar(ax3.collections[0], label="Penalty Value")
            plt.tight_layout()

        if plot_collision:
            fig4 = plt.figure(figsize=(8, 6))
            ax4 = fig4.add_subplot(111, projection="3d")
            ax4.scatter(
                self.pcd_pts[:, 0],
                self.pcd_pts[:, 1],
                self.pcd_pts[:, 2],
                c=self.collision,
                cmap="viridis",
                marker="o",
            )
            ax4.set_xlabel("X")
            ax4.set_ylabel("Y")
            ax4.set_zlabel("Z")
            ax4.set_title("Collision Visualization")
            ax4.set_xlim([self.pcd_pts[:, 0].min(), self.pcd_pts[:, 0].max()])
            ax4.set_ylim([self.pcd_pts[:, 1].min(), self.pcd_pts[:, 1].max()])
            ax4.set_zlim([self.pcd_pts[:, 2].min(), self.pcd_pts[:, 2].max()])
            plt.colorbar(ax4.collections[0], label="Collision Value")
            plt.tight_layout()

        if plot_height:
            fig5 = plt.figure(figsize=(8, 6))
            ax5 = fig5.add_subplot(111, projection="3d")
            ax5.scatter(
                self.pcd_pts[:, 0],
                self.pcd_pts[:, 1],
                self.pcd_pts[:, 2],
                c=self.height_dev,
                cmap="viridis",
                marker="o",
            )
            ax5.set_xlabel("X")
            ax5.set_ylabel("Y")
            ax5.set_zlabel("Z")
            ax5.set_title("Height Visualization")
            ax5.set_xlim([self.pcd_pts[:, 0].min(), self.pcd_pts[:, 0].max()])
            ax5.set_ylim([self.pcd_pts[:, 1].min(), self.pcd_pts[:, 1].max()])
            ax5.set_zlim([self.pcd_pts[:, 2].min(), self.pcd_pts[:, 2].max()])
            plt.colorbar(ax5.collections[0], label="Height Value")
            plt.tight_layout()

        fig_trav = plt.figure(figsize=(10, 8))
        ax_trav = fig_trav.add_subplot(111)
        traversability = self.collision + self.penalties + self.slope_weight * self.slopes

        if traversability.max() > traversability.min():
            normalized_trav = (traversability - traversability.min()) / (traversability.max() - traversability.min())
        else:
            normalized_trav = np.zeros_like(traversability)

        scatter = ax_trav.scatter(
            self.pcd_pts[:, 0],
            self.pcd_pts[:, 1],
            c=normalized_trav,
            cmap="jet_r",
            marker="o",
            s=30,
        )

        ax_trav.set_xlabel("X")
        ax_trav.set_ylabel("Y")
        ax_trav.set_title("Traversability Map (2D View)")
        ax_trav.set_xlim([self.pcd_pts[:, 0].min(), self.pcd_pts[:, 0].max()])
        ax_trav.set_ylim([self.pcd_pts[:, 1].min(), self.pcd_pts[:, 1].max()])

        cbar = plt.colorbar(scatter, ax=ax_trav)
        cbar.set_label("Traversability Score (Lower is Better)")

        plt.tight_layout()

        if plot_z_only:
            fig6 = plt.figure(figsize=(8, 6))
            ax6 = fig6.add_subplot(111)

            z_values = self.pcd_pts[:, 2]

            near_zero_mask = np.abs(z_values) <= 0.03
            negative_mask = z_values < -0.03
            positive_mask = z_values > 0.03

            ax6.scatter(
                self.pcd_pts[negative_mask, 0],
                self.pcd_pts[negative_mask, 1],
                color="blue",
                marker="o",
                label="Negative (<-2cm)",
            )
            ax6.scatter(
                self.pcd_pts[near_zero_mask, 0],
                self.pcd_pts[near_zero_mask, 1],
                color="green",
                marker="o",
                label="Near Zero (±2cm)",
            )

            ax6.scatter(
                self.pcd_pts[positive_mask, 0],
                self.pcd_pts[positive_mask, 1],
                color="red",
                marker="o",
                label="Positive (>2cm)",
            )

            ax6.set_xlabel("X")
            ax6.set_ylabel("Y")
            ax6.set_title("Z-value Classification")
            ax6.legend()
            plt.tight_layout()

        plt.show()

    def plot_graph_with_costs(self):
        node_costs = np.zeros(len(self.pcd_pts))
        for node, neighbors in self.graph.items():
            node_costs[node] = sum(cost for _, cost in neighbors)

        fig = plt.figure(figsize=(8, 6))
        ax = fig.add_subplot(111, projection="3d")

        scatter = ax.scatter(
            self.pcd_pts[:, 0],
            self.pcd_pts[:, 1],
            self.pcd_pts[:, 2],
            c=node_costs,
            cmap="viridis",
            marker="o",
        )
        plt.colorbar(scatter, label="Node Cost")

        for edge in self.edges:
            point1 = self.pcd_pts[edge[0]]
            point2 = self.pcd_pts[edge[1]]
            ax.plot(
                [point1[0], point2[0]],
                [point1[1], point2[1]],
                [point1[2], point2[2]],
                color="gray",
                linewidth=0.5,
            )

        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.view_init(elev=90, azim=-90)
        plt.show()


def build_waypoints_from_path(
    planner: GlobalPlanner, path: Sequence[Tuple[int, float]], goal_yaw: float = 0.0
) -> List[Waypoint]:
    waypoints: List[Waypoint] = []
    for i, (node_idx, yaw) in enumerate(path):
        position = planner.pcd_pts[int(node_idx), :3]
        pitch = 0.0
        roll = 0.0
        if i == len(path) - 1:
            yaw = goal_yaw

        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        orientation = np.array([qw, qx, qy, qz])
        waypoints.append(Waypoint(position, orientation))

    return waypoints


def build_default_params() -> Dict[str, float]:
    return {
        "lookahead_dist": 0.3,
        "k_v": 1.5,
        "k_omega": 0.75,
        "v_max": 0.4,
        "omega_max": 0.3,
        "v_omni_max": 0.2,
        "angle_thresh_deg": 10,
        "z_threshold": 0.05,
        "weight_time": True,
        "dt": 0.03,
        "max_time": 120.0,
        "goal_threshold": 0.1,
        "alpha_v": 0.8,
        "alpha_w": 0.8,
    }


def run_pure_pursuit_controllers(
    waypoints: Sequence[Waypoint], params: Dict[str, float]
) -> Tuple[OmnidirectionalPurePursuit3D, OmnidirectionalPurePursuit3D, OmnidirectionalPurePursuit3D]:
    params_uni = params.copy()
    params_uni["v_omni_max"] = 1e-5

    params_omni = params.copy()
    params_omni["angle_thresh_deg"] = 100.0
    params_omni["v_omni_max"] = 0.5
    params_omni["weight_time"] = False

    pp_controller = OmnidirectionalPurePursuit3D(waypoints=waypoints, params=params)
    uni_controller = OmnidirectionalPurePursuit3D(waypoints, params_uni)
    omni_controller = OmnidirectionalPurePursuit3D(waypoints, params_omni)

    pp_controller.run()
    uni_controller.run()
    omni_controller.run()

    return pp_controller, uni_controller, omni_controller


def build_trajectories(
    pp_controller: OmnidirectionalPurePursuit3D,
    uni_controller: OmnidirectionalPurePursuit3D,
    omni_controller: OmnidirectionalPurePursuit3D,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    traj_hybrid = np.array(
        [pp_controller.history["x"], pp_controller.history["y"], pp_controller.history["z"]]
    ).T
    traj_uni = np.array([uni_controller.history["x"], uni_controller.history["y"], uni_controller.history["z"]]).T
    traj_omni = np.array(
        [omni_controller.history["x"], omni_controller.history["y"], omni_controller.history["z"]]
    ).T
    return traj_hybrid, traj_uni, traj_omni


def plot_trajectory_comparison(
    traj_hybrid: np.ndarray,
    traj_uni: np.ndarray,
    traj_omni: np.ndarray,
    waypoints_position: np.ndarray,
) -> None:
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection="3d")

    ax.plot(traj_hybrid[:, 0], traj_hybrid[:, 1], traj_hybrid[:, 2], "b--", linewidth=2, label="Hybrid Model")
    ax.plot(traj_uni[:, 0], traj_uni[:, 1], traj_uni[:, 2], "r--", linewidth=2, label="Unicycle Only")
    ax.plot(traj_omni[:, 0], traj_omni[:, 1], traj_omni[:, 2], "g--", linewidth=2, label="Omnidirectional Only")

    ax.plot(
        waypoints_position[:-1, 0],
        waypoints_position[:-1, 1],
        waypoints_position[:-1, 2],
        "ko-",
        markersize=6,
        label="Waypoints",
    )

    ax.set_xlabel("X Position", fontsize=12)
    ax.set_ylabel("Y Position", fontsize=12)
    ax.set_zlabel("Z Position", fontsize=12)
    ax.set_title("Trajectory Comparison of Motion Models", fontsize=14)
    ax.legend(fontsize=10)
    ax.grid(True)

    ax.set_zlim(0, 1)
    ax.view_init(elev=90, azim=-90)

    plt.tight_layout()
    plt.show()


def compute_yaw_reference(
    waypoints_position: np.ndarray, waypoints_orientation: np.ndarray, targets_xy: np.ndarray
) -> np.ndarray:
    yaw_by_waypoint = np.array(
        [
            np.arctan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]))
            for q in waypoints_orientation
        ]
    )

    tree = cKDTree(waypoints_position[:, :2])
    _, indices = tree.query(targets_xy)
    return yaw_by_waypoint[indices]
