# === FILE: arm_visualizer.py ===
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

class ArmVisualizer:
    def __init__(self, limits=20):
        self.limits = limits

    def draw_gripper(self, ax, T, d=2, w=2):
        points_local = np.array([
            [0,  w/2, 0, 1],
            [0,  w/2, d, 1],
            [0, -w/2, 0, 1],
            [0, -w/2, d, 1],
        ]).T
        points_world = (T @ points_local)[:3, :]
        ax.plot(points_world[0, [0,1]], points_world[1, [0,1]], points_world[2, [0,1]], '-', lw=2)
        ax.plot(points_world[0, [2,3]], points_world[1, [2,3]], points_world[2, [2,3]], '-', lw=2)
        ax.plot(points_world[0, [0,2]], points_world[1, [0,2]], points_world[2, [0,2]], '-', lw=2)

    def draw(self, ax, Ts, cube_faces=None, cube_centroid=None):
        ax.cla()  # clear SAME plot

        X = [T[0,3] for T in Ts]
        Y = [T[1,3] for T in Ts]
        Z = [T[2,3] for T in Ts]

        ax.plot(X, Y, Z, '-o', linewidth=2, markersize=6)

        if cube_faces is not None:
            ax.add_collection3d(
                Poly3DCollection(cube_faces, facecolors='skyblue', edgecolors='k', alpha=0.5)
            )

        if cube_centroid is not None:
            ax.scatter(*cube_centroid, color='r', s=50)

        self.draw_gripper(ax, Ts[-1])

        ax.set_xlim3d([-self.limits, self.limits])
        ax.set_ylim3d([-self.limits, self.limits])
        ax.set_zlim3d([-self.limits, self.limits])
        ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
        ax.set_box_aspect([1,1,1])
