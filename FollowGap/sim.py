"""
SIMULATION
Tests sans robot réel.

Rôle :
- Générer de faux scans LiDAR
- Tester les behaviors et la navigation
- Permettre le debug rapide

Très utile pour :
- développer sans hardware
- valider les algorithmes
"""
import numpy as np
import os
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def generate_multi_gap_lidar_xyz(
    num_points=360,
    max_range=10.0,
    noise_std=0.05
):
    angles = np.linspace(-np.pi, np.pi, num_points)
    ranges = np.ones(num_points) * max_range

    # 🔹 Étape 1 : créer des obstacles (zones bloquées)
    obstacles = [
        (-1.8, -1.2, 1.5),   # obstacle gauche
        (-0.8, -0.3, 2.0),   # obstacle centre gauche
        (0.2, 0.6, 1.2),     # obstacle centre droit
        (1.0, 1.6, 1.8)      # obstacle droite
    ]

    for (a_min, a_max, dist) in obstacles:
        mask = (angles > a_min) & (angles < a_max)
        ranges[mask] = dist + np.random.normal(0, noise_std, np.sum(mask))

    # 🔹 Étape 2 : créer des gaps (zones ouvertes)
    gaps = [
        (-0.2, 0.1),   # gap 1 (petit)
        (0.7, 0.9)     # gap 2 (plus large)
    ]

    for (a_min, a_max) in gaps:
        mask = (angles > a_min) & (angles < a_max)
        ranges[mask] = max_range - np.random.uniform(0, 1, np.sum(mask))

    # 🔹 Étape 3 : ajouter bruit global
    ranges += np.random.normal(0, noise_std, num_points)

    # 🔹 Étape 4 : clamp (éviter valeurs négatives)
    ranges = np.clip(ranges, 0.05, max_range)

    # 🔹 Étape 5 : conversion XYZ
    x = ranges * np.cos(angles)
    y = ranges * np.sin(angles)
    z = np.zeros_like(x)

    points = np.stack((x, y, z), axis=1)

    return points, angles, ranges

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ===============================
# LiDAR simulation
# ===============================
def simulate_lidar(position, obstacles, n_rays=360, max_range=10.0):
    angles = np.linspace(-np.pi, np.pi, n_rays)
    distances = np.full(n_rays, max_range)

    for i, theta in enumerate(angles):
        direction = np.array([np.cos(theta), np.sin(theta)])

        for obs in obstacles:
            oc = position - obs["center"]

            b = 2 * np.dot(direction, oc)
            c = np.dot(oc, oc) - obs["radius"]**2
            delta = b**2 - 4*c

            if delta >= 0:
                t = (-b - np.sqrt(delta)) / 2
                if 0 < t < distances[i]:
                    distances[i] = t

    return np.column_stack((distances, angles))


# ===============================
# ENVIRONNEMENT
# ===============================
obstacles = [
    {"center": np.array([4, 0]), "radius": 1},
    {"center": np.array([0, 5]), "radius": 1},
    {"center": np.array([0, -3]), "radius": 1},
    {"center": np.array([4, 4]), "radius": 0.5},
    {"center": np.array([2, -2]), "radius": 1},
]

goal = np.array([10, -2])


# ===============================
# ANIMATION (FIX GIF STOP)
# ===============================
def animate_simulation(fg):
    fig, ax = plt.subplots(figsize=(6,6))

    pos = np.array([0.0, 0.0])
    heading = 0.0
    traj = [pos.copy()]

    done = False
    anim = None

    traj_line, = ax.plot([], [], '-b', label="Trajectory")
    robot_point = ax.scatter([], [], c='blue', s=50)

    # obstacles
    for obs in obstacles:
        circle = plt.Circle(obs["center"], obs["radius"], color='r', alpha=0.5)
        ax.add_patch(circle)

    # goal
    ax.scatter(goal[0], goal[1], c='g', s=100, label="Goal")

    ax.set_xlim(-2, 12)
    ax.set_ylim(-5, 12)
    ax.set_aspect('equal')
    ax.grid()
    ax.legend()

    def update(frame):
        nonlocal pos, heading, traj, anim, done

        if done:
            raise StopIteration  # 🔥 STOP GIF propre

        # --- TON CODE (inchangé) ---
        scan = simulate_lidar(pos, obstacles)

        scan[:,1] -= heading
        scan[:,1] = (scan[:,1] + np.pi) % (2*np.pi) - np.pi

        theta_goal = np.arctan2(goal[1] - pos[1], goal[0] - pos[0])
        theta_goal -= heading
        theta_goal = (theta_goal + np.pi) % (2*np.pi) - np.pi

        idx, theta_target, debug_scan = fg.compute(scan, theta_goal)

        if theta_target is None:
            done = True
            anim.event_source.stop()
            raise StopIteration

        max_turn_rate = 0.3
        angle_diff = (theta_target + np.pi) % (2*np.pi) - np.pi
        angle_diff = np.clip(angle_diff, -max_turn_rate, max_turn_rate)

        heading += angle_diff

        step_size = 0.2
        pos += step_size * np.array([np.cos(heading), np.sin(heading)])

        traj.append(pos.copy())

        # update plot
        traj_np = np.array(traj)
        traj_line.set_data(traj_np[:,0], traj_np[:,1])
        robot_point.set_offsets(pos)

        # 🔹 STOP AU GOAL
        if np.linalg.norm(pos - goal) < 0.1:
            print("Goal reached!")
            ax.set_title("Goal reached!")
            done = True
            anim.event_source.stop()
            raise StopIteration

        return traj_line, robot_point

    # 🔥 générateur infini MAIS stoppé proprement
    def frame_gen():
        while True:
            yield 0

    anim = FuncAnimation(fig, update, frames=frame_gen, interval=50, blit=True)

    # 🔹 sauvegarde GIF
    anim.save("simulation.gif", writer="pillow", fps=10)

    plt.show()


# ===============================
# RUN
# ===============================
if __name__ == "__main__":
    from follow_gap import FollowGap

    fg = FollowGap()

    animate_simulation(fg)