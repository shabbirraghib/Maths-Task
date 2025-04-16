import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def forward_kinematics(j1, j2, j3, j4, L=1.0):
    """
    Compute the 3D position of the end-effector for a 4-joint robotic arm.

    Args:
        j1, j2, j3, j4 (float): Joint angles in degrees
        L (float): Length of each link (default is 1.0 meter)

    Returns:
        list: [x_coords, y_coords, z_coords] of all joints and end-effector
    """
    # Convert joint angles to radians
    j1, j2, j3, j4 = np.radians([j1, j2, j3, j4])

    # Homogeneous transformation matrices
    def rot_z(theta):
        return np.array([
            [np.cos(theta), -np.sin(theta), 0, 0],
            [np.sin(theta),  np.cos(theta), 0, 0],
            [0,              0,             1, 0],
            [0,              0,             0, 1]
        ])

    def rot_y(theta):
        return np.array([
            [np.cos(theta),  0, np.sin(theta), 0],
            [0,              1, 0,             0],
            [-np.sin(theta), 0, np.cos(theta), 0],
            [0,              0, 0,             1]
        ])

    def rot_x(theta):
        return np.array([
            [1, 0,              0,             0],
            [0, np.cos(theta), -np.sin(theta), 0],
            [0, np.sin(theta),  np.cos(theta), 0],
            [0, 0,              0,             1]
        ])

    def trans_z(d):
        return np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, d],
            [0, 0, 0, 1]
        ])

    def trans_x(d):
        return np.array([
            [1, 0, 0, d],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

    def trans_y(d):
        return np.array([
            [1, 0, 0, 0],
            [0, 1, 0, d],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

    # Start from origin
    T = np.eye(4)
    points = [T[:3, 3].copy()]  # initial position at origin

    # Joint 1: rotate around Z, then move along X by L
    T = T @ rot_z(j1) @ trans_x(L)
    points.append(T[:3, 3].copy())

    # Joint 2: rotate around Y, then move along X by L
    T = T @ rot_y(j2) @ trans_x(L)
    points.append(T[:3, 3].copy())

    # Joint 3: rotate around X, then move along Z by L
    T = T @ rot_x(j3) @ trans_z(L)
    points.append(T[:3, 3].copy())

    # Joint 4: rotate around Y, then move along Z by L
    T = T @ rot_y(j4) @ trans_z(L)
    points.append(T[:3, 3].copy())

    # Extract X, Y, Z coordinates
    x_coords = [pt[0] for pt in points]
    y_coords = [pt[1] for pt in points]
    z_coords = [pt[2] for pt in points]

    return [x_coords, y_coords, z_coords]

def visualize_arm(j1, j2, j3, j4, L=1.0):
    """
    Plot the 3D visualization of the robotic arm.

    Args:
        j1, j2, j3, j4 (float): Joint angles in degrees
        L (float): Length of each link
    """
    x, y, z = forward_kinematics(j1, j2, j3, j4, L)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot links
    ax.plot(x, y, z, 'bo-', label='Arm links', markersize=6)

    # Mark joints
    for i in range(len(x)):
        ax.scatter(x[i], y[i], z[i], c='r', s=50)
        ax.text(x[i], y[i], z[i], f'J{i}', fontsize=10)

    ax.set_xlabel('X axis (m)')
    ax.set_ylabel('Y axis (m)')
    ax.set_zlabel('Z axis (m)')
    ax.set_title(f'Forward Kinematics\nJoint Angles: ({j1}°, {j2}°, {j3}°, {j4}°)')
    ax.set_box_aspect([1, 1, 1])
    ax.legend()
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # Example usage
    joint_angles = [30, 45, 60, 90]  # You can change this
    link_length = 1.0  # Adjustable

    print(f"\nJoint Angles (degrees): {joint_angles}")
    coords = forward_kinematics(*joint_angles, L=link_length)
    end_effector = (coords[0][-1], coords[1][-1], coords[2][-1])
    print(f"End-effector 3D Coordinates: x={end_effector[0]:.4f}, y={end_effector[1]:.4f}, z={end_effector[2]:.4f}")

    # Optional visualization
    visualize_arm(*joint_angles, L=link_length)
