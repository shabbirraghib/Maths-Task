import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R

def euler_to_quaternion(roll, pitch, yaw, degrees=True):
    """
    Convert Euler angles (roll, pitch, yaw) to quaternion (w, x, y, z).
    """
    if degrees:
        roll = np.radians(roll)
        pitch = np.radians(pitch)
        yaw = np.radians(yaw)

    cr = np.cos(roll / 2)
    sr = np.sin(roll / 2)
    cp = np.cos(pitch / 2)
    sp = np.sin(pitch / 2)
    cy = np.cos(yaw / 2)
    sy = np.sin(yaw / 2)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    quaternion = np.array([w, x, y, z])

    norm = np.linalg.norm(quaternion)
    if norm < 1e-10:
        return np.array([1.0, 0.0, 0.0, 0.0])  # Identity quaternion
    return quaternion / norm

def quaternion_to_euler(w, x, y, z, degrees=True):
    """
    Convert quaternion (w, x, y, z) to Euler angles (roll, pitch, yaw).
    """
    quaternion = np.array([w, x, y, z])
    norm = np.linalg.norm(quaternion)
    if norm < 1e-10:
        return (0.0, 0.0, 0.0) if degrees else (0.0, 0.0, 0.0)
    quaternion = quaternion / norm
    w, x, y, z = quaternion

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x**2 + y**2)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = np.sign(sinp) * np.pi / 2
    else:
        pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y**2 + z**2)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    if degrees:
        roll = np.degrees(roll)
        pitch = np.degrees(pitch)
        yaw = np.degrees(yaw)

        # Wrap to [-180, 180]
        roll = (roll + 180) % 360 - 180
        pitch = (pitch + 180) % 360 - 180
        yaw = (yaw + 180) % 360 - 180

    return roll, pitch, yaw

def visualize_orientation(roll, pitch, yaw, degrees=True):
    """
    Visualize the orientation defined by Euler angles as 3D axes.
    """
    if degrees:
        rotation = R.from_euler('xyz', [roll, pitch, yaw], degrees=True)
    else:
        rotation = R.from_euler('xyz', [roll, pitch, yaw])

    origin = np.zeros((3,))
    x_axis = np.array([1, 0, 0])
    y_axis = np.array([0, 1, 0])
    z_axis = np.array([0, 0, 1])

    x_rot = rotation.apply(x_axis)
    y_rot = rotation.apply(y_axis)
    z_rot = rotation.apply(z_axis)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.quiver(*origin, *x_rot, color='r', label='X-axis')
    ax.quiver(*origin, *y_rot, color='g', label='Y-axis')
    ax.quiver(*origin, *z_rot, color='b', label='Z-axis')

    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    ax.set_title(f"Orientation: Roll={roll:.1f}, Pitch={pitch:.1f}, Yaw={yaw:.1f}")
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    plt.show()

def test_conversions():
    """
    Test conversion functions with various edge and normal cases.
    """
    test_cases = [
        {"euler": (30, 45, 60), "desc": "Normal case"},
        {"euler": (0, 0, 0), "desc": "Zero rotation"},
        {"euler": (0, 90, 0), "desc": "Gimbal lock (pitch = 90°)"},
        {"euler": (0, -90, 0), "desc": "Gimbal lock (pitch = -90°)"},
        {"euler": (720, 360, -720), "desc": "Large angles"},
    ]

    for case in test_cases:
        roll, pitch, yaw = case["euler"]
        print(f"\n--- Testing {case['desc']} ---")
        print(f"Euler angles: (Roll={roll}, Pitch={pitch}, Yaw={yaw})")

        q = euler_to_quaternion(roll, pitch, yaw)
        print(f"Quaternion: [w={q[0]:.4f}, x={q[1]:.4f}, y={q[2]:.4f}, z={q[3]:.4f}]")

        roll_back, pitch_back, yaw_back = quaternion_to_euler(*q)
        print(f"Back to Euler: (Roll={roll_back:.4f}, Pitch={pitch_back:.4f}, Yaw={yaw_back:.4f})")

        norm = np.linalg.norm(q)
        print(f"Quaternion norm: {norm:.4f}")

        # Visualize orientation for normal case only
        if case["desc"] == "Normal case":
            visualize_orientation(roll, pitch, yaw)

if __name__ == "__main__":
    np.set_printoptions(precision=4, suppress=True)
    test_conversions()
