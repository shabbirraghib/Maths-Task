# 🧮 Maths Task

📌 GitHub Repository: [https://github.com/shabbirraghib/Maths-Task](https://github.com/shabbirraghib/Maths-Task)

📥 Clone this repo using:

```bash
git clone https://github.com/shabbirraghib/Maths-Task.git
```

This repository contains solutions to two mathematical robotics tasks written in Python:

- ✅ **Task 1:** Conversion between Euler angles and Quaternions
- ✅ **Task 2:** Forward Kinematics for a 4-joint robotic arm

---

## 📁 Files

- `task_1.py` — Euler to Quaternion conversion and back (with optional 3D visualization)
- `task_2.py` — Forward Kinematics for a 4-DOF robot arm (with optional 3D visualization)

---

## ✅ Task 1: Euler ↔ Quaternion Conversion

### Description:
This script performs conversion between **Euler angles (roll, pitch, yaw)** and **quaternions (w, x, y, z)**. It also includes:

- Normalization of quaternions
- Handling of edge cases (e.g., gimbal lock, zero rotation)
- Optional 3D visualization of the orientation using `matplotlib`

### How to Run:

```bash
python3 task_1.py
```
Sample Output:
```bash
--- Testing Normal case ---
Euler angles: (Roll=30, Pitch=45, Yaw=60)
Quaternion: [w=0.8224, x=0.2006, y=0.3919, z=0.3604]
Back to Euler: (Roll=30.0000, Pitch=45.0000, Yaw=60.0000)
```


## ✅ Task 2: Forward Kinematics of a Robotic Arm

### Description:

This script calculates the 3D position of each joint and the end-effector of a robotic arm with 4 revolute joints.

- Each link has a fixed length L (default: 1 meter)

- The axes of joints are perpendicular to each other

- Visualization of the robotic arm in 3D space is included

### How to Run:

```bash
python3 task_2.py
```

Sample Output:
```bash
Joint Angles (degrees): [30, 45, 60, 90]
End-effector 3D Coordinates: x=1.5731, y=1.0000, z=1.7071
```
