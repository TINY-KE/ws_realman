#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt

# ===============================
# 1. DH 参数（你给的参数）
# ===============================
alpha = np.array([np.pi/2,  -np.pi/2,  np.pi/2, -np.pi/2,np.pi/2, 0])
a     = np.array([0,  0,    0.256,  0,        0,        0])
d     = np.array([0,  0,   0,  0.2405,        0,    0.144])


# ===============================
# 2. 标准 DH 变换（Craig DH）
# ===============================
def dh_transform(a, alpha, d, theta):
    ca, sa = np.cos(alpha), np.sin(alpha)
    ct, st = np.cos(theta), np.sin(theta)

    return np.array([
        [ ct, -st*ca,  st*sa, a*ct],
        [ st,  ct*ca, -ct*sa, a*st],
        [  0,     sa,     ca,    d],
        [  0,      0,      0,    1]
    ])


# ===============================3
# 3. 前向运动学（返回各关节原点）
# ===============================
def forward_kinematics(alpha, a, d, q):
    T = np.eye(4)
    points = [T[:3, 3].copy()]  # base

    for i in range(6):
        Ti = dh_transform(a[i], alpha[i], d[i], q[i])
        T = T @ Ti
        points.append(T[:3, 3].copy())

    return np.array(points)


# ===============================
# 4. 绘图函数
# ===============================
def plot_arm(points, title):
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(points[:, 0], points[:, 1], points[:, 2],
            '-o', linewidth=3, markersize=8)

    # 标注关节编号
    for i, p in enumerate(points):
        ax.text(p[0], p[1], p[2], f'{i}', fontsize=12)

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title(title)

    ax.set_box_aspect([1, 1, 1])

    # 让视角更直观
    ax.view_init(elev=30, azim=45)

    plt.tight_layout()
    plt.show()


# ===============================
# 5. 主程序：三种验证
# ===============================
if __name__ == "__main__":

    # -------- 验证 1：零位姿 --------
    q_zero = np.zeros(6)
    pts_zero = forward_kinematics(alpha, a, d, q_zero)
    plot_arm(pts_zero, "DH Arm Skeleton (q = [0,0,0,0,0,0])")

    # # -------- 验证 2：只动 joint2 --------
    # q_j2 = np.zeros(6)
    # q_j2[1] = 0.6
    # pts_j2 = forward_kinematics(alpha, a, d, q_j2)
    # plot_arm(pts_j2, "Only Joint 2 Moves (q2 = 0.6 rad)")

    # # -------- 验证 3：只动 joint3 --------
    # q_j3 = np.zeros(6)
    # q_j3[2] = 0.6
    # pts_j3 = forward_kinematics(alpha, a, d, q_j3)
    # plot_arm(pts_j3, "Only Joint 3 Moves (q3 = 0.6 rad)")