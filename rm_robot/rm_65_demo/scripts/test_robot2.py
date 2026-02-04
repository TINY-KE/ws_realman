import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def standard_dh_transform(theta, d, a, alpha):
    """标准DH变换矩阵"""
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    
    T = np.array([
        [ct, -st, 0, 0],
        [st, ct, 0, 0],
        [0, 0, 1, d],
        [0, 0, 0, 1]
    ]) @ np.array([
        [1, 0, 0, a],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ]) @ np.array([
        [1, 0, 0, 0],
        [0, ca, -sa, 0],
        [0, sa, ca, 0],
        [0, 0, 0, 1]
    ])
    
    return T

def draw_frame(ax, T, scale=0.05, label=''):
    """绘制坐标系"""
    origin = T[:3, 3]
    x_axis = T[:3, 0] * scale
    y_axis = T[:3, 1] * scale
    z_axis = T[:3, 2] * scale
    
    # X轴 - 红色
    ax.quiver(origin[0], origin[1], origin[2],
              x_axis[0], x_axis[1], x_axis[2],
              color='r', arrow_length_ratio=0.3, linewidth=2)
    
    # Y轴 - 绿色
    ax.quiver(origin[0], origin[1], origin[2],
              y_axis[0], y_axis[1], y_axis[2],
              color='g', arrow_length_ratio=0.3, linewidth=2)
    
    # Z轴 - 蓝色
    ax.quiver(origin[0], origin[1], origin[2],
              z_axis[0], z_axis[1], z_axis[2],
              color='b', arrow_length_ratio=0.3, linewidth=2)
    
    # 标注
    if label:
        ax.text(origin[0], origin[1], origin[2], label, fontsize=10)

# DH参数
alpha = np.array([0, 0, np.pi/2, -np.pi/2, np.pi/2, 0])
a = np.array([0.2405, 0.256, 0, 0.0, 0, 0])
d = np.array([0, 0, 0, 0.210, 0, 0.144])
theta = np.array([0, 0, np.pi/2, 0, 0, 0])

# 基座变换
base_T = np.array([
    [1, 0, 0, 0],
    [0, 0, 1, 0],
    [0, -1, 0, 0],
    [0, 0, 0, 1]
])  # Ry(-90°)

# 计算每个关节的变换矩阵
transforms = [base_T]
current_T = base_T.copy()

for i in range(6):
    T_i = standard_dh_transform(theta[i], d[i], a[i], alpha[i])
    current_T = current_T @ T_i
    transforms.append(current_T.copy())

# 创建3D图形
fig = plt.figure(figsize=(12, 10))
ax = fig.add_subplot(111, projection='3d')

# 绘制所有坐标系
origins = []
for i, T in enumerate(transforms):
    label = f'Joint {i}' if i > 0 else 'Base'
    draw_frame(ax, T, scale=0.08, label=label)
    origins.append(T[:3, 3])

# 绘制连接线
origins = np.array(origins)
ax.plot(origins[:, 0], origins[:, 1], origins[:, 2], 
        'ko-', linewidth=2, markersize=6, label='Robot链')

# 设置图形属性
ax.set_xlabel('X (m)', fontsize=12)
ax.set_ylabel('Y (m)', fontsize=12)
ax.set_zlabel('Z (m)', fontsize=12)
ax.set_title('Realman65 机械臂 DH 坐标系可视化', fontsize=14, fontweight='bold')

# 设置相等的坐标轴比例
max_range = 0.6
ax.set_xlim([-0.1, max_range])
ax.set_ylim([-max_range/2, max_range/2])
ax.set_zlim([0, max_range])

# 添加网格
ax.grid(True, alpha=0.3)

# 添加图例
from matplotlib.lines import Line2D
legend_elements = [
    Line2D([0], [0], color='r', linewidth=2, label='X轴'),
    Line2D([0], [0], color='g', linewidth=2, label='Y轴'),
    Line2D([0], [0], color='b', linewidth=2, label='Z轴'),
    Line2D([0], [0], color='k', marker='o', linewidth=2, label='关节连接')
]
ax.legend(handles=legend_elements, loc='upper right')

# 打印关节位置
print("各关节在基坐标系中的位置：")
for i, origin in enumerate(origins):
    label = f"Joint {i}" if i > 0 else "Base"
    print(f"{label}: [{origin[0]:.4f}, {origin[1]:.4f}, {origin[2]:.4f}]")

plt.tight_layout()
plt.show()