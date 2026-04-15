# ======================================================
#                  雅可比矩阵的应用
# ======================================================

class Jacobian():
    def __init__(self):
        self.a = 0
    
import numpy as np

def compute_jacobian(robot_config, joint_angles):
    """
    计算6自由度机械臂的雅可比矩阵
    
    Args:
        robot_config: 机器人D-H参数
        joint_angles: 关节角度 [θ₁, θ₂, ..., θ₆]
    
    Returns:
        J: 6×6 雅可比矩阵
    """
    n = len(joint_angles)
    J = np.zeros((6, n))
    
    # 计算各关节位置和方向
    T = np.eye(4)
    positions = []
    axes = []
    
    for i in range(n):
        # 计算当前关节的变换矩阵
        T_i = dh_transform(robot_config[i], joint_angles[i])
        T = T @ T_i
        
        # 提取位置和旋转轴
        positions.append(T[:3, 3])
        axes.append(T[:3, 2])  # Z轴方向
    
    end_pos = positions[-1]
    
    # 构建雅可比
    for i in range(n):
        # 线速度分量
        J[:3, i] = np.cross(axes[i], end_pos - positions[i])
        # 角速度分量
        J[3:, i] = axes[i]
    
    return J

def dh_transform(dh_params, theta):
    """根据D-H参数计算变换矩阵"""
    d, a, alpha = dh_params['d'], dh_params['a'], dh_params['alpha']
    
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    
    T = np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,      sa,     ca,    d],
        [0,       0,      0,    1]
    ])
    
    return T



# ======================================================
#                   6个关节，3D
# ======================================================

# 6 个关节的角度，单位：弧度
current_angles = np.array([0.5, 0.3, 0.2, 0.0, 0.0, 0.0])

# dh参数
robot_config = [
    # 关节1
    {'d': 0.4, 'a': 0.0, 'alpha': np.pi/2},
    # 关节2
    {'d': 0.0, 'a': 0.5, 'alpha': 0.0},
    # 关节3
    {'d': 0.0, 'a': 0.4, 'alpha': 0.0},
    # 关节4
    {'d': 0.0, 'a': 0.0, 'alpha': -np.pi/2},
    # 关节5
    {'d': 0.1, 'a': 0.0, 'alpha': np.pi/2},
    # 关节6
    {'d': 0.0, 'a': 0.0, 'alpha': 0.0}
]
joint_velocities = np.array([0.1, 0.2, 0.1, 0.0, 0.0, 0.0])  # rad/s (每个关节速度)
J = compute_jacobian(robot_config, current_angles)
end_effector_velocity = J @ joint_velocities
print(f'末尾关节的速度为{end_effector_velocity}')


# ================================================================
#                   2维平面2个关节
#               2个关节长度为1,各自转45度，速度为1rad/s，求关节末速度
# ================================================================

robot_config_2dof = [
    # 关节1
    {'d': 0, 'a': 1, 'alpha': 0},
    # 关节2
    {'d': 0, 'a': 1, 'alpha': 0}
]

current_angles_2dof = np.array([np.pi/4, np.pi/4])   # 45° + 45°

J = compute_jacobian(robot_config_2dof, current_angles_2dof)
print("雅可比矩阵 J (6×2)：")
print(np.round(J, 3))
joint_velocities = np.array([1, 1])  # 两个关节都以 1 rad/s 转动
end_velocities = J @ joint_velocities
print("\n末端速度 [vx, vy, vz, wx, wy, wz]:")
print(np.round(end_velocities, 3))


