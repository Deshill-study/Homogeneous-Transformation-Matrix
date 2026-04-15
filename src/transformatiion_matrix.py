import numpy as np

class Transformation:
    def __init__(self):
        self.theta = 0
    def rot_y(self,theta):
        """绕 Y 轴旋转 theta 弧度"""
        c, s = np.cos(theta), np.sin(theta)
        return np.array([
            [c, 0, -s, 0],
            [0,  1, 0, 0],
            [s,  0, c, 0],
            [0,  0, 0, 1]
        ])
    def rot_x(self,theta):
        """绕 X 轴旋转 theta 弧度"""
        c, s = np.cos(theta), np.sin(theta)
        return np.array([
            [1, 0,  0, 0],
            [0, c, -s, 0],
            [0, s,  c, 0],
            [0, 0,  0, 1]
        ])
    # 定义变换矩阵函数
    def rot_z(self,theta):
        """绕 Z 轴旋转 theta 弧度"""
        c, s = np.cos(theta), np.sin(theta)
        return np.array([
            [c, -s, 0, 0],
            [s,  c, 0, 0],
            [0,  0, 1, 0],
            [0,  0, 0, 1]
        ])

    def translate(self,x, y, z=0):
        """平移变换"""
        return np.array([
            [1, 0, 0, x],
            [0, 1, 0, y],
            [0, 0, 1, z],
            [0, 0, 0, 1]
        ])
    

# ======================================================
#           齐次变换矩阵 - 移动坐标系变换
#           具体题目查看image文件夹下的q1.png
# ======================================================
transformation = Transformation()

# 参数
L1, L2 = 2, 1.5
theta1, theta2 = np.radians(30), np.radians(45)

# 变换链：基座 → 关节1 → 关节2 → 末端
T_base = np.eye(4)  # 基座（单位矩阵)
T_joint1 = transformation.rot_z(theta1) @ transformation.translate(L1, 0)  # 旋转后平移
T_joint2 = transformation.rot_z(theta2) @ transformation.translate(L2, 0)  # 第二个关节

# 总变换
# 因为是动坐标，所以坐标系要从左到右乘过去
T_total = T_base @ T_joint1 @ T_joint2

# 最终得到的T_total就是指的是，T_base坐标系到T_joint2坐标系的齐次变换矩阵,也就是说，在joint2坐标系下的某个点，在base坐标系下是哪个坐标，就用那个点在joint2的坐标右乘T_total

# 末端位置（齐次坐标）
end_effector = T_total @ np.array([0, 0, 0, 1])
print(f"末端位置: ({end_effector[0]:.3f}, {end_effector[1]:.3f})")
# 输出: 末端位置: (1.366, 2.299)

# ======================================================
#           几何法进行验证
#           具体题目查看image文件夹下的q1.png
#           验证结果和齐次变换矩阵一摸一样
# ======================================================

x = L1*np.cos(theta1)+L2*np.cos(theta2+theta1)
y = L1*np.sin(theta1)+L2*np.sin(theta2+theta1)

print(f"末端位置: ({x:.3f}, {y:.3f})")


# ======================================================
#           齐次变换矩阵 - 定坐标系变换
#           具体题目查看image文件夹下的q2.png
#           需要注意这时候先变换的要写在右边
# ======================================================

theta1, theta2, theta3 = np.radians(45), np.radians(30),np.radians(15)
T_total = transformation.translate(10,0,0)@transformation.rot_x(theta3)@transformation.rot_y(theta2)@transformation.rot_z(theta1)

end_effector = T_total @ np.array([1, 0, 0, 1])
print(f"末端位置: ({end_effector[0]:.3f}, {end_effector[1]:.3f},{end_effector[1]:.3f})")
