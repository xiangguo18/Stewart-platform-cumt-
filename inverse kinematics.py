import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
def rotx(phi): 
    if phi > np.pi:
        phi = np.radians(phi)
    rotX = np.array([
    [1, 0, 0],
    [0, np.cos(phi), -np.sin(phi)],
    [0, np.sin(phi), np.cos(phi)]
    ])
    return rotX 
 
def roty(theta): 
    if theta > np.pi:
        theta  = np.radians(theta)
    rotY = np.array([
    [np.cos(theta), 0, np.sin(theta)],
    [0, 1, 0],
    [-np.sin(theta), 0, np.cos(theta)]
    ])
    return rotY  

def rotz(psi): 
    if psi  > np.pi:
        psi = np.radians(psi)
    rotZ = np.array([
    [np.cos(psi), -np.sin(psi), 0],
    [np.sin(psi), np.cos(psi), 0],
    [ 0, 0, 1]
    ])
    return rotZ 


"""
计算平台参数
"""

#输入参数
r_b = 70
r_p = 54

servo_arm = 23
leg =  126                    #rod or leg   126 is leg 

alpha_b = 27.4307

alpha_p = 16.6714

home = np.array([                   #零位  位移  旋转
    0,
    0,
    127
    ])
orint = np.array([
    0,
    0,
    0
    ])
trans = np.array([
    0,
    0,
    0
    ])                  


#定义平台几何尺寸
pi = np.pi
beta = np.array([
    pi + pi / 2, 
    pi / 2,
    2 * pi / 3 + pi + pi / 2, 
    2 * pi / 3 + pi / 2,
    4 * pi / 3 + pi + pi / 2, 
    4 * pi / 3 + pi / 2
])

theta_b = np.array([
    alpha_b,
    alpha_b,
    pi/3+alpha_b,
    pi/3-alpha_b,
    pi/3-alpha_b,
    pi/3+alpha_b
    ])

theta_p = np.array([
    pi/3-alpha_p,
    pi/3-alpha_b,
    pi/3+alpha_b,
    alpha_b,
    alpha_b,
    pi/3+alpha_b
    ])

servo_b = r_b * np.array([
    [np.cos(theta_b[0]), -np.sin(theta_b[0]), 0],
    [np.cos(theta_b[1]),  np.sin(theta_b[1]), 0],
    [-np.cos(theta_b[2]), np.sin(theta_b[2]), 0],
    [-np.cos(theta_b[3]), np.sin(theta_b[3]), 0],
    [-np.cos(theta_b[4]), -np.sin(theta_b[4]), 0],
    [-np.cos(theta_b[5]), -np.sin(theta_b[5]), 0]
])
rod_p = r_p * np.array([
    [np.cos(theta_p[0]), -np.sin(theta_p[0]), 0],
    [np.cos(theta_p[1]),  np.sin(theta_p[1]), 0],
    [np.cos(theta_p[2]),  np.sin(theta_p[2]), 0],
    [-np.cos(theta_p[3]), np.sin(theta_p[3]), 0],
    [-np.cos(theta_p[4]), -np.sin(theta_p[4]), 0],
    [np.cos(theta_p[5]), -np.sin(theta_p[5]), 0]
])
"""
print('servo_b is /n',servo_b)
print('rod_p is /n',rod_p)
print(rod_p[:,1]) 

fig = plt.figure()                                 #创建图对象
ax = fig.add_subplot(111, projection='3d')         #三维图
ax.plot(servo_b[:,0], servo_b[:,1], servo_b[:,2], '-')
plt.grid(True)
plt.show()

"""
'''
计算  有效杆长(电动缸长度) rod
'''

#计算旋转矩阵
T_BP = np.dot(rotz(orint[2]),np.dot(roty(orint[1]),rotx(orint[0])))        #T_BP 有错可能   phi theta psi 换成 orient

#计算rod_p
rod_b = np.dot(T_BP,rod_p) + home + trans     

#计算rod
L_B = rod_b - servo_b    
rod = np.linalg.norm(L_B)                          #输出

#test
print("T_BP:",T_BP)
print(np.linalg.norm([1,2,3]))



'''
计算  舵机转角 angle
'''

 #输入
beta =     #角度值
beta = np.radions(beta)

 #获得坐标
x_p = rod_p[0:]
y_p = rod_p[1:]
z_p = rod_p[2:]

x_b = rod_b[0:]
y_b = rod_b[1:]
z_b = rod_b[2:]

 #更新舵机转角     矩阵运算还是数值运算？
rod2 = rod * rod        # matlab/数学运算中.*是等于numpy库中的矩阵A*矩阵B
leg2 = leg * leg
arm2 = servo_arm * servo_arm

l = rod2 - leg2 + arm2
M = 2 * servo_arm * (z_p - z_b)
N = 2 * servo_arm * (np.cos(beta) * (x_p - x_b) + np.sin(beta) * (y_p - y_b))   # .*?
M2 = M*M
N2 = N*N

#计算角度
angle = np.arcsin(l / (M2 - N2)**0.5) -np.arctan(N,M)     #输出

angles = np.zeros(6)
joint_b = np.zeros((3, 6))

for i in range(6):
    N = 2 * servo_arm * (np.cos(beta[i]) * (x_p[i] - x_b[i]) + np.sin(beta[i]) * (y_p[i] - y_b[i]))
    
    # 计算角度
    angles[i] = np.arcsin(l[i] / np.sqrt(M[i]**2 + N**2)) - np.arctan2(N, M[i])

    # 计算连接点位置
    joint_b[:, i] = [
        servo_arm * np.cos(angles[i]) * np.cos(beta[i]) + servo_b[0, i],
        servo_arm * np.cos(angles[i]) * np.sin(beta[i]) + servo_b[1, i],
        servo_arm * np.sin(angles[i])
    ]

"""
#画图
"""

fig = plt.figure()                                 #创建图对象
ax = fig.add_subplot(111, projection='3d')         #三维图

# 绘制三维填充多边形
ax.plot(servo_b[0], servo_b[1], servo_b[2], '-')
leg = leg + servo_b
ax.plot(leg[0], leg[1], leg[2], '-g')

# 设置坐标轴范围
r_B = 1  # 替换为适当的值

ax.set_xlim([-r_B-servo_arm, r_B+servo_arm])
ax.set_ylim([-r_B-servo_arm, r_B+servo_arm])
ax.set_zlim([-servo_arm, rod+servo_arm])

# 绘制线段
for i in range(6):
    ax.plot([servo_b[0, i], joint_b[0, i]], 
            [servo_b[1, i], joint_b[1, i]], 
            [servo_b[2, i], joint_b[2, i]], 
            color='r', linewidth=2)

    ax.plot([joint_b[0, i], leg[0, i]], 
            [joint_b[1, i], leg[1, i]], 
            [joint_b[2, i], leg[2, i]], 
            color='k')

    ax.plot([servo_b[0, i], leg[0, i]], 
            [servo_b[1, i], leg[1, i]], 
            [servo_b[2, i], leg[2, i]], 
            color='y')

# 启用交互式旋转
plt.grid(True)
plt.show()


