import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
def rotx(phi):                #输入度数
    phi = np.radians(phi)
    rotX = np.array([
    [1, 0, 0],
    [0, np.cos(phi), -np.sin(phi)],
    [0, np.sin(phi), np.cos(phi)]
    ])
    return rotX 
 
def roty(theta): 
    theta  = np.radians(theta)
    rotY = np.array([
    [np.cos(theta), 0, np.sin(theta)],
    [0, 1, 0],
    [-np.sin(theta), 0, np.cos(theta)]
    ])
    return rotY  

def rotz(psi): 
    psi = np.radians(psi)
    rotZ = np.array([
    [np.cos(psi), -np.sin(psi), 0],
    [np.sin(psi), np.cos(psi), 0],
    [ 0, 0, 1]
    ])
    return rotZ 

def atan(a):               #返回弧度
    return np.arctan(a)

def atand(a):              #返回角度
    pi=np.pi
    return np.arctan(a)*180/pi

def cos(radian):
    return np.cos(radian)

def sin(radain):
    return np.sin(radain)


"""
计算平台参数
"""
#输入参数
servo_arm = np.array([23,23,23,23,23,23])
leg =  np.array([126,126,126,126,126,126])                   #rod or leg   126 is leg 

                                  #零位  位移  旋转
 
home=np.array([[0],[0],[123.12]])       #舵机-球铰123.12，舵机- 顶面127.12，舵机-底面23.15，顶面-底面15.27，顶面-球铰4      
orint=np.array([[0],[0],[0]])
trans=np.array([[0],[0],[0]])

#定义平台几何尺寸
servo_b = np.array([
    [-30, -59.3, 0],
    [30, -59.3 , 0],
    [66.36,  3.67, 0],
    [36.36, 55.63, 0],
    [-36.36, 55.63, 0],
    [-66.36, 3.67, 0]
])
rod_p = np.array([
    [-39.34, -40.03, 0],
    [39.34, -40.03 , 0],
    [54.34,  -14.05, 0],
    [15, 54.08, 0],
    [-15, 54.08, 0],
    [-54.34, -14.05, 0]
])

#print('servo_b is : ')
#print(servo_b)
#print('rod_p is :')
#print(rod_p)

fig = plt.figure()                                 #创建图对象
ax = fig.add_subplot(111, projection='3d')         #三维图
ax.plot(servo_b[:,0], servo_b[:,1], servo_b[:,2], '-', marker='o', color="blue")
ax.plot((servo_b[0,0],servo_b[5,0]), (servo_b[0,1],servo_b[5,1]), (servo_b[0,2],servo_b[5,2]), '-', color="blue")

#ax.plot(rod_p[:,0], rod_p[:,1], rod_p[:,2], '-', marker='o', color="black")
#ax.plot((rod_p[0,0],rod_p[5,0]), (rod_p[0,1],rod_p[5,1]), (rod_p[0,2],rod_p[5,2]), '-', color="black")

plt.grid(True)
plt.show()


'''
计算  有效杆长(电动缸长度) rod
'''

#计算旋转矩阵
T_BP = rotz(orint[2,0]) @ roty(orint[1,0]) @ rotx(orint[0,0])        
print("T_BP:")
print (T_BP)


#计算rod_b
broke_rod_p = np.split(rod_p, rod_p.shape[0])    #拆分成6个1*3
print("broke_rod_p:")
print (broke_rod_p)
brokeT_rod_p = [np.transpose(every_rod_p) for every_rod_p in broke_rod_p]
print("brokeT_rod_p:")
print (brokeT_rod_p)
rodT_b = [T_BP@everyT_rod_p+home+trans for everyT_rod_p in brokeT_rod_p]   #得出rod_b
print("rodT_b:")
print (rodT_b)
rod_b = np.vstack([np.transpose(matrix) for matrix in rodT_b])
print("rod_b:")
print (rod_b)


#计算rod
L_B = rod_b - servo_b   
print("L_B:",type(L_B))
print (L_B) 
rod = np.array([np.linalg.norm(a) for a in L_B])                        #输出B
print("rod:",type(rod))
print(rod)




'''
计算  舵机转角 angle
'''
def atan(angle):
    return np.arctan(angle)
pi = np.pi

beta = np.array([
    atan(servo_b[0,1]/servo_b[0,0]), 
    atan(servo_b[1,1]/servo_b[1,0]),
    atan(servo_b[2,1]/servo_b[2,0]),
    atan(servo_b[3,1]/servo_b[3,0]),
    atan(servo_b[4,1]/servo_b[4,0]),
    atan(servo_b[5,1]/servo_b[5,0]),
])

beta_d = np.array([
    atand(servo_b[0,1]/servo_b[0,0]), 
    atand(servo_b[1,1]/servo_b[1,0]),
    atand(servo_b[2,1]/servo_b[2,0]),
    atand(servo_b[3,1]/servo_b[3,0]),
    atand(servo_b[4,1]/servo_b[4,0]),
    atand(servo_b[5,1]/servo_b[5,0]),
])
print('betad')
print(beta)
print()

 #获得坐标
x_p = rod_p[:,0]
y_p = rod_p[:,1]
z_p = rod_p[:,2]

x_b = rod_b[:,0]
y_b = rod_b[:,1]
z_b = rod_b[:,2]


 #更新舵机转角     
l = rod**2 - leg**2 + servo_arm**2
M = 2 * servo_arm * (z_p - z_b)
N = 2 * servo_arm * (cos(beta) * (x_p - x_b) + sin(beta) * (y_p - y_b))   # .*?
M2 = M**2
N2 = N**2


#计算角度
angle = np.arcsin(l / np.sqrt(M2 - N2)) -np.arctan(N,M)     #输出
print('angle:',type(angle))
print(angle)
'''
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
'''
