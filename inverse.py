"""
真正六维台
"""
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import tkinter as tk

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

def calculate_rod_b(orint,rod_p,home,trans):
    #计算旋转矩阵
    T_BP = rotz(orint[2,0]) @ roty(orint[1,0]) @ rotx(orint[0,0])        
    #print("T_BP:")
    #print (T_BP)

    #计算rod_b
    broke_rod_p = np.split(rod_p, rod_p.shape[0])    #拆分成6个1*3

    brokeT_rod_p = [np.transpose(every_rod_p) for every_rod_p in broke_rod_p]

    rodT_b = [T_BP@everyT_rod_p+home+trans for everyT_rod_p in brokeT_rod_p]   #得出rod_b

    rod_b = np.vstack([np.transpose(matrix) for matrix in rodT_b])
    #print("rod_b:")
    #print (rod_b)
    return T_BP,rod_b

def calculate_rod_length(rod_b,servo_b):
    L_B = rod_b - servo_b   

    rod = np.array([np.linalg.norm(a) for a in L_B])                        #输出B
    return rod ,L_B




"""
计算平台参数                     输入
"""
pi = np.pi

#零位  位移  旋转
 
home=np.array([[0],[0],[297.6984]])       #  400  为零位  297.6984 = 400 - 53.5047 - 48.7969
orint=np.array([[0],[0],[7]])        #单位，角度
trans=np.array([[0],[0],[0]])       #单位：mm

#定义底部平台几何尺寸                                 底部虎克铰中心距离地面  53.5047mm
base_b = np.array([
    [29.3443,	-135.5477, 0],
    [132.05985,	42.3609, 0],
    [102.7156,	93.1867, 0],
    [-102.7156,	93.1867, 0],
    [-132.05985,	42.3609, 0],
    [-29.3443,	-135.5477, 0]
])	

#定义顶部平台几何尺寸                                 顶部球铰铰中心距离地面  351.2031mm
top_p = np.array([
    [82.2046,	-75.8088,	0],
    [106.7546,	-33.2868,	0],		
    [24.5501,	109.0956,	0],		
    [-24.5501,	109.0956,	0],		
    [-106.7546,	-33.2868,	0],		
    [-82.2046,	-75.8088,	0]
])

#print('base_b is : ')
#print(base_b)
#print('top_p is :')
#print(top_p)


'''
计算  动平台铰点 top_b
'''

T_BP = calculate_rod_b(orint,top_p,home,trans)[0]
top_b = calculate_rod_b(orint,top_p,home,trans)[1]
print("T_BP:")
print (T_BP)
print("top_b:")
print (top_b)

'''
计算  有效杆长(电动缸长度) rod
'''

rod = calculate_rod_length(top_b,base_b)[0]
L_B = calculate_rod_length(top_b,base_b)[1]
print("rod:",type(rod))
print(rod)
print(L_B)




fig1 = plt.figure()                                 #创建图对象
ax1 = fig1.add_subplot(111, projection='3d')             #三维图
ax1.set_title("inverse kinematics")
ax1.set_xlabel('X axis')
ax1.set_ylabel('Y axis')
ax1.set_zlabel('Z axis')


ax1.plot(base_b[:,0], base_b[:,1], base_b[:,2], '-', marker='o', color="blue")
ax1.plot((base_b[0,0],base_b[5,0]), (base_b[0,1],base_b[5,1]), (base_b[0,2],base_b[5,2]), '-', color="blue")

ax1.plot(top_b[:,0], top_b[:,1], top_b[:,2], '-', marker='o', color="black")
ax1.plot((top_b[0,0],top_b[5,0]), (top_b[0,1],top_b[5,1]), (top_b[0,2],top_b[5,2]), '-', color="black")

for i in range(6):
    ax1.plot([top_b[i,0], base_b[i,0]], [top_b[i,1], base_b[i,1]], [top_b[i,2], base_b[i,2]], '-', color="y", label='rod' if i == 0 else "_nolegend_")

# 添加图例
ax1.legend()

# 启用交互式旋转
plt.grid(True)
plt.show()
