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

def acosd(a):              #返回角度
    pi=np.pi
    return np.arccos(a)*180/pi

def atand(a):              #返回角度
    pi=np.pi
    return np.arctan(a)*180/pi

def cos(radian):
    return np.cos(radian)

def cosd(degree):
    radian = degree * np.pi /180
    return np.cos(radian)

def sin(radian):
    return np.sin(radian)

def sind(degree):
    radian = degree * np.pi /180
    return np.cos(radian)

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
    return rod,L_B




"""
计算平台参数                     输入
"""
pi = np.pi

#零位  位移  旋转
 
home=np.array([[0],[0],[297.6984]])       #  400  为零位  297.6984 = 400 - 53.5047 - 48.7969
orint=np.array([[0],[0],[0]])        #单位，角度
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

'''
工作空间边界条件
'''


#杆长限制      不干涉limit=1 ，干涉limit=0
lmin = 280
lmax = 339
for i in range(6):
    if not 280 <= rod[i] <= 339:
        llimit = 0
        break 


#运动副限制      不干涉limit=1 ，干涉limit=0
theta_p_max = 23.5    #单位：度数
theta_b_max = 18.08   #单位：度数

n_b = np.array([[0.171513,   0.19383199, 0.96592583],
                [-0.08210724, -0.24544951, 0.96592583],
                [-0.25361951, 0.0516196,  0.96592583],
                [0.25361951, 0.0516196,  0.96592583],
                [0.08210724, -0.24544951, 0.96592583],
                [-0.171513,   0.19383199, 0.96592583]
 ])


n_p = -n_b

#print('n_p:')
#print(n_p)
#print('n_b:')
#print(n_b)

for i in range(6):
    deta_p = -L_B[i] @ (T_BP @ n_p[i].reshape(-1,1)) / np.linalg.norm(L_B[i])
    if 1 <= deta_b <= 1.00000013:
        deta_b = 1.0
    else:
        deta_b = deta_b[0]
    theta_p = acosd(deta_p)
    #print(deta_p)
    #print(theta_p)
    if not theta_p <= theta_p_max:         #或者   if not deta_p >= cos(theta_p_max):
        theta_p_limit = 0
        break
    else: 
        theta_t_limit = 1

 

for i in range(6):
    deta_b = L_B[i] @ (T_BP @ n_b[i].reshape(-1,1)) / np.linalg.norm(L_B[i])
    if 1 <= deta_b <= 1.00000013:
        deta_b = 1.0
    else:
        deta_b = deta_b[0]
    theta_b = acosd(deta_b)
    print(deta_b)
    #print(theta_b)
    if not theta_b <= theta_b_max:
        theta_b_limit = 0
        break
    else:
        theta_b_limit = 1


#连杆干涉（忽略不考虑）      不干涉limit=1 ，干涉limit=0
d = 22


'''
快速坐标搜索，工作空间
'''

#初始化边界
z = z0       #z0 < zmin
r = 1
alpha = 0
vz = 
vr =
valpha = 
x = r * cosd(alpha)
y = r * sind(alpha)
trans = np.array([[x],[y],[z]])
orint = np.array([[0],[0],[0]])
