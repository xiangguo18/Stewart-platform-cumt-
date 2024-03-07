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

def sin(radian):
    return np.sin(radian)

def cosd(degree):
    radian = degree * np.pi /180
    return np.cos(radian)

def sind(degree):
    radian = degree * np.pi /180
    return np.sin(radian)

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


   


'''
快速坐标搜索，工作空间
'''

#初始化边界
pi = np.pi
z0 = -33                     #大概是±30
r0 = 1
alpha0 = 0
zmax = 32
z = z0       #z0 < zmin


v_z = 1
v_r1 = 0.5               #小增量
v_r2 = 5              #大增量
v_alpha = 0.1         #单位： 度
orint = np.array([[0],[0],[0]])           # 定 姿态 搜索
home=np.array([[0],[0],[365]])       #  400  为零位  297.6984 = 400 - 53.5047 - 48.7969

#定义底部平台几何尺寸                                 底部虎克铰中心距离地面  53.5047mm
base_b = np.array([
    [29.3443,	-135.5477, 53.5047],
    [132.05985,	42.3609, 53.5047],
    [102.7156,	93.1867, 53.5047],
    [-102.7156,	93.1867, 53.5047],
    [-132.05985,	42.3609, 53.5047],
    [-29.3443,	-135.5477, 53.5047]
])	

#定义顶部平台几何尺寸                                 顶部球铰铰中心距离动平台顶面  48.7969mm   顶部球铰距离动平台底面13.7968mm    动平台厚35mm
top_p = np.array([
    [82.2046,	-75.8088,	-13.7969],
    [106.7546,	-33.2868,	-13.7969],		
    [24.5501,	109.0956,	-13.7969],		
    [-24.5501,	109.0956,	-13.7969],		
    [-106.7546,	-33.2868,	-13.7969],		
    [-82.2046,	-75.8088,	-13.7969]
])
#print('base_b is : ')
#print(base_b)
#print('top_p is :')
#print(top_p)

#静平台、动平台铰接座方向姿态向量
n_b = np.array([[0.171513,   0.19383199, 0.96592583],
                [-0.08210724, -0.24544951, 0.96592583],
                [-0.25361951, 0.0516196,  0.96592583],
                [0.25361951, 0.0516196,  0.96592583],
                [0.08210724, -0.24544951, 0.96592583],
                [-0.171513,   0.19383199, 0.96592583]
])

n_p = -n_b


'''
工作空间边界条件
'''
#杆长限制      不干涉limit=1 ，干涉limit=0
lmin = 280
lmax = 339

#运动副限制      不干涉limit=1 ，干涉limit=0
theta_p_max = 23.5    #单位：度数
theta_b_max = 18.08   #单位：度数

#连杆干涉（忽略不考虑）      不干涉limit=1 ，干涉limit=0
d = 22
point = []
while 1:
    r = r0
    alpha = alpha0
    while 1:
        print('Z坐标为：',z)
        print('搜索半径为：',r)
        print('当前搜索角度为：',alpha)
        x = r * cosd(alpha)
        y = r * sind(alpha)
        trans = np.array([[x],[y],[z]])

        T_BP = calculate_rod_b(orint,top_p,home,trans)[0]
        top_b = calculate_rod_b(orint,top_p,home,trans)[1]

        rod = calculate_rod_length(top_b,base_b)[0]
        L_B = calculate_rod_length(top_b,base_b)[1]

        l1 = max(rod)
        l2 = min(rod)

        if l2 < lmin:
            break                

        theta_b_list = []
        theta_p_list = []

        for i in range(6):
            deta_p = -L_B[i] @ (T_BP @ n_p[i].reshape(-1,1)) / np.linalg.norm(L_B[i])
            if 1 <= deta_p <= 1.01:              #实际上在零位400处大于1.00000013就OK了，但是因为精度可接受10um范围，故取1.01
                deta_p = 1.0
            else:
                deta_p = deta_p[0]
            theta_p = acosd(deta_p)
            theta_p_list.append(theta_p)

        for i in range(6):
            deta_b = L_B[i] @ (T_BP @ n_b[i].reshape(-1,1)) / np.linalg.norm(L_B[i])
            if 1 <= deta_b <= 1.01:
                deta_b = 1.0
            else:
                deta_b = deta_b[0]
            theta_b = acosd(deta_b)
            theta_b_list.append(theta_b)

        theta_p_m = max(theta_p_list)
        theta_b_m = max(theta_b_list)

        if lmax-1 <=l1<= lmax   or   theta_p_max-1 <=theta_p_m<= theta_p_max   or   theta_b_max-1 <=theta_b_m <= theta_b_max:        #最好是一个边缘范围
            point.append([x,y,z])
            print([x,y,z])
            alpha = alpha + v_alpha
        
        if alpha > 360:
            break
        elif l1 > lmax   or   theta_p_m > theta_p_max   or   theta_b_m > theta_b_max:
            r = r - v_r1                                                                  #       最好是判断一下大多少
        else :                              #是这样的情况吗?  if l1 < lmax   and   theta_p_m < theta_p_max   and   theta_b_m < theta_b_max：
            r = r + v_r1
    z = z + v_z
    if z == zmax:
        break

# 创建一个 3D 坐标系
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 解析出数据中的 x, y, z 坐标
x_list, y_list, z_list = np.array(point).T

# 绘制散点图
ax.scatter(x_list, y_list, z_list)

# 设置坐标轴标签
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

# 显示图形
plt.show()   
    








