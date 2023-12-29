'''
解析法求位置姿态，但是只有一组解，（并没有多组解的情况？）
'''
import numpy as np
from scipy.optimize import fsolve   #牛顿迭代法？  还是最小二乘法？  

def cos(angle):
    return np.cosh(angle)

def acosd(deta):
    return np.arccosh(deta)

# 定义方程组
def equations(vars,rod,servo_b,rod_p):
    l1, l2, l3, m1, m2, m3, x, y, z = vars
    eq1 = l1**2 + l2**2 + l3**2 - 1
    eq2 = m1**2 + m2**2 + m3**2 - 1
    eq3 = l1*l1 + l2*m2 + l3*m3 
    eq4 = rod[0]**2 - (rod_p[0,0]*l1 + rod_p[0,1]*m1  + x - servo_b[0,0])**2 - (rod_p[0,0]*l2 + rod_p[0,1]*m2 + y - servo_b[0,1])**2 - (rod_p[0,0]*l3 + rod_p[0,1]*m3 + z)**2
    eq5 = rod[1]**2 - (rod_p[1,0]*l1 + rod_p[1,1]*m1  + x - servo_b[1,0])**2 - (rod_p[1,0]*l2 + rod_p[1,1]*m2 + y - servo_b[1,1])**2 - (rod_p[1,0]*l3 + rod_p[1,1]*m3 + z)**2
    eq6 = rod[2]**2 - (rod_p[2,0]*l1 + rod_p[2,1]*m1  + x - servo_b[2,0])**2 - (rod_p[2,0]*l2 + rod_p[2,1]*m2 + y - servo_b[2,1])**2 - (rod_p[2,0]*l3 + rod_p[2,1]*m3 + z)**2
    eq7 = rod[3]**2 - (rod_p[3,0]*l1 + rod_p[3,1]*m1  + x - servo_b[3,0])**2 - (rod_p[3,0]*l2 + rod_p[3,1]*m2 + y - servo_b[3,1])**2 - (rod_p[3,0]*l3 + rod_p[3,1]*m3 + z)**2
    eq8 = rod[4]**2 - (rod_p[4,0]*l1 + rod_p[4,1]*m1  + x - servo_b[4,0])**2 - (rod_p[4,0]*l2 + rod_p[4,1]*m2 + y - servo_b[4,1])**2 - (rod_p[4,0]*l3 + rod_p[4,1]*m3 + z)**2
    eq9 = rod[5]**2 - (rod_p[5,0]*l1 + rod_p[5,1]*m1  + x - servo_b[5,0])**2 - (rod_p[5,0]*l2 + rod_p[5,1]*m2 + y - servo_b[5,1])**2 - (rod_p[5,0]*l3 + rod_p[5,1]*m3 + z)**2


    # 最终返回所有方程构成的数组,eq1,eq2是单位向量；eq3两个向量垂直点积为0；eq4-9长度相等；eq10-12右手定则叉乘
    return [eq1 , eq2 , eq3 , eq4 , eq5 , eq6 , eq7, eq8 , eq9 ]  

rod = np.array([119.73186293, 119.73186293, 119.73091108, 119.73138628, 119.73138628,119.73091108])
servo_b = np.array([
        [-30, -59, 0],
        [30, -59 , 0],
        [66.0955,  3.5192, 0],
        [36.0955, 55.4808, 0],
        [-36.0955, 55.4808, 0],
        [-66.0955, 3.5192, 0]
])
rod_p = np.array([
        [-39.34, -40.03, 0],
        [39.34, -40.03 , 0],
        [54.34,  -14.05, 0],
        [15, 54.08, 0],
        [-15, 54.08, 0],
        [-54.34, -14.05, 0]
])

# 初始猜测值
initial_guess = np.zeros(9)

# 求解方程组
solution = fsolve(equations, initial_guess , args=(rod, servo_b, rod_p), full_output=True)  #添加额外参数args=(rod, servo_b, rod_p),默认容差xtol=2.220446049250313e-12, 迭代次数maxfev=200*（方程数目+1）,
#full_output=True返回诊断信息（默认False）:
'''
nfev  函数调用次数;
fvec  函数在输出解处的值;
fjac  估计的Jacobian矩阵; 
r  Jacobian矩阵的上三角部分。
qtf  Jacobian矩阵QR分解的最后一步的向量。
ier 是一个整数标志，它表示优化过程是否成功。
ier=1 表明找到了一个解，其他值表明有不同类型的错误或者问题。
msg 是一个描述优化结果的消息，它提供了关于求解过程是否成功以及为何停止的信息。
'''   
print("Solution:", solution)
