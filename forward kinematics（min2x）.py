import numpy as np

# 固有参数
servo_b = np.array([[-30, -59, 0], [30, -59, 0], [66.0955, 3.5192, 0], [36.0955, 55.4808, 0], [-36.0955, 55.4808, 0], [-66.0955, 3.5192, 0]])
rod_p = np.array([[-39.34, -40.03, 0], [39.34, -40.03, 0], [54.34, -14.05, 0], [15, 54.08, 0], [-15, 54.08, 0], [-54.34, -14.05, 0]])
home = np.array([[0], [0], [117.85]])

# 初始化参数
rod = np.array([118.55088562, 121.78820736, 118.55036911, 121.78725506, 118.55086632, 121.78689215])
params = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 10.0])  # 初始参数 [x, y, z, thx, thy, thz]
alpha = 0.01  # 学习率
max_iter = 1000  # 最大迭代次数
tolerance = 1e-6  # 收敛容忍度

# 目标函数
def objective(params):
    return np.sum((rod - l_length(params, rod_p, servo_b, home)) ** 2)

# 计算L的函数
def l_length(params, rod_p, servo_b, home):
    x, y, z, thx, thy, thz = params
    trans = np.array([[x], [y], [z]])
    rotx = np.array([
        [1, 0, 0],
        [0, np.cos(np.radians(thx)), -np.sin(np.radians(thx))],
        [0, np.sin(np.radians(thx)), np.cos(np.radians(thx))]
        ])
    roty = np.array([
        [np.cos(np.radians(thy)), 0, np.sin(np.radians(thy))], 
        [0, 1, 0],
        [-np.sin(np.radians(thy)), 0, np.cos(np.radians(thy))]
        ])
    rotz = np.array([
        [np.cos(np.radians(thz)), -np.sin(np.radians(thz)), 0],
        [np.sin(np.radians(thz)), np.cos(np.radians(thz)), 0], 
        [0, 0, 1]
        ])
    T = rotz @ roty @ rotx
    print("T:")
    print (T)

    broke_rod_p = np.split(rod_p, rod_p.shape[0])    #拆分成6个1*3

    brokeT_rod_p = [np.transpose(every_rod_p) for every_rod_p in broke_rod_p]


    rodT_b = [T@everyT_rod_p+home+trans for everyT_rod_p in brokeT_rod_p]   #得出rod_b

    rod_b = np.vstack([np.transpose(matrix) for matrix in rodT_b])   
    print("rod_b:")
    print (rod_b)
    L_B = rod_b - servo_b   

    l = np.array([np.linalg.norm(a) for a in L_B])                        #输出B
    return l

# 计算梯度的函数
def new_grad(params, rod_p, servo_b, home, rod):
    h = 1e-4  # 微小的步长
    grad = np.zeros_like(params)  # 初始化梯度向量
    
    for i in range(len(params)):
        params_forward = np.copy(params)
        params_backward = np.copy(params)
        
        params_forward[i] += h
        params_backward[i] -= h
    
        obj_forward = np.sum((rod - l_length(params_forward, rod_p, servo_b, home)) ** 2)
        obj_backward = np.sum((rod - l_length(params_backward, rod_p, servo_b, home)) ** 2)
    
        grad[i] = (obj_forward - obj_backward) / (2 * h)
    
    return grad

# 迭代过程
for iter in range(max_iter):

    grad = new_grad(params, rod_p, servo_b, home, rod)
    params -= alpha * grad

    if np.linalg.norm(grad) < tolerance:
        break

# 输出结果
print('Optimized Parameters:')
print(params)

