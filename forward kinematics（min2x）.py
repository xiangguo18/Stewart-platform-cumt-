import numpy as np

# 固有参数

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

home = np.array([[0], [0], [365]])

# 初始化参数
rod = np.array([308.2 ,308.2 ,308.2 ,308.2 ,308.2 ,308.2 ])   #零位参数[308.2 ,308.2 ,308.2 ,308.2 ,308.2 ,308.2]
params = np.array([2.0, 1.0, -1.0, 1.0, 0.1, -2.0])           # 初始参数 [x, y, z, thx, thy, thz],零位参数[308.2 ,308.2 ,308.2 ,308.2 ,308.2 ,308.2]
alpha = 0.01  # 学习率
max_iter = 1000  # 最大迭代次数
tolerance = 1e-4  # 收敛容忍度

# 目标函数
def objective(params):
    return np.sum((rod - l_length(params, top_p, base_b, home)) ** 2)

# 计算L的函数
def l_length(params, top_p, base_b, home):
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

    broke_top_p = np.split(top_p, top_p.shape[0])    #拆分成6个1*3

    brokeT_top_p = [np.transpose(every_top_p) for every_top_p in broke_top_p]

    topT_b = [T@everyT_top_p+home+trans for everyT_top_p in brokeT_top_p]   #得出top_b

    top_b = np.vstack([np.transpose(matrix) for matrix in   topT_b])   
    print("top_b:")
    print (top_b)
    L_B = top_b - base_b   

    l = np.array([np.linalg.norm(a) for a in L_B])                        #输出B
    return l

# 计算梯度的函数
def new_grad(params, top_p, base_b, home, rod):
    h = 1e-4  # 微小的步长
    grad = np.zeros_like(params)  # 初始化梯度向量
    
    for i in range(len(params)):
        params_forward = np.copy(params)
        params_backward = np.copy(params)
        
        params_forward[i] += h
        params_backward[i] -= h
    
        obj_forward = np.sum((rod - l_length(params_forward, top_p, base_b, home)) ** 2)
        obj_backward = np.sum((rod - l_length(params_backward, top_p, base_b, home)) ** 2)
    
        grad[i] = (obj_forward - obj_backward) / (2 * h)
    
    return grad

# 迭代过程
for iter in range(max_iter):

    grad = new_grad(params, top_p, base_b, home, rod)
    params -= alpha * grad

    if np.linalg.norm(grad) < tolerance:
        break

# 输出结果
print('Optimized Parameters:')
print(params)

