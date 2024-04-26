import tkinter as tk
import numpy as np
import serial

# 定义逆运动学和正运动学计算的模拟函数
# 这里简单地使用示例算法，实际应用中需要使用复杂的计算
def alm_clear(num):
    print(f"清除 {num} 的报警")

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

'''
逆运动学
'''
def calculate_rod_b(orint,rod_p,home,trans):
    #计算旋转矩阵
    T_BP = rotz(orint[2,0]) @ roty(orint[1,0]) @ rotx(orint[0,0])        
    #计算rod_b
    broke_rod_p = np.split(rod_p, rod_p.shape[0])    #拆分成6个1*3
    brokeT_rod_p = [np.transpose(every_rod_p) for every_rod_p in broke_rod_p]
    rodT_b = [T_BP@everyT_rod_p+home+trans for everyT_rod_p in brokeT_rod_p]   #得出rod_b
    rod_b = np.vstack([np.transpose(matrix) for matrix in rodT_b])
    return T_BP,rod_b

def calculate_rod_length(rod_b,servo_b):
    L_B = rod_b - servo_b   
    rod = np.array([np.linalg.norm(a) for a in L_B])                        #输出B
    return rod ,L_B

def inverse(X,Y,Z,Rx,Ry,Rz):
    #零位  位移  旋转
    home=np.array([[0],[0],[365]])       #  400  为零位  297.6984 = 400 - 53.5047 - 48.7969     346.4953=400-53.5047    351.2031=400-48.7969
    orint=np.array([[Rx],[Ry],[Rz]])         #旋转单位，角度
    trans=np.array([[X],[Y],[Z]])      #平移单位：mm     53.5047+48.7968=102.3016
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
    #计算  动平台铰点 top_b
    T_BP = calculate_rod_b(orint,top_p,home,trans)[0]
    top_b = calculate_rod_b(orint,top_p,home,trans)[1]
    print("T_BP:")
    print (T_BP)
    print("top_b:")
    print (top_b)

    #计算  有效杆长(电动缸长度) rod
    rod = calculate_rod_length(top_b,base_b)[0]
    print("rod:",type(rod))
    print(rod)

'''
正运动学
'''
def forward_kinematics(leg_lengths):
    # 这是个模拟函数，返回新的 6 个数值
    return (sum(leg_lengths) / 6, ) * 6  # 简单模拟，实际计算会更复杂

# 处理输入数据的函数
def submit_values():
    # 获取输入字段中的值
    x = float(entry_x.get())
    y = float(entry_y.get())
    z = float(entry_z.get())
    rx = float(entry_rx.get())
    ry = float(entry_ry.get())
    rz = float(entry_rz.get())

    # 进行逆运动学计算
    leg_lengths = inverse(x, y, z, rx, ry, rz)
    '''
    # 进行正运动学计算
    new_values = forward_kinematics(leg_lengths)
    
    # 更新当前值显示框中的文本
    current_values.config(
        text=f"当前值：X={new_values[0]:.2f}, Y={new_values[1]:.2f}, Z={new_values[2]:.2f}, Rx={new_values[3]:.2f}, Ry={new_values[4]:.2f}, Rz={new_values[5]:.2f}"
    )
    '''

#回零操作
def back_zero():
    x=1


'''
串口连接设置
'''
def ser_connect():
    global ser
    try:
        serial_port = ser_port.get()
        baudrate = int(ser_baudrate.get())
        parity = ser_parity.get()
        bytesize = int(ser_bytesize.get())
        stopbits = int(ser_stopbits.get())
        timeout = int(ser_timeout.get())
        ser = serial.Serial(serial_port, baudrate, parity=parity, bytesize=bytesize, stopbits=stopbits, timeout=timeout)
        print("串口连接成功")
        canvas.itemconfig(light, fill="green")
    except serial.SerialException as e:
        print("串口连接错误:", e)
        canvas.itemconfig(light, fill="green")

def ser_disconnect():
    global ser
    if ser is not None:
        ser.close()
        print("串口连接已关闭")
        canvas.itemconfig(light, fill="green")




pi = np.pi
ser = None
# 创建主窗口
root = tk.Tk()
root.title("六维调姿平台")


'''
创建串口设置框架
'''
frame0 = tk.Frame(root, padx=10, pady=10, bd=2, relief="solid")
frame0.grid(row=0, column=0, padx=10, pady=10)
# 添加fram0标题
title_label0 = tk.Label(frame0, text="串口设置", font=("Helvetica", 12, "bold"))
title_label0.grid(row=0, column=0, columnspan=2, padx=10, pady=10)
# 创建并添加串口设置框
ser_labels = ["串口号:", "波特率:", "校验位:", "数据位:", "停止位:", "超时时间:"]
ser_entries = [tk.Entry(frame0) for _ in range(6)]
setting = ['COM8','9600','N','8','1','1']
for i, label in enumerate(ser_labels):
    j = i*2
    tk.Label(frame0, text=label).grid(row=0, column=j+2)
    ser_entries[i].insert(0,setting[i])
    ser_entries[i].grid(row=0, column=j+3)
# 将这些输入框分别赋值给变量
ser_port, ser_baudrate, ser_parity, ser_bytesize, ser_stopbits, ser_timeout = ser_entries
#创建提示灯
canvas = tk.Canvas(frame0, width=50, height=50)
light = canvas.create_oval(10, 10, 40, 40, fill="gray")
canvas.grid(row=0, column=16, padx=10, pady=10)
# 创建连接按钮
connect_button = tk.Button(frame0, text="连接", command=submit_values)
connect_button.grid(row=0, column=14, padx=10, pady=10)
#创建断开操作
disconnect_button = tk.Button(frame0, text="断开", command=back_zero)
disconnect_button.grid(row=0, column=15, padx=10, pady=10)


#frame5 = tk.Frame(root, width=200, height=100, padx=10, pady=10, bd=2, relief="solid")
#frame5.grid(row=2, column=1, padx=10, pady=10)          框架大小测试数值为像素大小

'''
创建输入框架
'''
frame1 = tk.Frame(root,padx=10, pady=10, bd=2, relief="solid")
frame1.grid(row=1, column=0, padx=10, pady=10)
# 添加frame1标题
title_label1 = tk.Label(frame1, text="目标位置姿态", font=("Helvetica", 12, "bold"))
title_label1.grid(row=0, column=0, columnspan=2, padx=10, pady=10)
# 创建并添加输入框
labels = ["X(mm):", "Y(mm):", "Z(mm):", "Rx(°):", "Ry(°):", "Rz(°):"]
entries = [tk.Entry(frame1) for _ in range(6)]
for i, label in enumerate(labels):
    tk.Label(frame1, text=label).grid(row=i+1, column=0)
    entries[i].grid(row=i+1, column=1)
# 将这些输入框分别赋值给变量
entry_x, entry_y, entry_z, entry_rx, entry_ry, entry_rz = entries
# 创建提交按钮
submit_button = tk.Button(frame1, text="提交", command=submit_values)
submit_button.grid(row=7, column=1, padx=10, pady=10)
#创建回原点操作
zero_button = tk.Button(frame1, text="回原点", command=back_zero)
zero_button.grid(row=7, column=0, padx=10, pady=10)


'''
创建当前值的显示框
'''
frame2 = tk.Frame(root, padx=10, pady=10, bd=2, relief="solid")
frame2.grid(row=1, column=1, padx=10, pady=10)
# 添加frame2标题
title_label2 = tk.Label(frame2, text="当前位置姿态", font=("Helvetica", 12, "bold"))
title_label2.grid(row=0, column=0, padx=10, pady=10)
# 创建显示当前值的标签
current_values = tk.Label(frame2, text="", font=("Helvetica", 10))
current_values.grid(row=1, column=0, padx=10, pady=10)


'''
创建报警提示框架
'''
frame3 = tk.Frame(root, padx=10, pady=10, bd=2, relief="solid")
frame3.grid(row=2, column=0, padx=10, pady=10)
# 添加frame3标题
title_label3 = tk.Label(frame3, text="报警状态", font=("Helvetica", 12, "bold"))
title_label3.grid(row=0, column=0, padx=10, pady=10)
# 创建显示当前报警的标签
alm_values = tk.Label(frame3, text="当前报警", font=("Helvetica", 10))
alm_values.grid(row=1, column=1, padx=10, pady=10)
# 创建提交按钮
alm_buttons = []
for i in range(6):
    button = tk.Button(frame3, text=f"清除报警{i+1}", command=lambda index=i: alm_clear(f"伺服电机{index+1}"))
    button.grid(row=i+1, column=0, pady=5)
    alm_buttons.append(button)

'''
运行主循环
'''
root.mainloop()