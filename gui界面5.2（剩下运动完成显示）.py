import tkinter as tk
import numpy as np
import serial
import time


'''
串口连接设置
'''
def ser_connect():
    global ser
    global alm_states
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
        #while ser.is_open:
        for i in range(6):
            alm_read(i+1)
        alm_values.config(text=f"所有报警状态: {alm_states}")
    except serial.SerialException as e:
        print("串口连接错误:", e)
        canvas.itemconfig(light, fill="red")

def ser_disconnect():
    global ser
    if ser is not None:
        ser.close()
        print("串口连接已关闭")
        canvas.itemconfig(light, fill="gray")


'''
发送modbus RTU报文
'''
def calculate_crc(data):
    data = bytearray.fromhex(data.replace(" ", ""))
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    # 返回 CRC 校验码，低八位在前，两位两位以空格隔开
    return "{:02X} {:02X}".format((crc & 0xFF), ((crc & 0xFF00) >> 8))

def send(request):
    global ser
    request = bytearray.fromhex(request.replace(" ", ""))
    try:
        # 发送请求
        ser.write(request)
        print("已发送 Modbus RTU 请求:", ":".join("{:02X}".format(byte) for byte in request))
        time.sleep(0.03)
        # 等待响应
        response = ser.read_all()
        response_hex = "".join("{:02X}".format(byte) for byte in response)
        response_list = [response_hex[i:i+2] for i in range(0, len(response_hex), 2)]
        print("收到 Modbus RTU 响应:", response_list)
        return response_list
    except serial.SerialException as e:
        print("串口错误:", e)
    

def modbus_message(hex_string):    #不带 CRC 的16进制形式的 Modbus RTU 报文，以空格分隔每个字节， 输出可以直接运行函数send_modbus_rtu_request
    crc = calculate_crc(hex_string)
    #print("计算得到的 CRC 校验码为:", crc)
    #print(hex_string)
    # 添加 CRC 校验码
    hex_crc_string = hex_string + " " + crc
    #print("带 CRC 校验码的十六进制输入为:", hex_crc_string)
 
    #return hex_crc_bytes        #执行该行，上一行要注释掉
    return send(hex_crc_string)

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

def dec_to_hex(dec_num):
    if dec_num < 0:  # 处理负数
        dec_num = (1 << 32) + dec_num
        
    hex_str = hex(dec_num & 0xFFFFFFFF)[2:].zfill(8).upper()  # 将10进制数转换为16进制
    hex_list = [hex_str[i:i+2] for i in range(0, len(hex_str), 2)]  # 每两位分割为一组
    while len(hex_list) % 4 != 0:  # 补足缺失的组
        hex_list.append('00')
    for i in range(0, len(hex_list), 4):
        hex_list[i], hex_list[i+1], hex_list[i+2], hex_list[i+3] = hex_list[i+2], hex_list[i+3], hex_list[i], hex_list[i+1]  # 调整高低位顺序
    hex_result = ' '.join(hex_list)  # 以空格连接各组16进制数
    return hex_result


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
    #T_BP = calculate_rod_b(orint,top_p,home,trans)[0]
    top_b = calculate_rod_b(orint,top_p,home,trans)[1]
    #print("T_BP:")
    #print (T_BP)
    #print("top_b:")
    #print (top_b)
    #计算  有效杆长(电动缸长度) rod
    rod = calculate_rod_length(top_b,base_b)[0]
    #print("rod:",type(rod))
    rod = np.round(rod,3)
    print(rod)
    return rod

'''
正运动学
'''
def forward_kinematics(leg_lengths):
    # 这是个模拟函数，返回新的 6 个数值
    return (sum(leg_lengths) / 6, ) * 6  # 简单模拟，实际计算会更复杂


'''
操作按钮
'''
# 处理输入数据的函数
def submit_values():
    global rod0
    global previous_rod
    # 获取输入字段中的值
    x = float(entry_x.get())
    y = float(entry_y.get())
    z = float(entry_z.get())
    rx = float(entry_rx.get())
    ry = float(entry_ry.get())
    rz = float(entry_rz.get())
    # 进行逆运动学计算
    rod = inverse(x, y, z, rx, ry, rz)
    if 'previous_rod' not in globals():
        previous_rod = rod
        mov_rod = rod - rod0
    else:
        mov_rod = rod - previous_rod
    previous_rod = rod
    print(mov_rod)

    for i, length in enumerate(mov_rod):
        mov_i =  np.round(length,3)
        movi_pluse = mov_i * 10487
        movi_pluse = int(movi_pluse)
        movi_pluse_hex = dec_to_hex(movi_pluse)
        slave_NO = '0'+ str(i+1)
        slave_NO_block_command = slave_NO + ' ' + '10 48 00 00 02 04 00 00 01 00'   #写入block指令  0x4800
        modbus_message(slave_NO_block_command)
        slave_NO_block_date = slave_NO + ' ' + '10 48 02 00 02 04' + ' ' + movi_pluse_hex           
        modbus_message(slave_NO_block_date)                                        #写入block数据   0x4802
    modbus_message('00 05 00 60 FF 00')    #伺服使能
    time.sleep(0.5)
    modbus_message('10 0F 00 00 00 06 01 3F')    #制动器松闸
    time.sleep(0.5)
    modbus_message('00 06 4414 00 00')    #指定block NO
    time.sleep(0.5)
    modbus_message('00 05 01 20 FF 00')    #启动STB信号  
    time.sleep(0.5)
    modbus_message()    #监视block motion状态

    '''
    #运动完成，制动器抱闸，伺服使能关闭
    while 1 :
        if modbus_message('00 01 01 40 00 01') == '00 01 01 40 00 01 FC 33':       # 0x0140 block motion动作执行状态     CRC校验码  FC 33  待定，因为不确定是不是广播信号
            modbus_message('10 0F 00 00 00 06 01 00')    #制动器关闸
            time.sleep(0.5)
            modbus_message('00 05 00 60 00 00')    #伺服使能关闭
    '''

    # 进行正运动学计算
    new_values = forward_kinematics(previous_rod)
    # 更新当前值显示框中的文本
    current_values.config(
        text=f"当前值：X={new_values[0]:.3f}, Y={new_values[1]:.3f}, Z={new_values[2]:.3f}, Rx={new_values[3]:.3f}, Ry={new_values[4]:.3f}, Rz={new_values[5]:.3f}"
    )


#回零操作
def back_zero():
    global previous_rod
    global rod0
    mov_rod = rod0 - previous_rod
    for i, length in enumerate(mov_rod):
        mov_i =  np.round(length,3)
        movi_pluse = mov_i * 10526
        movi_pluse_hex = dec_to_hex(movi_pluse)
        slave_NO = '0'+ str(i+1)
        slave_NO_block_command = slave_NO + ' ' + '10 48 00 00 02 04 00 00 01 00'   #写入block指令  0x4800
        modbus_message(slave_NO_block_command)
        slave_NO_block_date = slave_NO + ' ' + '10 48 02 00 02 04' + ' ' + movi_pluse_hex           
        modbus_message(slave_NO_block_date)                                        #写入block数据   0x4802
    modbus_message('00 05 00 60 FF 00')    #伺服使能
    time.sleep(0.5)
    modbus_message('10 0F 00 00 00 06 01 3F')    #制动器松闸
    time.sleep(0.5)
    modbus_message('00 06 4414 00 00')    #指定block NO
    time.sleep(0.5)
    modbus_message('00 05 01 20 FF 00')    #启动STB信号  
    time.sleep(0.5)
    modbus_message()    #监视block motion状态
'''      
    #运动完成，制动器抱闸，伺服使能关闭
    while 1 :
        if modbus_message('00 01 01 40 00 01') == '00 01 01 40 00 01 FC 33':       #CRC校验码  FC 33  待定，因为不确定是不是广播信号
        modbus_message('10 0F 00 00 00 06 01 00')    #制动器关闸
        time.sleep(0.5)
        modbus_message('00 05 00 60 00 00')    #伺服使能关闭
'''  

#读取报警状态
def alm_read(slave):
    slave = str(slave)
    baowen = '0' + slave + ' ' + '03 40 01 00 01'
    alm_response = modbus_message(baowen)
    almNO = [int(alm_response[3],16), int(alm_response[4],16)]
    index = int(slave)-1
    alm_states[index] = almNO

#清除报警
def alm_clear(slave):
    slave = str(slave)
    massage = '0' + slave + ' ' + '05 00 61 FF 00'
    modbus_message(massage)
    alm_read(slave)
    alm_values.config(text=f"所有报警状态: {alm_states}")










flag = True
pi = np.pi
ser = None
rod0 = np.array([308.2, 308.2, 308.2, 308.2, 308.2, 308.2])
alm_states = [None, None, None, None, None, None]  # 初始化报警状态
# 创建主窗口
root = tk.Tk()
root.title("六维调姿平台")
root.configure(background='green')  # 设置背景颜色




'''
创建串口设置框架
'''
frame0 = tk.Frame(root, padx=10, pady=10, bd=2, relief="solid")
frame0.grid(row=0, column=0, padx=10, pady=10, columnspan=4)
# 添加fram0标题
title_label0 = tk.Label(frame0, text="串口设置", font=("Helvetica", 12, "bold"))
title_label0.grid(row=0, column=0, columnspan=2, padx=10, pady=10)
# 创建并添加串口设置框
ser_labels = ["串口号:", "波特率:", "校验位:", "数据位:", "停止位:", "超时时间:"]
ser_entries = [tk.Entry(frame0,width=10) for _ in range(6)]
setting = ['COM8','9600','E','8','1','1']
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
connect_button = tk.Button(frame0, text="连接", command=ser_connect)
connect_button.grid(row=0, column=14, padx=10, pady=10)
#创建断开操作
disconnect_button = tk.Button(frame0, text="断开", command=ser_disconnect)
disconnect_button.grid(row=0, column=15, padx=10, pady=10)


#frame5 = tk.Frame(root, width=200, height=100, padx=10, pady=10, bd=2, relief="solid")
#frame5.grid(row=2, column=1, padx=10, pady=10)          框架大小测试数值为像素大小

'''
创建输入框架
'''
frame1 = tk.Frame(root,padx=10, pady=10, bd=2, relief="solid")
frame1.grid(row=1, column=0, padx=10, pady=10, sticky="w")
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
frame2.grid(row=2, column=0, padx=10, pady=10, columnspan=4)

# 添加frame2标题
title_label2 = tk.Label(frame2, text="当前位置姿态", font=("Helvetica", 12, "bold"))
title_label2.grid(row=0, column=0, padx=10, pady=10)
# 创建显示当前值的标签
current_values = tk.Label(frame2, text="X=0 Y=0 Z=0 Rx=0 Ry=0 Rz=0", font=("Helvetica", 10))
current_values.grid(row=1, column=0, padx=10, pady=10)


'''
创建报警提示框架
'''
frame3 = tk.Frame(root, padx=10, pady=10, bd=2, relief="solid")
frame3.grid(row=1, column=1, padx=10, pady=10, sticky="w", columnspan=3)
# 添加frame3标题
title_label3 = tk.Label(frame3, text="报警状态", font=("Helvetica", 12, "bold"))
title_label3.grid(row=0, column=0, padx=10, pady=10)
# 创建显示当前报警的标签
alm_values = tk.Label(frame3, text="等待连接查询", font=("Helvetica", 10),width=60, height=5)
alm_values.grid(row=0, column=1, padx=10, pady=10,columnspan=2)
# 创建报警清除按钮
alm_buttons = []
for i in range(6):
    button = tk.Button(frame3, text=f"清除报警{i+1}", command=lambda index=i: alm_clear(f"{index+1}"))
    button.place(x=i+i*100, y=80, anchor="nw")
    alm_buttons.append(button)

'''
运行主循环
'''
root.mainloop()