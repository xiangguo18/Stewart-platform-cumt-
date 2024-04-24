import serial

# 串口配置
SERIAL_PORT = 'COM8'  # 串口号
BAUDRATE = 9600  # 波特率
PARITY = serial.PARITY_EVEN  # 校验位
BYTESIZE = 8  # 数据位
STOPBITS = 1  # 停止位
TIMEOUT = 1  # 超时时间

def calculate_crc(data):
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

def send_modbus_rtu_request(request):
    try:
        # 初始化串口
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, parity=PARITY, bytesize=BYTESIZE, stopbits=STOPBITS, timeout=TIMEOUT)
        
        # 发送请求
        ser.write(request)
        print("已发送 Modbus RTU 请求:", ":".join("{:02X}".format(byte) for byte in request))
        
        # 等待响应
        response = ser.read_all()
        print("收到 Modbus RTU 响应:", ":".join("{:02X}".format(byte) for byte in response))
        
        # 关闭串口
        ser.close()
    except serial.SerialException as e:
        print("串口错误:", e)

def main():
    while True:
        try:
            # 手动输入带 CRC 校验码的 Modbus RTU 报文
            hex_string = input("请输入带 CRC 校验码的十六进制形式的 Modbus RTU 报文，以空格分隔每个字节，输入 q 退出：\n")
            if hex_string.lower() == 'q':
                break

            hex_bytes = bytearray.fromhex(hex_string.replace(" ", ""))
            send_modbus_rtu_request(hex_bytes)
        except ValueError:
            print("请输入有效的十六进制字符串。")

if __name__ == "__main__":
    main()







'''CRC校验码计算'''
def calculate_crc(data):
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

def main():
    while True:
        try:
            # 手动输入需要计算 CRC 的数据，以空格分隔每个字节
            hex_string = input("请输入需要计算 CRC 的十六进制数据，以空格分隔每个字节，输入 q 退出：\n")
            if hex_string.lower() == 'q':
                break

            hex_bytes = bytearray.fromhex(hex_string.replace(" ", ""))
            crc = calculate_crc(hex_bytes)
            print("计算得到的 CRC 校验码为:", crc)
        except ValueError:
            print("请输入有效的十六进制字符串。")

if __name__ == "__main__":
    main()