from pymodbus.client import ModbusSerialClient as ModbusClient
from pymodbus.exceptions import ModbusException

# 设置串行连接
client = ModbusClient(method='rtu', port='COM8', baudrate=9600, stopbits=1, bytesize=8, parity='N')

if client.connect():
    print("Modbus 连接成功")
else:
    print("无法连接到 Modbus 设备")

# 连接到从站
connection = client.connect()
if connection:
    try:
        # 向从站发送写单个线圈命令
        # 从站地址为4，线圈地址为0x0060，要写入的值为0xFF00
        response = client.write_coil(address=0x0060, value=0xFF00, unit=4)
        print("Response:", response)
    except ModbusException as e:
        print("Modbus Exception:", str(e))
    except Exception as e:
        print("Exception:", str(e))
    finally:
        client.close()
else:
    print("Unable to connect to the Modbus Server/Slave")
