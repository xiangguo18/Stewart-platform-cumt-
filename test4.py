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


