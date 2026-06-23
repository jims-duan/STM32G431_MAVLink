import serial
from ymodem.Socket import ModemSocket
import os
import time
import sys
import argparse

# 解析命令行参数
parser = argparse.ArgumentParser(description="Ymodem 串口文件发送工具")
parser.add_argument("bin_path", help="待发送bin文件路径，例如 G431_MAVLink.bin")
parser.add_argument("--port", default="/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_20783986534B-if00", help="串口设备路径")
parser.add_argument("--baud", type=int, default=115200, help="波特率")
args = parser.parse_args()

# 校验文件是否存在
file_path = args.bin_path
if not os.path.isfile(file_path):
    print(f"错误：文件 {file_path} 不存在！")
    sys.exit(1)

# 打开串口
ser = serial.Serial(
    args.port,
    args.baud,
    timeout=5.0
)

# 新增进度条全局变量
total_data_bytes = 0
file_size = 0

def format_hex(data):
    if not data:
        return "(空)"
    hex_str = data.hex().upper()
    hex_bytes = ' '.join(hex_str[i:i+2] for i in range(0, len(hex_str), 2))
    bytes_list = hex_bytes.split(' ')
    lines = []
    for i in range(0, len(bytes_list), 16):
        line = ' '.join(bytes_list[i:i+16])
        lines.append(line)
    return '\n'.join(lines)

# 新增进度条绘制函数
def show_progress():
    if file_size <= 0:
        return
    progress = total_data_bytes / file_size
    progress = min(progress, 1.0)
    bar_len = 40
    fill = int(bar_len * progress)
    bar = '█' * fill + ' ' * (bar_len - fill)
    pct = round(progress * 100, 2)
    disp = min(total_data_bytes, file_size)
    sys.stdout.write(f"\r[传输进度] |{bar}| {pct}% ({disp}/{file_size} bytes)")
    sys.stdout.flush()

def read_from_port(size, timeout=5):
    data = ser.read(size)
    if data:
        print(f"\n📥 <<<收[{len(data)}] 字节:")
        print(format_hex(data))
    else:
        print(f"\n📥 <<<收: 超时")
    return data or None

def write_to_port(data, timeout=5):
    global total_data_bytes
    bytes_written = ser.write(data)
    
    if bytes_written > 0:
        if data[0] in (0x01, 0x02) and len(data) > 5:
            data_len = len(data) - 5
            total_data_bytes += data_len
            print(f"\n📤 >>>发[{bytes_written}] 字节 (有效数据 {data_len} 字节):")
        else:
            print(f"\n📤 >>>发[{bytes_written}] 字节 (控制包):")
        
        print(format_hex(data))
    
    # 仅新增一行刷新进度条，原有逻辑完全不动
    show_progress()
    return bytes_written

ser.reset_input_buffer()
ser.reset_output_buffer()

# ====================== 新增：先检测是否收到字符C ======================
print("=" * 60)
print("检测设备是否已进入Ymodem等待(等待字符C)...")
has_C = False
# 短暂读取缓冲区，看是否存在 'C'
recv_buf = ser.read(128)
if b'C' in recv_buf:
    has_C = True
    print("✅ 检测到设备已发送 'C'，直接开始Ymodem传输")
else:
    print("❌ 未检测到设备Ymodem标志C，需要发送复位指令")
    # 等待用户回车发送复位命令
    input("准备就绪，按回车键进入升级模式...\r\n")
    reset_cmd = b"BOOTLOADER RESET\r\n"
    ser.write(reset_cmd)
    # print(f"\n已发送复位指令: {reset_cmd.decode('utf-8').strip()}")

    # 关键修改：关闭串口，等待5秒后重新打开
    ser.close()
    print("串口重启...")
    time.sleep(3)
    # 重新打开串口
    ser = serial.Serial(args.port, args.baud, timeout=5.0)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    print("串口重新打开完成")

    # 复位后等待设备输出C
    print("等待设备返回Ymodem起始字符C...")
    wait_start = time.time()
    found = False
    while time.time() - wait_start < 5.0:  # 最多等待3秒
        chunk = ser.read(64)
        if b'C' in chunk:
            found = True
            break
    if not found:
        print("⚠️ 复位后未收到字符C，继续尝试传输，可能失败")
    else:
        print("✅ 收到设备 Ymodem 起始字符 C")

print("开始Ymodem文件传输流程...")
print("=" * 60)
# ======================================================================

ym = ModemSocket(
    read_from_port, 
    write_to_port,
    packet_size=128
)

print(f"发送文件: {file_path}")
print("=" * 50)

file_size = os.path.getsize(file_path)
print(f"文件大小: {file_size} 字节")
print(f"预计包数: {(file_size + 127) // 128} 包")
print("=" * 50)

# ========== 开始计时 ==========
start_time = time.time()

success = ym.send([file_path])
# 传输完成换行分隔进度条
print("\n")

# ========== 结束计时 ==========
end_time = time.time()
elapsed_time = end_time - start_time

print("=" * 50)

# 方法1: 如果库支持获取传输统计
if hasattr(ym, 'bytes_sent'):
    total_data_bytes = ym.bytes_sent
    print(f"📊 从库获取的发送字节数: {total_data_bytes} 字节")
elif hasattr(ym, 'total_bytes'):
    total_data_bytes = ym.total_bytes
    print(f"📊 从库获取的发送字节数: {total_data_bytes} 字节")
else:
    if success:
        total_data_bytes = file_size
    else:
        total_data_bytes = 0
        print(f"📊 传输失败，无法获取统计信息")

if success:
    print(f"✅ 发送成功")
    print(f"📊 有效数据: {total_data_bytes} 字节")
    print(f"📊 文件大小: {file_size} 字节")
    
    # 时间统计
    print(f"⏱️  用时: {elapsed_time:.3f} 秒")
    
    # 速度计算
    if elapsed_time > 0:
        speed_bps = total_data_bytes / elapsed_time
        speed_kbps = speed_bps / 1024
        print(f"📈 速度: {speed_kbps:.2f} KB/s")
    
    # 完整性检查
    if total_data_bytes == file_size:
        print("✅ 数据完整！")
    else:
        print(f"⚠️ 数据不完整！差 {file_size - total_data_bytes} 字节")
else:
    print(f"❌ 发送失败")
    print(f"📊 已发送有效数据: {total_data_bytes} 字节")
    print(f"⏱️  用时: {elapsed_time:.3f} 秒")

ser.close()