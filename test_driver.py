import serial
import time
from DRIVER_AUTO_AGV import WriteRegister16

ser = serial.Serial("COM8", 115200, timeout=0.2)
print("Đang gửi thử lệnh kích hoạt xuống Driver (ID 10)...")

while True:
    ser.reset_input_buffer()
    WriteRegister16(ser, 10, 2000, 1)
    time.sleep(0.05)
    so_byte = ser.in_waiting
    du_lieu = ser.read(so_byte).hex() if so_byte > 0 else "Không có tín hiệu"
    print(f"Phản hồi từ Driver ID 10: {so_byte} bytes | Dữ liệu: {du_lieu}")
    time.sleep(0.5)