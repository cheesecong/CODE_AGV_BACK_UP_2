import time
import msvcrt
from pymodbus.client import ModbusSerialClient

# =====================================================
# 1. CẤU HÌNH MODBUS & THÔNG SỐ CƠ KHÍ
# =====================================================
COM_PORT = "COM7"
BAUDRATE = 115200
SLAVE_ID = 10

WHEEL_RADIUS_CM = 9
PI = 3.14
KHOANG_CACH_2_BANH = 58  # cm

# Biến lưu trạng thái để không phải gửi lại toàn bộ 14 thanh ghi mỗi vòng PID
_da_khoi_dong_tien = False

# =====================================================
# 2. CÁC HÀM CON ĐỂ MAIN.PY GỌI SANG
# =====================================================
def ghi_thanh_ghi_fc16(client, reg, value, slave_id=SLAVE_ID):
    """
    Thay thế hàm WriteRegister16 cũ bằng pymodbus (vẫn dùng FC16 - 0x10).
    """
    value = int(max(0, min(65535, value)))  # Giới hạn an toàn 0 -> 65535
    try:
        client.write_registers(address=reg, values=[value], device_id=slave_id)
    except Exception as e:
        print(f"Lỗi ghi Driver (Reg {reg}): {e}")

def tinh_rpm_hai_banh(van_toc_xe, W_bo_cua):
    """
    Tính toán RPM cho 2 bánh từ vận tốc dài V (cm/s) và vận tốc góc W (rad/s).
    """
    V_left = van_toc_xe - (W_bo_cua * KHOANG_CACH_2_BANH / 2)
    V_right = van_toc_xe + (W_bo_cua * KHOANG_CACH_2_BANH / 2)

    rpm_left = int(V_left * 30 * 60 / (2 * PI * WHEEL_RADIUS_CM))
    rpm_right = int(V_right * 30 * 60 / (2 * PI * WHEEL_RADIUS_CM))

    # Không cho phép RPM âm vì thanh ghi Modbus nhận từ 0 - 65535
    rpm_left = max(0, min(65535, rpm_left))
    rpm_right = max(0, min(65535, rpm_right))

    return rpm_left, rpm_right

def phat_xung_chot_lenh(client, rpm_2006, rpm_2007, tien=True, slave_id=SLAVE_ID, delay=0.01):
    """
    Gửi chuỗi thanh ghi 2000 -> 2013 theo đúng quy trình của Driver.
    """
    reg_2004 = 1 if tien else 0
    reg_2005 = 0 if tien else 1

    ghi_thanh_ghi_fc16(client, 2000, 1, slave_id)
    time.sleep(delay)
    ghi_thanh_ghi_fc16(client, 2000, 0, slave_id)
    time.sleep(delay)

    ghi_thanh_ghi_fc16(client, 2001, 1, slave_id)
    time.sleep(delay)
    ghi_thanh_ghi_fc16(client, 2002, 1, slave_id)
    time.sleep(delay)
    ghi_thanh_ghi_fc16(client, 2003, 1, slave_id)
    time.sleep(delay)

    ghi_thanh_ghi_fc16(client, 2004, reg_2004, slave_id)
    time.sleep(delay)
    ghi_thanh_ghi_fc16(client, 2005, reg_2005, slave_id)
    time.sleep(delay)

    ghi_thanh_ghi_fc16(client, 2006, rpm_2006, slave_id)
    time.sleep(delay)
    ghi_thanh_ghi_fc16(client, 2007, rpm_2007, slave_id)
    time.sleep(delay)

    ghi_thanh_ghi_fc16(client, 2008, 0, slave_id)
    time.sleep(delay)
    ghi_thanh_ghi_fc16(client, 2009, 1, slave_id)
    time.sleep(delay)
    ghi_thanh_ghi_fc16(client, 2010, 0, slave_id)
    time.sleep(delay)
    ghi_thanh_ghi_fc16(client, 2011, 0, slave_id)
    time.sleep(delay)
    ghi_thanh_ghi_fc16(client, 2012, 0, slave_id)
    time.sleep(delay)
    ghi_thanh_ghi_fc16(client, 2013, 0, slave_id)

def dieu_khien_tu_dong(client, van_toc_xe, W_bo_cua, slave_id=SLAVE_ID):
    """
    Hàm chính được gọi liên tục trong vòng lặp PID của main.py:
    - van_toc_xe (cm/s): Lấy theo trạng thái thẻ RFID.
    - W_bo_cua (rad/s): Lấy từ ngõ ra của bộ điều khiển PID (2 cảm biến từ).
    """
    global _da_khoi_dong_tien

    if van_toc_xe <= 0:
        dung_xe(client, slave_id)
        return 0, 0

    rpm_left, rpm_right = tinh_rpm_hai_banh(van_toc_xe, W_bo_cua)

    if not _da_khoi_dong_tien:
        # Lần đầu chạy: Gửi đầy đủ chuỗi khởi tạo 2000 -> 2013
        phat_xung_chot_lenh(client, rpm_left, rpm_right, tien=True, slave_id=slave_id, delay=0.01)
        _da_khoi_dong_tien = True
    else:
        # Đang chạy trên đường: Chỉ cập nhật tốc độ 2 bánh (2006, 2007) và chốt lệnh (2000) để PID phản ứng nhanh
        ghi_thanh_ghi_fc16(client, 2000, 1, slave_id)
        ghi_thanh_ghi_fc16(client, 2000, 0, slave_id)
        ghi_thanh_ghi_fc16(client, 2006, rpm_left, slave_id)
        ghi_thanh_ghi_fc16(client, 2007, rpm_right, slave_id)
        ghi_thanh_ghi_fc16(client, 2009, 1, slave_id)

    return rpm_left, rpm_right

def dung_xe(client, slave_id=SLAVE_ID):
    """Dừng khẩn cấp hoặc dừng tại trạm RFID"""
    global _da_khoi_dong_tien
    _da_khoi_dong_tien = False
    ghi_thanh_ghi_fc16(client, 2000, 1, slave_id)
    ghi_thanh_ghi_fc16(client, 2000, 0, slave_id)
    ghi_thanh_ghi_fc16(client, 2004, 0, slave_id)
    ghi_thanh_ghi_fc16(client, 2005, 0, slave_id)
    ghi_thanh_ghi_fc16(client, 2006, 0, slave_id)
    ghi_thanh_ghi_fc16(client, 2007, 0, slave_id)

# =====================================================
# 3. CHẾ ĐỘ CHẠY ĐỘC LẬP BẰNG BÀN PHÍM (ĐỂ TEST RIÊNG)
# =====================================================
def main():
    client = ModbusSerialClient(port=COM_PORT, baudrate=BAUDRATE, timeout=0.1)

    if not client.connect():
        print(f"Lỗi: Không thể mở cổng {COM_PORT}")
        return

    print("\nModbus Driver (Pymodbus) đã sẵn sàng!")

    try:
        van_toc_xe = float(input("Nhập vận tốc xe (cm/s): "))
        W_bo_cua = float(input("Nhập tốc độ góc khi bo cua (rad/s): "))

        rpm_thang, _ = tinh_rpm_hai_banh(van_toc_xe, 0)
        rpm_left, rpm_right = tinh_rpm_hai_banh(van_toc_xe, W_bo_cua)

        print(f"\nRPM thẳng: {rpm_thang} | RPM trái: {rpm_left} | RPM phải: {rpm_right}")
        print("W: Tiến | S: Lùi | A: Rẽ trái | F: Rẽ phải | Space: Dừng | ESC: Thoát")

        while True:
            key = msvcrt.getch().decode("utf-8").lower()

            if key == '\x1b':
                dung_xe(client, SLAVE_ID)
                print("\nThoát chương trình!")
                break
            elif key == ' ':
                dung_xe(client, SLAVE_ID)
                print("Đã dừng xe")
            elif key == 'w':
                phat_xung_chot_lenh(client, rpm_thang, rpm_thang, tien=True, delay=0.02)
            elif key == 's':
                phat_xung_chot_lenh(client, rpm_thang, rpm_thang, tien=False, delay=0.02)
            elif key == 'f':
                phat_xung_chot_lenh(client, rpm_left, rpm_right, tien=True, delay=0.02)
            elif key == 'a':
                phat_xung_chot_lenh(client, rpm_right, rpm_left, tien=True, delay=0.02)

    except Exception as e:
        print("\nLỖI:", e)
    finally:
        client.close()
        print("Đã đóng cổng COM!")

if __name__ == "__main__":
    main()