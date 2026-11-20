import time
from pymodbus.client import ModbusSerialClient
import modbus_rs485_AGV

# 1. CẤU HÌNH GIAO TIẾP
COM_PORT = 'COM7'
BAUDRATE = 115200
RFID_ID  = 15

# 2. KHAI BÁO BẢNG LỆNH (Để ngoài hàm để main_AGV.py có thể import sang dùng chung)
TAG_QUEO_TRAI = "20092004"
TAG_QUEO_PHAI = "13032004"
TAG_DI_THANG  = "22072004"

SO_THE_DUNG_HAN = 3
THOI_GIAN_DUNG_MOI_THE_GIAY = 10.0


def mo_ta_the(ma_the: str) -> str:
    """Trả về ghi chú tên thẻ (nếu nằm trong danh sách đã biết)."""
    if ma_the == TAG_QUEO_TRAI:
        return "QUẸO TRÁI"
    if ma_the == TAG_QUEO_PHAI:
        return "QUẸO PHẢI"
    if ma_the == TAG_DI_THANG:
        return "ĐI THẲNG"
    return "THẺ TRẠM"


# ==========================================================
# HÀM CON ĐỂ CHƯƠNG TRÌNH CHÍNH (MAIN_AGV.PY) GỌI SANG
# ==========================================================
def doc_the_rfid(client, rfid_id=RFID_ID):
    """
    Đọc nhanh 8 thanh ghi từ địa chỉ 1003 của đầu đọc RFID (ID 15) có chống xung đột RS485.
    - Nhận vào: đối tượng 'client' (ModbusSerialClient hoặc serial.Serial) đang mở cổng COM.
    - Trả về:
        + Chuỗi 8 số (VD: "22072004") nếu đang có thẻ dưới gầm.
        + Chuỗi rỗng "" nếu không có thẻ (sum == 0).
        + None nếu xảy ra lỗi đường truyền Modbus.
    """
    regs = modbus_rs485_AGV.doc_input_registers_nhanh(
        client_or_ser=client,
        slave_id=rfid_id,
        address=1003,
        count=8,
        timeout=0.035,
    )
    if regs is not None and len(regs) == 8:
        if sum(regs) > 0:
            return "".join(str(x) for x in regs)
        else:
            return ""  # Không có thẻ dưới cảm biến

    return None  # Lỗi đọc Modbus


# ==========================================================
# CHẾ ĐỘ CHỈ TEST ĐẦU ĐỌC RFID ĐỘC LẬP (KHÔNG CHẠY XE)
# ==========================================================
def test_doc_rfid_doc_lap():
    ser = modbus_rs485_AGV.ket_noi_rs485_nhanh(COM_PORT, BAUDRATE, timeout=0.035)
    if ser is None:
        print(f"[!] Lỗi: Không thể mở cổng {COM_PORT}")
        return

    print(" Đã kết nối RS485 thành công.")
    print(" Đang chờ thẻ RFID...")
    print("=" * 40)

    last_tag_read = ""

    try:
        while True:
            ma_the_hien_tai = doc_the_rfid(ser, RFID_ID)

            if ma_the_hien_tai is None:
                print(f"[{time.strftime('%H:%M:%S')}] LỖI ĐỌC MODBUS RFID (ID {RFID_ID})")
            elif ma_the_hien_tai != "":
                if ma_the_hien_tai != last_tag_read:
                    print(
                        f"[{time.strftime('%H:%M:%S')}] Quét được ID thẻ: {ma_the_hien_tai} ({mo_ta_the(ma_the_hien_tai)})"
                    )
                    print("-" * 40)
                    last_tag_read = ma_the_hien_tai
            else:
                last_tag_read = ""

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n Đã dừng chương trình.")
    finally:
        if ser.is_open:
            ser.close()


def main():
    """
    Mặc định khi chạy file này sẽ khởi chạy chương trình chính (main_AGV.py):
    Vừa chạy bám line bằng PID + vừa đọc thẻ RFID (gặp thẻ hiện ID, dừng 10s rồi chạy tiếp, đủ 3 thẻ dừng hẳn).
    """
    import sys
    if "--only-rfid" in sys.argv:
        sys.argv.remove("--only-rfid")
        test_doc_rfid_doc_lap()
    else:
        import main_AGV
        main_AGV.main()


if __name__ == "__main__":
    main()
