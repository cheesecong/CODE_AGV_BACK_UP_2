import time
from pymodbus.client import ModbusSerialClient
import modbus_rs485_AGV

COM_PORT = 'COM7'
BAUDRATE = 115200
SENSOR_ID = 20


def unsigned_to_signed(val):
    """Ép kiểu từ 16-bit không dấu sang có dấu (-75 đến +75 mm)"""
    if val > 32767:
        return val - 65536
    return val


def doc_cam_bien_truoc(client, sensor_id=SENSOR_ID, loc_nhieu=True, tra_ve_raw=False):
    """
    Đọc trọn bộ 16 thanh ghi (1000..1015) của cảm biến từ trước Hinson CNS-MGS-160N (ID 20)
    trong 1 khung FC04 duy nhất:
    - Tái tạo vị trí từ trường thực từ cả 3 vạch (Trái 1005, Giữa 1006, Phải 1007),
      cường độ từ trường (1008..1010) và 16 mắt Hall S1..S16 (1014..1015),
      khắc phục triệt để lỗi nhảy lùi vị trí do quy tắc ưu tiên gần tâm L1 < L2 của thanh ghi 1002.
    - Nhận vào:
        + client: đối tượng ModbusSerialClient hoặc serial.Serial.
        + loc_nhieu: True (mặc định) trả về tín hiệu đã lọc Alpha-Beta cho PID.
        + tra_ve_raw: True -> trả về tuple (gia_tri_da_loc, gia_tri_tho_1002).
    - Trả về: Giá trị lệch tâm 'deviation' (-75.0 đến +75.0 mm), hoặc None nếu lỗi đọc.
    """
    return modbus_rs485_AGV.doc_cam_bien_da_vach_mgs160n(
        client_or_ser=client,
        sensor_id=sensor_id,
        loc_nhieu=loc_nhieu,
        tra_ve_raw=tra_ve_raw,
    )


def main():
    """
    Khi chạy trực tiếp file này: Mở đồ thị Matplotlib thời gian thực
    cho Cảm biến từ trước (ID 20).
    """
    import visualize_cam_bien_tu

    visualize_cam_bien_tu.chay_visualize(
        com_port=COM_PORT,
        baudrate=BAUDRATE,
        che_do="front",
        target_hz=120.0,
    )


if __name__ == "__main__":
    main()
