import threading
import time
import serial
import serial.tools.list_ports

SLAVE_ID = 10
WHEEL_RADIUS_CM = 9.0
PI = 3.141592653589793
KHOANG_CACH_2_BANH = 58.0  # cm
MAX_RPM = 3000

_lenh_huong_da_gui = ""
_van_toc_da_gui = -1.0
_thoi_gian_gui_cuoi = 0.0
_da_khoi_dong_tien = False
_last_rpm_2006 = -1
_last_rpm_2007 = -1


def liet_ke_cong_com() -> str:
    """Trả về chuỗi danh sách các cổng COM đang kết nối trên máy tính."""
    try:
        ports = list(serial.tools.list_ports.comports())
        if not ports:
            return "Không tìm thấy cổng COM nào!"
        return ", ".join(f"{p.device} ({p.description})" for p in ports)
    except Exception:
        return "Không xác định"


def ModbusCRC(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def ket_noi_driver(com_port, baudrate=115200, in_loi=True):
    """Mở cổng COM riêng cho mạch Driver động cơ."""
    try:
        ser = serial.Serial(
            port=com_port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.05,
            write_timeout=0.05,
        )
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        return ser
    except Exception as e:
        if in_loi:
            print(f"[!] Lỗi mở cổng Driver ({com_port}): {e}")
            print(f"    -> Các cổng COM hiện có trên máy: {liet_ke_cong_com()}")
        return None


def WriteRegister16(ser, slaveID, reg, value):
    value = int(max(0, min(65535, value)))

    frame = bytearray(11)
    frame[0] = slaveID
    frame[1] = 0x10
    frame[2] = (reg >> 8) & 0xFF
    frame[3] = reg & 0xFF
    frame[4] = 0x00
    frame[5] = 0x01
    frame[6] = 0x02
    frame[7] = (value >> 8) & 0xFF
    frame[8] = value & 0xFF

    crc = ModbusCRC(frame[:9])
    frame[9] = crc & 0xFF
    frame[10] = (crc >> 8) & 0xFF

    ser.write(frame)
    ser.flush()


def van_toc_sang_rpm(van_toc_cm_s: float) -> int:
    """Chuyển đổi vận tốc dài bánh xe (cm/s) sang tốc độ động cơ (RPM, hộp số 1:30)."""
    if van_toc_cm_s <= 0.0:
        return 0
    rpm = int(round(van_toc_cm_s * 30.0 * 60.0 / (2.0 * PI * WHEEL_RADIUS_CM)))
    return max(0, min(MAX_RPM, rpm))


def tinh_rpm_pid(van_toc_xe: float, w_pid: float, dao_chieu_lai: bool = False):
    """
    Tính toán tốc độ 2 bánh (rpm_2006, rpm_2007) từ:
      - van_toc_xe (cm/s): Vận tốc dài mục tiêu của tâm xe.
      - w_pid (rad/s): Ngõ ra từ bộ điều khiển PID.
        Quy ước chuẩn khớp với thực tế phần cứng:
          + error > 0 (vạch lệch Phải -> cần RẼ PHẢI 'f') => w_pid > 0 => rpm_2006 < rpm_2007
          + error < 0 (vạch lệch Trái -> cần RẼ TRÁI 'a') => w_pid < 0 => rpm_2006 > rpm_2007
          + error = 0 (đi thẳng 'w')                      => w_pid = 0 => rpm_2006 == rpm_2007
    """
    if van_toc_xe <= 0.0:
        return 0, 0

    w_eff = -w_pid if dao_chieu_lai else w_pid
    delta_v = w_eff * (KHOANG_CACH_2_BANH / 2.0)

    v_2006 = van_toc_xe - delta_v
    v_2007 = van_toc_xe + delta_v

    if v_2006 < 0.0:
        v_2006 = 0.0
    if v_2007 < 0.0:
        v_2007 = 0.0

    rpm_2006 = van_toc_sang_rpm(v_2006)
    rpm_2007 = van_toc_sang_rpm(v_2007)
    return rpm_2006, rpm_2007


def _gui_chuoi_khoi_dong_tien(ser, slaveID, rpm_2006, rpm_2007, delay=0.018):
    """Gửi đầy đủ 15 thanh ghi khởi tạo tiến (2000 -> 2013) ở lần chạy đầu tiên."""
    ser.reset_input_buffer()
    seq = [
        (2000, 1),
        (2000, 0),
        (2001, 1),
        (2002, 1),
        (2003, 1),
        (2004, 1),
        (2005, 0),
        (2006, rpm_2006),
        (2007, rpm_2007),
        (2008, 0),
        (2009, 1),
        (2010, 0),
        (2011, 0),
        (2012, 0),
        (2013, 0),
    ]
    for reg, val in seq:
        WriteRegister16(ser, slaveID, reg, val)
        time.sleep(delay)
    ser.reset_input_buffer()


def _gui_cap_nhat_rpm_nhanh(ser, slaveID, rpm_2006, rpm_2007, delay=0.010):
    """
    Cập nhật nhanh tốc độ 2 bánh (2006, 2007) và chốt lệnh (2000, 2009)
    để đáp ứng PID liên tục mà không bị trễ 375ms.
    """
    WriteRegister16(ser, slaveID, 2000, 1)
    time.sleep(delay)
    WriteRegister16(ser, slaveID, 2000, 0)
    time.sleep(delay)
    WriteRegister16(ser, slaveID, 2006, rpm_2006)
    time.sleep(delay)
    WriteRegister16(ser, slaveID, 2007, rpm_2007)
    time.sleep(delay)
    WriteRegister16(ser, slaveID, 2009, 1)
    time.sleep(delay)
    if ser.in_waiting:
        ser.reset_input_buffer()


class BoDieuKhienDriverPID:
    """
    Luồng điều khiển Driver độc lập trên cổng COM8:
    - Giúp vòng lặp đọc cảm biến từ + tính PID trên COM7 luôn chạy ~55-60Hz (0ms bị chặn bởi COM8).
    - Chỉ gửi chuỗi 15 thanh ghi khởi tạo (2000..2013) khi bắt đầu có lệnh RPM > 0 (tránh gửi 0,0 sớm).
    - Cập nhật ngay lập tức RPM 2 bánh (2006, 2007) khi PID thay đổi hoặc gửi Heartbeat mỗi 0.30s.
    """

    def __init__(
        self,
        ser,
        slave_id=SLAVE_ID,
        rpm_deadband=2,
        heartbeat_interval=0.30,
    ):
        self.ser = ser
        self.slave_id = slave_id
        self.rpm_deadband = rpm_deadband
        self.heartbeat_interval = heartbeat_interval

        self._lock = threading.Lock()
        self._event = threading.Event()
        self._running = False
        self._thread = None

        self._target_2006 = 0
        self._target_2007 = 0
        self._want_stop = True  # Chờ cho đến khi có lệnh RPM > 0 đầu tiên từ PID

        self.applied_2006 = 0
        self.applied_2007 = 0
        self.so_lan_cap_nhat = 0
        self.da_khoi_dong = False
        self.loi_ket_noi = False

    def bat_dau(self):
        if self.ser is None:
            return
        self._running = True
        self._thread = threading.Thread(target=self._vong_lap_driver, daemon=True)
        self._thread.start()

    def cap_nhat_rpm(self, rpm_2006: int, rpm_2007: int):
        """Được gọi từ vòng lặp PID (không gây block cổng COM7)."""
        rpm_2006 = max(0, min(MAX_RPM, int(rpm_2006)))
        rpm_2007 = max(0, min(MAX_RPM, int(rpm_2007)))
        with self._lock:
            changed = (
                abs(rpm_2006 - self._target_2006) >= self.rpm_deadband
                or abs(rpm_2007 - self._target_2007) >= self.rpm_deadband
                or not self.da_khoi_dong
            )
            self._target_2006 = rpm_2006
            self._target_2007 = rpm_2007
            self._want_stop = (rpm_2006 == 0 and rpm_2007 == 0)
        if changed:
            self._event.set()

    def dung_khan_cap(self):
        """Dừng xe an toàn và kết thúc luồng Driver."""
        with self._lock:
            self._target_2006 = 0
            self._target_2007 = 0
            self._want_stop = True
        self._running = False
        self._event.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        dung_xe(self.ser, self.slave_id, delay=0.015)
        self.applied_2006 = 0
        self.applied_2007 = 0
        self.da_khoi_dong = False

    def _vong_lap_driver(self):
        last_send_t = 0.0
        last_sent_2006 = 0
        last_sent_2007 = 0

        while self._running:
            self._event.wait(timeout=0.025)
            self._event.clear()
            if not self._running:
                break

            with self._lock:
                r2006 = self._target_2006
                r2007 = self._target_2007
                stop_req = self._want_stop

            now = time.perf_counter()

            try:
                if stop_req:
                    if self.da_khoi_dong or last_sent_2006 != 0 or last_sent_2007 != 0:
                        dung_xe(self.ser, self.slave_id, delay=0.015)
                        self.da_khoi_dong = False
                        last_sent_2006 = 0
                        last_sent_2007 = 0
                        self.applied_2006 = 0
                        self.applied_2007 = 0
                        last_send_t = time.perf_counter()
                    continue

                if not self.da_khoi_dong:
                    _gui_chuoi_khoi_dong_tien(
                        self.ser, self.slave_id, r2006, r2007, delay=0.018
                    )
                    self.da_khoi_dong = True
                    self.loi_ket_noi = False
                    last_sent_2006 = r2006
                    last_sent_2007 = r2007
                    self.applied_2006 = r2006
                    self.applied_2007 = r2007
                    self.so_lan_cap_nhat += 1
                    last_send_t = time.perf_counter()
                else:
                    diff_2006 = abs(r2006 - last_sent_2006)
                    diff_2007 = abs(r2007 - last_sent_2007)
                    need_heartbeat = (now - last_send_t) >= self.heartbeat_interval

                    if (
                        diff_2006 >= self.rpm_deadband
                        or diff_2007 >= self.rpm_deadband
                        or need_heartbeat
                    ):
                        _gui_cap_nhat_rpm_nhanh(
                            self.ser, self.slave_id, r2006, r2007, delay=0.010
                        )
                        self.loi_ket_noi = False
                        last_sent_2006 = r2006
                        last_sent_2007 = r2007
                        self.applied_2006 = r2006
                        self.applied_2007 = r2007
                        self.so_lan_cap_nhat += 1
                        last_send_t = time.perf_counter()
            except Exception as e:
                self.loi_ket_noi = True
                self.da_khoi_dong = False
                print(f"[Driver COM8] Lỗi gửi lệnh: {e}")
                time.sleep(0.05)


def dieu_khien_pid(
    ser,
    van_toc_xe: float,
    w_pid: float,
    slaveID=SLAVE_ID,
    dao_chieu_lai: bool = False,
):
    """
    Hàm điều khiển trực tiếp bằng PID (nếu gọi đồng bộ không qua luồng):
    - Lần đầu khởi tạo đầy đủ 15 thanh ghi 2000..2013.
    - Các chu kỳ tiếp theo chỉ cập nhật nhanh 2006, 2007 và chốt lệnh khi RPM thay đổi hoặc mỗi 0.30s.
    """
    global _da_khoi_dong_tien, _thoi_gian_gui_cuoi, _last_rpm_2006, _last_rpm_2007

    if van_toc_xe <= 0.0:
        dung_xe(ser, slaveID)
        return 0, 0

    rpm_2006, rpm_2007 = tinh_rpm_pid(van_toc_xe, w_pid, dao_chieu_lai=dao_chieu_lai)
    hien_tai = time.perf_counter()

    if not _da_khoi_dong_tien:
        _gui_chuoi_khoi_dong_tien(ser, slaveID, rpm_2006, rpm_2007, delay=0.018)
        _da_khoi_dong_tien = True
        _last_rpm_2006 = rpm_2006
        _last_rpm_2007 = rpm_2007
        _thoi_gian_gui_cuoi = hien_tai
    elif (
        abs(rpm_2006 - _last_rpm_2006) >= 2
        or abs(rpm_2007 - _last_rpm_2007) >= 2
        or (hien_tai - _thoi_gian_gui_cuoi) >= 0.30
    ):
        _gui_cap_nhat_rpm_nhanh(ser, slaveID, rpm_2006, rpm_2007, delay=0.010)
        _last_rpm_2006 = rpm_2006
        _last_rpm_2007 = rpm_2007
        _thoi_gian_gui_cuoi = hien_tai

    return rpm_2006, rpm_2007


def dieu_khien_if_else(ser, van_toc_xe, W_bo_cua, lenh_huong, slaveID=SLAVE_ID, delay=0.025):
    """Giữ tương thích ngược với chế độ IF-ELSE cũ."""
    global _lenh_huong_da_gui, _van_toc_da_gui, _thoi_gian_gui_cuoi

    Toc_do_dong_co = van_toc_sang_rpm(van_toc_xe)
    V_left = van_toc_xe - (W_bo_cua * KHOANG_CACH_2_BANH / 2.0)
    V_right = van_toc_xe + (W_bo_cua * KHOANG_CACH_2_BANH / 2.0)

    Toc_do_dong_co_left = van_toc_sang_rpm(V_left)
    Toc_do_dong_co_right = van_toc_sang_rpm(V_right)

    if lenh_huong == "w":
        rpm_2006, rpm_2007 = Toc_do_dong_co, Toc_do_dong_co
    elif lenh_huong == "f":
        rpm_2006, rpm_2007 = Toc_do_dong_co_left, Toc_do_dong_co_right
    elif lenh_huong == "a":
        rpm_2006, rpm_2007 = Toc_do_dong_co_right, Toc_do_dong_co_left
    else:
        dung_xe(ser, slaveID)
        return 0, 0

    hien_tai = time.time()

    if lenh_huong != _lenh_huong_da_gui or van_toc_xe != _van_toc_da_gui:
        _gui_chuoi_khoi_dong_tien(ser, slaveID, rpm_2006, rpm_2007, delay=delay)
        _lenh_huong_da_gui = lenh_huong
        _van_toc_da_gui = van_toc_xe
        _thoi_gian_gui_cuoi = hien_tai
    elif hien_tai - _thoi_gian_gui_cuoi >= 0.4:
        WriteRegister16(ser, slaveID, 2006, rpm_2006)
        time.sleep(0.01)
        WriteRegister16(ser, slaveID, 2007, rpm_2007)
        time.sleep(0.01)
        WriteRegister16(ser, slaveID, 2009, 1)
        time.sleep(0.01)
        ser.reset_input_buffer()
        _thoi_gian_gui_cuoi = hien_tai

    return rpm_2006, rpm_2007


def dung_xe(ser, slaveID=SLAVE_ID, delay=0.020):
    global _lenh_huong_da_gui, _van_toc_da_gui, _da_khoi_dong_tien, _last_rpm_2006, _last_rpm_2007
    if ser is not None and getattr(ser, "is_open", False):
        try:
            WriteRegister16(ser, slaveID, 2000, 1)
            time.sleep(delay)
            WriteRegister16(ser, slaveID, 2000, 0)
            time.sleep(delay)
            WriteRegister16(ser, slaveID, 2004, 0)
            time.sleep(delay)
            WriteRegister16(ser, slaveID, 2005, 0)
            time.sleep(delay)
            WriteRegister16(ser, slaveID, 2006, 0)
            time.sleep(delay)
            WriteRegister16(ser, slaveID, 2007, 0)
            time.sleep(delay)
            ser.reset_input_buffer()
        except Exception:
            pass
    _lenh_huong_da_gui = ""
    _van_toc_da_gui = -1.0
    _da_khoi_dong_tien = False
    _last_rpm_2006 = -1
    _last_rpm_2007 = -1
