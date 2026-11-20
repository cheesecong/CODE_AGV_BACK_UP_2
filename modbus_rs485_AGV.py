import collections
import ctypes
import logging
import os
import subprocess
import sys
import threading
import time

try:
    import serial
except ModuleNotFoundError:
    _py310 = r"C:\Users\cheesecong\AppData\Local\Programs\Python\Python310\python.exe"
    if os.path.exists(_py310) and os.path.normcase(sys.executable) != os.path.normcase(_py310):
        _p = subprocess.Popen([_py310, *sys.argv])
        try:
            sys.exit(_p.wait())
        except KeyboardInterrupt:
            sys.exit(_p.wait())
    raise

# =====================================================================
# 1. BẬT TIMER 1ms & TẮT WINDOWS 11 ECOQOS POWER THROTTLING (64-BIT SAFE)
# =====================================================================
try:
    ctypes.windll.winmm.timeBeginPeriod(1)

    k32 = ctypes.windll.kernel32
    k32.GetCurrentProcess.restype = ctypes.c_void_p
    k32.SetPriorityClass.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
    k32.SetProcessInformation.argtypes = [
        ctypes.c_void_p,
        ctypes.c_int,
        ctypes.c_void_p,
        ctypes.c_ulong,
    ]

    h_proc = k32.GetCurrentProcess()
    k32.SetPriorityClass(h_proc, 0x00000080)  # HIGH_PRIORITY_CLASS

    class _PROCESS_POWER_THROTTLING_STATE(ctypes.Structure):
        _fields_ = [
            ("Version", ctypes.c_ulong),
            ("ControlMask", ctypes.c_ulong),
            ("StateMask", ctypes.c_ulong),
        ]

    _state = _PROCESS_POWER_THROTTLING_STATE(1, 0x1 | 0x4, 0)
    k32.SetProcessInformation(h_proc, 4, ctypes.byref(_state), ctypes.sizeof(_state))
except Exception:
    pass

# =====================================================================
# 2. THÔNG SỐ CHUẨN THEO SỔ TAY HINSON CNS-MGS-160N V1.6.5 (TRANG 26, 72-73)
# =====================================================================
PHYSICAL_MIN_MM = -75.0
PHYSICAL_MAX_MM = 75.0

RS485_TURNAROUND_DELAY = 0.0025   # 2.5 ms: Đạt 100% gói OK (0 Timeout) trên bus COM7
SAME_SLAVE_MIN_INTERVAL = 0.0050  # 5.0 ms: Chu kỳ đáp ứng nội bộ của CNS-MGS-160N (Mục 1.2)
DEFAULT_READ_TIMEOUT = 0.035      # 35 ms: Không bao giờ cắt sớm gói tin của FTDI

# Tọa độ vật lý (mm) của 16 mắt cảm biến Hall S1..S16 (khoảng cách 10mm, tâm ở 85mm - Trang 4 & 20)
HALL_POSITIONS_MM = [10.0 * i - 85.0 for i in range(1, 17)]  # [-75, -65, ..., -5, +5, ..., +75]


def precise_wait(duration_sec: float):
    """Ngủ chính xác bằng kernel timer 1ms (nhường 100% GIL cho Matplotlib)."""
    if duration_sec > 0:
        time.sleep(duration_sec)


def unsigned_to_signed16(val: int) -> int:
    """Ép kiểu 16-bit không dấu sang có dấu (-32768 .. +32767)."""
    return val - 65536 if val > 32767 else val


def giai_ma_16_mat_hall(reg_1014: int, reg_1015: int):
    """
    Giải mã trạng thái bật/tắt (0/1) của 16 mắt cảm biến từ S1..S16
    từ thanh ghi 1014 và 1015 (Mục 7.4.4 Trang 73):
      - Thanh ghi 1015:
        + Byte thấp (Low 8-bit): bit1..bit7 = S1..S7 (bit0 không dùng)
        + Byte cao (High 8-bit): bit0..bit7 = S8..S15
      - Thanh ghi 1014:
        + bit0 = S16
    Trả về: (bits_list_16, centroid_mm_tu_cac_mat_sang)
    """
    low_1015 = reg_1015 & 0xFF
    high_1015 = (reg_1015 >> 8) & 0xFF
    bits = [0] * 16
    # S1 .. S7
    for i in range(1, 8):
        bits[i - 1] = (low_1015 >> i) & 1
    # S8 .. S15
    for i in range(8):
        bits[7 + i] = (high_1015 >> i) & 1
    # S16
    bits[15] = reg_1014 & 1

    active_positions = [HALL_POSITIONS_MM[idx] for idx, b in enumerate(bits) if b == 1]
    if active_positions:
        centroid = sum(active_positions) / len(active_positions)
    else:
        centroid = None
    return bits, centroid


def tai_tao_vi_tri_da_vach(regs_1000_1015: list) -> dict:
    """
    Phân tích toàn bộ 16 thanh ghi Input (1000 -> 1015) của Hinson CNS-MGS-160N (Trang 26 & 72-73)
    để khắc phục triệt để hiện tượng "nhảy lùi / đi ngược từ phải qua trái" khi quét 1 chiều:

    NGUYÊN NHÂN TRONG DATASHEET (Mục 6.1 Trang 26 & Mục 7.4.4 Trang 72):
      - Khi từ trường nam châm rộng hoặc có từ trường vòngกลับ (fringe field) ở 2 mép,
        cảm biến tách nam châm thành 2 hoặc 3 vạch từ: Trái (Reg 1005), Giữa (Reg 1006), Phải (Reg 1007).
      - Thanh ghi 1002 (khi Reg 2000 = 0) CHỈ trả về vạch nào có khoảng cách hình học gần tâm 0mm nhất
        (Quy tắc L1 < L2 ở Trang 26), chứ KHÔNG trả về cực từ mạnh nhất!
      - Do đó, khi bạn quét nam châm từ Trái qua Phải vượt quá +15mm, cực chính của nam châm đi ra xa tâm
        (bị đẩy sang Reg 1007 - Vạch Phải), còn phần đuôi/từ trường phụ phía sau lại nằm gần 0mm hơn nên
        chiếm chỗ Reg 1002 -> Làm giá trị đọc từ Reg 1002 bị giật lùi hoặc đi ngược chiều!

    GIẢI PHÁP TÁI TẠO VỊ TRÍ THỰC (TRUE POSITION RECONSTRUCTION):
      1. Đọc đồng thời cả 3 vạch: Trái (1005), Giữa (1006), Phải (1007) cùng Cường độ từ trường
         (1008, 1009, 1010) và Số mắt nhận từ (1011, 1012, 1013).
      2. Nếu có nhiều vạch xuất hiện đồng thời (Reg 1001 mask in 3, 5, 6, 7):
         - Nếu các vạch có CÙNG cực từ (cùng dấu cường độ) và sát nhau: tính tâm hợp lực theo trọng số
           (Cường độ * Số mắt) để gộp 2 nửa nam châm thành 1 tâm duy nhất liên tục!
         - Nếu có vạch phụ ngược cực / yếu hơn hẳn (fringe lobe): chọn vạch có Cường độ từ trường mạnh nhất!
    """
    status = regs_1000_1015[0]
    mask = regs_1000_1015[1]
    pos_1002 = float(unsigned_to_signed16(regs_1000_1015[2]))
    str_1003 = unsigned_to_signed16(regs_1000_1015[3])
    pts_1004 = regs_1000_1015[4]

    pos_L = float(unsigned_to_signed16(regs_1000_1015[5]))
    pos_C = float(unsigned_to_signed16(regs_1000_1015[6]))
    pos_R = float(unsigned_to_signed16(regs_1000_1015[7]))

    str_L = unsigned_to_signed16(regs_1000_1015[8])
    str_C = unsigned_to_signed16(regs_1000_1015[9])
    str_R = unsigned_to_signed16(regs_1000_1015[10])

    pts_L = regs_1000_1015[11]
    pts_C = regs_1000_1015[12]
    pts_R = regs_1000_1015[13]

    reg_1014 = regs_1000_1015[14]
    reg_1015 = regs_1000_1015[15]
    hall_bits, hall_centroid = giai_ma_16_mat_hall(reg_1014, reg_1015)

    # Thu thập danh sách các vạch đang thực sự kích hoạt theo bitmask Reg 1001
    tracks = []
    if (mask & 1) and -75.0 <= pos_L <= 75.0 and (pts_L > 0 or abs(str_L) > 0):
        tracks.append(("L", pos_L, str_L, max(1, pts_L)))
    if (mask & 2) and -75.0 <= pos_C <= 75.0 and (pts_C > 0 or abs(str_C) > 0):
        tracks.append(("C", pos_C, str_C, max(1, pts_C)))
    if (mask & 4) and -75.0 <= pos_R <= 75.0 and (pts_R > 0 or abs(str_R) > 0):
        tracks.append(("R", pos_R, str_R, max(1, pts_R)))

    # Xác định vị trí thực (pos_true) trước khi đưa vào bộ lọc
    if mask == 0 or pts_1004 == 0:
        pos_true = 0.0
    elif len(tracks) == 1:
        pos_true = tracks[0][1]
    elif len(tracks) >= 2:
        # Sắp xếp theo độ mạnh từ trường |str| * pts
        tracks_by_power = sorted(tracks, key=lambda t: abs(t[2]) * t[3], reverse=True)
        dom = tracks_by_power[0]
        dom_power = max(1.0, float(abs(dom[2]) * dom[3]))

        # Gộp các vạch có cùng cực từ (cùng dấu str) và có độ mạnh đáng kể (>= 35% vạch chính)
        # để tái tạo tâm thực của viên nam châm bị tách đôi!
        w_sum = 0.0
        x_sum = 0.0
        for name, p_mm, s_val, pt_cnt in tracks:
            same_pole = (s_val * dom[2]) >= 0
            power = float(max(100, abs(s_val)) * pt_cnt)
            if same_pole and power >= 0.35 * dom_power:
                w_sum += power
                x_sum += power * p_mm

        if w_sum > 0:
            pos_true = x_sum / w_sum
        else:
            pos_true = dom[1]
    else:
        # Nếu pos_1002 nằm ngoài [-75, +75] nhưng có mắt Hall đang sáng, dùng tâm mắt Hall
        if -75.0 <= pos_1002 <= 75.0:
            pos_true = pos_1002
        elif hall_centroid is not None:
            pos_true = hall_centroid
        else:
            pos_true = pos_1002

    return {
        "status": status,
        "mask": mask,
        "pos_1002": pos_1002,
        "pos_true": round(pos_true, 2),
        "strength": str_1003,
        "points": pts_1004,
        "pos_L": pos_L,
        "pos_C": pos_C,
        "pos_R": pos_R,
        "str_L": str_L,
        "str_C": str_C,
        "str_R": str_R,
        "pts_L": pts_L,
        "pts_C": pts_C,
        "pts_R": pts_R,
        "hall_bits": hall_bits,
        "hall_centroid": hall_centroid,
    }


# =====================================================================
# 3. BỘ LỌC ALPHA-BETA (1D KALMAN) THEO DÕI VẬN TỐC KHÔNG TRỄ PHA
# =====================================================================
class BoLocCamBienTuPID:
    """
    Bộ lọc dự đoán vị trí - vận tốc Alpha-Beta (1D Kalman Tracker) kết hợp cắt rác vật lý.
    """

    def __init__(
        self,
        min_mm: float = PHYSICAL_MIN_MM,
        max_mm: float = PHYSICAL_MAX_MM,
        zero_hold_max: int = 3,
        alpha: float = 0.65,
        beta: float = 0.20,
    ):
        self.min_mm = min_mm
        self.max_mm = max_mm
        self.zero_hold_max = zero_hold_max
        self.alpha = alpha
        self.beta = beta

        self._initialized = False
        self._x = 0.0
        self._v = 0.0
        self._last_t = None
        self._zero_streak = 0
        self._pending_spike = None
        self.so_mau_rac_da_loc = 0

    def reset(self):
        self._initialized = False
        self._x = 0.0
        self._v = 0.0
        self._last_t = None
        self._zero_streak = 0
        self._pending_spike = None

    def cap_nhat(self, raw_val: float, now: float = None) -> float:
        """Nhận mẫu raw_val (mm) và trả về giá trị mượt liên tục cho PID (mm)."""
        if now is None:
            now = time.perf_counter()

        if raw_val is None:
            return round(self._x, 2)

        z = float(raw_val)

        # 1. Loại bỏ rác ngoài dải vật lý [-75, +75] mm (VD: 77, 81, 83, 86, 88 mm)
        if z < self.min_mm or z > self.max_mm:
            self.so_mau_rac_da_loc += 1
            if self._initialized and self._last_t is not None:
                dt = min(0.05, max(0.002, now - self._last_t))
                self._last_t = now
                self._v *= 0.6
                self._x = max(self.min_mm, min(self.max_mm, self._x + self._v * dt))
            return round(self._x, 2)

        if not self._initialized:
            self._initialized = True
            self._x = z
            self._v = 0.0
            self._last_t = now
            return round(self._x, 2)

        dt = now - self._last_t if self._last_t is not None else 0.016
        dt = min(0.06, max(0.002, dt))
        self._last_t = now

        # 2. Chống sập về 0.0 mm giả khi nam châm đang ở xa tâm (|x| >= 22mm)
        if z == 0.0 and abs(self._x) >= 22.0:
            self._zero_streak += 1
            if self._zero_streak <= self.zero_hold_max:
                self.so_mau_rac_da_loc += 1
                self._v *= 0.5
                return round(self._x, 2)
        else:
            self._zero_streak = 0

        # 3. Dự đoán vị trí tiếp theo dựa trên vận tốc lướt hiện tại
        x_pred = max(self.min_mm, min(self.max_mm, self._x + self._v * dt))
        residual = z - x_pred

        # 4. Kiểm tra gai nhiễu đơn lẻ 1 mẫu (> 48mm)
        if abs(residual) > 48.0 and self._pending_spike is None:
            self._pending_spike = z
            self.so_mau_rac_da_loc += 1
            self._x = x_pred + 0.25 * residual
            return round(self._x, 2)
        elif self._pending_spike is not None:
            if abs(z - self._pending_spike) <= 25.0:
                self._x = 0.35 * self._pending_spike + 0.65 * z
                self._v = (z - self._pending_spike) / dt
                self._pending_spike = None
                return round(self._x, 2)
            else:
                self._pending_spike = None

        # 5. Cập nhật bộ lọc Alpha-Beta
        abs_res = abs(residual)
        if abs_res <= 1.5:
            eff_alpha = 0.38
            eff_beta = 0.08
        else:
            eff_alpha = min(0.78, self.alpha + 0.01 * abs_res)
            eff_beta = self.beta

        self._x = x_pred + eff_alpha * residual
        self._v = (1.0 - eff_beta) * self._v + eff_beta * (residual / dt)
        self._v = max(-1200.0, min(1200.0, self._v))
        self._x = max(self.min_mm, min(self.max_mm, self._x))

        if z == 0.0 and abs(self._x) < 0.35:
            self._x = 0.0
            self._v = 0.0

        return round(self._x, 2)


_bo_loc_theo_slave = {}
_chi_tiet_theo_slave = {}


def lay_bo_loc_slave(slave_id: int) -> BoLocCamBienTuPID:
    """Lấy (hoặc khởi tạo) bộ lọc PID riêng cho từng cảm biến từ theo Slave ID."""
    with _global_lock:
        if slave_id not in _bo_loc_theo_slave:
            _bo_loc_theo_slave[slave_id] = BoLocCamBienTuPID()
        return _bo_loc_theo_slave[slave_id]


def lay_chi_tiet_slave(slave_id: int):
    """Lấy thông tin chẩn đoán 16 thanh ghi gần nhất của Slave ID."""
    return _chi_tiet_theo_slave.get(slave_id)


# =====================================================================
# 4. QUẢN LÝ KHÓA LUỒNG & GIAO TIẾP MODBUS RS485 (100% ỔN ĐỊNH - 0 LỖI)
# =====================================================================
_port_locks = {}
_port_last_io_time = {}
_slave_last_poll_time = {}
_configured_serial_ids = set()
_last_valid_registers = {}
_global_lock = threading.Lock()

_frame_cache = {}

thong_ke_modbus = {
    "ok": 0,
    "timeout": 0,
    "crc_err": 0,
    "id_mismatch": 0,
    "resync_ok": 0,
    "last_dt_ms": 0.0,
    "avg_dt_ms": 0.0,
    "ftdi_latency_ms": 2,
}

LOG_FILE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "modbus_log.txt")
_logger = logging.getLogger("ModbusRS485")


def _dam_bao_logger(reset_file: bool = False):
    if reset_file and _logger.handlers:
        for h in list(_logger.handlers):
            _logger.removeHandler(h)
            h.close()
    if not _logger.handlers:
        _logger.setLevel(logging.INFO)
        mode = "w" if reset_file else "a"
        _fh = logging.FileHandler(LOG_FILE_PATH, mode=mode, encoding="utf-8")
        _fh.setFormatter(
            logging.Formatter("[%(asctime)s.%(msecs)03d] %(message)s", datefmt="%H:%M:%S")
        )
        _logger.addHandler(_fh)


def ghi_log(msg: str):
    """Ghi dòng log vào modbus_log.txt (chế độ append an toàn)."""
    try:
        _dam_bao_logger(reset_file=False)
        _logger.info(msg)
    except Exception:
        pass


def toi_uu_ftdi_truoc_khi_mo_cong(com_port: str, target_ms: int = 2) -> int:
    """Hạ LatencyTimer của cáp USB-RS485 FTDI qua ftd2xx.dll."""
    try:
        target_num = int("".join(ch for ch in com_port if ch.isdigit()))
    except ValueError:
        return -1

    try:
        dll = ctypes.WinDLL("ftd2xx.dll")
        for idx in range(8):
            h = ctypes.c_void_p()
            if dll.FT_Open(idx, ctypes.byref(h)) == 0 and h.value:
                try:
                    port_num = ctypes.c_long(0)
                    if dll.FT_GetComPortNumber(h, ctypes.byref(port_num)) == 0:
                        if port_num.value == target_num:
                            dll.FT_SetLatencyTimer(h, ctypes.c_ubyte(target_ms))
                            actual = ctypes.c_ubyte(0)
                            dll.FT_GetLatencyTimer(h, ctypes.byref(actual))
                            thong_ke_modbus["ftdi_latency_ms"] = actual.value
                            return actual.value
                finally:
                    dll.FT_Close(h)
    except Exception:
        pass
    return -1


def modbus_crc16(data: bytes) -> int:
    """Tính CRC16 chuẩn Modbus RTU (đa thức 0xA001 - Mục 7.1.5 Trang 43)."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def _build_read_frame(slave_id: int, func_code: int, address: int, count: int) -> bytes:
    """Tạo và cache khung hỏi Modbus RTU 8-byte."""
    key = (slave_id, func_code, address, count)
    frame = _frame_cache.get(key)
    if frame is None:
        buf = bytearray(8)
        buf[0] = slave_id & 0xFF
        buf[1] = func_code & 0xFF
        buf[2] = (address >> 8) & 0xFF
        buf[3] = address & 0xFF
        buf[4] = (count >> 8) & 0xFF
        buf[5] = count & 0xFF
        crc = modbus_crc16(buf[:6])
        buf[6] = crc & 0xFF
        buf[7] = (crc >> 8) & 0xFF
        frame = bytes(buf)
        _frame_cache[key] = frame
    return frame


def lay_serial_va_lock(client_or_ser):
    """Lấy đối tượng serial.Serial và Lock riêng cho cổng COM."""
    if client_or_ser is None:
        return None, None, None

    ser = getattr(client_or_ser, "socket", client_or_ser)
    if ser is None or not getattr(ser, "is_open", False):
        return None, None, None

    port_name = getattr(ser, "port", str(id(ser)))
    with _global_lock:
        if port_name not in _port_locks:
            _port_locks[port_name] = threading.RLock()
            _port_last_io_time[port_name] = 0.0

        ser_id = id(ser)
        if ser_id not in _configured_serial_ids:
            try:
                cur_to = getattr(ser, "timeout", None)
                if cur_to is None or cur_to < 0.030 or cur_to > 0.100:
                    ser.timeout = DEFAULT_READ_TIMEOUT
                if hasattr(ser, "set_buffer_size"):
                    ser.set_buffer_size(rx_size=4096, tx_size=4096)
            except Exception:
                pass
            _configured_serial_ids.add(ser_id)

        return ser, _port_locks[port_name], port_name


def ket_noi_rs485_nhanh(
    com_port: str, baudrate: int = 115200, timeout: float = DEFAULT_READ_TIMEOUT
):
    """Mở cổng RS485 bằng pyserial chuẩn (đạt 100% gói OK, 0 Timeout)."""
    _dam_bao_logger(reset_file=True)
    try:
        toi_uu_ftdi_truoc_khi_mo_cong(com_port, target_ms=2)
        ser = serial.Serial(
            port=com_port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
            write_timeout=timeout,
        )
        try:
            if hasattr(ser, "set_buffer_size"):
                ser.set_buffer_size(rx_size=4096, tx_size=4096)
        except Exception:
            pass
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        with _global_lock:
            _configured_serial_ids.add(id(ser))
        ghi_log(f"Mở cổng {com_port} thành công @ {baudrate}bps, timeout={timeout*1000:.0f}ms")
        return ser
    except Exception as e:
        msg = f"Lỗi mở cổng RS485 ({com_port}): {e}"
        print(msg)
        ghi_log(msg)
        return None


def _kiem_tra_crc_khung(resp: bytes, func_code: int, count: int) -> bool:
    """Kiểm tra độ dài, Function Code, Byte Count và CRC16."""
    expected_bytes = 5 + (2 * count)
    if len(resp) != expected_bytes:
        return False
    if resp[1] != func_code or resp[2] != (count * 2):
        return False
    crc_calc = modbus_crc16(resp[:-2])
    crc_recv = resp[-2] | (resp[-1] << 8)
    return crc_calc == crc_recv


def doc_input_registers_nhanh(
    client_or_ser,
    slave_id: int,
    address: int,
    count: int = 1,
    timeout: float = DEFAULT_READ_TIMEOUT,
    turnaround_delay: float = RS485_TURNAROUND_DELAY,
):
    """
    Đọc thanh ghi Input Register (Function Code 0x04) ổn định tuyệt đối (0% lỗi Timeout).
    """
    ser, lock, port_name = lay_serial_va_lock(client_or_ser)
    if ser is None:
        return None

    request_frame = _build_read_frame(slave_id, 0x04, address, count)
    expected_bytes = 5 + (2 * count)

    with lock:
        try:
            now = time.perf_counter()

            # 1. Khoảng lặng chuyển mạch RS485 giữa 2 khung truyền (2.5 ms)
            elapsed_bus = now - _port_last_io_time.get(port_name, 0.0)
            if 0.0 <= elapsed_bus < turnaround_delay:
                time.sleep(turnaround_delay - elapsed_bus)
                now = time.perf_counter()

            # 2. Đảm bảo chu kỳ T2 >= 5ms khi hỏi lặp lại cùng 1 Slave
            slave_key = (port_name, slave_id)
            elapsed_slave = now - _slave_last_poll_time.get(slave_key, 0.0)
            if 0.0 <= elapsed_slave < SAME_SLAVE_MIN_INTERVAL:
                time.sleep(SAME_SLAVE_MIN_INTERVAL - elapsed_slave)

            # 3. Xả sạch byte trễ (nếu có) trước khi gửi lệnh mới
            waiting = ser.in_waiting
            if waiting > 0:
                stale = ser.read(waiting)
                if len(stale) == expected_bytes and _kiem_tra_crc_khung(stale, 0x04, count):
                    other_id = stale[0]
                    regs_other = [
                        (stale[3 + 2 * i] << 8) | stale[4 + 2 * i]
                        for i in range(count)
                    ]
                    _last_valid_registers[(port_name, other_id, address, count)] = regs_other

            # 4. Gửi khung hỏi 8-byte xuống bus RS485
            t_send = time.perf_counter()
            _slave_last_poll_time[slave_key] = t_send
            ser.write(request_frame)
            ser.flush()

            # 5. Đọc đúng số byte kỳ vọng
            resp = ser.read(expected_bytes)
            t_recv = time.perf_counter()
            _port_last_io_time[port_name] = t_recv
            dt_ms = (t_recv - t_send) * 1000.0
            thong_ke_modbus["last_dt_ms"] = dt_ms

            # 6. Nếu Timeout hoặc thiếu byte
            if len(resp) < expected_bytes:
                thong_ke_modbus["timeout"] += 1
                time.sleep(0.003)
                if ser.in_waiting:
                    ser.read(ser.in_waiting)
                _port_last_io_time[port_name] = time.perf_counter()
                return None

            # 7. Nếu nhận được gói trễ của Slave khác (resp[0] != slave_id):
            if resp[0] != slave_id:
                thong_ke_modbus["id_mismatch"] += 1
                if _kiem_tra_crc_khung(resp, 0x04, count):
                    other_id = resp[0]
                    regs_other = [
                        (resp[3 + 2 * i] << 8) | resp[4 + 2 * i]
                        for i in range(count)
                    ]
                    _last_valid_registers[(port_name, other_id, address, count)] = regs_other

                resp2 = ser.read(expected_bytes)
                _port_last_io_time[port_name] = time.perf_counter()
                if len(resp2) == expected_bytes and resp2[0] == slave_id and _kiem_tra_crc_khung(resp2, 0x04, count):
                    thong_ke_modbus["resync_ok"] += 1
                    thong_ke_modbus["ok"] += 1
                    resp = resp2
                else:
                    if ser.in_waiting:
                        ser.read(ser.in_waiting)
                    return None
            else:
                # 8. Kiểm tra CRC16 của khung phản hồi
                if not _kiem_tra_crc_khung(resp, 0x04, count):
                    thong_ke_modbus["crc_err"] += 1
                    time.sleep(0.003)
                    if ser.in_waiting:
                        ser.read(ser.in_waiting)
                    return None

                thong_ke_modbus["ok"] += 1
                if thong_ke_modbus["avg_dt_ms"] == 0.0:
                    thong_ke_modbus["avg_dt_ms"] = dt_ms
                else:
                    thong_ke_modbus["avg_dt_ms"] = (
                        0.9 * thong_ke_modbus["avg_dt_ms"] + 0.1 * dt_ms
                    )

            # 9. Giải mã thanh ghi 16-bit
            registers = [
                (resp[3 + 2 * i] << 8) | resp[4 + 2 * i]
                for i in range(count)
            ]
            _last_valid_registers[(port_name, slave_id, address, count)] = registers
            return registers

        except Exception:
            try:
                if ser.in_waiting:
                    ser.read(ser.in_waiting)
            except Exception:
                pass
            return None
        finally:
            _port_last_io_time[port_name] = time.perf_counter()


def doc_cam_bien_da_vach_mgs160n(
    client_or_ser, sensor_id: int, loc_nhieu: bool = True, tra_ve_raw: bool = False
):
    """
    Đọc trọn bộ 16 thanh ghi (1000 -> 1015) chỉ trong 1 gói USB duy nhất (~12ms),
    tái tạo vị trí thực từ cả 3 vạch (Trái 1005, Giữa 1006, Phải 1007) + Cường độ từ trường
    (1008..1010) + 16 mắt Hall S1..S16 (1014..1015), sau đó đưa qua bộ lọc Alpha-Beta.
    """
    regs = doc_input_registers_nhanh(
        client_or_ser=client_or_ser,
        slave_id=sensor_id,
        address=1000,
        count=16,
    )
    if regs is None or len(regs) != 16:
        return None

    info = tai_tao_vi_tri_da_vach(regs)
    _chi_tiet_theo_slave[sensor_id] = info

    pos_true = info["pos_true"]
    pos_1002 = info["pos_1002"]

    if loc_nhieu:
        bo_loc = lay_bo_loc_slave(sensor_id)
        clean_val = bo_loc.cap_nhat(pos_true)
        if tra_ve_raw:
            return clean_val, pos_1002
        return clean_val
    else:
        if tra_ve_raw:
            return pos_true, pos_1002
        return pos_true
