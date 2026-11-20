import argparse
import collections
import csv
import math
import os
import subprocess
import sys
import threading
import time

try:
    import serial
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
except ModuleNotFoundError:
    _py310 = r"C:\Users\cheesecong\AppData\Local\Programs\Python\Python310\python.exe"
    if os.path.exists(_py310) and os.path.normcase(sys.executable) != os.path.normcase(_py310):
        _p = subprocess.Popen([_py310, *sys.argv])
        try:
            sys.exit(_p.wait())
        except KeyboardInterrupt:
            sys.exit(_p.wait())
    raise

import modbus_rs485_AGV
import magnetic_sensor_AGV
import magnetic_sensor_2_AGV
import RFID_AGV
import DRIVER_AUTO_AGV

# =====================================================================
# 1. CẤU HÌNH CỔNG GIAO TIẾP RS485
# =====================================================================
COM_SENSORS = "COM7"   # Cổng RS485 cho 2 Cảm biến từ (ID 20, ID 87) + 1 RFID (ID 15)
COM_DRIVER = "COM8"    # Cổng RS485 riêng cho Driver động cơ (ID 10)
BAUDRATE = 115200

SENSOR_TRUOC_ID = 20
SENSOR_SAU_ID = 87

# =====================================================================
# 2. CẤU HÌNH VẬN TỐC GIỚI HẠN 10 CM/S & CHẾ ĐỘ ĐỌC THẺ RFID (3 THẺ)
# =====================================================================
VAN_TOC_TOI_DA = 25.0            # Giới hạn vận tốc tối đa của xe: 15.0 cm/s (~477 RPM)
VAN_TOC_DI_THANG = 25.0          # Vận tốc dài khi đi thẳng (cm/s)
VAN_TOC_VAO_CUA = 10.0            # Vận tốc dài khi lệch lớn / vào cua (cm/s) giúp thu hồi tâm êm
GIAM_TOC_TUDONG_KHI_CUA = True   # Tự động giảm tốc mượt về 7.0 cm/s khi |error| lớn (> 18mm)
THOI_GIAN_KHOI_HANH_EM = 1.0     # Tăng tốc êm từ  5cm/s -> 10cm/s trong 1.0s đầu

SO_THE_MUC_TIEU = 3              # Gặp đủ 3 thẻ RFID thì dừng hẳn xe
THOI_GIAN_DUNG_MOI_THE = 10.0    # Khi gặp thẻ (thẻ 1, thẻ 2) thì dừng xe 10.0 giây rồi chạy lại
THOI_GIAN_ROI_THE_TOI_THIEU = 1.5  # Thời gian chạy thoát khỏi thẻ vừa dừng trước khi nhận lại cùng ID (giây)

QUANG_DUONG_MUC_TIEU_CM = 0.0    # 0.0 = Không giới hạn quãng đường (chạy đến khi gặp đủ 3 thẻ RFID)
THOI_GIAN_GIAM_TOC_DUNG = 3.0    # Thời gian giảm tốc từ từ nếu dùng giới hạn quãng đường (giây)

# =====================================================================
# 3. BỘ THÔNG SỐ PID TỐI ƯU CHO VẬN TỐC 10 CM/S (ĐIỂM VÀNG TỪ CÁC LƯỢT CHẠY)
# =====================================================================
KP = 0.0035            # Hệ số Tỷ lệ tối ưu cho 10cm/s (rad/s trên mỗi mm lệch)
KI = 0.00020           # Hệ số Tích phân tối ưu cho 10cm/s (khử sai số xác lập, chống windup)
KD = 0.00135           # Hệ số Vi phân tối ưu cho 10cm/s (dập dao động / phanh góc mượt)

W_MAX = 0.24           # Giới hạn tốc độ góc tối đa ở V=10cm/s (rad/s)
I_MAX = 0.035          # Giới hạn chống bão hòa tích phân Anti-Windup (rad/s)
DEADBAND_MM = 1.5      # Vùng chết quanh tâm (±1.5 mm)
ALPHA_LOC_D = 0.65     # Hệ số lọc thông thấp khâu D

# Trọng số ưu tiên mũi xe (ID 20) dẫn hướng chính khi tiến
TRONG_SO_TRUOC = 0.90
TRONG_SO_SAU = 0.10

DAO_CHIEU_LAI = False

LINE_LOSS_HOLD_SEC = 1.50
DUNG_XE_KHI_MAT_LINE_LAU = False
THOI_GIAN_DUNG_MAT_LINE = 4.0

TARGET_LOOP_HZ = 65.0
RFID_POLL_INTERVAL = 0.10  # Đọc thẻ RFID tốc độ 10Hz (~100ms/lần) để bắt nhạy 100% thẻ khi xe chạy
HISTORY_LEN = 350          # Lưu ~10 giây dữ liệu trên đồ thị thời gian thực
HALL_POSITIONS_MM = [-75.0 + 10.0 * i for i in range(16)]

LOG_TXT_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "pid_log.txt")
LOG_CSV_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "pid_log.csv")


class BoDieuKhienPIDBamLine:
    """Bộ điều khiển PID chuyên dụng cho AGV bám vạch từ Hinson CNS-MGS-160N."""

    def __init__(
        self,
        kp: float = KP,
        ki: float = KI,
        kd: float = KD,
        w_max: float = W_MAX,
        i_max: float = I_MAX,
        deadband_mm: float = DEADBAND_MM,
        alpha_d: float = ALPHA_LOC_D,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.w_max = w_max
        self.i_max = i_max
        self.deadband_mm = deadband_mm
        self.alpha_d = alpha_d

        self.integral = 0.0
        self.prev_error = None
        self.prev_mode = None
        self.d_filtered = 0.0
        self.last_t = None

        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0
        self.w_out = 0.0
        self.de_dt = 0.0

    def cap_nhat(
        self,
        error_mm: float,
        now: float = None,
        on_line: bool = True,
        sensor_mode: str = "2_CAM_BIEN",
    ) -> float:
        if now is None:
            now = time.perf_counter()

        if self.last_t is None:
            dt = 0.018
        else:
            dt = max(0.003, min(0.10, now - self.last_t))
        self.last_t = now

        if abs(error_mm) <= self.deadband_mm:
            e_eff = 0.0
        else:
            e_eff = (
                error_mm - math.copysign(self.deadband_mm, error_mm)
                if self.deadband_mm > 0
                else error_mm
            )

        # 1. Khâu Tỷ lệ (P) thích nghi hai vùng (Dual-Zone Gain Scheduling):
        if abs(e_eff) <= 10.0:
            kp_eff = self.kp * (0.80 + 0.20 * (abs(e_eff) / 10.0))
        else:
            kp_eff = self.kp
        self.p_term = kp_eff * e_eff

        # 2. Khâu Vi phân (D) có lọc thông thấp & chống gai đạo hàm khi chuyển mode cảm biến
        if self.prev_error is None or (self.prev_mode is not None and sensor_mode != self.prev_mode):
            raw_de_dt = 0.0
        else:
            raw_de_dt = (error_mm - self.prev_error) / dt
            raw_de_dt = max(-140.0, min(140.0, raw_de_dt))

        self.prev_error = error_mm
        self.prev_mode = sensor_mode

        self.d_filtered = (self.alpha_d * self.d_filtered) + (
            (1.0 - self.alpha_d) * raw_de_dt
        )
        self.de_dt = self.d_filtered
        self.d_term = self.kd * self.d_filtered if on_line else 0.0

        # 3. Khâu Tích phân (I) có chống bão hòa (Anti-Windup) & xả tức thời khi đổi dấu qua tâm
        if not on_line or e_eff == 0.0:
            self.integral *= 0.80
        else:
            if (error_mm * self.integral) < 0.0:
                self.integral = 0.0
            self.integral += e_eff * dt

        if self.ki > 0.0:
            max_int = self.i_max / self.ki
            self.integral = max(-max_int, min(max_int, self.integral))
            self.i_term = self.ki * self.integral
        else:
            self.i_term = 0.0

        # 4. Tổng hợp ngõ ra tốc độ góc W_pid (rad/s)
        w_raw = self.p_term + self.i_term + self.d_term
        self.w_out = max(-self.w_max, min(self.w_max, w_raw))
        return self.w_out


_last_valid_tr_mm = 0.0
_last_valid_tr_time = 0.0


def tinh_sai_so_hop_luc(
    e_truoc,
    e_sau,
    mask_tr: int,
    mask_sa: int,
    w_tr: float,
    w_sa: float,
    last_active_error: float,
    last_active_time: float,
    now: float,
):
    global _last_valid_tr_mm, _last_valid_tr_time

    tr_has_line = (e_truoc is not None) and (mask_tr > 0 or abs(e_truoc) >= 1.0)
    sa_has_line = (e_sau is not None) and (mask_sa > 0 or abs(e_sau) >= 1.0)

    if tr_has_line:
        _last_valid_tr_mm = float(e_truoc)
        _last_valid_tr_time = now

    if tr_has_line and sa_has_line:
        e_tr_f = float(e_truoc)
        e_sa_f = float(e_sau)
        if (e_tr_f * e_sa_f) < 0.0 and abs(e_sa_f) > 20.0:
            e_sa_clamped = math.copysign(min(abs(e_sa_f), 10.0), e_sa_f)
            err = (w_tr * e_tr_f) + (w_sa * e_sa_clamped)
        else:
            err = (w_tr * e_tr_f) + (w_sa * e_sa_f)
        return round(err, 2), True, "2_CAM_BIEN", err, now
    elif tr_has_line:
        err = float(e_truoc)
        return round(err, 2), True, "CHI_TRUOC", err, now
    elif sa_has_line:
        e_sa_f = float(e_sau)
        if (
            abs(_last_valid_tr_mm) >= 38.0
            and (now - _last_valid_tr_time) <= 1.20
            and (_last_valid_tr_mm * e_sa_f) < 0.0
        ):
            return round(_last_valid_tr_mm, 2), False, "GIU_HUONG_MUI_XE", _last_valid_tr_mm, last_active_time
        return round(e_sa_f, 2), True, "CHI_SAU", e_sa_f, now
    else:
        dt_lost = now - last_active_time
        if abs(last_active_error) >= 28.0 and dt_lost <= LINE_LOSS_HOLD_SEC:
            return round(last_active_error, 2), False, "GIU_HUONG_TIM_LINE", last_active_error, last_active_time
        return 0.0, False, "MAT_LINE", last_active_error, last_active_time


class BoVanHanhAGVPID:
    """
    Quản lý vòng lặp đọc Cảm biến từ + RFID + tính PID + điều khiển Driver ở luồng nền tốc độ cao,
    đồng thời cung cấp dữ liệu thread-safe cho giao diện Matplotlib thời gian thực.
    """

    def __init__(
        self,
        ser_sensors,
        ser_driver,
        bo_driver,
        kp,
        ki,
        kd,
        v_thang,
        v_cua,
        quang_duong_muc_tieu,
        thoi_gian_giam_toc,
        w_max_val,
        deadband_val,
        dao_chieu,
        so_the_muc_tieu=SO_THE_MUC_TIEU,
        thoi_gian_dung_the=THOI_GIAN_DUNG_MOI_THE,
        dry_run=False,
        com_driver=COM_DRIVER,
    ):
        self.ser_sensors = ser_sensors
        self.ser_driver = ser_driver
        self.bo_driver = bo_driver
        self.dry_run = dry_run
        self.com_driver = com_driver

        self.kp_val = kp
        self.ki_val = ki
        self.kd_val = kd
        self.v_thang = v_thang
        self.v_cua = v_cua
        self.quang_duong_muc_tieu = quang_duong_muc_tieu
        self.thoi_gian_giam_toc = thoi_gian_giam_toc
        self.w_max_val = w_max_val
        self.deadband_val = deadband_val
        self.dao_chieu = dao_chieu
        self.so_the_muc_tieu = max(1, int(so_the_muc_tieu))
        self.thoi_gian_dung_the = max(0.5, float(thoi_gian_dung_the))

        self.pid = BoDieuKhienPIDBamLine(
            kp=kp,
            ki=ki,
            kd=kd,
            w_max=w_max_val,
            i_max=I_MAX,
            deadband_mm=deadband_val,
            alpha_d=ALPHA_LOC_D,
        )

        self.lock = threading.Lock()
        self.running = False
        self.thread = None
        self.da_hoan_tat_5m = False
        self.da_hoan_tat_3_the = False

        # Bộ đệm lịch sử để vẽ đồ thị Matplotlib thời gian thực
        self.t_buf = collections.deque(maxlen=HISTORY_LEN)
        self.e_tr_buf = collections.deque(maxlen=HISTORY_LEN)
        self.e_sa_buf = collections.deque(maxlen=HISTORY_LEN)
        self.e_pid_buf = collections.deque(maxlen=HISTORY_LEN)
        self.rpm_2006_buf = collections.deque(maxlen=HISTORY_LEN)
        self.rpm_2007_buf = collections.deque(maxlen=HISTORY_LEN)
        self.p_buf = collections.deque(maxlen=HISTORY_LEN)
        self.i_buf = collections.deque(maxlen=HISTORY_LEN)
        self.d_buf = collections.deque(maxlen=HISTORY_LEN)
        self.w_buf = collections.deque(maxlen=HISTORY_LEN)

        # Trạng thái tức thời hiển thị lên GUI
        self.cur_hz = 0.0
        self.cur_dist_cm = 0.0
        self.cur_v_cm_s = 0.0
        self.cur_e_tr = 0.0
        self.cur_e_sa = 0.0
        self.cur_e_pid = 0.0
        self.cur_de_dt = 0.0
        self.cur_p = 0.0
        self.cur_i = 0.0
        self.cur_d = 0.0
        self.cur_w = 0.0
        self.cur_rpm_2006 = 0
        self.cur_rpm_2007 = 0
        self.cur_trang_thai = "ĐANG KHỞI ĐỘNG..."
        self.cur_mode = "2_CAM_BIEN"
        self.cur_tag = "CHƯA GẶP THẺ"
        self.so_the_da_gap = 0
        self.danh_sach_the_da_gap = []  # List[(stt, tag_id, rel_t, dist_cm)]
        self.dang_dung_tai_the = False
        self.thoi_gian_con_lai_dung_the = 0.0
        self.driver_connected = (self.bo_driver is not None) or self.dry_run

        # Thống kê phiên chạy
        self.t_start = time.perf_counter()
        self.tong_mau_pid = 0
        self.tong_abs_e = 0.0
        self.tong_sq_e = 0.0
        self.max_abs_e = 0.0
        self.so_lan_doi_dau = 0
        self.mau_bam_tam = 0
        self.mau_lech_vua = 0
        self.mau_lech_nang = 0
        self.mau_mat_line = 0
        self.max_overshoot_after_cross = 0.0

        self.f_txt = open(LOG_TXT_PATH, mode="w", encoding="utf-8", buffering=1)
        self.f_csv = open(LOG_CSV_PATH, mode="w", encoding="utf-8", newline="", buffering=1)
        self.csv_writer = csv.writer(self.f_csv)
        self.csv_writer.writerow([
            "time_s",
            "hz",
            "dist_cm",
            "e_truoc_mm",
            "raw_truoc_mm",
            "mask_truoc",
            "e_sau_mm",
            "raw_sau_mm",
            "mask_sau",
            "error_pid_mm",
            "de_dt_mm_s",
            "P_rad_s",
            "I_rad_s",
            "D_rad_s",
            "W_pid_rad_s",
            "v_cm_s",
            "rpm_2006",
            "rpm_2007",
            "mode",
            "rfid_tag",
            "tag_count",
        ])

        mo_ta_quang_duong = (
            f"Giới hạn S={quang_duong_muc_tieu:.0f}cm"
            if quang_duong_muc_tieu > 0
            else "Chạy liên tục theo thẻ RFID"
        )
        header_info = (
            "=" * 118 + "\n"
            f"PHIÊN CHẠY AGV BÁM LINE PID + ĐỌC THẺ RFID (DỪNG {self.thoi_gian_dung_the:.0f}S/THẺ - ĐỦ {self.so_the_muc_tieu} THẺ DỪNG HẲN) - {time.strftime('%Y-%m-%d %H:%M:%S')}\n"
            f"Thông số PID : KP={kp:.5f} | KI={ki:.5f} | KD={kd:.5f} | W_MAX={w_max_val:.2f} rad/s | Deadband=±{deadband_val:.1f} mm\n"
            f"Chế độ RFID  : Gặp thẻ hiện ID -> Dừng {self.thoi_gian_dung_the:.1f}s rồi chạy tiếp | Gặp đủ {self.so_the_muc_tieu} thẻ -> DỪNG HẲN ({mo_ta_quang_duong})\n"
            f"File Log     : {LOG_TXT_PATH}\n"
            f"File CSV     : {LOG_CSV_PATH}\n"
            + "=" * 118
        )
        print(header_info)
        self.f_txt.write(header_info + "\n")

    def bat_dau(self):
        self.running = True
        self.t_start = time.perf_counter()
        self.thread = threading.Thread(target=self._vong_lap_chinh, daemon=True)
        self.thread.start()

    def dung_va_luu_log(self):
        self.running = False
        if self.thread is not None and self.thread.is_alive():
            self.thread.join(timeout=1.2)

        if self.bo_driver is not None:
            self.bo_driver.dung_khan_cap()
        elif self.ser_driver is not None:
            DRIVER_AUTO_AGV.dung_xe(self.ser_driver)

        total_Duration = max(0.001, time.perf_counter() - self.t_start)
        mae = (self.tong_abs_e / self.tong_mau_pid) if self.tong_mau_pid > 0 else 0.0
        rmse = math.sqrt(self.tong_sq_e / self.tong_mau_pid) if self.tong_mau_pid > 0 else 0.0
        pct_tam = (self.mau_bam_tam * 100.0 / self.tong_mau_pid) if self.tong_mau_pid > 0 else 0.0
        pct_vua = (self.mau_lech_vua * 100.0 / self.tong_mau_pid) if self.tong_mau_pid > 0 else 0.0
        pct_nang = (self.mau_lech_nang * 100.0 / self.tong_mau_pid) if self.tong_mau_pid > 0 else 0.0
        osc_per_min = (self.so_lan_doi_dau * 60.0 / total_Duration) if total_Duration > 0 else 0.0

        goi_y = []
        if self.tong_mau_pid < 10:
            goi_y.append("Chưa có đủ dữ liệu cảm biến trên vạch từ (hãy đặt xe lên line từ để thu thập log).")
        else:
            if self.so_lan_doi_dau >= 6 and self.max_overshoot_after_cross >= 18.0:
                goi_y.append(
                    f"Xe có hiện tượng dao động / vọt lố qua tâm ({self.so_lan_doi_dau} lần đổi dấu, biên độ lố {self.max_overshoot_after_cross:.1f}mm) -> Nên GIẢM KP xuống ~{self.kp_val*0.8:.4f} hoặc TĂNG KD lên ~{self.kd_val*1.4:.4f}."
                )
            if pct_nang > 25.0 and self.so_lan_doi_dau <= 2:
                goi_y.append(
                    f"Xe phản ứng hơi yếu khi lệch lớn (tỷ lệ |e|>30mm chiếm {pct_nang:.1f}%, ít đổi dấu) -> Nên TĂNG KP lên ~{self.kp_val*1.25:.4f}."
                )
            if mae <= 8.0 and pct_tam >= 75.0:
                goi_y.append("Độ bám tâm rất tốt (MAE <= 8mm, trên 75% thời gian nằm trong vùng ±10mm)!")
            if not goi_y:
                goi_y.append("Hệ thống phản ứng cân bằng. Hãy gửi/đọc pid_log.txt để tinh chỉnh sâu hơn theo từng khúc cua.")

        if self.danh_sach_the_da_gap:
            chuoi_the_log = " | ".join(
                f"Thẻ #{stt}: ID={tid} (t={t_gap:.1f}s, S={s_gap:.1f}cm)"
                for stt, tid, t_gap, s_gap in self.danh_sach_the_da_gap
            )
        else:
            chuoi_the_log = "Chưa gặp thẻ RFID nào"

        summary_text = (
            "\n" + "=" * 118 + "\n"
            "BẢNG TỔNG KẾT HÀNH TRÌNH PID + ĐỌC THẺ RFID (TỰ ĐỘNG LƯU VÀO pid_log.txt)\n"
            + "-" * 118 + "\n"
            f"  - Thời gian & Quãng đường: {total_Duration:.2f} giây | Tổng quãng đường: {self.cur_dist_cm:.1f} cm\n"
            f"  - Kết quả quét thẻ RFID  : Đã gặp {self.so_the_da_gap}/{self.so_the_muc_tieu} thẻ -> {chuoi_the_log}\n"
            f"  - Mẫu thu thập           : Trên line = {self.tong_mau_pid} mẫu | Ngoài line = {self.mau_mat_line} mẫu\n"
            f"  - Thông số vừa chạy      : KP = {self.kp_val:.5f} | KI = {self.ki_val:.5f} | KD = {self.kd_val:.5f} | V_max = {self.v_thang:.1f} cm/s\n"
            f"  - Sai số trung bình (MAE): {mae:.2f} mm | Sai số hiệu dụng (RMSE): {rmse:.2f} mm | Lệch cực đại (Max |e|): {self.max_abs_e:.1f} mm\n"
            f"  - Phân bố độ lệch        : Bám tâm (<=10mm): {pct_tam:.1f}% | Lệch vừa (10..30mm): {pct_vua:.1f}% | Lệch nặng (>30mm): {pct_nang:.1f}%\n"
            f"  - Dao động qua tâm 0mm   : {self.so_lan_doi_dau} lần đổi dấu (~{osc_per_min:.1f} lần/phút) | Vọt lố cực đại sau qua tâm: {self.max_overshoot_after_cross:.1f} mm\n"
            f"  - Giao tiếp Modbus COM7  : OK = {modbus_rs485_AGV.thong_ke_modbus['ok']} | Timeout = {modbus_rs485_AGV.thong_ke_modbus['timeout']} | CRC Err = {modbus_rs485_AGV.thong_ke_modbus['crc_err']}\n"
            f"  - Gợi ý Tune PID         : {' '.join(goi_y)}\n"
            + "=" * 118 + "\n"
        )
        print(summary_text)
        try:
            self.f_txt.write(summary_text)
            self.f_txt.close()
            self.f_csv.close()
        except Exception:
            pass

        if self.ser_sensors is not None and self.ser_sensors.is_open:
            self.ser_sensors.close()
        if self.ser_driver is not None and self.ser_driver.is_open:
            self.ser_driver.close()
        print(f"Đã đóng cổng COM và lưu toàn bộ log vào: {LOG_TXT_PATH}")

    def _vong_lap_chinh(self):
        van_toc_co_so = self.v_thang
        the_hien_tai = "CHƯA GẶP THẺ"

        last_loop_t = self.t_start
        last_rfid_time = 0.0
        last_print_time = 0.0
        last_log_time = 0.0
        last_driver_retry_t = 0.0
        chu_ky_muc_tieu = 1.0 / TARGET_LOOP_HZ
        window_timestamps = collections.deque(maxlen=60)

        fail_truoc = 0
        fail_sau = 0
        fail_rfid = 0
        next_probe_truoc = 0.0
        next_probe_sau = 0.0
        next_probe_rfid = 0.0

        e_truoc_val = None
        raw_truoc_val = 0.0
        e_sau_val = None
        raw_sau_val = 0.0
        last_active_error = 0.0
        last_active_time = self.t_start

        quang_duong_da_di_cm = 0.0
        da_bat_dau_giam_toc = False
        thoi_diem_bat_dau_giam_toc = None
        van_toc_luc_bat_dau_giam_toc = self.v_thang

        # Biến quản lý trạng thái đọc thẻ RFID, dừng 10s và dừng hẳn khi đủ 3 thẻ
        so_the_da_gap = 0
        dang_dung_tai_the = False
        thoi_diem_bat_dau_dung_the = 0.0
        thoi_diem_bat_dau_chay = self.t_start
        ma_the_vua_dung = ""
        da_roi_khoi_the = True
        dem_khong_the_lien_tiep = 0
        hoan_tat_du_3_the = False

        dau_truoc_do = 0
        cur_peak_after_cross = 0.0

        while self.running:
            t0 = time.perf_counter()
            dt_step = max(0.0, min(0.10, t0 - last_loop_t))
            last_loop_t = t0
            rel_t = t0 - self.t_start

            # 0. Nếu chưa mở được cổng Driver (COM8), tự động thử kết nối lại mỗi 1.0 giây
            if not self.dry_run and self.bo_driver is None and (t0 - last_driver_retry_t >= 1.0):
                last_driver_retry_t = t0
                ser_drv = DRIVER_AUTO_AGV.ket_noi_driver(self.com_driver, BAUDRATE, in_loi=False)
                if ser_drv is not None:
                    self.ser_driver = ser_drv
                    self.bo_driver = DRIVER_AUTO_AGV.BoDieuKhienDriverPID(
                        ser_drv, slave_id=DRIVER_AUTO_AGV.SLAVE_ID
                    )
                    self.bo_driver.bat_dau()
                    thoi_diem_bat_dau_chay = t0
                    msg_reconn = f"\n[DRIVER] >>> ĐÃ KẾT NỐI ĐƯỢC DRIVER ({self.com_driver})! BẮT ĐẦU CHẠY XE! <<<\n"
                    print(msg_reconn)
                    self.f_txt.write(msg_reconn)

            # 1. Đọc RFID định kỳ (~10Hz) trên COM7
            if (t0 - last_rfid_time >= RFID_POLL_INTERVAL) and (
                fail_rfid < 15 or t0 >= next_probe_rfid
            ):
                last_rfid_time = t0
                ma_the_moi = RFID_AGV.doc_the_rfid(self.ser_sensors)
                if ma_the_moi is not None:
                    fail_rfid = 0
                    if not dang_dung_tai_the and not hoan_tat_du_3_the:
                        if ma_the_moi == "":
                            dem_khong_the_lien_tiep += 1
                            if dem_khong_the_lien_tiep >= 2:
                                da_roi_khoi_the = True
                        else:
                            la_the_moi = (ma_the_moi != ma_the_vua_dung) or (
                                da_roi_khoi_the
                                and (t0 - thoi_diem_bat_dau_chay) >= THOI_GIAN_ROI_THE_TOI_THIEU
                            )
                            dem_khong_the_lien_tiep = 0

                            if la_the_moi:
                                so_the_da_gap += 1
                                the_hien_tai = ma_the_moi
                                ma_the_vua_dung = ma_the_moi
                                da_roi_khoi_the = False
                                ten_mo_ta = RFID_AGV.mo_ta_the(ma_the_moi)

                                with self.lock:
                                    self.so_the_da_gap = so_the_da_gap
                                    self.cur_tag = ma_the_moi
                                    self.danh_sach_the_da_gap.append(
                                        (so_the_da_gap, ma_the_moi, rel_t, quang_duong_da_di_cm)
                                    )

                                if so_the_da_gap >= self.so_the_muc_tieu:
                                    hoan_tat_du_3_the = True
                                    dang_dung_tai_the = False
                                    thong_bao_the = (
                                        "\n" + "=" * 95 + "\n"
                                        f"[RFID] >>> PHÁT HIỆN THẺ #{so_the_da_gap}/{self.so_the_muc_tieu} | ID THẺ: {ma_the_moi} ({ten_mo_ta}) <<<\n"
                                        f"[RFID] >>> ĐÃ GẶP ĐỦ {self.so_the_muc_tieu} THẺ RFID! DỪNG HẲN XE TẠI S = {quang_duong_da_di_cm:.1f} cm (t = {rel_t:.2f}s)! <<<\n"
                                        + "=" * 95
                                    )
                                    print(thong_bao_the)
                                    self.f_txt.write(thong_bao_the + "\n")
                                else:
                                    dang_dung_tai_the = True
                                    thoi_diem_bat_dau_dung_the = t0
                                    thong_bao_the = (
                                        "\n" + "=" * 95 + "\n"
                                        f"[RFID] >>> PHÁT HIỆN THẺ #{so_the_da_gap}/{self.so_the_muc_tieu} | ID THẺ: {ma_the_moi} ({ten_mo_ta}) <<<\n"
                                        f"[RFID] >>> Dừng xe {self.thoi_gian_dung_the:.0f} giây tại thẻ {ma_the_moi}, sau đó sẽ tự động chạy tiếp PID...\n"
                                        + "=" * 95
                                    )
                                    print(thong_bao_the)
                                    self.f_txt.write(thong_bao_the + "\n")

                                if self.bo_driver is not None:
                                    self.bo_driver.cap_nhat_rpm(0, 0)
                                elif self.ser_driver is not None:
                                    DRIVER_AUTO_AGV.dung_xe(self.ser_driver)
                else:
                    fail_rfid += 1
                    if fail_rfid >= 15:
                        next_probe_rfid = t0 + 1.5

            # 2. Đọc Cảm biến từ Trước (ID 20)
            if fail_truoc < 25 or t0 >= next_probe_truoc:
                res_tr = magnetic_sensor_AGV.doc_cam_bien_truoc(
                    self.ser_sensors, SENSOR_TRUOC_ID, loc_nhieu=True, tra_ve_raw=True
                )
                if res_tr is not None:
                    e_truoc_val, raw_truoc_val = res_tr
                    fail_truoc = 0
                else:
                    fail_truoc += 1
                    if fail_truoc >= 25:
                        next_probe_truoc = t0 + 1.0

            # 3. Đọc Cảm biến từ Sau (ID 87)
            now_mid = time.perf_counter()
            if fail_sau < 25 or now_mid >= next_probe_sau:
                res_sa = magnetic_sensor_2_AGV.doc_cam_bien_sau(
                    self.ser_sensors, SENSOR_SAU_ID, loc_nhieu=True, tra_ve_raw=True
                )
                if res_sa is not None:
                    e_sau_val, raw_sau_val = res_sa
                    fail_sau = 0
                else:
                    fail_sau += 1
                    if fail_sau >= 25:
                        next_probe_sau = now_mid + 1.0

            info_tr = modbus_rs485_AGV.lay_chi_tiet_slave(SENSOR_TRUOC_ID)
            info_sa = modbus_rs485_AGV.lay_chi_tiet_slave(SENSOR_SAU_ID)
            mask_tr = info_tr["mask"] if info_tr else 0
            mask_sa = info_sa["mask"] if info_sa else 0

            valid_e_tr = e_truoc_val if (e_truoc_val is not None and fail_truoc < 5) else None
            valid_e_sa = e_sau_val if (e_sau_val is not None and fail_sau < 5) else None

            now_pid = time.perf_counter()
            error_pid, on_line, che_do_cam_bien, last_active_error, last_active_time = (
                tinh_sai_so_hop_luc(
                    e_truoc=valid_e_tr,
                    e_sau=valid_e_sa,
                    mask_tr=mask_tr,
                    mask_sa=mask_sa,
                    w_tr=TRONG_SO_TRUOC,
                    w_sa=TRONG_SO_SAU,
                    last_active_error=last_active_error,
                    last_active_time=last_active_time,
                    now=now_pid,
                )
            )

            # Kiểm tra đếm ngược thời gian dừng 10s tại thẻ RFID
            thoi_gian_con_lai_dung = 0.0
            if dang_dung_tai_the:
                da_dung_duoc = now_pid - thoi_diem_bat_dau_dung_the
                if da_dung_duoc >= self.thoi_gian_dung_the:
                    dang_dung_tai_the = False
                    thoi_diem_bat_dau_chay = now_pid
                    last_active_time = now_pid
                    self.pid.integral = 0.0
                    self.pid.prev_error = error_pid
                    self.pid.last_t = now_pid
                    msg_resume = (
                        f"\n[RFID] >>> Đã dừng đủ {self.thoi_gian_dung_the:.0f}s tại Thẻ #{so_the_da_gap} "
                        f"(ID: {ma_the_vua_dung}). Xe tiếp tục chạy bám line PID! <<<"
                    )
                    print(msg_resume)
                    self.f_txt.write(msg_resume + "\n")
                else:
                    thoi_gian_con_lai_dung = max(0.0, self.thoi_gian_dung_the - da_dung_duoc)

            # 4. Tính toán PID -> W_pid (rad/s)
            if dang_dung_tai_the or hoan_tat_du_3_the:
                self.pid.integral = 0.0
                self.pid.prev_error = error_pid
                self.pid.last_t = now_pid
                self.pid.p_term = 0.0
                self.pid.i_term = 0.0
                self.pid.d_term = 0.0
                self.pid.w_out = 0.0
                w_pid = 0.0
                van_toc_thuc = 0.0
            else:
                w_pid = self.pid.cap_nhat(
                    error_pid, now=now_pid, on_line=on_line, sensor_mode=che_do_cam_bien
                )

                # 5. Tính vận tốc dài mục tiêu (giới hạn <= 10.0 cm/s) & khởi hành êm
                if DUNG_XE_KHI_MAT_LINE_LAU and not on_line and (now_pid - last_active_time) > THOI_GIAN_DUNG_MAT_LINE:
                    van_toc_thuc = 0.0
                elif GIAM_TOC_TUDONG_KHI_CUA and abs(error_pid) > 18.0:
                    ty_le_cua = min(1.0, (abs(error_pid) - 18.0) / 38.0)
                    van_toc_thuc = van_toc_co_so - ty_le_cua * max(0.0, van_toc_co_so - self.v_cua)
                else:
                    van_toc_thuc = van_toc_co_so

                # Khởi hành êm (Soft-start) trong 1.0s đầu hoặc sau khi dừng trạm 10s
                dt_tu_luc_chay = now_pid - thoi_diem_bat_dau_chay
                if dt_tu_luc_chay < THOI_GIAN_KHOI_HANH_EM and van_toc_thuc > 5.0:
                    ty_le_start = max(0.0, min(1.0, dt_tu_luc_chay / THOI_GIAN_KHOI_HANH_EM))
                    van_toc_thuc = 5.0 + (van_toc_thuc - 5.0) * ty_le_start

                van_toc_thuc = min(VAN_TOC_TOI_DA, max(0.0, van_toc_thuc))

            hoan_tat_dung = hoan_tat_du_3_the
            if (
                not hoan_tat_du_3_the
                and not dang_dung_tai_the
                and self.quang_duong_muc_tieu > 0.0
                and quang_duong_da_di_cm >= self.quang_duong_muc_tieu
            ):
                if not da_bat_dau_giam_toc:
                    da_bat_dau_giam_toc = True
                    thoi_diem_bat_dau_giam_toc = now_pid
                    van_toc_luc_bat_dau_giam_toc = max(1.0, van_toc_thuc)
                    msg_target = (
                        f"\n[>>>] ĐÃ ĐẠT ĐỦ {quang_duong_da_di_cm:.1f} cm ({self.quang_duong_muc_tieu/100.0:.1f}m) tại t={rel_t:.2f}s! "
                        f"Bắt đầu GIẢM TỐC TỪ TỪ về 0 trong {self.thoi_gian_giam_toc:.1f}s..."
                    )
                    print(msg_target)
                    self.f_txt.write(msg_target + "\n")

                dt_decel = now_pid - thoi_diem_bat_dau_giam_toc
                if dt_decel >= self.thoi_gian_giam_toc:
                    van_toc_thuc = 0.0
                    w_pid = 0.0
                    hoan_tat_dung = True
                else:
                    he_so_con_lai = 0.5 * (1.0 + math.cos(math.pi * (dt_decel / self.thoi_gian_giam_toc)))
                    van_toc_thuc = van_toc_luc_bat_dau_giam_toc * he_so_con_lai
                    w_pid = w_pid * he_so_con_lai

            # 6. Tính RPM 2 bánh (2006: Bánh Phải, 2007: Bánh Trái) và gửi xuống Driver COM8
            rpm_2006, rpm_2007 = DRIVER_AUTO_AGV.tinh_rpm_pid(
                van_toc_thuc, w_pid, dao_chieu_lai=self.dao_chieu
            )
            if self.bo_driver is not None:
                self.bo_driver.cap_nhat_rpm(rpm_2006, rpm_2007)

            driver_ok = (self.bo_driver is not None and not self.bo_driver.loi_ket_noi)
            driver_active = self.dry_run or (self.bo_driver is not None and self.bo_driver.da_khoi_dong)
            if driver_active and van_toc_thuc > 0.0 and not dang_dung_tai_the:
                quang_duong_da_di_cm += van_toc_thuc * dt_step

            # 7. Nhãn trạng thái
            if not self.dry_run and not driver_ok:
                trang_thai = f"LỖI MẤT KẾT NỐI DRIVER ({self.com_driver}) - XE ĐỨNG IM!"
            elif hoan_tat_du_3_the:
                trang_thai = f"ĐÃ DỪNG HẲN - ĐỦ {so_the_da_gap}/{self.so_the_muc_tieu} THẺ (ID CUỐI: {the_hien_tai})"
            elif dang_dung_tai_the:
                trang_thai = (
                    f"DỪNG THẺ #{so_the_da_gap}/{self.so_the_muc_tieu} [ID: {the_hien_tai}] "
                    f"(Còn {thoi_gian_con_lai_dung:.1f}s)"
                )
            elif hoan_tat_dung:
                trang_thai = f"ĐÃ DỪNG ÊM (SAU {self.quang_duong_muc_tieu/100.0:.0f}M)"
            elif da_bat_dau_giam_toc:
                trang_thai = f"GIẢM TỐC SAU {self.quang_duong_muc_tieu/100.0:.0f}M ({van_toc_thuc:.1f}cm/s)"
            elif van_toc_thuc <= 0.0:
                trang_thai = "DỪNG XE"
            elif not on_line and che_do_cam_bien == "GIU_HUONG_TIM_LINE":
                trang_thai = "TÌM LINE PHẢI" if error_pid > 0 else "TÌM LINE TRÁI"
            elif not on_line:
                trang_thai = "MẤT LINE (0mm)"
            elif abs(error_pid) <= self.deadband_val:
                trang_thai = "ĐI THẲNG (TÂM)"
            elif w_pid > 0:
                trang_thai = "LÁI PHẢI (PID)"
            else:
                trang_thai = "LÁI TRÁI (PID)"

            # 8. Thống kê PID (chỉ thống kê khi xe đang lăn bánh)
            if not dang_dung_tai_the and not hoan_tat_du_3_the:
                if on_line:
                    self.tong_mau_pid += 1
                    abs_e = abs(error_pid)
                    self.tong_abs_e += abs_e
                    self.tong_sq_e += error_pid * error_pid
                    if abs_e > self.max_abs_e:
                        self.max_abs_e = abs_e

                    if abs_e <= 10.0:
                        self.mau_bam_tam += 1
                    elif abs_e <= 30.0:
                        self.mau_lech_vua += 1
                    else:
                        self.mau_lech_nang += 1

                    if abs_e >= 3.5:
                        dau_hien_tai = 1 if error_pid > 0 else -1
                        if dau_truoc_do != 0 and dau_hien_tai != dau_truoc_do:
                            self.so_lan_doi_dau += 1
                            if cur_peak_after_cross > self.max_overshoot_after_cross:
                                self.max_overshoot_after_cross = cur_peak_after_cross
                            cur_peak_after_cross = abs_e
                        else:
                            if abs_e > cur_peak_after_cross:
                                cur_peak_after_cross = abs_e
                        dau_truoc_do = dau_hien_tai
                else:
                    self.mau_mat_line += 1

            # 9. Tính tần số vòng lặp thực tế (Hz)
            t_now = time.perf_counter()
            window_timestamps.append(t_now)
            if len(window_timestamps) >= 2:
                dt_win = window_timestamps[-1] - window_timestamps[0]
                hz = (len(window_timestamps) - 1) / dt_win if dt_win > 0 else 0.0
            else:
                hz = 0.0

            e_tr_log = e_truoc_val if e_truoc_val is not None else 0.0
            e_sa_log = e_sau_val if e_sau_val is not None else 0.0

            # Đẩy dữ liệu vào bộ đệm Matplotlib
            with self.lock:
                self.t_buf.append(rel_t)
                self.e_tr_buf.append(e_tr_log)
                self.e_sa_buf.append(e_sa_log)
                self.e_pid_buf.append(error_pid)
                self.rpm_2006_buf.append(rpm_2006)
                self.rpm_2007_buf.append(rpm_2007)
                self.p_buf.append(self.pid.p_term)
                self.i_buf.append(self.pid.i_term)
                self.d_buf.append(self.pid.d_term)
                self.w_buf.append(w_pid)

                self.cur_hz = hz
                self.cur_dist_cm = quang_duong_da_di_cm
                self.cur_v_cm_s = van_toc_thuc
                self.cur_e_tr = e_tr_log
                self.cur_e_sa = e_sa_log
                self.cur_e_pid = error_pid
                self.cur_de_dt = self.pid.de_dt
                self.cur_p = self.pid.p_term
                self.cur_i = self.pid.i_term
                self.cur_d = self.pid.d_term
                self.cur_w = w_pid
                self.cur_rpm_2006 = rpm_2006
                self.cur_rpm_2007 = rpm_2007
                self.cur_trang_thai = trang_thai
                self.cur_mode = che_do_cam_bien
                self.cur_tag = the_hien_tai
                self.so_the_da_gap = so_the_da_gap
                self.dang_dung_tai_the = dang_dung_tai_the
                self.thoi_gian_con_lai_dung_the = thoi_gian_con_lai_dung
                self.driver_connected = driver_ok or self.dry_run

            # 10. Ghi log chi tiết (mỗi 80ms)
            if (t_now - last_log_time) >= 0.080 or hoan_tat_dung:
                last_log_time = t_now
                self.csv_writer.writerow([
                    f"{rel_t:.3f}",
                    f"{hz:.1f}",
                    f"{quang_duong_da_di_cm:.2f}",
                    f"{e_tr_log:.2f}",
                    f"{raw_truoc_val:.0f}",
                    mask_tr,
                    f"{e_sa_log:.2f}",
                    f"{raw_sau_val:.0f}",
                    mask_sa,
                    f"{error_pid:.2f}",
                    f"{self.pid.de_dt:.1f}",
                    f"{self.pid.p_term:.4f}",
                    f"{self.pid.i_term:.4f}",
                    f"{self.pid.d_term:.4f}",
                    f"{w_pid:.4f}",
                    f"{van_toc_thuc:.2f}",
                    rpm_2006,
                    rpm_2007,
                    che_do_cam_bien,
                    the_hien_tai,
                    so_the_da_gap,
                ])

                log_line = (
                    f"[{time.strftime('%H:%M:%S')}.{int((t_now % 1)*1000):03d}] "
                    f"t={rel_t:>6.2f}s | S={quang_duong_da_di_cm:>6.1f}cm | Thẻ:{so_the_da_gap}/{self.so_the_muc_tieu}({the_hien_tai}) | "
                    f"Tr:{e_tr_log:>+6.1f}(M{mask_tr}) Sa:{e_sa_log:>+6.1f}(M{mask_sa}) | "
                    f"e:{error_pid:>+6.1f}mm | W:{w_pid:>+6.3f} | "
                    f"V:{van_toc_thuc:>4.2f} | 2006/2007:{rpm_2006:>3}/{rpm_2007:<3} | {trang_thai}"
                )
                self.f_txt.write(log_line + "\n")

            # 11. In ra màn hình Console mỗi 200ms
            if (t_now - last_print_time) >= 0.20 or hoan_tat_dung:
                last_print_time = t_now
                print(
                    f"[{time.strftime('%H:%M:%S')}] Thẻ:{so_the_da_gap}/{self.so_the_muc_tieu} (ID:{the_hien_tai}) | "
                    f"S:{quang_duong_da_di_cm:>5.1f}cm | V:{van_toc_thuc:>4.2f}cm/s | "
                    f"Tr:{e_tr_log:>+5.1f} Sa:{e_sa_log:>+5.1f} -> e:{error_pid:>+5.1f}mm | "
                    f"RPM:{rpm_2006:>3}/{rpm_2007:<3} | {trang_thai}"
                )

            if hoan_tat_dung:
                if self.bo_driver is not None:
                    self.bo_driver.cap_nhat_rpm(0, 0)
                if hoan_tat_du_3_the:
                    ds_id = ", ".join(
                        f"#{stt}:{tid}" for stt, tid, _, _ in self.danh_sach_the_da_gap
                    )
                    print(
                        f"\n[HOÀN TẤT] Xe đã gặp đủ {so_the_da_gap}/{self.so_the_muc_tieu} thẻ RFID ({ds_id}) và DỪNG HẲN tại S = {quang_duong_da_di_cm:.1f} cm!"
                    )
                    self.da_hoan_tat_3_the = True
                else:
                    print(
                        f"\n[HOÀN TẤT] Xe đã chạy đủ {self.quang_duong_muc_tieu/100.0:.1f}m và giảm tốc dừng êm ái tại tổng quãng đường S = {quang_duong_da_di_cm:.1f} cm!"
                    )
                    self.da_hoan_tat_5m = True
                self.running = False
                break

            elapsed = time.perf_counter() - t0
            remain = chu_ky_muc_tieu - elapsed
            if remain > 0.0005:
                time.sleep(remain)


def hien_thi_do_thi_pid(bo_agv: BoVanHanhAGVPID):
    """Giao diện Matplotlib thời gian thực giám sát trực tiếp PID, thẻ RFID, 2 cảm biến từ và tốc độ 2 bánh."""
    plt.style.use(
        "seaborn-v0_8-darkgrid"
        if "seaborn-v0_8-darkgrid" in plt.style.available
        else "default"
    )
    fig, (ax_err, ax_rpm, ax_bar) = plt.subplots(
        3, 1, figsize=(12.6, 8.2), gridspec_kw={"height_ratios": [2.2, 1.6, 1.15]}
    )
    fig.canvas.manager.set_window_title(
        f"Giám sát AGV Bám Line PID + Đọc Thẻ RFID (Dừng {bo_agv.thoi_gian_dung_the:.0f}s/thẻ | Đủ {bo_agv.so_the_muc_tieu} thẻ dừng hẳn | V_max={bo_agv.v_thang:.1f}cm/s)"
    )

    # 1. Đồ thị Sai số Cảm biến Trước, Sau & Sai số tổng hợp PID
    ax_err.set_title(
        "1. ĐỘ LỆCH VẠCH TỪ (TRƯỚC ID 20, SAU ID 87), SAI SỐ PID & TRẠNG THÁI THẺ RFID (ID 15)",
        fontsize=10.5,
        fontweight="bold",
    )
    ax_err.set_ylabel("Độ lệch (mm)")
    ax_err.set_xlim(0.0, 10.0)
    ax_err.set_ylim(-82, 82)
    ax_err.axhline(0, color="black", linestyle="-", linewidth=1.1, alpha=0.6)
    ax_err.axhspan(-10, 10, color="#d4edda", alpha=0.35, label="Vùng bám tâm tốt (±10 mm)")

    (line_tr,) = ax_err.plot([], [], color="#0275d8", linewidth=1.8, alpha=0.85, label="Trước ID 20 (mm)")
    (line_sa,) = ax_err.plot([], [], color="#f0ad4e", linewidth=1.6, linestyle="--", alpha=0.80, label="Sau ID 87 (mm)")
    (line_pid,) = ax_err.plot([], [], color="#28a745", linewidth=2.6, label="Sai số e_PID (0.9*Trước + 0.1*Sau)")
    ax_err.legend(loc="upper right", fontsize=8.5, ncol=4, framealpha=0.92)

    info_box = ax_err.text(
        0.012,
        0.95,
        "Đang khởi tạo vòng lặp PID + RFID...",
        transform=ax_err.transAxes,
        fontsize=8.8,
        fontweight="bold",
        verticalalignment="top",
        bbox=dict(boxstyle="round,pad=0.4", facecolor="#f8f9fa", edgecolor="#343a40", alpha=0.93),
    )

    rfid_box = ax_err.text(
        0.988,
        0.18,
        "THẺ RFID: Chưa gặp thẻ (0/3)",
        transform=ax_err.transAxes,
        fontsize=9.5,
        fontweight="bold",
        horizontalalignment="right",
        verticalalignment="bottom",
        bbox=dict(boxstyle="round,pad=0.45", facecolor="#e7f1ff", edgecolor="#0275d8", alpha=0.95),
    )

    # 2. Đồ thị Tốc độ 2 bánh xe (RPM 2006 Bánh Phải vs RPM 2007 Bánh Trái)
    rpm_chuan = DRIVER_AUTO_AGV.van_toc_sang_rpm(bo_agv.v_thang)
    ax_rpm.set_title(
        "2. ĐÁP ỨNG ĐỘNG CƠ 2 BÁNH TỪ BỘ ĐIỀU KHIỂN PID (BÁNH PHẢI REG 2006 vs BÁNH TRÁI REG 2007)",
        fontsize=10.2,
        fontweight="bold",
    )
    ax_rpm.set_ylabel("Tốc độ (RPM)")
    ax_rpm.set_xlim(0.0, 10.0)
    ax_rpm.set_ylim(-15, max(350, int(rpm_chuan * 1.95)))
    ax_rpm.axhline(
        rpm_chuan,
        color="gray",
        linestyle=":",
        linewidth=1.0,
        label=f"RPM chuẩn {bo_agv.v_thang:.1f} cm/s ({rpm_chuan} RPM)",
    )

    (line_rpm2006,) = ax_rpm.plot([], [], color="#d9534f", linewidth=2.2, label="Bánh Phải - Reg 2006 (RPM)")
    (line_rpm2007,) = ax_rpm.plot([], [], color="#0275d8", linewidth=2.2, label="Bánh Trái - Reg 2007 (RPM)")
    ax_rpm.legend(loc="upper right", fontsize=8.5, ncol=3, framealpha=0.92)

    pid_terms_box = ax_rpm.text(
        0.012,
        0.92,
        "",
        transform=ax_rpm.transAxes,
        fontsize=9.0,
        fontweight="bold",
        verticalalignment="top",
        bbox=dict(boxstyle="round,pad=0.35", facecolor="#fff3cd", edgecolor="#856404", alpha=0.92),
    )

    # 3. Thanh 16 mắt cảm biến Hall S1..S16 & Vị trí tức thời
    ax_bar.set_title(
        "3. THANH 16 MẮT HALL S1..S16 (-75mm .. +75mm) & TRẠNG THÁI ĐÁNH LÁI TỨC THỜI",
        fontsize=10.0,
        fontweight="bold",
    )
    ax_bar.set_xlim(-82, 82)
    ax_bar.set_ylim(-1.05, 1.05)
    ax_bar.set_yticks([])
    ax_bar.set_xticks(HALL_POSITIONS_MM)
    ax_bar.set_xticklabels([f"S{i+1}\n{int(p)}" for i, p in enumerate(HALL_POSITIONS_MM)], fontsize=7.5)
    ax_bar.set_xlabel("Tọa độ 16 mắt cảm biến Hall (mm) — Đóng cửa sổ đồ thị hoặc nhấn Ctrl+C để dừng xe an toàn")

    ax_bar.axvspan(-10, 10, color="#d4edda", alpha=0.55)
    ax_bar.axvline(0, color="black", linestyle="-", linewidth=1.2)

    ax_bar.plot(HALL_POSITIONS_MM, [0.55] * 16, "o", markersize=7.5, color="#ced4da", alpha=0.7)
    (hall_on_tr,) = ax_bar.plot([], [], "o", markersize=9.5, color="#ff3b30", label="Mắt Hall Trước (ID 20)")
    ax_bar.plot(HALL_POSITIONS_MM, [-0.55] * 16, "o", markersize=7.5, color="#ced4da", alpha=0.7)
    (hall_on_sa,) = ax_bar.plot([], [], "o", markersize=9.5, color="#ff9500", label="Mắt Hall Sau (ID 87)")

    (marker_tr,) = ax_bar.plot([0], [0.22], marker="v", markersize=13, color="#0275d8", label="Mũi xe (ID 20)")
    (marker_sa,) = ax_bar.plot([0], [-0.22], marker="^", markersize=13, color="#f0ad4e", label="Đuôi xe (ID 87)")
    (marker_pid,) = ax_bar.plot([0], [0.0], marker="o", markersize=14, color="#28a745", label="Tâm e_PID")
    ax_bar.legend(loc="upper right", fontsize=7.8, ncol=5)

    state_label = ax_bar.text(
        0.015,
        0.78,
        "",
        transform=ax_bar.transAxes,
        fontsize=10.0,
        fontweight="bold",
    )

    plt.tight_layout()

    def cap_nhat_gui(_frame):
        with bo_agv.lock:
            if not bo_agv.t_buf:
                return

            t_list = list(bo_agv.t_buf)
            tr_list = list(bo_agv.e_tr_buf)
            sa_list = list(bo_agv.e_sa_buf)
            pid_list = list(bo_agv.e_pid_buf)
            r2006_list = list(bo_agv.rpm_2006_buf)
            r2007_list = list(bo_agv.rpm_2007_buf)

            hz = bo_agv.cur_hz
            dist_cm = bo_agv.cur_dist_cm
            v_cm_s = bo_agv.cur_v_cm_s
            e_tr = bo_agv.cur_e_tr
            e_sa = bo_agv.cur_e_sa
            e_pid = bo_agv.cur_e_pid
            de_dt = bo_agv.cur_de_dt
            p_val = bo_agv.cur_p
            i_val = bo_agv.cur_i
            d_val = bo_agv.cur_d
            w_val = bo_agv.cur_w
            r2006 = bo_agv.cur_rpm_2006
            r2007 = bo_agv.cur_rpm_2007
            trang_thai = bo_agv.cur_trang_thai
            cur_tag = bo_agv.cur_tag
            so_the = bo_agv.so_the_da_gap
            ds_the = list(bo_agv.danh_sach_the_da_gap)
            dang_dung_the = bo_agv.dang_dung_tai_the
            con_lai_s = bo_agv.thoi_gian_con_lai_dung_the
            drv_ok = bo_agv.driver_connected

        t_latest = t_list[-1]
        x_min = max(0.0, t_latest - 10.0)
        x_max = max(10.0, t_latest + 0.25)
        ax_err.set_xlim(x_min, x_max)
        ax_rpm.set_xlim(x_min, x_max)

        line_tr.set_data(t_list, tr_list)
        line_sa.set_data(t_list, sa_list)
        line_pid.set_data(t_list, pid_list)

        line_rpm2006.set_data(t_list, r2006_list)
        line_rpm2007.set_data(t_list, r2007_list)

        info_tr = modbus_rs485_AGV.lay_chi_tiet_slave(SENSOR_TRUOC_ID)
        if info_tr and info_tr.get("hall_bits"):
            xs_tr = [HALL_POSITIONS_MM[i] for i, b in enumerate(info_tr["hall_bits"]) if b]
            hall_on_tr.set_data(xs_tr, [0.55] * len(xs_tr))
        else:
            hall_on_tr.set_data([], [])

        info_sa = modbus_rs485_AGV.lay_chi_tiet_slave(SENSOR_SAU_ID)
        if info_sa and info_sa.get("hall_bits"):
            xs_sa = [HALL_POSITIONS_MM[i] for i, b in enumerate(info_sa["hall_bits"]) if b]
            hall_on_sa.set_data(xs_sa, [-0.55] * len(xs_sa))
        else:
            hall_on_sa.set_data([], [])

        marker_tr.set_data([e_tr], [0.22])
        marker_sa.set_data([e_sa], [-0.22])
        marker_pid.set_data([e_pid], [0.0])

        info_box.set_text(
            f"Thời gian: {t_latest:>5.1f}s | Quãng đường S: {dist_cm:>5.1f} cm | Số thẻ RFID: {so_the}/{bo_agv.so_the_muc_tieu} | Vận tốc: {v_cm_s:.2f} cm/s ({hz:.1f} Hz)\n"
            f"Trước (ID 20) : {e_tr:>+6.1f} mm | Sau (ID 87): {e_sa:>+6.1f} mm | Sai số e_PID: {e_pid:>+6.1f} mm (de/dt: {de_dt:>+5.0f} mm/s)"
        )

        if ds_the:
            lich_su_the = " -> ".join(f"#{stt}:{tid}" for stt, tid, _, _ in ds_the)
        else:
            lich_su_the = "Chưa quét được thẻ nào"

        if not drv_ok:
            rfid_box.set_text(
                f"[!] LỖI KHÔNG MỞ ĐƯỢC DRIVER ({bo_agv.com_driver}) - ĐANG THỬ KẾT NỐI LẠI...\n"
                f"Kiểm tra cáp USB-RS485 {bo_agv.com_driver} hoặc đóng cửa sổ Python cũ đang chạy ngầm!"
            )
        elif so_the >= bo_agv.so_the_muc_tieu:
            rfid_box.set_text(
                f"ĐÃ GẶP ĐỦ {so_the}/{bo_agv.so_the_muc_tieu} THẺ - DỪNG HẲN XE!\n"
                f"ID thẻ vừa gặp: {cur_tag} | Lịch sử: {lich_su_the}"
            )
        elif dang_dung_the:
            rfid_box.set_text(
                f"ĐANG DỪNG TẠI THẺ #{so_the}/{bo_agv.so_the_muc_tieu} - ID: {cur_tag} (Còn {con_lai_s:.1f}s)\n"
                f"Lịch sử thẻ: {lich_su_the}"
            )
        else:
            rfid_box.set_text(
                f"THẺ RFID GẦN NHẤT ({so_the}/{bo_agv.so_the_muc_tieu}): {cur_tag}\n"
                f"Lịch sử thẻ: {lich_su_the}"
            )

        pid_terms_box.set_text(
            f"Chi tiết PID  : P = {p_val:>+6.3f} | I = {i_val:>+6.3f} | D = {d_val:>+6.3f}  ==>  W_pid = {w_val:>+6.3f} rad/s\n"
            f"Tốc độ 2 bánh : Bánh Phải (2006) = {r2006:>3} RPM | Bánh Trái (2007) = {r2007:>3} RPM | Chênh lệch dRPM = {r2007 - r2006:>+4} RPM"
        )

        state_label.set_text(f"TRẠNG THÁI XE: {trang_thai}")
        if not drv_ok:
            state_label.set_color("#d9534f")
        elif "DỪNG" in trang_thai or "GIẢM TỐC" in trang_thai:
            state_label.set_color("#0275d8")
        elif abs(e_pid) <= 10.0:
            state_label.set_color("#28a745")
        else:
            state_label.set_color("#d9534f")

    _ani = FuncAnimation(fig, cap_nhat_gui, interval=60, blit=False, cache_frame_data=False)
    plt.show()


def main():
    if os.name == "nt":
        for stream in (sys.stdout, sys.stderr):
            if hasattr(stream, "reconfigure"):
                try:
                    stream.reconfigure(encoding="utf-8")
                except Exception:
                    pass

    parser = argparse.ArgumentParser(description="Điều khiển AGV bám line bằng PID + Đọc thẻ RFID (Dừng 10s/thẻ, đủ 3 thẻ dừng hẳn)")
    parser.add_argument("--com-sensors", type=str, default=COM_SENSORS, help=f"Cổng COM cảm biến từ + RFID (mặc định: {COM_SENSORS})")
    parser.add_argument("--com-driver", type=str, default=COM_DRIVER, help=f"Cổng COM Driver động cơ (mặc định: {COM_DRIVER})")
    parser.add_argument("--kp", type=float, default=KP, help=f"Hệ số KP (mặc định: {KP})")
    parser.add_argument("--ki", type=float, default=KI, help=f"Hệ số KI (mặc định: {KI})")
    parser.add_argument("--kd", type=float, default=KD, help=f"Hệ số KD (mặc định: {KD})")
    parser.add_argument("--v", type=float, default=VAN_TOC_DI_THANG, help=f"Vận tốc đi thẳng cm/s, tối đa {VAN_TOC_TOI_DA} (mặc định: {VAN_TOC_DI_THANG})")
    parser.add_argument("--v-cua", type=float, default=VAN_TOC_VAO_CUA, help=f"Vận tốc vào cua cm/s (mặc định: {VAN_TOC_VAO_CUA})")
    parser.add_argument("--max-tags", type=int, default=SO_THE_MUC_TIEU, help=f"Số thẻ RFID cần gặp để dừng hẳn (mặc định: {SO_THE_MUC_TIEU})")
    parser.add_argument("--tag-stop", type=float, default=THOI_GIAN_DUNG_MOI_THE, help=f"Thời gian dừng xe tại mỗi thẻ (giây, mặc định: {THOI_GIAN_DUNG_MOI_THE}s)")
    parser.add_argument("--dist", type=float, default=QUANG_DUONG_MUC_TIEU_CM, help="Giới hạn quãng đường tối đa (cm, mặc định 0 = chỉ dừng theo 3 thẻ RFID)")
    parser.add_argument("--ramp-down", type=float, default=THOI_GIAN_GIAM_TOC_DUNG, help=f"Thời gian giảm tốc từ từ nếu dùng --dist (s, mặc định: {THOI_GIAN_GIAM_TOC_DUNG}s)")
    parser.add_argument("--w-max", type=float, default=W_MAX, help=f"Giới hạn tốc độ góc W_max rad/s (mặc định: {W_MAX})")
    parser.add_argument("--deadband", type=float, default=DEADBAND_MM, help=f"Vùng chết tâm mm (mặc định: {DEADBAND_MM})")
    parser.add_argument("--invert-steer", action="store_true", help="Đảo chiều đánh lái Trái/Phải")
    parser.add_argument("--dry-run", action="store_true", help="Chỉ đọc cảm biến + tính PID + ghi log, không quay động cơ")
    parser.add_argument("--no-gui", action="store_true", help="Chạy không mở cửa sổ đồ thị Matplotlib")
    args = parser.parse_args()

    com_sensors = args.com_sensors.strip().upper()
    com_driver = args.com_driver.strip().upper()
    kp_val = args.kp
    ki_val = args.ki
    kd_val = args.kd
    v_thang = min(VAN_TOC_TOI_DA, max(0.5, args.v))
    v_cua = min(v_thang, max(0.5, args.v_cua))
    quang_duong_muc_tieu = max(0.0, args.dist)
    thoi_gian_giam_toc = max(0.5, args.ramp_down)
    w_max_val = args.w_max
    deadband_val = args.deadband
    dao_chieu = DAO_CHIEU_LAI or args.invert_steer
    so_the_muc_tieu = max(1, args.max_tags)
    thoi_gian_dung_the = max(0.5, args.tag_stop)

    print(f"[*] Các cổng COM đang cắm trên máy: {DRIVER_AUTO_AGV.liet_ke_cong_com()}")

    # 1. Mở cổng RS485 cho cụm cảm biến (COM7)
    ser_sensors = modbus_rs485_AGV.ket_noi_rs485_nhanh(
        com_sensors, BAUDRATE, timeout=0.035
    )
    if ser_sensors is None:
        print(f"[!] Lỗi: Không thể mở cổng Cảm biến ({com_sensors}). Hãy tắt cửa sổ đang giữ {com_sensors} (nếu có).")
        return

    # 2. Mở cổng RS485 riêng cho Driver (COM8)
    ser_driver = None
    bo_driver = None
    if not args.dry_run:
        ser_driver = DRIVER_AUTO_AGV.ket_noi_driver(com_driver, BAUDRATE, in_loi=True)
        if ser_driver is None:
            print(
                f"[!] CẢNH BÁO: Chưa mở được cổng Driver ({com_driver})! "
                f"Hệ thống sẽ tự động thử kết nối lại {com_driver} mỗi 1 giây..."
            )
        else:
            print(f"[OK] Đã kết nối cổng Driver động cơ ({com_driver}) thành công!")
            bo_driver = DRIVER_AUTO_AGV.BoDieuKhienDriverPID(
                ser_driver, slave_id=DRIVER_AUTO_AGV.SLAVE_ID
            )
            bo_driver.bat_dau()

    bo_agv = BoVanHanhAGVPID(
        ser_sensors=ser_sensors,
        ser_driver=ser_driver,
        bo_driver=bo_driver,
        kp=kp_val,
        ki=ki_val,
        kd=kd_val,
        v_thang=v_thang,
        v_cua=v_cua,
        quang_duong_muc_tieu=quang_duong_muc_tieu,
        thoi_gian_giam_toc=thoi_gian_giam_toc,
        w_max_val=w_max_val,
        deadband_val=deadband_val,
        dao_chieu=dao_chieu,
        so_the_muc_tieu=so_the_muc_tieu,
        thoi_gian_dung_the=thoi_gian_dung_the,
        dry_run=args.dry_run,
        com_driver=com_driver,
    )
    bo_agv.bat_dau()

    try:
        if args.no_gui:
            while bo_agv.running:
                time.sleep(0.1)
        else:
            hien_thi_do_thi_pid(bo_agv)
    except KeyboardInterrupt:
        print("\n[!] Nhận lệnh dừng (Ctrl+C). Đang phanh dừng xe an toàn...")
    finally:
        bo_agv.dung_va_luu_log()


if __name__ == "__main__":
    main()
