import argparse
import collections
import math
import os
import random
import subprocess
import sys
import threading
import time

try:
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

# =====================================================================
# CẤU HÌNH MẶC ĐỊNH
# =====================================================================
DEFAULT_COM = "COM7"
DEFAULT_BAUD = 115200
SENSOR_TRUOC_ID = 20
SENSOR_SAU_ID = 87

TARGET_SAMPLE_HZ = 200.0
HISTORY_LEN = 500  # Lưu ~5 giây dữ liệu liên tục

# Tọa độ vật lý 16 mắt cảm biến Hall S1..S16 trên thanh CNS-MGS-160N (Mục 5.2 Trang 20)
HALL_POSITIONS_MM = [-75.0 + 10.0 * i for i in range(16)]


def _dinh_dang_chi_tiet_vach(sensor_id: int) -> str:
    """Định dạng thông tin đa vạch từ bộ thanh ghi 1000..1015 để in log và hiển thị."""
    info = modbus_rs485_AGV.lay_chi_tiet_slave(sensor_id)
    if not info:
        return "Chưa có dữ liệu 1000..1015"
    pole = "N" if info["strength"] > 0 else ("S" if info["strength"] < 0 else "-")
    hall_str = f"{info['hall_centroid']:.0f}mm" if info["hall_centroid"] is not None else "-"
    return (
        f"Mask={info['mask']}(n={info['status']}) "
        f"L/C/R=({info['pos_L']:.0f},{info['pos_C']:.0f},{info['pos_R']:.0f}) "
        f"Str=({info['str_L']:.0f},{info['str_C']:.0f},{info['str_R']:.0f})[{pole}] "
        f"Hall={hall_str}"
    )


class BoDocCamBienTuTocDoCao:
    """
    Luồng đọc Modbus RTU tốc độ cao kết hợp:
    1. Đọc đồng thời 16 thanh ghi 1000..1015 (FC04) chỉ trong 1 gói USB FTDI (37 bytes, ~12ms).
    2. Tái tạo vị trí từ trường thực từ 3 vạch (Trái 1005, Giữa 1006, Phải 1007) + Cường độ (1008..1010)
       + 16 mắt Hall S1..S16 (1014..1015), triệt tiêu hoàn toàn hiện tượng nhảy lùi ngược chiều
       khi quét 1 chiều do quy tắc ưu tiên gần tâm 0mm (L1 < L2, Trang 26 Datasheet) của thanh ghi 1002.
    3. Lập lịch ưu tiên động (Dynamic Priority Polling): tự động dồn 80% băng thông cho cảm biến
       đang có nam châm quét qua để đạt tốc độ lấy mẫu sóng sin cao nhất.
    """

    def __init__(
        self,
        com_port=DEFAULT_COM,
        baudrate=DEFAULT_BAUD,
        che_do="both",
        target_hz=TARGET_SAMPLE_HZ,
        mo_phong=False,
    ):
        self.com_port = com_port
        self.baudrate = baudrate
        self.che_do = che_do
        self.target_hz = target_hz
        self.mo_phong = mo_phong

        self.ser = None
        self.running = False
        self.thread = None
        self.data_lock = threading.Lock()

        self.t_buf = collections.deque(maxlen=HISTORY_LEN)
        self.raw_truoc_buf = collections.deque(maxlen=HISTORY_LEN)
        self.raw_sau_buf = collections.deque(maxlen=HISTORY_LEN)
        self.e_truoc_buf = collections.deque(maxlen=HISTORY_LEN)
        self.e_sau_buf = collections.deque(maxlen=HISTORY_LEN)
        self.e_tong_buf = collections.deque(maxlen=HISTORY_LEN)

        self.raw_truoc = 0.0
        self.raw_sau = 0.0
        self.e_truoc = 0.0
        self.e_sau = 0.0
        self.online_truoc = False
        self.online_sau = False
        self.e_tong = 0.0

        self.tan_so_mau_hz = 0.0
        self.so_mau_thanh_cong = 0
        self.so_loi_modbus = 0
        self.so_rac_da_loc = 0

        self._fail_streak_truoc = 0
        self._fail_streak_sau = 0
        self._next_probe_truoc = 0.0
        self._next_probe_sau = 0.0

        self._last_active_truoc = 0.0
        self._last_active_sau = 0.0

    def bat_dau(self) -> bool:
        if not self.mo_phong:
            self.ser = modbus_rs485_AGV.ket_noi_rs485_nhanh(
                self.com_port, self.baudrate, timeout=0.035
            )
            if self.ser is None:
                return False

        self.running = True
        self.thread = threading.Thread(target=self._vong_lap_doc_modbus, daemon=True)
        self.thread.start()
        return True

    def dung_lai(self):
        self.running = False
        if self.thread is not None and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        if self.ser is not None and self.ser.is_open:
            self.ser.close()

    def _ghi_diem_do_thi(self, t_start, sample_timestamps):
        """Đẩy ngay mẫu vừa đọc vào bộ đệm đồ thị để sóng sin có mật độ điểm cao nhất."""
        now = time.perf_counter()
        sample_timestamps.append(now)
        if len(sample_timestamps) >= 2:
            dt_s = sample_timestamps[-1] - sample_timestamps[0]
            mau_hz = (len(sample_timestamps) - 1) / dt_s if dt_s > 0 else 0.0
        else:
            mau_hz = 0.0

        rac_tr = modbus_rs485_AGV.lay_bo_loc_slave(SENSOR_TRUOC_ID).so_mau_rac_da_loc
        rac_sa = modbus_rs485_AGV.lay_bo_loc_slave(SENSOR_SAU_ID).so_mau_rac_da_loc

        with self.data_lock:
            self.tan_so_mau_hz = mau_hz
            self.so_rac_da_loc = rac_tr + rac_sa

            if self.online_truoc and self.online_sau:
                self.e_tong = round((self.e_truoc * 0.7) + (self.e_sau * 0.3), 2)
            elif self.online_truoc:
                self.e_tong = round(self.e_truoc, 2)
            elif self.online_sau:
                self.e_tong = round(self.e_sau, 2)
            else:
                self.e_tong = 0.0

            self.t_buf.append(now - t_start)
            self.raw_truoc_buf.append(self.raw_truoc)
            self.raw_sau_buf.append(self.raw_sau)
            self.e_truoc_buf.append(self.e_truoc)
            self.e_sau_buf.append(self.e_sau)
            self.e_tong_buf.append(self.e_tong)

    def _doc_va_cap_nhat_1_slave(self, is_truoc: bool, t_start: float, sample_timestamps):
        now = time.perf_counter()
        if is_truoc:
            if self._fail_streak_truoc >= 25 and now < self._next_probe_truoc:
                return
            res = magnetic_sensor_AGV.doc_cam_bien_truoc(
                self.ser, SENSOR_TRUOC_ID, loc_nhieu=True, tra_ve_raw=True
            )
            if res is not None:
                clean_val, raw_val = res
                self._fail_streak_truoc = 0
                self.online_truoc = True
                if clean_val != 0.0 or raw_val != 0.0:
                    self._last_active_truoc = time.perf_counter()
                with self.data_lock:
                    self.e_truoc = float(clean_val)
                    self.raw_truoc = float(raw_val)
                    self.so_mau_thanh_cong += 1
                self._ghi_diem_do_thi(t_start, sample_timestamps)
            else:
                self._fail_streak_truoc += 1
                if self._fail_streak_truoc < 25:
                    with self.data_lock:
                        self.so_loi_modbus += 1
                else:
                    self.online_truoc = False
                    self._next_probe_truoc = now + 1.0
        else:
            if self._fail_streak_sau >= 25 and now < self._next_probe_sau:
                return
            res = magnetic_sensor_2_AGV.doc_cam_bien_sau(
                self.ser, SENSOR_SAU_ID, loc_nhieu=True, tra_ve_raw=True
            )
            if res is not None:
                clean_val, raw_val = res
                self._fail_streak_sau = 0
                self.online_sau = True
                if clean_val != 0.0 or raw_val != 0.0:
                    self._last_active_sau = time.perf_counter()
                with self.data_lock:
                    self.e_sau = float(clean_val)
                    self.raw_sau = float(raw_val)
                    self.so_mau_thanh_cong += 1
                self._ghi_diem_do_thi(t_start, sample_timestamps)
            else:
                self._fail_streak_sau += 1
                if self._fail_streak_sau < 25:
                    with self.data_lock:
                        self.so_loi_modbus += 1
                else:
                    self.online_sau = False
                    self._next_probe_sau = now + 1.0

    def _vong_lap_doc_modbus(self):
        t_start = time.perf_counter()
        sample_timestamps = collections.deque(maxlen=150)
        last_console_print = 0.0
        step_counter = 0

        bo_loc_sim_tr = modbus_rs485_AGV.lay_bo_loc_slave(SENSOR_TRUOC_ID)
        bo_loc_sim_sa = modbus_rs485_AGV.lay_bo_loc_slave(SENSOR_SAU_ID)

        while self.running:
            now = time.perf_counter()
            rel_t = now - t_start

            if self.mo_phong:
                # Mô phỏng quét nam châm từ Trái (-65mm) sang Phải (+65mm),
                # trong đó thanh ghi 1002 bị lật giả sang vạch phụ -15mm khi đi qua +18..+30mm
                true_tr = round(60.0 * math.sin(2.0 * math.pi * 0.45 * rel_t))
                if 16 <= true_tr <= 30:
                    # Mô phỏng lỗi L1 < L2: vạch phụ ở -14mm gần tâm 0mm hơn vạch chính +18..+30mm
                    reg1002_tr = -14
                    regs_sim = [
                        2,                  # 1000: 2 vạch
                        6,                  # 1001: Giữa + Phải (0b110)
                        reg1002_tr & 0xFFFF,# 1002: xuất nhầm vạch phụ -14mm
                        180,                # 1003: cường độ vạch giữa
                        2,                  # 1004: 2 mắt
                        0,                  # 1005: Trái = 0
                        reg1002_tr & 0xFFFF,# 1006: Giữa = -14mm (vạch phụ)
                        int(true_tr) & 0xFFFF, # 1007: Phải = vị trí nam châm chính!
                        0,                  # 1008: Str Trái
                        180,                # 1009: Str Giữa (yếu)
                        620,                # 1010: Str Phải (mạnh gấp 3.4 lần!)
                        0, 2, 3,            # 1011..1013: số mắt
                        0, 0                # 1014..1015: Hall bits
                    ]
                else:
                    reg1002_tr = int(true_tr)
                    regs_sim = [
                        1, 2, reg1002_tr & 0xFFFF, 600, 3,
                        0, reg1002_tr & 0xFFFF, 0,
                        0, 600, 0,
                        0, 3, 0,
                        0, 0
                    ]
                # Gắn bit Hall S1..S16 tương ứng vị trí true_tr
                idx_hall = max(0, min(15, int(round((true_tr + 75.0) / 10.0))))
                mask16 = (1 << idx_hall) | (1 << min(15, idx_hall + 1))
                regs_sim[14] = (mask16 >> 8) & 0xFF
                regs_sim[15] = mask16 & 0xFF

                info_tr = modbus_rs485_AGV.tai_tao_vi_tri_da_vach(regs_sim)
                modbus_rs485_AGV._chi_tiet_theo_slave[SENSOR_TRUOC_ID] = info_tr
                clean_tr = bo_loc_sim_tr.cap_nhat(info_tr["pos_true"], now)
                raw_tr = info_tr["pos_1002"]

                base_sa = round(35.0 * math.sin(2.0 * math.pi * 0.45 * rel_t - 0.5))
                clean_sa = bo_loc_sim_sa.cap_nhat(float(base_sa), now)

                with self.data_lock:
                    self.online_truoc = True
                    self.online_sau = True
                    self.e_truoc = clean_tr
                    self.raw_truoc = raw_tr
                    self.e_sau = clean_sa
                    self.raw_sau = float(base_sa)
                    self.so_mau_thanh_cong += 1
                self._ghi_diem_do_thi(t_start, sample_timestamps)
                time.sleep(0.015)
            else:
                if self.che_do == "front":
                    self._doc_va_cap_nhat_1_slave(True, t_start, sample_timestamps)
                elif self.che_do == "rear":
                    self._doc_va_cap_nhat_1_slave(False, t_start, sample_timestamps)
                else:
                    truoc_active = (now - self._last_active_truoc) < 1.2
                    sau_active = (now - self._last_active_sau) < 1.2

                    if truoc_active and not sau_active:
                        if step_counter % 5 == 4:
                            self._doc_va_cap_nhat_1_slave(False, t_start, sample_timestamps)
                        else:
                            self._doc_va_cap_nhat_1_slave(True, t_start, sample_timestamps)
                    elif sau_active and not truoc_active:
                        if step_counter % 5 == 4:
                            self._doc_va_cap_nhat_1_slave(True, t_start, sample_timestamps)
                        else:
                            self._doc_va_cap_nhat_1_slave(False, t_start, sample_timestamps)
                    else:
                        if step_counter % 2 == 0:
                            self._doc_va_cap_nhat_1_slave(True, t_start, sample_timestamps)
                        else:
                            self._doc_va_cap_nhat_1_slave(False, t_start, sample_timestamps)
                    step_counter += 1

            now_after = time.perf_counter()
            if now_after - last_console_print >= 0.40:
                last_console_print = now_after
                s_tr = (
                    f"{self.e_truoc:>5.1f}mm (1002:{self.raw_truoc:>4.0f})"
                    if self.online_truoc
                    else "  ---  "
                )
                s_sa = (
                    f"{self.e_sau:>5.1f}mm (1002:{self.raw_sau:>4.0f})"
                    if self.online_sau
                    else "  ---  "
                )
                avg_ms = modbus_rs485_AGV.thong_ke_modbus["avg_dt_ms"]
                active_id = (
                    SENSOR_TRUOC_ID
                    if abs(self.e_truoc) >= abs(self.e_sau)
                    else SENSOR_SAU_ID
                )
                chi_tiet = _dinh_dang_chi_tiet_vach(active_id)
                line_log = (
                    f"[{time.strftime('%H:%M:%S')}] {self.tan_so_mau_hz:>5.1f} mẫu/s (~{avg_ms:.1f}ms) | "
                    f"Trước: {s_tr} | Sau: {s_sa} | PID:{self.e_tong:>5.1f}mm | "
                    f"ID{active_id}[{chi_tiet}] | OK:{self.so_mau_thanh_cong} Lỗi:{self.so_loi_modbus}"
                )
                print(line_log)
                modbus_rs485_AGV.ghi_log(line_log)


def chay_visualize(
    com_port=DEFAULT_COM,
    baudrate=DEFAULT_BAUD,
    che_do="both",
    target_hz=TARGET_SAMPLE_HZ,
    mo_phong=False,
):
    """Khởi chạy giao diện Matplotlib hiển thị Đa Vạch (1000..1015) + 16 Mắt Hall S1..S16."""
    bo_doc = BoDocCamBienTuTocDoCao(
        com_port=com_port,
        baudrate=baudrate,
        che_do=che_do,
        target_hz=target_hz,
        mo_phong=mo_phong,
    )

    if not bo_doc.bat_dau():
        print(
            f"\n[!] Không thể mở cổng {com_port} (có thể đang có cửa sổ Python khác giữ cổng COM7).\n"
            f"    Hãy tắt cửa sổ cũ trước khi chạy lại, hoặc chạy thử chế độ mô phỏng:\n"
            f"    python visualize_cam_bien_tu.py --sim"
        )
        return

    print("=" * 105)
    print(
        f"ĐANG VISUALIZE ĐA VẠCH (1000..1015) + 16 MẮT HALL S1..S16 + BỘ LỌC ALPHA-BETA ({'MÔ PHỎNG' if mo_phong else com_port})"
    )
    print(f"File log chẩn đoán: {modbus_rs485_AGV.LOG_FILE_PATH}")
    print("Đóng cửa sổ đồ thị Matplotlib hoặc nhấn Ctrl+C để dừng.")
    print("=" * 105)

    plt.style.use(
        "seaborn-v0_8-darkgrid"
        if "seaborn-v0_8-darkgrid" in plt.style.available
        else "default"
    )
    fig, (ax_wave, ax_bar) = plt.subplots(
        2, 1, figsize=(12.0, 7.2), gridspec_kw={"height_ratios": [2.6, 1.25]}
    )
    fig.canvas.manager.set_window_title(
        f"Kiểm tra Cảm Biến Từ Hinson CNS-MGS-160N (Đa Vạch 1000..1015 + 16 Mắt Hall) - {com_port}"
    )

    ax_wave.set_title(
        "ĐỒ THỊ TÍN HIỆU: THANH GHI 1002 THÔ (NÉT ĐỨT) vs TÁI TẠO ĐA VẠCH 1005..1015 + LỌC PID (NÉT ĐẬM)",
        fontsize=11.0,
        fontweight="bold",
    )
    ax_wave.set_xlabel("Thời gian tương đối so với hiện tại (giây)")
    ax_wave.set_ylabel("Độ lệch tâm (mm)")
    ax_wave.set_xlim(-5.0, 0.05)
    ax_wave.set_ylim(-92, 92)
    ax_wave.axhline(0, color="black", linestyle="-", linewidth=1.0, alpha=0.5)
    ax_wave.axhline(75, color="gray", linestyle=":", linewidth=0.9, alpha=0.6, label="Giới hạn vật lý (±75 mm)")
    ax_wave.axhline(-75, color="gray", linestyle=":", linewidth=0.9, alpha=0.6)
    ax_wave.axhline(10, color="#d9534f", linestyle="--", linewidth=0.9, alpha=0.6, label="Ngưỡng rẽ (±10 mm)")
    ax_wave.axhline(-10, color="#d9534f", linestyle="--", linewidth=0.9, alpha=0.6)

    (line_raw_tr,) = ax_wave.plot(
        [], [], color="#0275d8", linestyle="--", linewidth=1.0, alpha=0.35, label="Trước Thanh ghi 1002 Thô (ID 20)"
    )
    (line_raw_sa,) = ax_wave.plot(
        [], [], color="#f0ad4e", linestyle="--", linewidth=1.0, alpha=0.35, label="Sau Thanh ghi 1002 Thô (ID 87)"
    )

    (line_truoc,) = ax_wave.plot(
        [], [], color="#0275d8", linewidth=2.4, label="Trước Đa-Vạch + Lọc PID (ID 20)"
    )
    (line_sau,) = ax_wave.plot(
        [], [], color="#f0ad4e", linewidth=2.4, label="Sau Đa-Vạch + Lọc PID (ID 87)"
    )
    (line_tong,) = ax_wave.plot(
        [], [], color="#28a745", linewidth=2.6, linestyle="-", label="Sai số PID gộp (0.7*Trước + 0.3*Sau)"
    )
    ax_wave.legend(loc="upper right", fontsize=8.3, ncol=2, framealpha=0.92)

    status_text = ax_wave.text(
        0.012,
        0.96,
        "Đang đồng bộ RS485 (16 thanh ghi 1000..1015)...",
        transform=ax_wave.transAxes,
        fontsize=8.8,
        fontweight="bold",
        verticalalignment="top",
        bbox=dict(boxstyle="round,pad=0.4", facecolor="#f8f9fa", edgecolor="#343a40", alpha=0.92),
    )

    ax_bar.set_title(
        "16 MẮT CẢM BIẾN HALL (S1: -75mm ... S16: +75mm) & VỊ TRÍ TÂM TỪ TRƯỜNG SAU TÁI TẠO ĐA VẠCH",
        fontsize=10,
        fontweight="bold",
    )
    ax_bar.set_xlim(-82, 82)
    ax_bar.set_ylim(-1.05, 1.05)
    ax_bar.set_yticks([])
    ax_bar.set_xticks(HALL_POSITIONS_MM)
    ax_bar.set_xticklabels([f"S{i+1}\n{int(p)}" for i, p in enumerate(HALL_POSITIONS_MM)], fontsize=7.5)
    ax_bar.set_xlabel("Tọa độ 16 mắt cảm biến Hall S1..S16 (mm) — Cách nhau 10mm (Mục 5.2 Trang 20)")

    ax_bar.axvspan(-10, 10, color="#d4edda", alpha=0.55, label="Vùng đi thẳng (±10mm)")
    ax_bar.axvline(0, color="black", linestyle="-", linewidth=1.2)
    ax_bar.axvline(-10, color="#d9534f", linestyle="--", linewidth=1.0)
    ax_bar.axvline(10, color="#d9534f", linestyle="--", linewidth=1.0)

    # Vẽ 16 đèn LED tương ứng 16 mắt Hall S1..S16 của cảm biến Trước (hàng trên y=0.55) và Sau (hàng dưới y=-0.55)
    (hall_off_tr,) = ax_bar.plot(
        HALL_POSITIONS_MM, [0.55] * 16, "o", markersize=8, color="#ced4da", alpha=0.7
    )
    (hall_on_tr,) = ax_bar.plot(
        [], [], "o", markersize=10, color="#ff3b30", label="Mắt Hall S1..S16 đang sáng (Reg 1014-1015)"
    )
    (hall_off_sa,) = ax_bar.plot(
        HALL_POSITIONS_MM, [-0.55] * 16, "o", markersize=8, color="#ced4da", alpha=0.7
    )
    (hall_on_sa,) = ax_bar.plot(
        [], [], "o", markersize=10, color="#ff9500"
    )

    (marker_truoc,) = ax_bar.plot([0], [0.22], marker="v", markersize=14, color="#0275d8", label="Trước (ID 20)")
    (marker_sau,) = ax_bar.plot([0], [-0.22], marker="^", markersize=14, color="#f0ad4e", label="Sau (ID 87)")
    (marker_tong,) = ax_bar.plot([0], [0.0], marker="o", markersize=15, color="#28a745", label="Tâm PID hợp lực")
    ax_bar.legend(loc="upper right", fontsize=7.8, ncol=4)

    huong_text = ax_bar.text(
        0.015,
        0.82,
        "",
        transform=ax_bar.transAxes,
        fontsize=10.0,
        fontweight="bold",
    )

    plt.tight_layout()

    def cap_nhat_do_thi(_frame):
        with bo_doc.data_lock:
            if not bo_doc.t_buf:
                return (
                    line_raw_tr,
                    line_raw_sa,
                    line_truoc,
                    line_sau,
                    line_tong,
                    hall_on_tr,
                    hall_on_sa,
                    marker_truoc,
                    marker_sau,
                    marker_tong,
                    status_text,
                    huong_text,
                )

            t_raw = list(bo_doc.t_buf)
            r_tr_list = list(bo_doc.raw_truoc_buf)
            r_sa_list = list(bo_doc.raw_sau_buf)
            e_tr_list = list(bo_doc.e_truoc_buf)
            e_sa_list = list(bo_doc.e_sau_buf)
            e_tg_list = list(bo_doc.e_tong_buf)

            r_tr = bo_doc.raw_truoc
            r_sa = bo_doc.raw_sau
            e_tr = bo_doc.e_truoc
            e_sa = bo_doc.e_sau
            on_tr = bo_doc.online_truoc
            on_sa = bo_doc.online_sau
            e_tg = bo_doc.e_tong
            mau_hz = bo_doc.tan_so_mau_hz
            ok_cnt = bo_doc.so_mau_thanh_cong
            err_cnt = bo_doc.so_loi_modbus
            rac_cnt = bo_doc.so_rac_da_loc

        t_latest = t_raw[-1]
        t_rel = [t - t_latest for t in t_raw]

        if on_tr:
            line_raw_tr.set_data(t_rel, r_tr_list)
            line_truoc.set_data(t_rel, e_tr_list)
        else:
            line_raw_tr.set_data([], [])
            line_truoc.set_data([], [])

        if on_sa:
            line_raw_sa.set_data(t_rel, r_sa_list)
            line_sau.set_data(t_rel, e_sa_list)
        else:
            line_raw_sa.set_data([], [])
            line_sau.set_data([], [])

        line_tong.set_data(t_rel, e_tg_list)

        # Cập nhật đèn LED 16 mắt Hall S1..S16 từ thanh ghi 1014-1015
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

        marker_truoc.set_data([e_tr], [0.22])
        marker_truoc.set_alpha(1.0 if on_tr else 0.15)

        marker_sau.set_data([e_sa], [-0.22])
        marker_sau.set_alpha(1.0 if on_sa else 0.15)

        marker_tong.set_data([e_tg], [0.0])

        if e_tg < -10.0:
            trang_thai = "LỆCH TRÁI -> RẼ TRÁI (A)"
            mau_tt = "#d9534f"
        elif e_tg > 10.0:
            trang_thai = "LỆCH PHẢI -> RẼ PHẢI (F)"
            mau_tt = "#d9534f"
        else:
            trang_thai = "CÂN BẰNG TÂM -> ĐI THẲNG (W)"
            mau_tt = "#28a745"

        str_tr = (
            f"{e_tr:>5.1f} mm (Reg1002: {r_tr:>4.0f}mm | {_dinh_dang_chi_tiet_vach(SENSOR_TRUOC_ID)})"
            if on_tr
            else "Đang dò..."
        )
        str_sa = (
            f"{e_sa:>5.1f} mm (Reg1002: {r_sa:>4.0f}mm | {_dinh_dang_chi_tiet_vach(SENSOR_SAU_ID)})"
            if on_sa
            else "Đang dò..."
        )
        avg_ms = modbus_rs485_AGV.thong_ke_modbus["avg_dt_ms"]
        total_p = ok_cnt + err_cnt
        pct_ok = (ok_cnt * 100.0 / total_p) if total_p > 0 else 100.0

        status_text.set_text(
            f"Tốc độ đọc 16 thanh ghi: {mau_hz:>5.1f} mẫu/s (~{avg_ms:.1f} ms/gói | Ổn định: {pct_ok:.1f}%)\n"
            f"Trước (ID 20): {str_tr}\n"
            f"Sau   (ID 87): {str_sa}\n"
            f"Sai số PID e : {e_tg:>5.1f} mm | OK: {ok_cnt} | Lỗi: {err_cnt} | Đã lọc: {rac_cnt}"
        )

        huong_text.set_text(f"NGÕ RA CHO PID: {trang_thai} (e_PID = {e_tg:.1f} mm)")
        huong_text.set_color(mau_tt)

        return (
            line_raw_tr,
            line_raw_sa,
            line_truoc,
            line_sau,
            line_tong,
            hall_on_tr,
            hall_on_sa,
            marker_truoc,
            marker_sau,
            marker_tong,
            status_text,
            huong_text,
        )

    _ani = FuncAnimation(fig, cap_nhat_do_thi, interval=18, blit=True, cache_frame_data=False)

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        bo_doc.dung_lai()
        print("\nĐã dừng luồng đọc cảm biến và đóng cổng COM an toàn.")


def main():
    parser = argparse.ArgumentParser(
        description="Visualize Cảm Biến Từ AGV (Đa Vạch 1000..1015 + 16 Mắt Hall + Bộ lọc Alpha-Beta)"
    )
    parser.add_argument("--port", default=DEFAULT_COM, help="Cổng COM RS485 (mặc định: COM7)")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Baudrate (mặc định: 115200)")
    parser.add_argument(
        "--mode",
        choices=["both", "front", "rear"],
        default="both",
        help="Chế độ đọc: 'both' (cả 2 cảm biến), 'front' (chỉ ID 20), 'rear' (chỉ ID 87)",
    )
    parser.add_argument("--hz", type=float, default=TARGET_SAMPLE_HZ, help="Tần số đọc mục tiêu")
    parser.add_argument("--sim", action="store_true", help="Chạy mô phỏng lỗi lật vạch L1<L2 & cách khử")
    args = parser.parse_args()

    chay_visualize(
        com_port=args.port,
        baudrate=args.baud,
        che_do=args.mode,
        target_hz=args.hz,
        mo_phong=args.sim,
    )


if __name__ == "__main__":
    main()
