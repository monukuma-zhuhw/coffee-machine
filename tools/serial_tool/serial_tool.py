# -*- coding: utf-8 -*-
"""
咖啡机固件调试用 Windows 串口工具
================================

功能：
  1. 自动枚举 COM 端口；支持配置波特率、数据位、校验位、停止位。
  2. 实时显示串口接收数据（自动滚屏、时间戳、保存日志）。
  3. 实时绘制两类关键字曲线：
       - "pressure"  -> 蓝色（右 Y 轴，压力）
       - "temp"      -> 红色（左 Y 轴，温度）
     匹配规则：`关键字[:=]<数值>`（大小写不敏感）。

运行：
    pip install -r requirements.txt
    python serial_tool.py
"""

import re
import sys
import time
import threading
import queue
from collections import deque
from datetime import datetime

import tkinter as tk
from tkinter import ttk, filedialog, messagebox, scrolledtext

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("ERROR: pyserial 未安装，请运行: pip install pyserial")
    sys.exit(1)

try:
    import matplotlib
    matplotlib.use("TkAgg")
    # 让 matplotlib 支持中文显示。
    matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "Arial Unicode MS"]
    matplotlib.rcParams["axes.unicode_minus"] = False
    from matplotlib.figure import Figure
    from matplotlib.ticker import MultipleLocator
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
except ImportError:
    print("ERROR: matplotlib 未安装，请运行: pip install matplotlib")
    sys.exit(1)


# ----- 配置 ------------------------------------------------------------------

BAUD_RATES = [1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]
DATA_BITS = [5, 6, 7, 8]
PARITIES = [("无",   serial.PARITY_NONE),
            ("偶校验", serial.PARITY_EVEN),
            ("奇校验", serial.PARITY_ODD),
            ("Mark",  serial.PARITY_MARK),
            ("Space", serial.PARITY_SPACE)]
STOP_BITS = [(1.0, serial.STOPBITS_ONE),
             (1.5, serial.STOPBITS_ONE_POINT_FIVE),
             (2.0, serial.STOPBITS_TWO)]

# 曲线最多保留点数（滑动窗口）。
PLOT_MAX_POINTS = 600

# 关键字 + 数值 提取正则：pressure: 12.34   temp = 25   pressure:7   TEMP : -3.5
KEYWORD_PATTERN = re.compile(
    r"(?P<key>pressure|temp)\s*[:=]\s*(?P<val>-?\d+(?:\.\d+)?)",
    re.IGNORECASE,
)


# ----- 串口接收线程 -----------------------------------------------------------

class SerialWorker(threading.Thread):
    """从串口读取字节流，按行切分后投递到队列。"""

    def __init__(self, ser, out_queue, stop_event):
        super().__init__(daemon=True)
        self.ser = ser
        self.q = out_queue
        self.stop_event = stop_event
        self._buf = bytearray()

    def run(self):
        while not self.stop_event.is_set():
            try:
                data = self.ser.read(self.ser.in_waiting or 1)
            except (serial.SerialException, OSError) as e:
                self.q.put(("ERROR", f"串口读取错误: {e}"))
                return
            if not data:
                continue
            self._buf.extend(data)
            # 按 \n 切行；尾部不完整的行留在缓冲区。
            while True:
                nl = self._buf.find(b"\n")
                if nl < 0:
                    # 缓冲区无换行但已过大，强制冲刷。
                    if len(self._buf) > 4096:
                        line = bytes(self._buf)
                        self._buf.clear()
                        self.q.put(("LINE", self._decode(line)))
                    break
                line = bytes(self._buf[:nl])
                del self._buf[:nl + 1]
                if line.endswith(b"\r"):
                    line = line[:-1]
                self.q.put(("LINE", self._decode(line)))

    @staticmethod
    def _decode(b):
        try:
            return b.decode("utf-8", errors="replace")
        except Exception:
            return repr(b)


# ----- 主应用 ----------------------------------------------------------------

class SerialToolApp:
    def __init__(self, root):
        self.root = root
        self.root.title("串口工具 - 咖啡机调试")
        self.root.geometry("1100x780")

        self.ser = None
        self.worker = None
        self.stop_event = threading.Event()
        self.rx_queue = queue.Queue()

        # 曲线数据缓存。
        self.t_pressure = deque(maxlen=PLOT_MAX_POINTS)
        self.v_pressure = deque(maxlen=PLOT_MAX_POINTS)
        self.t_temp = deque(maxlen=PLOT_MAX_POINTS)
        self.v_temp = deque(maxlen=PLOT_MAX_POINTS)
        self.start_time = None

        self._build_ui()
        self._refresh_ports()

        # 周期任务。
        self.root.after(50, self._drain_rx_queue)
        self.root.after(200, self._redraw_plot)

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---- UI 构建 ----

    def _build_ui(self):
        # ===== 顶部：连接配置栏 =====
        bar = ttk.LabelFrame(self.root, text="串口设置")
        bar.pack(side=tk.TOP, fill=tk.X, padx=6, pady=4)

        ttk.Label(bar, text="端口:").grid(row=0, column=0, padx=4, pady=4, sticky="w")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(bar, textvariable=self.port_var, width=28, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=4, pady=4)

        ttk.Button(bar, text="刷新", command=self._refresh_ports).grid(row=0, column=2, padx=4)

        ttk.Label(bar, text="波特率:").grid(row=0, column=3, padx=4, sticky="w")
        self.baud_var = tk.IntVar(value=115200)
        ttk.Combobox(bar, textvariable=self.baud_var, values=BAUD_RATES, width=10).grid(row=0, column=4)

        ttk.Label(bar, text="数据位:").grid(row=0, column=5, padx=4, sticky="w")
        self.data_var = tk.IntVar(value=8)
        ttk.Combobox(bar, textvariable=self.data_var, values=DATA_BITS, width=4, state="readonly").grid(row=0, column=6)

        ttk.Label(bar, text="校验位:").grid(row=0, column=7, padx=4, sticky="w")
        self.parity_var = tk.StringVar(value="无")
        ttk.Combobox(bar, textvariable=self.parity_var,
                     values=[p[0] for p in PARITIES], width=8, state="readonly").grid(row=0, column=8)

        ttk.Label(bar, text="停止位:").grid(row=0, column=9, padx=4, sticky="w")
        self.stop_var = tk.StringVar(value="1.0")
        ttk.Combobox(bar, textvariable=self.stop_var,
                     values=[str(s[0]) for s in STOP_BITS], width=4, state="readonly").grid(row=0, column=10)

        self.connect_btn = ttk.Button(bar, text="打开串口", command=self._toggle_connection)
        self.connect_btn.grid(row=0, column=11, padx=8)

        self.status_var = tk.StringVar(value="未连接")
        ttk.Label(bar, textvariable=self.status_var, foreground="gray").grid(row=0, column=12, padx=8)

        # ===== 中部：上下分栏（文本 + 曲线）=====
        paned = ttk.PanedWindow(self.root, orient=tk.VERTICAL)
        paned.pack(fill=tk.BOTH, expand=True, padx=6, pady=4)

        # --- 文本区 ---
        text_frame = ttk.LabelFrame(paned, text="接收数据")
        paned.add(text_frame, weight=1)

        toolbar = ttk.Frame(text_frame)
        toolbar.pack(side=tk.TOP, fill=tk.X)

        self.autoscroll_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(toolbar, text="自动滚屏", variable=self.autoscroll_var).pack(side=tk.LEFT, padx=4)

        self.show_ts_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(toolbar, text="显示时间戳", variable=self.show_ts_var).pack(side=tk.LEFT, padx=4)

        ttk.Button(toolbar, text="清空", command=self._clear_text).pack(side=tk.LEFT, padx=4)
        ttk.Button(toolbar, text="保存日志...", command=self._save_log).pack(side=tk.LEFT, padx=4)
        ttk.Button(toolbar, text="清空曲线", command=self._clear_plot).pack(side=tk.LEFT, padx=4)

        # 发送行。
        ttk.Label(toolbar, text="  发送:").pack(side=tk.LEFT)
        self.send_var = tk.StringVar()
        send_entry = ttk.Entry(toolbar, textvariable=self.send_var, width=40)
        send_entry.pack(side=tk.LEFT, padx=4)
        send_entry.bind("<Return>", lambda e: self._send_line())
        ttk.Button(toolbar, text="发送", command=self._send_line).pack(side=tk.LEFT)
        self.append_nl_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(toolbar, text="加\\r\\n", variable=self.append_nl_var).pack(side=tk.LEFT, padx=2)

        self.text = scrolledtext.ScrolledText(text_frame, wrap=tk.NONE, height=14,
                                              font=("Consolas", 10), background="#101010",
                                              foreground="#e0e0e0", insertbackground="white")
        self.text.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)
        self.text.tag_config("rx",   foreground="#d0d0d0")
        self.text.tag_config("tx",   foreground="#7ec8ff")
        self.text.tag_config("err",  foreground="#ff7878")
        self.text.tag_config("info", foreground="#9ade5e")
        self.text.tag_config("hit",  foreground="#ffd166")

        # --- 曲线区（双 Y 轴：左温度红色，右压力蓝色）---
        plot_frame = ttk.LabelFrame(paned, text="实时曲线（左轴=温度/红色，右轴=压力/蓝色）")
        paned.add(plot_frame, weight=1)

        self.fig = Figure(figsize=(8, 3.4), dpi=100)
        # 左 Y 轴：温度。
        self.ax_temp = self.fig.add_subplot(111)
        self.ax_temp.set_xlabel("时间 (秒)")
        self.ax_temp.set_ylabel("温度 (°C)", color="red")
        self.ax_temp.tick_params(axis="y", labelcolor="red")
        self.ax_temp.grid(True, linestyle="--", alpha=0.4)
        # X 轴主刻度：每 5 秒一格。
        self.ax_temp.xaxis.set_major_locator(MultipleLocator(5))

        # 右 Y 轴：压力（共享 X）。
        self.ax_pres = self.ax_temp.twinx()
        self.ax_pres.set_ylabel("压力 (Bar)", color="blue")
        self.ax_pres.tick_params(axis="y", labelcolor="blue")

        (self.line_temp,)     = self.ax_temp.plot([], [], color="red",  lw=1.4, label="温度 (temp)")
        (self.line_pressure,) = self.ax_pres.plot([], [], color="blue", lw=1.4, label="压力 (pressure)")

        # 合并两个轴的图例到一处。
        lines = [self.line_temp, self.line_pressure]
        self.ax_temp.legend(lines, [ln.get_label() for ln in lines], loc="upper right")

        self.fig.tight_layout()

        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        nav = NavigationToolbar2Tk(self.canvas, plot_frame)
        nav.update()

    # ---- 端口枚举 ----

    def _refresh_ports(self):
        ports = serial.tools.list_ports.comports()
        items = [f"{p.device}  -  {p.description}" for p in ports]
        self.port_combo["values"] = items
        if items and not self.port_var.get():
            self.port_combo.current(0)

    def _selected_port_device(self):
        s = self.port_var.get()
        if not s:
            return None
        # 形如: "COM5  -  USB-SERIAL CH340"
        return s.split()[0]

    # ---- 打开 / 关闭 ----

    def _toggle_connection(self):
        if self.ser and self.ser.is_open:
            self._close_serial()
        else:
            self._open_serial()

    def _open_serial(self):
        device = self._selected_port_device()
        if not device:
            messagebox.showwarning("打开串口", "未选择端口。")
            return

        parity = next(v for n, v in PARITIES if n == self.parity_var.get())
        stop = next(v for n, v in STOP_BITS if str(n) == self.stop_var.get())

        try:
            self.ser = serial.Serial(
                port=device,
                baudrate=int(self.baud_var.get()),
                bytesize=int(self.data_var.get()),
                parity=parity,
                stopbits=stop,
                timeout=0.05,
                write_timeout=0.5,
            )
        except serial.SerialException as e:
            messagebox.showerror("打开失败", str(e))
            self.ser = None
            return

        self.stop_event.clear()
        self.worker = SerialWorker(self.ser, self.rx_queue, self.stop_event)
        self.worker.start()

        self.connect_btn.config(text="关闭串口")
        self.status_var.set(f"已连接: {device} @ {self.baud_var.get()}")
        self.start_time = time.time()
        self._append_text(f"[信息] 已打开 {device} @ {self.baud_var.get()} "
                          f"{self.data_var.get()}-{self.parity_var.get()}-{self.stop_var.get()}\n",
                          tag="info")

    def _close_serial(self):
        self.stop_event.set()
        if self.worker:
            self.worker.join(timeout=1.0)
            self.worker = None
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        self.connect_btn.config(text="打开串口")
        self.status_var.set("未连接")
        self._append_text("[信息] 串口已关闭\n", tag="info")

    # ---- 发送 ----

    def _send_line(self):
        if not (self.ser and self.ser.is_open):
            messagebox.showinfo("发送", "串口未打开。")
            return
        s = self.send_var.get()
        if not s:
            return
        if self.append_nl_var.get():
            s = s + "\r\n"
        try:
            self.ser.write(s.encode("utf-8", errors="replace"))
        except serial.SerialException as e:
            self._append_text(f"[错误] 写入失败: {e}\n", tag="err")
            return
        self._append_text(f"> {s}", tag="tx")

    # ---- 接收队列消费 ----

    def _drain_rx_queue(self):
        try:
            while True:
                kind, payload = self.rx_queue.get_nowait()
                if kind == "LINE":
                    self._handle_line(payload)
                elif kind == "ERROR":
                    self._append_text(f"[错误] {payload}\n", tag="err")
                    self._close_serial()
        except queue.Empty:
            pass
        self.root.after(50, self._drain_rx_queue)

    def _handle_line(self, line):
        # 写入文本区。
        ts = ""
        if self.show_ts_var.get():
            ts = datetime.now().strftime("[%H:%M:%S.%f]")[:-3] + " "
        self._append_text(f"{ts}{line}\n", tag="rx")

        # 关键字解析。
        if self.start_time is None:
            self.start_time = time.time()
        t = time.time() - self.start_time
        for m in KEYWORD_PATTERN.finditer(line):
            key = m.group("key").lower()
            try:
                val = float(m.group("val"))
            except ValueError:
                continue
            if key == "pressure":
                self.t_pressure.append(t)
                self.v_pressure.append(val)
            elif key == "temp":
                self.t_temp.append(t)
                self.v_temp.append(val)
            self._append_text(f"    -> {key}={val}\n", tag="hit")

    # ---- 文本辅助 ----

    def _append_text(self, s, tag="rx"):
        self.text.insert(tk.END, s, tag)
        # 截断防止内存膨胀。
        line_count = int(self.text.index('end-1c').split('.')[0])
        if line_count > 5000:
            self.text.delete("1.0", "1000.end+1c")
        if self.autoscroll_var.get():
            self.text.see(tk.END)

    def _clear_text(self):
        self.text.delete("1.0", tk.END)

    def _save_log(self):
        path = filedialog.asksaveasfilename(
            defaultextension=".log",
            filetypes=[("日志文件", "*.log"), ("全部文件", "*.*")],
            initialfile=datetime.now().strftime("serial_%Y%m%d_%H%M%S.log"),
        )
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8") as f:
                f.write(self.text.get("1.0", tk.END))
            messagebox.showinfo("保存日志", f"已保存到:\n{path}")
        except OSError as e:
            messagebox.showerror("保存日志", str(e))

    # ---- 曲线辅助 ----

    def _clear_plot(self):
        self.t_pressure.clear(); self.v_pressure.clear()
        self.t_temp.clear();     self.v_temp.clear()
        self.start_time = time.time()

    def _redraw_plot(self):
        try:
            self.line_temp.set_data(list(self.t_temp), list(self.v_temp))
            self.line_pressure.set_data(list(self.t_pressure), list(self.v_pressure))

            # X 轴范围：温度和压力共享。
            xs = list(self.t_pressure) + list(self.t_temp)
            if xs:
                xmin, xmax = min(xs), max(xs)
                if xmax - xmin < 1.0:
                    xmax = xmin + 1.0
                self.ax_temp.set_xlim(xmin, xmax)

            # 左 Y 轴（温度）独立缩放。
            if self.v_temp:
                ymin, ymax = min(self.v_temp), max(self.v_temp)
                pad = max(0.5, (ymax - ymin) * 0.1)
                self.ax_temp.set_ylim(ymin - pad, ymax + pad)

            # 右 Y 轴（压力）独立缩放。
            if self.v_pressure:
                ymin, ymax = min(self.v_pressure), max(self.v_pressure)
                pad = max(0.2, (ymax - ymin) * 0.1)
                self.ax_pres.set_ylim(ymin - pad, ymax + pad)

            self.canvas.draw_idle()
        except Exception as e:
            print("曲线刷新错误:", e)
        self.root.after(200, self._redraw_plot)

    # ---- 关闭 ----

    def _on_close(self):
        try:
            self._close_serial()
        finally:
            self.root.destroy()


def main():
    root = tk.Tk()
    try:
        ttk.Style().theme_use("vista")
    except tk.TclError:
        pass
    SerialToolApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
