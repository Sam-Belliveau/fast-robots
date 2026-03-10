#!/usr/bin/env python3
"""
Lab 3 ToF Sensor Characterization Tool

Workflow: set robot at known distance → press "Measure" → get stats + plot.
Results table tracks true distance, mode, mean, std, error for the lab writeup.

Run from the ble_python directory:
    python tof_gui.py
"""

import sys
import os

os.chdir(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ".")

import tkinter as tk
from tkinter import ttk
import queue
import time
import csv
import numpy as np

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

from ble import get_ble_controller
from cmd_types import CMD


class ToFGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Lab 3 — ToF Characterization")
        self.root.geometry("1000x850")

        self.ble = None
        self.connected = False

        self.tof1_data = {"time": [], "dist": []}
        self.tof2_data = {"time": [], "dist": []}
        self.tof_stats = {}
        self.transfer_done = False
        self._pending_logs = []

        self._build_gui()
        self._flush_logs()

    # ── BLE notification handler ─────────────────────────────────────

    def _handle(self, _uuid, raw):
        """Called from bleak's thread — no tkinter calls here."""
        msg = raw.decode()
        self._pending_logs.append(msg)

        if msg == "END":
            self.transfer_done = True
            return

        prefix, _, payload = msg.partition(":")
        parts = payload.split("|")
        try:
            if prefix == "D1":
                self.tof1_data["time"].append(float(parts[0]))
                self.tof1_data["dist"].append(float(parts[1]))
            elif prefix == "D2":
                self.tof2_data["time"].append(float(parts[0]))
                self.tof2_data["dist"].append(float(parts[1]))
            elif prefix == "S1":
                self.tof_stats["s1"] = {"n": int(parts[0]), "mean": float(parts[1]), "std": float(parts[2])}
            elif prefix == "S2":
                self.tof_stats["s2"] = {"n": int(parts[0]), "mean": float(parts[1]), "std": float(parts[2])}
        except (ValueError, IndexError):
            pass

    def _flush_logs(self):
        """Drain pending log messages onto the tkinter log widget. Runs on main thread."""
        if self._pending_logs:
            self.log_text.config(state="normal")
            for msg in self._pending_logs:
                self.log_text.insert("end", f"<< {msg}\n")
            self.log_text.see("end")
            self.log_text.config(state="disabled")
            self._pending_logs.clear()
        self.root.after(100, self._flush_logs)

    # ── GUI layout ───────────────────────────────────────────────────

    def _build_gui(self):
        # Connection
        f = ttk.LabelFrame(self.root, text="Connection", padding=8)
        f.pack(fill="x", padx=10, pady=(8, 4))
        self.btn_connect = ttk.Button(f, text="Connect", command=self._on_connect)
        self.btn_connect.pack(side="left", padx=4)
        self.btn_disconnect = ttk.Button(f, text="Disconnect", command=self._on_disconnect, state="disabled")
        self.btn_disconnect.pack(side="left", padx=4)
        self.lbl_status = ttk.Label(f, text="● Disconnected", foreground="gray")
        self.lbl_status.pack(side="left", padx=16)

        # Controls
        f = ttk.LabelFrame(self.root, text="Measurement", padding=8)
        f.pack(fill="x", padx=10, pady=4)

        ttk.Label(f, text="Mode:").pack(side="left", padx=(0, 4))
        self.mode_var = tk.StringVar(value="long")
        ttk.Radiobutton(f, text="Short", variable=self.mode_var, value="short", command=self._on_mode).pack(side="left", padx=4)
        ttk.Radiobutton(f, text="Long", variable=self.mode_var, value="long", command=self._on_mode).pack(side="left", padx=4)

        ttk.Separator(f, orient="vertical").pack(side="left", fill="y", padx=8)

        ttk.Label(f, text="True dist (mm):").pack(side="left", padx=(0, 4))
        self.var_true = tk.StringVar(value="")
        ttk.Entry(f, textvariable=self.var_true, width=8).pack(side="left", padx=4)

        ttk.Separator(f, orient="vertical").pack(side="left", fill="y", padx=8)

        self.btn_measure = ttk.Button(f, text="Measure", command=self._on_measure, state="disabled")
        self.btn_measure.pack(side="left", padx=8)
        self.lbl_result = ttk.Label(f, text="", font=("Menlo", 12))
        self.lbl_result.pack(side="left", padx=8)

        # Plot
        f = ttk.LabelFrame(self.root, text="Last Measurement", padding=4)
        f.pack(fill="both", expand=True, padx=10, pady=4)
        self.fig = Figure(figsize=(8, 3), dpi=100)
        self.ax1 = self.fig.add_subplot(121)
        self.ax2 = self.fig.add_subplot(122)
        for ax, label in [(self.ax1, "Sensor 1"), (self.ax2, "Sensor 2")]:
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Distance (mm)")
            ax.set_title(label)
            ax.grid(True, alpha=0.3)
        self.line1, = self.ax1.plot([], [], "steelblue", linewidth=1.2)
        self.line2, = self.ax2.plot([], [], "tab:orange", linewidth=1.2)
        self.fig.tight_layout()
        self.canvas = FigureCanvasTkAgg(self.fig, master=f)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        # Results Table
        f = ttk.LabelFrame(self.root, text="Results", padding=4)
        f.pack(fill="both", padx=10, pady=(4, 4))
        cols = ("sensor", "mode", "true", "mean", "std", "error", "n")
        self.tree = ttk.Treeview(f, columns=cols, show="headings", height=6)
        for cid, heading, width in [
            ("sensor", "Sensor", 70), ("mode", "Mode", 60),
            ("true", "True (mm)", 80), ("mean", "Mean (mm)", 90),
            ("std", "Std (mm)", 80), ("error", "Error (mm)", 90), ("n", "N", 50),
        ]:
            self.tree.heading(cid, text=heading)
            self.tree.column(cid, width=width, anchor="center")
        self.tree.pack(fill="x")

        bf = ttk.Frame(self.root)
        bf.pack(pady=(0, 4))
        ttk.Button(bf, text="Export CSV", command=self._export_csv).pack(side="left", padx=4)
        ttk.Button(bf, text="Clear", command=self._clear).pack(side="left", padx=4)

        # Log
        f = ttk.LabelFrame(self.root, text="BLE Log", padding=4)
        f.pack(fill="both", padx=10, pady=(0, 8))
        self.log_text = tk.Text(f, height=5, font=("Menlo", 9), state="disabled", wrap="none")
        scrollbar = ttk.Scrollbar(f, orient="vertical", command=self.log_text.yview)
        self.log_text.config(yscrollcommand=scrollbar.set)
        scrollbar.pack(side="right", fill="y")
        self.log_text.pack(fill="both", expand=True)

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ── Connection ───────────────────────────────────────────────────

    def _on_connect(self):
        self.btn_connect.config(state="disabled")
        self.lbl_status.config(text="● Connecting…", foreground="orange")
        self.root.update()
        try:
            self.ble = get_ble_controller()
            self.ble.connect()
            self.ble.start_notify(self.ble.uuid["RX_STRING"], self._handle)
            self.connected = True
            self.lbl_status.config(text="● Connected", foreground="green")
            self.btn_disconnect.config(state="normal")
            self.btn_measure.config(state="normal")
        except Exception as e:
            self.lbl_status.config(text=f"● {str(e)[:40]}", foreground="red")
            self.btn_connect.config(state="normal")

    def _on_disconnect(self):
        if self.ble:
            try:
                self.ble.stop_notify(self.ble.uuid["RX_STRING"])
                self.ble.disconnect()
            except Exception:
                pass
        self.connected = False
        self.lbl_status.config(text="● Disconnected", foreground="gray")
        self.btn_connect.config(state="normal")
        self.btn_disconnect.config(state="disabled")
        self.btn_measure.config(state="disabled")

    def _on_mode(self):
        if not self.connected:
            return
        cmd = CMD.TOF_SHORT if self.mode_var.get() == "short" else CMD.TOF_LONG
        self.ble.send_command(cmd, "")

    # ── Measure ──────────────────────────────────────────────────────

    def _on_measure(self):
        self.btn_measure.config(state="disabled")
        self.lbl_result.config(text="Collecting 3s…", foreground="orange")
        # Wait 3s for fresh samples, then request stats + data
        self.root.after(3000, self._request_stats)

    def _request_stats(self):
        self.tof_stats = {}
        self.transfer_done = False
        self.ble.send_command(CMD.TOF_STATS, "")
        self._wait_for_end(then=self._request_data)

    def _request_data(self):
        self.tof1_data = {"time": [], "dist": []}
        self.tof2_data = {"time": [], "dist": []}
        self.transfer_done = False
        self.ble.send_command(CMD.SEND_TOF_DATA, "")
        self._wait_for_end(then=self._measure_done)

    def _wait_for_end(self, then, deadline=None):
        if deadline is None:
            deadline = time.time() + 10
        if self.transfer_done or time.time() > deadline:
            then()
        else:
            self.root.after(100, lambda: self._wait_for_end(then, deadline))

    def _measure_done(self):
        mode = self.mode_var.get()
        true_str = self.var_true.get().strip()
        true_mm = float(true_str) if true_str else None

        # Update plots
        for data, line, ax in [
            (self.tof1_data, self.line1, self.ax1),
            (self.tof2_data, self.line2, self.ax2),
        ]:
            if data["time"]:
                t = np.array(data["time"]) / 1e6
                t -= t[0]
                line.set_data(t, data["dist"])
                ax.relim()
                ax.autoscale_view()
            else:
                line.set_data([], [])
                ax.set_xlim(0, 1)
                ax.set_ylim(0, 1)
        self.canvas.draw()

        # Add rows to table
        for key, label in [("s1", "S1"), ("s2", "S2")]:
            s = self.tof_stats.get(key)
            if not s:
                continue
            err_str = f"{s['mean'] - true_mm:+.1f}" if true_mm is not None else "—"
            true_disp = f"{true_mm:.0f}" if true_mm is not None else "—"
            self.tree.insert("", "end", values=(
                label, mode, true_disp,
                f"{s['mean']:.1f}", f"{s['std']:.2f}", err_str, str(s["n"]),
            ))

        s1 = self.tof_stats.get("s1", {"mean": 0, "std": 0, "n": 0})
        self.lbl_result.config(
            text=f"S1: {s1['mean']:.1f} ± {s1['std']:.2f} mm (n={s1['n']})",
            foreground="green",
        )
        self.btn_measure.config(state="normal")

    # ── Export / Clear ───────────────────────────────────────────────

    def _export_csv(self):
        rows = []
        for item in self.tree.get_children():
            rows.append(self.tree.item(item)["values"])
        if not rows:
            return
        path = "tof_results.csv"
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["sensor", "mode", "true_mm", "mean_mm", "std_mm", "error_mm", "n"])
            w.writerows(rows)
        self.lbl_result.config(text=f"Exported to {path}", foreground="blue")

    def _clear(self):
        for row in self.tree.get_children():
            self.tree.delete(row)
        self.lbl_result.config(text="")
        for line in [self.line1, self.line2]:
            line.set_data([], [])
        self.canvas.draw()

    def _on_close(self):
        self._on_disconnect()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    ToFGUI(root)
    root.mainloop()
