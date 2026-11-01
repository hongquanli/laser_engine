#! /usr/bin/env python3
# coding=utf-8

"""
 description:   Tkinter status panel for the laser engine. Shows the same data
                the terminal script prints, one row per channel, with
                per-line Wake and Sleep buttons.
 create date:   2026-08-14
 version:       1.1.0

 Reuses TeensyController from pc-side-python.py so the wire protocol
 (framing, CRC, commands) lives in exactly one place.
"""

import importlib.util
import os
import queue
import threading
import tkinter as tk
from tkinter import ttk, messagebox

# pc-side-python.py is not a valid module name (hyphens), so load it by path.
_HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location(
    "pc_side_python", os.path.join(_HERE, "pc-side-python.py"))
pcs = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(pcs)

# Firmware reports 6 thermal channels for 5 laser lines. Channel 4 is the 555nm
# NECSEL diode and channel 5 is its PPLN doubling crystal oven - the two halves
# of the same line, which is why a 'W' or 'S' addressed to 4 wakes or sleeps
# both (laser_engine.ino:736-767) and why they share TTL index 4.
CHANNEL_LINES = {0: '405', 1: '470', 2: '638', 3: '730', 4: '55x', 5: 'PPLN'}
CHANNEL_NOTES = {4: 'NECSEL 555', 5: 'oven for 555'}
LINE_KEY = {0: '405', 1: '470', 2: '638', 3: '730', 4: '55x', 5: None}
TTL_INDEX = {0: 0, 1: 1, 2: 2, 3: 3, 4: 4, 5: 4}

STATE_COLORS = {
    'ACTIVE':        ('#0b6b2f', '#d7f5e1'),
    'CHECK_ACTIVE':  ('#7a5b00', '#fdf3d0'),
    'WARMING_UP':    ('#7a5b00', '#fdf3d0'),
    'WAKE_UP':       ('#7a5b00', '#fdf3d0'),
    'SLEEP':         ('#4a4a4a', '#e8e8e8'),
    'PREPARE_SLEEP': ('#4a4a4a', '#e8e8e8'),
    'CHECK_ERROR':   ('#8a1c1c', '#fbdcdc'),
    'ERROR':         ('#8a1c1c', '#fbdcdc'),
}


class LaserEngineGUI:
    def __init__(self, root, controller):
        self.root = root
        self.controller = controller
        self.queue = queue.Queue()
        self.rows = {}
        self.ttl = [False] * 5          # laser_ttl from the newest packet

        root.title("Laser Engine")

        # Status packets arrive on the serial thread; hand them to Tk via a queue.
        controller.status_callback = self.queue.put

        self._build_header()
        self._build_table()
        self._build_log()

        root.protocol("WM_DELETE_WINDOW", self.on_close)

        # Size to whatever the content actually needs. A hardcoded width clips
        # the Wake column on high-DPI displays, where Tk renders the same
        # layout wider.
        root.update_idletasks()
        root.minsize(root.winfo_reqwidth(), root.winfo_reqheight())

        self.root.after(100, self.drain_queue)

    def _build_header(self):
        bar = ttk.Frame(self.root, padding=(12, 10, 12, 6))
        bar.pack(fill='x')

        port = getattr(self.controller, 'port', '?')
        sn = pcs.USBSN or '-'
        ttk.Label(bar, text=f"SN {sn}  on  {port}",
                  font=('Segoe UI', 11, 'bold')).pack(side='left')

        self.conn_var = tk.StringVar(value="polling")
        ttk.Label(bar, textvariable=self.conn_var,
                  foreground='#0b6b2f').pack(side='left', padx=(14, 0))

        self.updated_var = tk.StringVar(value="waiting for first packet...")
        ttk.Label(bar, textvariable=self.updated_var,
                  foreground='#555').pack(side='right')

        self.crc_var = tk.StringVar(value="CRC mismatches: 0")
        ttk.Label(bar, textvariable=self.crc_var,
                  foreground='#555').pack(side='right', padx=(0, 18))

    def _build_table(self):
        wrap = ttk.Frame(self.root, padding=(12, 0, 12, 8))
        wrap.pack(fill='both', expand=True)

        headers = ["Ch", "Line", "", "State", "Temp", "Diff", "Hi-T limit",
                   "TEC V", "TEC A", "TTL", "", ""]
        widths = [4, 8, 13, 15, 10, 9, 10, 9, 9, 6, 10, 10]
        for col, (text, w) in enumerate(zip(headers, widths)):
            ttk.Label(wrap, text=text, width=w,
                      font=('Segoe UI', 9, 'bold')).grid(row=0, column=col,
                                                         sticky='w', padx=3, pady=(0, 4))

        ttk.Separator(wrap, orient='horizontal').grid(
            row=1, column=0, columnspan=len(headers), sticky='ew', pady=(0, 4))

        for ch in range(6):
            r = ch + 2
            ttk.Label(wrap, text=str(ch)).grid(row=r, column=0, sticky='w', padx=3, pady=2)
            ttk.Label(wrap, text=CHANNEL_LINES[ch], font=('Segoe UI', 9, 'bold')).grid(
                row=r, column=1, sticky='w', padx=3)
            ttk.Label(wrap, text=CHANNEL_NOTES.get(ch, ''), foreground='#888').grid(
                row=r, column=2, sticky='w', padx=3)

            state = tk.Label(wrap, text="--", width=14, anchor='center',
                             font=('Segoe UI', 9, 'bold'), relief='flat', padx=4, pady=2)
            state.grid(row=r, column=3, sticky='w', padx=3)

            cells = {}
            for col, key in [(4, 'temp'), (5, 'diff'), (6, 'hi_temp'),
                             (7, 'volt'), (8, 'amp'), (9, 'ttl')]:
                var = tk.StringVar(value="--")
                lbl = ttk.Label(wrap, textvariable=var, font=('Consolas', 9))
                lbl.grid(row=r, column=col, sticky='w', padx=3)
                cells[key] = (var, lbl)

            if LINE_KEY[ch] is not None:
                ttk.Button(wrap, text="Wake", width=8,
                           command=lambda c=ch: self.wake(c)).grid(
                    row=r, column=10, padx=3)
                ttk.Button(wrap, text="Sleep", width=8,
                           command=lambda c=ch: self.sleep(c)).grid(
                    row=r, column=11, padx=3)
            else:
                ttk.Label(wrap, text="follows 55x", foreground='#888').grid(
                    row=r, column=10, columnspan=2, padx=3, sticky='w')

            self.rows[ch] = {'state': state, 'cells': cells}

        note = ("Wake starts TEC warm-up only - the diode is enabled by firmware "
                "once the channel reaches ACTIVE and its TTL is asserted. "
                "Sleep cuts the diode and switches the TEC off; PREPARE_SLEEP can "
                "take up to ~15 s to reach SLEEP.")
        ttk.Label(wrap, text=note, foreground='#666', wraplength=820).grid(
            row=9, column=0, columnspan=12, sticky='w', padx=3, pady=(10, 0))

    def _build_log(self):
        frame = ttk.Frame(self.root, padding=(12, 0, 12, 10))
        frame.pack(fill='x')
        self.log = tk.Text(frame, height=6, font=('Consolas', 8),
                           background='#f6f6f6', relief='flat', wrap='none')
        self.log.pack(side='left', fill='both', expand=True)
        sb = ttk.Scrollbar(frame, command=self.log.yview)
        sb.pack(side='right', fill='y')
        self.log.configure(yscrollcommand=sb.set, state='disabled')

    def log_line(self, text):
        self.log.configure(state='normal')
        self.log.insert('end', text + "\n")
        self.log.see('end')
        self.log.configure(state='disabled')

    def wake(self, ch):
        line = LINE_KEY[ch]
        current = self.rows[ch]['state']['text']
        if current in ('ACTIVE', 'WARMING_UP', 'CHECK_ACTIVE', 'WAKE_UP'):
            # Re-waking an awake channel forces it back through WARMING_UP.
            if not messagebox.askyesno(
                    "Already awake",
                    f"Line {line} is {current}.\n\n"
                    "Waking it again forces it back to WARMING_UP and it will be "
                    "unavailable until it re-stabilises.\n\nSend wake anyway?"):
                return
        try:
            self.controller.wake_up(line)
            self.log_line(f"-> WAKE {line} (ch{ch}, was {current})")
        except Exception as e:
            self.log_line(f"!! wake {line} failed: {e}")

    def sleep(self, ch):
        line = LINE_KEY[ch]
        current = self.rows[ch]['state']['text']
        if current in ('SLEEP', 'PREPARE_SLEEP'):
            self.log_line(f"-- {line} is already {current}, sleep not sent")
            return
        if self.ttl[TTL_INDEX[ch]]:
            # Firmware wakes a SLEEP channel whose TTL is high
            # (laser_engine.ino:1261-1266), so this only forces a re-warm.
            if not messagebox.askyesno(
                    "TTL is on",
                    f"Line {line} has its TTL asserted, so something is using it.\n\n"
                    "Sleeping cuts the diode and switches the TEC off, but firmware "
                    "wakes the line again as soon as it reaches SLEEP while TTL is "
                    "high. It will be unavailable until it re-stabilises.\n\n"
                    "Send sleep anyway?"):
                return
        try:
            self.controller.put_to_sleep(line)
            self.log_line(f"-> SLEEP {line} (ch{ch}, was {current})")
        except Exception as e:
            self.log_line(f"!! sleep {line} failed: {e}")

    def drain_queue(self):
        status = None
        try:
            while True:                      # keep only the newest packet
                status = self.queue.get_nowait()
        except queue.Empty:
            pass
        if status is not None:
            self.apply_status(status)
        self.root.after(100, self.drain_queue)

    def apply_status(self, status):
        ttl = status['laser_ttl']
        self.ttl = ttl
        for ch, data in enumerate(status['channels']):
            row = self.rows[ch]
            state = data['state_str']

            fg, bg = STATE_COLORS.get(state, ('#000', '#fff'))
            row['state'].configure(text=state, foreground=fg, background=bg)

            on = ttl[TTL_INDEX[ch]]
            values = {
                'temp': f"{data['temp']:.2f}°C",
                'diff': f"{data['diff_temp']:+.2f}",
                'hi_temp': f"{data['hi_temp_setpoint']:.2f}°C",
                'volt': f"{data['tec_voltage']:.2f}",
                'amp': f"{data['tec_current']:.2f}",
                'ttl': "ON" if on else "off",
            }
            for key, text in values.items():
                var, lbl = row['cells'][key]
                var.set(text)
            row['cells']['ttl'][1].configure(
                foreground='#0b6b2f' if on else '#999')

        self.crc_var.set(f"CRC mismatches: {self.controller.crc_mismatch}")
        self.updated_var.set("updated " + _clock())

    def on_close(self):
        self.conn_var.set("stopping...")
        self.root.update_idletasks()
        try:
            self.controller.stop()
        except Exception:
            pass
        self.root.destroy()


def _clock():
    import datetime
    return datetime.datetime.now().strftime("%H:%M:%S")


def main():
    try:
        controller = pcs.TeensyController(device=pcs.DEVICE, SN=pcs.USBSN)
    except Exception as e:
        root = tk.Tk()
        root.withdraw()
        messagebox.showerror("Laser Engine - connection failed", str(e))
        return

    # Per-monitor DPI awareness: without it Windows bitmap-stretches the window
    # and the text renders blurry on scaled displays.
    try:
        import ctypes
        ctypes.windll.shcore.SetProcessDpiAwareness(1)
    except Exception:
        pass

    root = tk.Tk()
    try:
        ttk.Style().theme_use('vista')
    except tk.TclError:
        pass

    gui = LaserEngineGUI(root, controller)
    controller.start()
    gui.log_line(f"Connected: SN {pcs.USBSN} on {controller.port}")
    root.mainloop()


if __name__ == "__main__":
    main()
