#!/usr/bin/env python3
"""
GUI OSC Sender for Raspberry Pi Show Daemon

- Ping the Raspberry Pi to check reachability
- Send /show/start and /show/stop OSC messages
- Show status and log messages in a small GUI

Defaults:
  Hosts: 10.213.190.1,10.213.190.2,10.213.190.3  (comma-separated)
  Port: 9000
  Start path: /show/start
  Stop  path: /show/stop
"""

import sys
import threading
import subprocess
import tkinter as tk
from tkinter import ttk, messagebox

from pythonosc.udp_client import SimpleUDPClient

DEFAULT_HOSTS = "10.213.190.1"  # you can change this to a comma-separated list
DEFAULT_PORT = 9000
DEFAULT_START_PATH = "/show/start"
DEFAULT_STOP_PATH  = "/show/stop"


class ShowSenderGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("RPi Show OSC Sender")

        # Now this can contain comma-separated IPs
        self.host_var = tk.StringVar(value=DEFAULT_HOSTS)
        self.port_var = tk.IntVar(value=DEFAULT_PORT)
        self.start_path_var = tk.StringVar(value=DEFAULT_START_PATH)
        self.stop_path_var = tk.StringVar(value=DEFAULT_STOP_PATH)
        self.value_var = tk.DoubleVar(value=1.0)
        self.status_var = tk.StringVar(value="Idle")

        self._build_ui()

    # ---------- UI ----------
    def _build_ui(self):
        frm = ttk.Frame(self.root, padding=10)
        frm.pack(fill=tk.BOTH, expand=True)

        # Connection settings
        row = 0
        ttk.Label(frm, text="RPi Host(s):").grid(row=row, column=0, sticky=tk.W)
        ttk.Entry(frm, textvariable=self.host_var, width=40).grid(
            row=row, column=1, sticky=tk.W, padx=4
        )

        ttk.Label(frm, text="Port:").grid(row=row, column=2, sticky=tk.W, padx=(16, 0))
        ttk.Entry(frm, textvariable=self.port_var, width=6).grid(row=row, column=3, sticky=tk.W)

        row += 1
        ttk.Label(frm, text="Start path:").grid(row=row, column=0, sticky=tk.W, pady=(6, 0))
        ttk.Entry(frm, textvariable=self.start_path_var, width=20).grid(
            row=row, column=1, sticky=tk.W, padx=4, pady=(6, 0)
        )

        ttk.Label(frm, text="Stop path:").grid(
            row=row, column=2, sticky=tk.W, padx=(16, 0), pady=(6, 0)
        )
        ttk.Entry(frm, textvariable=self.stop_path_var, width=20).grid(
            row=row, column=3, sticky=tk.W, pady=(6, 0)
        )

        row += 1
        ttk.Label(frm, text="Value:").grid(row=row, column=0, sticky=tk.W, pady=(6, 0))
        ttk.Entry(frm, textvariable=self.value_var, width=8).grid(
            row=row, column=1, sticky=tk.W, padx=4, pady=(6, 0)
        )

        # Buttons
        row += 1
        btn_frame = ttk.Frame(frm)
        btn_frame.grid(row=row, column=0, columnspan=4, sticky=tk.W, pady=(10, 4))

        ttk.Button(btn_frame, text="Ping RPi", command=self.on_ping).pack(side=tk.LEFT)
        ttk.Button(btn_frame, text="Send START", command=self.on_start).pack(
            side=tk.LEFT, padx=6
        )
        ttk.Button(btn_frame, text="Send STOP", command=self.on_stop).pack(side=tk.LEFT)

        # Status
        row += 1
        ttk.Label(frm, text="Status:").grid(row=row, column=0, sticky=tk.W, pady=(8, 0))
        ttk.Label(frm, textvariable=self.status_var, foreground="blue").grid(
            row=row, column=1, columnspan=3, sticky=tk.W, pady=(8, 0)
        )

        # Log box
        row += 1
        ttk.Label(frm, text="Log:").grid(row=row, column=0, sticky=tk.W, pady=(6, 0))
        self.log_box = tk.Text(frm, height=12, width=70)
        self.log_box.grid(row=row + 1, column=0, columnspan=4, sticky=tk.NSEW, pady=(2, 0))

        # Make log area expandable
        frm.rowconfigure(row + 1, weight=1)
        frm.columnconfigure(1, weight=1)
        frm.columnconfigure(3, weight=1)

    # ---------- Logging / status helpers ----------
    def set_status(self, text):
        self.status_var.set(text)

    def append_log(self, text):
        self.log_box.insert(tk.END, text + "\n")
        self.log_box.see(tk.END)

    # ---------- Helpers ----------
    def _get_hosts_list(self):
        """
        Parse comma-separated hosts into a clean list.
        """
        raw = self.host_var.get().strip()
        hosts = [h.strip() for h in raw.split(",") if h.strip()]
        return hosts

    # ---------- Actions ----------
    def on_ping(self):
        hosts = self._get_hosts_list()
        if not hosts:
            messagebox.showerror("Error", "Host cannot be empty")
            return

        # For simplicity, just ping the first host
        host = hosts[0]
        self.set_status(f"Pinging {host}...")
        threading.Thread(target=self._ping_thread, args=(host,), daemon=True).start()

    def _ping_thread(self, host):
        ok, output = ping_host(host)

        def update():
            if ok:
                self.set_status(f"Ping OK: {host}")
                self.append_log(f"[PING OK] {host}")
            else:
                self.set_status(f"Ping FAILED: {host}")
                self.append_log(f"[PING FAILED] {host}\n{output.strip()}")

        self.root.after(0, update)

    def on_start(self):
        self._send_osc(self.start_path_var.get().strip(), "START")

    def on_stop(self):
        self._send_osc(self.stop_path_var.get().strip(), "STOP")

    def _send_osc(self, path, label):
        hosts = self._get_hosts_list()
        port = self.port_var.get()
        value = self.value_var.get()

        if not hosts:
            messagebox.showerror("Error", "Host cannot be empty")
            return
        if not path:
            messagebox.showerror("Error", "Path cannot be empty")
            return

        host_list_str = ", ".join(hosts)
        self.set_status(f"Sending {label} to [{host_list_str}]:{port} {path}...")
        threading.Thread(
            target=self._send_osc_thread_multi,
            args=(hosts, port, path, value, label),
            daemon=True,
        ).start()

    def _send_osc_thread_multi(self, hosts, port, path, value, label):
        errors = []

        for host in hosts:
            try:
                client = SimpleUDPClient(host, port)
                client.send_message(path, value)
            except Exception as e:
                errors.append((host, e))

        def update():
            if not errors:
                self.set_status(f"{label} sent to {len(hosts)} host(s)")
                for host in hosts:
                    self.append_log(
                        f"[OSC {label}] host={host} port={port} path={path} value={value}"
                    )
            else:
                self.set_status(f"{label} partially FAILED")
                for host, e in errors:
                    self.append_log(f"[OSC {label} FAILED] host={host} error={e}")
                messagebox.showerror(
                    "OSC Error",
                    "Some hosts failed:\n"
                    + "\n".join(f"{host}: {err}" for host, err in errors),
                )

        self.root.after(0, update)


# ---------- Ping helper ----------
def ping_host(host: str):
    """
    Return (ok: bool, output: str) using system ping.
    """
    if sys.platform.startswith("win"):
        cmd = ["ping", "-n", "1", "-w", "1000", host]
    else:
        cmd = ["ping", "-c", "1", "-W", "1", host]

    try:
        proc = subprocess.run(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
        )
        ok = proc.returncode == 0
        return ok, proc.stdout
    except Exception as e:
        return False, str(e)


# ---------- Main ----------
if __name__ == "__main__":
    root = tk.Tk()
    app = ShowSenderGUI(root)
    root.mainloop()
