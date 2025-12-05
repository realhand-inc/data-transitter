import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox
from tkinter import scrolledtext

from test_adb_simple import ADB_PACKAGE_NAME, execute_adb_command, get_connected_adb_devices


DEVICE_REFRESH_MS = 5000


class AdbControlApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("RealHand TeleOp")
        self.devices = []
        self.status_var = tk.StringVar(value="Idle")
        self.ip_var = tk.StringVar()
        self.actions_window = None  # Lazy-created extra actions menu

        main = ttk.Frame(root, padding=12)
        main.grid(column=0, row=0, sticky="nsew")
        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)

        title = ttk.Label(main, text="RealHand TeleOp", font=("TkDefaultFont", 16, "bold"))
        title.grid(column=0, row=0, padx=8, pady=(0, 8), sticky="w")

        self.build_devices_frame(main)
        self.build_actions_frame(main)
        self.build_log_frame(main)

        self.refresh_devices()
        self.schedule_refresh()

    def build_devices_frame(self, parent: ttk.Frame):
        frame = ttk.LabelFrame(parent, text="Devices")
        frame.grid(column=0, row=1, padx=8, pady=4, sticky="nsew")
        parent.columnconfigure(0, weight=1)
        parent.rowconfigure(1, weight=1)

        self.devices_list = tk.Listbox(frame, height=8, selectmode=tk.MULTIPLE, exportselection=False)
        self.devices_list.grid(column=0, row=0, rowspan=4, padx=(8, 4), pady=8, sticky="nsew")
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(3, weight=1)

        refresh_btn = ttk.Button(frame, text="Refresh", command=self.refresh_devices)
        refresh_btn.grid(column=1, row=0, padx=4, pady=(8, 2), sticky="ew")

        ttk.Label(frame, text="IP:Port").grid(column=1, row=1, padx=4, pady=2, sticky="w")
        ip_entry = ttk.Entry(frame, textvariable=self.ip_var, width=18)
        ip_entry.grid(column=1, row=2, padx=4, pady=2, sticky="ew")
        connect_btn = ttk.Button(frame, text="Connect over IP", command=self.connect_ip)
        connect_btn.grid(column=1, row=3, padx=4, pady=(2, 8), sticky="ew")

        self.status_label = ttk.Label(frame, textvariable=self.status_var, foreground="green")
        self.status_label.grid(column=0, row=4, columnspan=2, padx=8, pady=(0, 8), sticky="w")

    def build_actions_frame(self, parent: ttk.Frame):
        frame = ttk.LabelFrame(parent, text="Actions")
        frame.grid(column=0, row=2, padx=8, pady=4, sticky="ew")

        open_btn = ttk.Button(frame, text="Open App", command=self.open_app)
        open_btn.grid(column=0, row=0, padx=6, pady=6, sticky="ew")

        stop_btn = ttk.Button(frame, text="Stop App", command=self.stop_app)
        stop_btn.grid(column=1, row=0, padx=6, pady=6, sticky="ew")

        restart_btn = ttk.Button(frame, text="Restart App", command=self.restart_app)
        restart_btn.grid(column=2, row=0, padx=6, pady=6, sticky="ew")

        disconnect_btn = ttk.Button(frame, text="Disconnect All", command=self.disconnect_all)
        disconnect_btn.grid(column=3, row=0, padx=6, pady=6, sticky="ew")

        more_btn = ttk.Button(frame, text="More ADB Actions", command=self.open_actions_menu)
        more_btn.grid(column=4, row=0, padx=6, pady=6, sticky="ew")

    def build_log_frame(self, parent: ttk.Frame):
        frame = ttk.LabelFrame(parent, text="Log")
        frame.grid(column=0, row=3, padx=8, pady=4, sticky="nsew")
        parent.rowconfigure(3, weight=1)

        self.log_widget = scrolledtext.ScrolledText(frame, wrap=tk.WORD, height=12, state="disabled")
        self.log_widget.grid(column=0, row=0, padx=8, pady=8, sticky="nsew")
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(0, weight=1)

    def schedule_refresh(self):
        self.root.after(DEVICE_REFRESH_MS, self.periodic_refresh)

    def periodic_refresh(self):
        self.refresh_devices()
        self.schedule_refresh()

    def set_status(self, text: str):
        self.status_var.set(text)

    def log(self, message: str):
        timestamp = time.strftime("%H:%M:%S")
        def _append():
            self.log_widget.configure(state="normal")
            self.log_widget.insert(tk.END, f"[{timestamp}] {message}\n")
            self.log_widget.see(tk.END)
            self.log_widget.configure(state="disabled")
        self.root.after(0, _append)

    def refresh_devices(self):
        self.set_status("Refreshing devices...")
        devices = get_connected_adb_devices()
        self.devices = devices
        self.devices_list.delete(0, tk.END)
        for dev in devices:
            self.devices_list.insert(tk.END, dev)
        if devices:
            self.set_status(f"Found {len(devices)} device(s)")
        else:
            self.set_status("No devices found")

    def selected_devices(self):
        indices = self.devices_list.curselection()
        if indices:
            return [self.devices[i] for i in indices]
        return self.devices[:1]  # Default to first device if none selected

    def connect_ip(self):
        ip = self.ip_var.get().strip()
        if not ip:
            messagebox.showinfo("ADB Control", "Enter an IP (with optional port) before connecting.")
            return

        def _connect():
            self.set_status(f"Connecting to {ip}...")
            success, output = execute_adb_command(f"adb connect {ip}")
            self.log(f"connect {ip}: {'ok' if success else 'failed'} | {output.strip()}")
            self.refresh_devices()
            self.set_status("Idle")

        threading.Thread(target=_connect, daemon=True).start()

    def disconnect_all(self):
        def _disconnect():
            self.set_status("Disconnecting all devices...")
            success, output = execute_adb_command("adb disconnect")
            self.log(f"disconnect all: {'ok' if success else 'failed'} | {output.strip()}")
            self.refresh_devices()
            self.set_status("Idle")

        threading.Thread(target=_disconnect, daemon=True).start()

    def open_app(self):
        self.run_for_devices("open app", self._open_app_on_device)

    def stop_app(self):
        self.run_for_devices("stop app", self._stop_app_on_device)

    def restart_app(self):
        self.run_for_devices("restart app", self._restart_app_on_device)

    def run_for_devices(self, label: str, action):
        devices = self.selected_devices()
        if not devices:
            self.log(f"{label}: no connected devices")
            self.set_status("No devices")
            return

        for device in devices:
            threading.Thread(target=self._run_action, args=(device, label, action), daemon=True).start()

    def _run_action(self, device: str, label: str, action):
        self.set_status(f"{label} on {device}")
        try:
            action(device)
        finally:
            self.set_status("Idle")

    def _open_app_on_device(self, device: str):
        cmd = f"adb -s {device} shell monkey -p {ADB_PACKAGE_NAME} -c android.intent.category.LAUNCHER 1"
        success, output = execute_adb_command(cmd)
        self.log(f"[{device}] open app: {'ok' if success else 'failed'} | {output.strip()}")

    def _stop_app_on_device(self, device: str):
        cmd = f"adb -s {device} shell am force-stop {ADB_PACKAGE_NAME}"
        success, output = execute_adb_command(cmd)
        self.log(f"[{device}] stop app: {'ok' if success else 'failed'} | {output.strip()}")

    def _restart_app_on_device(self, device: str):
        self._stop_app_on_device(device)
        time.sleep(1)
        self._open_app_on_device(device)

    def _reboot_device(self, device: str):
        cmd = f"adb -s {device} reboot"
        success, output = execute_adb_command(cmd)
        self.log(f"[{device}] reboot: {'ok' if success else 'failed'} | {output.strip()}")

    def _power_off_device(self, device: str):
        cmd = f"adb -s {device} shell reboot -p"
        success, output = execute_adb_command(cmd)
        self.log(f"[{device}] power off: {'ok' if success else 'failed'} | {output.strip()}")

    def open_actions_menu(self):
        if self.actions_window and tk.Toplevel.winfo_exists(self.actions_window):
            self.actions_window.lift()
            return

        self.actions_window = tk.Toplevel(self.root)
        self.actions_window.title("ADB Extra Actions")
        self.actions_window.resizable(False, False)

        ttk.Label(self.actions_window, text="Extra ADB Actions", font=("TkDefaultFont", 12, "bold")).grid(column=0, row=0, columnspan=2, padx=12, pady=(12, 6))

        reboot_btn = ttk.Button(self.actions_window, text="Reboot Device(s)", command=lambda: self.run_for_devices("reboot", self._reboot_device))
        reboot_btn.grid(column=0, row=1, padx=10, pady=6, sticky="ew")

        power_btn = ttk.Button(self.actions_window, text="Power Off Device(s)", command=lambda: self.run_for_devices("power off", self._power_off_device))
        power_btn.grid(column=1, row=1, padx=10, pady=6, sticky="ew")

        ttk.Label(self.actions_window, text="Uses selected devices, or the first device if none selected.").grid(column=0, row=2, columnspan=2, padx=12, pady=(6, 12))

        for i in range(2):
            self.actions_window.columnconfigure(i, weight=1)


def main():
    root = tk.Tk()
    app = AdbControlApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
