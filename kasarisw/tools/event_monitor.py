import socket
import struct
import tkinter as tk
from tkinter import ttk
import threading
import time
from collections import defaultdict
import queue
from parse_events import parse_event

# Shared queues for events and status (thread-safe)
event_queue = queue.Queue()
status_queue = queue.Queue()

# GUI class
class SensorGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Sensor Data Monitor")
        self.root.geometry("1000x400")
        
        # Make the root grid expandable
        self.root.rowconfigure(0, weight=1)
        self.root.columnconfigure(0, weight=1)
        
        # Dictionary to hold latest data
        self.latest_data = defaultdict(lambda: "No data yet")
        
        # Connection management
        self.host_port = "192.168.1.248:8080"  # Default host:port
        self.running = False
        self.thread = None
        self.sock = None
        
        # Create widgets
        self.create_widgets()
        
        # Bind resize event
        self.frame.bind("<Configure>", self.on_resize)
        
        # Start updating GUI
        self.update_gui()

        # Connect at startup
        self.connect()

    def create_widgets(self):
        self.frame = ttk.Frame(self.root, padding="10")
        self.frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Make frame columns expandable if needed, but primarily rely on wrap
        self.frame.columnconfigure(0, weight=1)
        self.frame.columnconfigure(1, weight=1)
        self.frame.columnconfigure(2, weight=1)
        
        # Connection controls
        ttk.Label(self.frame, text="Host:Port:").grid(row=0, column=0, sticky=tk.W)
        self.host_entry = ttk.Entry(self.frame)
        self.host_entry.insert(0, "192.168.1.248:8080")
        self.host_entry.grid(row=0, column=1, sticky=tk.W)
        
        self.connect_button = ttk.Button(self.frame, text="Connect", command=self.toggle_connect)
        self.connect_button.grid(row=0, column=2, sticky=tk.W)
        
        self.status_label = ttk.Label(self.frame, text="Status: Disconnected")
        self.status_label.grid(row=1, column=0, columnspan=3, sticky=tk.W)
        
        # Lidar section
        ttk.Label(self.frame, text="Lidar:").grid(row=2, column=0, sticky=tk.W)
        self.lidar_label = ttk.Label(self.frame, text=self.format_lidar())
        self.lidar_label.grid(row=3, column=0, columnspan=3, sticky=tk.W)
        
        # Accelerometer section
        ttk.Label(self.frame, text="Accelerometer:").grid(row=4, column=0, sticky=tk.W)
        self.accel_label = ttk.Label(self.frame, text=self.format_accel())
        self.accel_label.grid(row=5, column=0, columnspan=3, sticky=tk.W)
        
        # Receiver section (latest one, regardless of channel)
        ttk.Label(self.frame, text="Receiver:").grid(row=6, column=0, sticky=tk.W)
        self.receiver_label = ttk.Label(self.frame, text=self.format_receiver())
        self.receiver_label.grid(row=7, column=0, columnspan=3, sticky=tk.W)
        
        # Vbat section
        ttk.Label(self.frame, text="Vbat:").grid(row=8, column=0, sticky=tk.W)
        self.vbat_label = ttk.Label(self.frame, text=self.format_vbat())
        self.vbat_label.grid(row=9, column=0, columnspan=3, sticky=tk.W)
        
        # Command section
        ttk.Label(self.frame, text="Command:").grid(row=10, column=0, sticky=tk.W)
        self.command_entry = ttk.Entry(self.frame)
        self.command_entry.grid(row=10, column=1, sticky=tk.W)
        self.send_button = ttk.Button(self.frame, text="Send", command=self.on_send)
        self.send_button.grid(row=10, column=2, sticky=tk.W)

    def on_resize(self, event):
        # Update wraplength based on current frame width
        width = event.width - 40  # Subtract padding
        if width > 0:
            self.status_label.config(wraplength=width)
            self.lidar_label.config(wraplength=width)
            self.accel_label.config(wraplength=width)
            self.receiver_label.config(wraplength=width)
            self.vbat_label.config(wraplength=width)

    def toggle_connect(self):
        if self.connect_button.cget("text") == "Connect":
            self.connect()
        else:
            self.disconnect()

    def connect(self):
        self.host_port = self.host_entry.get()
        self.host, port_str = self.host_port.split(':')
        self.port = int(port_str)
        self.status_label.config(text="Status: Connecting...")
        status_queue.put("Connecting...")
        self.running = True
        self.thread = threading.Thread(target=self.streaming, daemon=True)
        self.thread.start()
        self.connect_button.config(text="Disconnect")

    def disconnect(self):
        self.running = False
        if self.sock:
            try:
                self.sock.close()
            except:
                pass
            self.sock = None
        self.status_label.config(text="Status: Disconnected")
        status_queue.put("Disconnected")
        self.connect_button.config(text="Connect")

    def streaming(self):
        while self.running:
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(10)
                self.sock.connect((self.host, self.port))
                self.sock.settimeout(None)
                status_queue.put("Connected")
                buffer = b''
                while self.running:
                    chunk = self.sock.recv(1024)
                    if not chunk:
                        raise EOFError("Connection closed by server")
                    buffer += chunk
                    while len(buffer) >= 1:
                        old_len = len(buffer)
                        event, buffer = parse_event(buffer)
                        if event is not None:
                            event_queue.put(event)
                        else:
                            if len(buffer) >= old_len:
                                break
            except Exception as e:
                if self.running:
                    status_queue.put(f"Error: {str(e)} - Retrying in 5 seconds...")
                    time.sleep(5)
            finally:
                if self.sock:
                    try:
                        self.sock.close()
                    except:
                        pass
                    self.sock = None

    def format_lidar(self):
        if self.latest_data['Lidar'] == "No data yet":
            return "No data yet"
        ts, d1, d2, d3, d4 = self.latest_data['Lidar']
        return f"Timestamp: {ts}, d1: {d1:.2f}, d2: {d2:.2f}, d3: {d3:.2f}, d4: {d4:.2f}"

    def format_accel(self):
        if self.latest_data['Accelerometer'] == "No data yet":
            return "No data yet"
        ts, accel_y, accel_z = self.latest_data['Accelerometer']
        return f"Timestamp: {ts}, Acceleration Y: {accel_y:.2f}, Z: {accel_z:.2f}"

    def format_receiver(self):
        if self.latest_data['Receiver'] == "No data yet":
            return "No data yet"
        ts, channel, value = self.latest_data['Receiver']
        if value is None:
            return f"Timestamp: {ts}, Channel: {channel}, Flag: 0 (None)"
        else:
            return f"Timestamp: {ts}, Channel: {channel}, Pulse Length: {value:.2f}"

    def format_vbat(self):
        if self.latest_data['Vbat'] == "No data yet":
            return "No data yet"
        ts, voltage = self.latest_data['Vbat']
        return f"Timestamp: {ts}, Voltage: {voltage:.2f}"

    def on_send(self):
        cmd = self.command_entry.get()
        if cmd and self.sock:
            try:
                self.sock.send(cmd.encode() + b'\n')
                self.command_entry.delete(0, tk.END)
            except Exception as e:
                self.status_label.config(text=f"Send error: {e}")

    def update_gui(self):
        # Process all queued events
        while not event_queue.empty():
            event = event_queue.get()
            sensor_type = event[0]
            if sensor_type == 'Lidar':
                self.latest_data['Lidar'] = event[1:]
            elif sensor_type == 'Accelerometer':
                self.latest_data['Accelerometer'] = event[1:]
            elif sensor_type == 'Receiver':
                self.latest_data['Receiver'] = event[1:]
            elif sensor_type == 'Vbat':
                self.latest_data['Vbat'] = event[1:]
        
        # Process status updates
        while not status_queue.empty():
            msg = status_queue.get()
            self.status_label.config(text=f"Status: {msg}")
        
        # Update sensor labels
        self.lidar_label.config(text=self.format_lidar())
        self.accel_label.config(text=self.format_accel())
        self.receiver_label.config(text=self.format_receiver())
        self.vbat_label.config(text=self.format_vbat())
        
        # Schedule next update
        self.root.after(100, self.update_gui)

# Main entry point
if __name__ == "__main__":
    root = tk.Tk()
    app = SensorGUI(root)
    root.mainloop()
