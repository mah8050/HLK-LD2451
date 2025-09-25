import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import serial
import serial.tools.list_ports
import threading
import time
import struct
import math
import queue

# --- Constants ---
# Old Data Frame constants (from original implementation)
OLD_FRAME_HEADER = b'\x53\x59'
OLD_FRAME_TAIL = b'\x54\x43'
OLD_CMD_MOVING_TARGET = 0x01

# New Target Data Frame constants (from user feedback)
TARGET_FRAME_HEADER = b'\xF4\xF3\xF2\xF1'
TARGET_FRAME_TAIL = b'\xF8\xF7\xF6\xF5'

# Configuration command constants
CONFIG_HEADER = b'\xFD\xFC\xFB\xFA'
CONFIG_TAIL = b'\x04\x03\x02\x01'
CONFIG_CMD_ENABLE = 0x00FF
CONFIG_CMD_DISABLE = 0x00FE
CONFIG_CMD_ENABLE_ACK = 0x01FF
CONFIG_CMD_DISABLE_ACK = 0x01FE

# Write Commands
CONFIG_CMD_SET_TARGET_PARAMS = 0x0002
CONFIG_CMD_SET_SENSITIVITY = 0x0003
CONFIG_CMD_RESTART = 0x00A3

# Read Commands
CONFIG_CMD_READ_TARGET_PARAMS = 0x0012
CONFIG_CMD_READ_SENSITIVITY = 0x0013

# Acknowledgment (Response) Commands from Module
CONFIG_ACK_TARGET_PARAMS = 0x0112
CONFIG_ACK_SENSITIVITY = 0x0113

# Read firmware version
CONFIG_CMD_READ_FIRMWARE_VERSION = 0x00A0
CONFIG_ACK_FIRMWARE_VERSION = 0x01A0

# Canvas settings
CANVAS_WIDTH = 500
CANVAS_HEIGHT = 500
INITIAL_MAX_DISTANCE_M = 100 

class LD2451_GUI:
    """
    Main application class for the HLK-LD2451 GUI.
    Handles the user interface, serial communication, and data processing.
    """
    def __init__(self, root):
        self.root = root
        self.root.title("HLK-LD2451 Radar GUI")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        # --- Member Variables ---
        self.serial_port = None
        self.is_connected = False
        self.read_thread = None
        self.data_queue = queue.Queue()
        self.current_max_dist_m = INITIAL_MAX_DISTANCE_M
        self.config_mode = False  # Track if we're in configuration mode
        self.response_timeout = 2.0  # Timeout for waiting responses

        # --- UI Setup ---
        self.create_widgets()
        self.update_ports()
        self.update_gui_from_queue()

    def create_widgets(self):
        """Creates and arranges all the GUI widgets."""
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        # --- Left Panel: Connection and Configuration ---
        left_panel = ttk.Frame(main_frame, padding="10")
        left_panel.grid(row=0, column=0, sticky=(tk.N, tk.S))
        
        # Connection Frame
        conn_frame = ttk.LabelFrame(left_panel, text="Connection", padding="10")
        conn_frame.grid(row=0, column=0, sticky=(tk.W, tk.E), ipady=5)
        conn_frame.columnconfigure(1, weight=1)

        ttk.Label(conn_frame, text="COM Port:").grid(row=0, column=0, sticky=tk.W, pady=2)
        self.port_combobox = ttk.Combobox(conn_frame, state="readonly")
        self.port_combobox.grid(row=0, column=1, sticky=(tk.W, tk.E), pady=2, padx=5)

        self.refresh_button = ttk.Button(conn_frame, text="Refresh", command=self.update_ports)
        self.refresh_button.grid(row=1, column=0, sticky=tk.W, pady=5)
        
        self.connect_button = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_button.grid(row=1, column=1, sticky=tk.E, pady=5)

        self.status_label = ttk.Label(conn_frame, text="Status: Disconnected", foreground="red")
        self.status_label.grid(row=2, column=0, columnspan=2, sticky=tk.W, pady=5)
        
        # --- Target Detection Parameters Frame ---
        target_frame = ttk.LabelFrame(left_panel, text="Target Detection Parameters", padding="10")
        target_frame.grid(row=1, column=0, sticky=(tk.W, tk.E), pady=10, ipady=5)
        target_frame.columnconfigure(1, weight=1)

        # Max Distance
        self.max_dist_val = tk.IntVar(value=int(INITIAL_MAX_DISTANCE_M)) # in meters
        ttk.Label(target_frame, text="Max Distance (m):").grid(row=0, column=0, sticky=tk.W, pady=2)
        ttk.Scale(target_frame, from_=10, to=100, orient=tk.HORIZONTAL, variable=self.max_dist_val, command=lambda s: self.max_dist_val.set(int(float(s)))).grid(row=0, column=1, sticky=(tk.W, tk.E), padx=5)
        self.max_dist_label = ttk.Label(target_frame, text=f"{INITIAL_MAX_DISTANCE_M}")
        self.max_dist_label.grid(row=0, column=2, padx=2)
        self.max_dist_val.trace_add("write", self.update_dist_label_and_grid)

        ttk.Label(target_frame, text="Direction:").grid(row=1, column=0, sticky=tk.W, pady=2)
        self.direction_var = tk.StringVar(value="Approach & Away")
        direction_cb = ttk.Combobox(target_frame, textvariable=self.direction_var, values=["Away Only", "Approach Only", "Approach & Away"], state="readonly")
        direction_cb.grid(row=1, column=1, columnspan=2, sticky=(tk.W, tk.E), padx=5)
        
        self.min_speed_val = tk.IntVar(value=0) # in Km/h
        ttk.Label(target_frame, text="Min Speed (Km/h):").grid(row=2, column=0, sticky=tk.W, pady=2)
        ttk.Scale(target_frame, from_=0, to=120, orient=tk.HORIZONTAL, variable=self.min_speed_val, command=lambda s: self.min_speed_val.set(int(float(s)))).grid(row=2, column=1, sticky=(tk.W, tk.E), padx=5)
        ttk.Label(target_frame, textvariable=self.min_speed_val).grid(row=2, column=2, padx=2)
        
        self.target_delay_val = tk.IntVar(value=0) # in seconds
        ttk.Label(target_frame, text="Target Delay (s):").grid(row=3, column=0, sticky=tk.W, pady=2)
        ttk.Scale(target_frame, from_=0, to=255, orient=tk.HORIZONTAL, variable=self.target_delay_val, command=lambda s: self.target_delay_val.set(int(float(s)))).grid(row=3, column=1, sticky=(tk.W, tk.E), padx=5)
        ttk.Label(target_frame, textvariable=self.target_delay_val).grid(row=3, column=2, padx=2)

        ttk.Button(target_frame, text="Apply Detection Params", command=self.apply_detection_config).grid(row=4, column=0, columnspan=3, pady=10)
        ttk.Button(target_frame, text="Read Current Params", command=self.read_target_params).grid(row=5, column=0, columnspan=3, pady=5)

        # --- Radar Sensitivity Frame ---
        sensitivity_frame = ttk.LabelFrame(left_panel, text="Radar Sensitivity", padding="10")
        sensitivity_frame.grid(row=2, column=0, sticky=(tk.W, tk.E), pady=10, ipady=5)
        sensitivity_frame.columnconfigure(1, weight=1)
        
        self.trigger_count_val = tk.IntVar(value=0)
        ttk.Label(sensitivity_frame, text="Trigger Count:").grid(row=0, column=0, sticky=tk.W, pady=2)
        ttk.Scale(sensitivity_frame, from_=1, to=10, orient=tk.HORIZONTAL, variable=self.trigger_count_val, command=lambda s: self.trigger_count_val.set(int(float(s)))).grid(row=0, column=1, sticky=(tk.W, tk.E), padx=5)
        ttk.Label(sensitivity_frame, textvariable=self.trigger_count_val).grid(row=0, column=2, padx=2)

        self.snr_val = tk.IntVar(value=0)
        ttk.Label(sensitivity_frame, text="SNR:").grid(row=1, column=0, sticky=tk.W, pady=2)
        ttk.Scale(sensitivity_frame, from_=0, to=8, orient=tk.HORIZONTAL, variable=self.snr_val, command=lambda s: self.snr_val.set(int(float(s)))).grid(row=1, column=1, sticky=(tk.W, tk.E), padx=5)
        ttk.Label(sensitivity_frame, textvariable=self.snr_val).grid(row=1, column=2, padx=2)

        ttk.Button(sensitivity_frame, text="Apply Sensitivity", command=self.apply_sensitivity_config).grid(row=2, column=0, columnspan=3, pady=10)
        ttk.Button(sensitivity_frame, text="Read Current Sensitivity", command=self.read_sensitivity_params).grid(row=3, column=0, columnspan=3, pady=5)

        # --- Module Control Frame ---
        module_frame = ttk.LabelFrame(left_panel, text="Module Control", padding="10")
        module_frame.grid(row=3, column=0, sticky=(tk.W, tk.E), pady=10, ipady=5)
        ttk.Button(module_frame, text="Restart Module", command=self.restart_module).grid(row=0, column=0, pady=5)
        ttk.Button(module_frame, text="Enable Config Mode", command=self.enable_config_mode).grid(row=0, column=1, pady=5, padx=5)
        ttk.Button(module_frame, text="Disable Config Mode", command=self.disable_config_mode).grid(row=1, column=0, columnspan=2, pady=5)

        # Config mode indicator
        self.config_status_label = ttk.Label(module_frame, text="Config Mode: OFF", foreground="red")
        self.config_status_label.grid(row=2, column=0, columnspan=2, pady=5)

        # --- Right Panel: Data Visualization ---
        right_panel = ttk.Frame(main_frame)
        right_panel.grid(row=0, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=10)
        main_frame.columnconfigure(1, weight=1)
        right_panel.rowconfigure(2, weight=1) # Allow log window row to expand

        self.canvas = tk.Canvas(right_panel, width=CANVAS_WIDTH, height=CANVAS_HEIGHT, bg="black")
        self.canvas.grid(row=0, column=0, columnspan=3)
        self.draw_canvas_grid()

        self.target_labels = {}
        data_labels_frame = ttk.Frame(right_panel)
        data_labels_frame.grid(row=1, column=0, columnspan=3, pady=10, sticky=tk.W)
        
        headers = ["Target", "X (m)", "Y (m)", "Speed (km/h)", "Distance (m)", "Angle (°)"]
        for i, header in enumerate(headers):
            ttk.Label(data_labels_frame, text=header, font=('Helvetica', 10, 'bold')).grid(row=0, column=i, padx=10, sticky=tk.W)

        # Updated to handle up to 5 targets in the UI
        for i in range(5):
            self.target_labels[i] = {
                "id": ttk.Label(data_labels_frame, text=f"#{i+1}"), "x": ttk.Label(data_labels_frame, text="---"),
                "y": ttk.Label(data_labels_frame, text="---"), "speed": ttk.Label(data_labels_frame, text="---"),
                "distance": ttk.Label(data_labels_frame, text="---"), "angle": ttk.Label(data_labels_frame, text="---"),
            }
            self.target_labels[i]["id"].grid(row=i+1, column=0, padx=10, sticky=tk.W)
            self.target_labels[i]["x"].grid(row=i+1, column=1, padx=10, sticky=tk.W)
            self.target_labels[i]["y"].grid(row=i+1, column=2, padx=10, sticky=tk.W)
            self.target_labels[i]["speed"].grid(row=i+1, column=3, padx=10, sticky=tk.W)
            self.target_labels[i]["distance"].grid(row=i+1, column=4, padx=10, sticky=tk.W)
            self.target_labels[i]["angle"].grid(row=i+1, column=5, padx=10, sticky=tk.W)
        
        # --- Log Window ---
        log_frame = ttk.LabelFrame(right_panel, text="Communication Log", padding="10")
        log_frame.grid(row=2, column=0, columnspan=3, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(10, 0))
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)

        self.log_text = scrolledtext.ScrolledText(log_frame, height=12, width=50, wrap=tk.WORD, bg="black", fg="white")
        self.log_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S)) # Use grid for better resizing
        
        # Define color tags
        self.log_text.tag_config('sent', foreground='#66b3ff') # Light Blue
        self.log_text.tag_config('recv', foreground='#77dd77') # Light Green
        self.log_text.tag_config('info', foreground='#ffff99') # Light Yellow
        self.log_text.tag_config('error', foreground='#ff6666') # Light Red
        self.log_text.configure(state='disabled')

    def update_dist_label_and_grid(self, *args):
        value_m = self.max_dist_val.get()
        self.current_max_dist_m = value_m
        self.max_dist_label.config(text=f"{value_m}")
        self.draw_canvas_grid()

    def draw_canvas_grid(self):
        self.canvas.delete("grid")
        cx, cy = CANVAS_WIDTH / 2, CANVAS_HEIGHT / 2

        if self.current_max_dist_m <= 0: return

        # Base lines
        self.canvas.create_line(cx, 0, cx, CANVAS_HEIGHT, fill="#404040", tags="grid")
        self.canvas.create_line(0, cy, CANVAS_WIDTH, cy, fill="#404040", tags="grid")
        
        # Determine number of rings based on distance
        max_m = self.current_max_dist_m
        if max_m <= 2: num_rings = 4 # 0.5m intervals
        elif max_m <= 6: num_rings = int(max_m) # 1m intervals
        else: num_rings = 6 # Max 6 rings for clarity
        
        for i in range(1, num_rings + 1):
            dist_m = i * (max_m / num_rings)
            radius_px = (dist_m / max_m) * (CANVAS_WIDTH / 2)
            if radius_px > CANVAS_WIDTH / 2: continue
            
            self.canvas.create_oval(cx - radius_px, cy - radius_px, cx + radius_px, cy + radius_px,
                                     outline="#404040", dash=(2, 2), tags="grid")
            self.canvas.create_text(cx + 5, cy - radius_px, text=f"{dist_m:.1f}m", fill="white", anchor=tk.NW, tags="grid")

    def update_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combobox['values'] = ports
        if ports:
            self.port_combobox.set(ports[0])

    def toggle_connection(self):
        if not self.is_connected:
            port_name = self.port_combobox.get()
            if not port_name:
                messagebox.showerror("Error", "No COM port selected.")
                return
            try:
                self.serial_port = serial.Serial(port_name, 115200, timeout=0.1)
                self.serial_port.reset_input_buffer()
                self.serial_port.reset_output_buffer()
                
                self.is_connected = True
                self.status_label.config(text=f"Status: Connected to {port_name}", foreground="green")
                self.connect_button.config(text="Disconnect")
                self.read_thread = threading.Thread(target=self.serial_reader, daemon=True)
                self.read_thread.start()
                
                self.data_queue.put(("log", ("info", f"Connected to {port_name} at 115200 baud")))
                
            except serial.SerialException as e:
                messagebox.showerror("Connection Error", f"Failed to connect: {e}")
        else:
            self.is_connected = False
            self.config_mode = False
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
            self.status_label.config(text="Status: Disconnected", foreground="red")
            self.config_status_label.config(text="Config Mode: OFF", foreground="red")
            self.connect_button.config(text="Connect")
            self.data_queue.put(("log", ("info", "Disconnected")))

    def serial_reader(self):
        """
        Reads serial data, and extracts frames for both normal and configuration modes.
        """
        buffer = b''
        while self.is_connected:
            try:
                if self.serial_port.in_waiting > 0:
                    buffer += self.serial_port.read(self.serial_port.in_waiting)

                while True:
                    # Check for both frame types
                    target_start = buffer.find(TARGET_FRAME_HEADER)
                    config_start = buffer.find(CONFIG_HEADER)

                    # Determine which frame starts first, or if neither is present
                    if target_start != -1 and (config_start == -1 or target_start < config_start):
                        # Normal mode (target) frame found
                        start = target_start
                        tail = TARGET_FRAME_TAIL
                        end = buffer.find(tail, start + len(TARGET_FRAME_HEADER))
                        if end != -1:
                            frame = buffer[start:end + len(tail)]
                            self.data_queue.put(("log", ("recv", f"RECEIVED (Target): {self.format_hex(frame)}")))
                            self.parse_target_frame(frame)
                            buffer = buffer[end + len(tail):]
                        else:
                            break # Incomplete frame, wait for more data

                    elif config_start != -1 and (target_start == -1 or config_start < target_start):
                        # Configuration mode frame found
                        start = config_start
                        tail = CONFIG_TAIL
                        end = buffer.find(tail, start + len(CONFIG_HEADER))
                        if end != -1:
                            frame = buffer[start:end + len(tail)]
                            self.data_queue.put(("log", ("recv", f"RECEIVED (Config): {self.format_hex(frame)}")))
                            self.parse_config_frame(frame)
                            buffer = buffer[end + len(tail):]
                        else:
                            break # Incomplete frame, wait for more data

                    else:
                        # No complete frame of either type found, flush old data if buffer gets too large
                        if len(buffer) > 256:
                            self.data_queue.put(("log", ("error", "Buffer overflow, flushing old data...")))
                            buffer = buffer[-256:] # Keep the last 256 bytes
                        break

                time.sleep(0.01)
            except Exception as e:
                self.data_queue.put(("log", ("error", f"Serial error: {e}")))
                self.is_connected = False
                self.data_queue.put(("disconnect", None))
                break

    def parse_target_frame(self, frame):
        """
        Parses a target frame and updates the UI/radar.
        Frame format:
        F4 F3 F2 F1 [LEN_L] [LEN_H] [NUM] [ALARM] [TARGETS...] F8 F7 F6 F5
        Each target: [ANGLE] [DIST] [SPEED_H] [SPEED_L] [SNR]
        """
        try:
            # Check for header and tail bytes
            if not frame.startswith(TARGET_FRAME_HEADER) or not frame.endswith(TARGET_FRAME_TAIL):
                self.data_queue.put(("log", ("error", "Invalid target frame format.")))
                return
            
            # The rest of your existing parsing logic is correct
            if len(frame) < 12:
                return  # Too short to be valid

            # Length is little-endian
            data_len = frame[4] + (frame[5] << 8)
            num_targets = frame[6]
            # alarm_info = frame[7]  # Not used here

            targets = []
            offset = 8
            for i in range(num_targets):
                if offset + 5 > len(frame) - 4:
                    break  # Not enough data for another target

                angle_raw = frame[offset]
                dist_raw = frame[offset + 1]
                speed_raw = (frame[offset + 2] << 8) | frame[offset + 3]
                snr = frame[offset + 4]

                # Parse values
                angle_deg = angle_raw - 0x80  # 0x80 offset
                distance_m = dist_raw
                speed_kmh = speed_raw & 0xFF
                if (speed_raw >> 8) == 1:
                    speed_kmh = -speed_kmh  # Away

                # Convert polar to cartesian for radar plot
                angle_rad = math.radians(angle_deg)
                x_m = distance_m * math.sin(angle_rad)
                y_m = distance_m * math.cos(angle_rad)

                targets.append({
                    "x_m": x_m,
                    "y_m": y_m,
                    "speed_kmh": speed_kmh,
                    "distance_m": distance_m,
                    "angle_deg": angle_deg,
                    "snr": snr
                })
                offset += 5

            self.data_queue.put(("targets", targets))
        except Exception as e:
            self.data_queue.put(("log", ("error", f"Error parsing target frame: {e}")))

    def parse_config_frame(self, frame):
        """Parse configuration response frames."""
        try:
            # Check for header and tail bytes
            if not frame.startswith(CONFIG_HEADER) or not frame.endswith(CONFIG_TAIL):
                self.data_queue.put(("log", ("error", "Invalid config frame format.")))
                return

            if len(frame) < 12: return
            
            # Use '<H' for little-endian unsigned short (2 bytes)
            cmd = struct.unpack('<H', frame[6:8])[0]
            
            if cmd == CONFIG_ACK_TARGET_PARAMS and len(frame) >= 16:
                dist, direction, speed, delay = struct.unpack('<BBBB', frame[10:14])
                self.data_queue.put(("config_target_params", (dist, direction, speed, delay)))
                self.data_queue.put(("log", ("info", f"Received target params: dist={dist}m, dir={direction}, speed={speed}km/h, delay={delay}s")))
                
            elif cmd == CONFIG_ACK_SENSITIVITY and len(frame) >= 16:
                count, snr, _, _ = struct.unpack('<BBBB', frame[10:14])
                self.data_queue.put(("config_sensitivity", (count, snr)))
                self.data_queue.put(("log", ("info", f"Received sensitivity: count={count}, snr={snr}")))

            elif cmd == CONFIG_CMD_ENABLE_ACK:
                self.config_mode = True
                self.data_queue.put(("config_mode_enabled", None))
                self.data_queue.put(("log", ("info", "Configuration mode enabled")))
                
            elif cmd == CONFIG_CMD_DISABLE_ACK:
                self.config_mode = False
                self.data_queue.put(("config_mode_disabled", None))
                self.data_queue.put(("log", ("info", "Configuration mode disabled")))
            
            elif cmd == CONFIG_ACK_FIRMWARE_VERSION:
                firmware_version = struct.unpack('<H', frame[10:12])[0]
                self.data_queue.put(("config_firmware_version", firmware_version))
                self.data_queue.put(("log", ("info", f"Received firmware version: {firmware_version}")))
            else:
                self.data_queue.put(("log", ("info", f"Unknown config response: 0x{cmd:04X}")))
        except Exception as e:
            self.data_queue.put(("log", ("error", f"Error parsing config frame: {e}")))

    def update_gui_from_queue(self):
        """Process queued messages and update GUI accordingly."""
        try:
            while not self.data_queue.empty():
                msg_type, data = self.data_queue.get_nowait()
                if msg_type == "targets": self.update_visualization(data)
                elif msg_type == "config_target_params":
                    dist, direction, speed, delay = data
                    self.max_dist_val.set(dist)
                    direction_map = {0: "Away Only", 1: "Approach Only", 2: "Approach & Away"}
                    self.direction_var.set(direction_map.get(direction, "Approach & Away"))
                    self.min_speed_val.set(speed)
                    self.target_delay_val.set(delay)
                elif msg_type == "config_sensitivity":
                    count, snr = data
                    self.trigger_count_val.set(count)
                    self.snr_val.set(snr)
                elif msg_type == "config_mode_enabled": self.config_status_label.config(text="Config Mode: ON", foreground="green")
                elif msg_type == "config_mode_disabled": self.config_status_label.config(text="Config Mode: OFF", foreground="red")
                elif msg_type == "log":
                    log_type, message = data
                    self.log_message(message, log_type)
                elif msg_type == "disconnect":
                    self.toggle_connection()
                    messagebox.showinfo("Info", "Device disconnected.")
        finally:
            self.root.after(50, self.update_gui_from_queue)

    def update_visualization(self, targets):
        """Update the radar visualization with current targets."""
        self.canvas.delete("target")
        cx, cy = CANVAS_WIDTH / 2, CANVAS_HEIGHT / 2
        max_dist_m = self.current_max_dist_m
        if max_dist_m <= 0: return

        # Updated to handle up to 5 targets
        for i in range(5):
            if i < len(targets):
                target = targets[i]
                x_m, y_m = target['x_m'], target['y_m']
                
                px = cx + (x_m / max_dist_m) * (CANVAS_WIDTH / 2)
                py = cy - (y_m / max_dist_m) * (CANVAS_HEIGHT / 2)
                
                radius = 6
                color = ["red", "orange", "yellow", "cyan", "magenta"][i]
                self.canvas.create_oval(px - radius, py - radius, px + radius, py + radius, fill=color, outline="white", width=2, tags="target")
                self.canvas.create_text(px, py - 20, text=f"T{i+1}", fill="white", font=("Arial", 10, "bold"), tags="target")
                
                # Update labels with data in meters
                self.target_labels[i]["x"].config(text=f"{x_m:.2f}")
                self.target_labels[i]["y"].config(text=f"{y_m:.2f}")
                self.target_labels[i]["speed"].config(text=f"{target['speed_kmh']:.1f}")
                self.target_labels[i]["distance"].config(text=f"{target['distance_m']:.2f}")
                self.target_labels[i]["angle"].config(text=f"{target['angle_deg']:.1f}")
            else:
                for key in ["x", "y", "speed", "distance", "angle"]:
                    self.target_labels[i][key].config(text="---")

    def serial_write_and_log(self, frame):
        """Write data to serial port and log the transmission."""
        if not self.is_connected or not self.serial_port:
            self.data_queue.put(("log", ("error", "Not connected - cannot send data")))
            return False
        try:
            self.serial_port.write(frame)
            self.serial_port.flush()
            self.data_queue.put(("log", ("sent", f"SENT: {self.format_hex(frame)}")))
            return True
        except Exception as e:
            self.data_queue.put(("log", ("error", f"Serial write error: {e}")))
            return False

    def enable_config_mode(self):
        """Enable configuration mode on the module."""
        if not self.is_connected:
            messagebox.showwarning("Warning", "Not connected to device.")
            return
        enable_frame = CONFIG_HEADER + b'\x02\x00' + struct.pack('<H', CONFIG_CMD_ENABLE) + CONFIG_TAIL
        if self.serial_write_and_log(enable_frame):
            self.data_queue.put(("log", ("info", "Enabling configuration mode...")))

    def disable_config_mode(self):
        """Disable configuration mode on the module."""
        if not self.is_connected:
            messagebox.showwarning("Warning", "Not connected to device.")
            return
        disable_frame = CONFIG_HEADER + b'\x02\x00' + struct.pack('<H', CONFIG_CMD_DISABLE) + CONFIG_TAIL
        if self.serial_write_and_log(disable_frame):
            self.data_queue.put(("log", ("info", "Disabling configuration mode...")))

    def send_write_config_command(self, command_word, value_bytes=b''):
        """Send a write configuration command with proper protocol."""
        if not self.is_connected:
            messagebox.showwarning("Warning", "Not connected to the device.")
            return False
        try:
            self.data_queue.put(("log", ("info", "Starting configuration sequence...")))
            enable_frame = CONFIG_HEADER + b'\x02\x00' + struct.pack('<H', CONFIG_CMD_ENABLE) + CONFIG_TAIL
            if not self.serial_write_and_log(enable_frame): return False
            time.sleep(0.2)
            
            length = len(value_bytes) + 2
            packet = struct.pack('<H', command_word) + value_bytes
            main_frame = CONFIG_HEADER + struct.pack('<H', length) + packet + CONFIG_TAIL
            if not self.serial_write_and_log(main_frame): return False
            time.sleep(0.2)
            
            disable_frame = CONFIG_HEADER + b'\x02\x00' + struct.pack('<H', CONFIG_CMD_DISABLE) + CONFIG_TAIL
            if not self.serial_write_and_log(disable_frame): return False
            
            self.data_queue.put(("log", ("info", "Configuration command completed")))
            return True
        except Exception as e:
            self.data_queue.put(("log", ("error", f"Failed to send config command: {e}")))
            messagebox.showerror("Error", f"Failed to send command: {e}")
            return False

    def send_read_command(self, command_word):
        """Send a read configuration command."""
        if not self.is_connected:
            messagebox.showwarning("Warning", "Not connected to device.")
            return False
        try:
            length = 0x0002
            frame = CONFIG_HEADER + struct.pack('<HH', length, command_word) + CONFIG_TAIL
            if self.serial_write_and_log(frame):
                self.data_queue.put(("log", ("info", f"Sent read command: 0x{command_word:04X}")))
                return True
            return False
        except Exception as e:
            self.data_queue.put(("log", ("error", f"Failed to send read command: {e}")))
            return False

    def read_initial_config(self):
        """Read initial configuration from the module."""
        self.data_queue.put(("log", ("info", "Reading initial configuration...")))
        self.root.after(100, lambda: self.send_read_command(CONFIG_CMD_READ_TARGET_PARAMS))
        self.root.after(300, lambda: self.send_read_command(CONFIG_CMD_READ_SENSITIVITY))

    def read_target_params(self):
        if self.send_read_command(CONFIG_CMD_READ_TARGET_PARAMS):
            self.data_queue.put(("log", ("info", "Requesting target parameters...")))

    def read_sensitivity_params(self):
        if self.send_read_command(CONFIG_CMD_READ_SENSITIVITY):
            self.data_queue.put(("log", ("info", "Requesting sensitivity parameters...")))

    def apply_detection_config(self):
        if not self.is_connected:
            messagebox.showwarning("Warning", "Not connected to device.")
            return
        dist_m = self.max_dist_val.get()
        dir_str = self.direction_var.get()
        speed_kmh = self.min_speed_val.get()
        delay_s = self.target_delay_val.get()
        direction_map = {"Away Only": 0x00, "Approach Only": 0x01, "Approach & Away": 0x02}
        dir_code = direction_map.get(dir_str, 0x02)
        value = struct.pack('<BBBB', dist_m, dir_code, speed_kmh, delay_s)
        if self.send_write_config_command(CONFIG_CMD_SET_TARGET_PARAMS, value):
            messagebox.showinfo("Success", "Target detection parameters applied successfully.")
            self.root.after(1000, lambda: self.send_read_command(CONFIG_CMD_READ_TARGET_PARAMS))
        else:
            messagebox.showerror("Error", "Failed to apply target detection parameters.")
            
    def apply_sensitivity_config(self):
        if not self.is_connected:
            messagebox.showwarning("Warning", "Not connected to device.")
            return
        trigger_count = self.trigger_count_val.get()
        snr = self.snr_val.get()
        value = struct.pack('<BBBB', trigger_count, snr, 0x00, 0x00)
        if self.send_write_config_command(CONFIG_CMD_SET_SENSITIVITY, value):
            messagebox.showinfo("Success", "Radar sensitivity applied successfully.")
            self.root.after(1000, lambda: self.send_read_command(CONFIG_CMD_READ_SENSITIVITY))
        else:
            messagebox.showerror("Error", "Failed to apply radar sensitivity.")
            
    def restart_module(self):
        if not self.is_connected:
            messagebox.showwarning("Warning", "Not connected to device.")
            return
        if self.send_write_config_command(CONFIG_CMD_RESTART):
            messagebox.showinfo("Success", "Restart command sent to module.")
            self.data_queue.put(("log", ("info", "Module restart initiated")))
            self.root.after(2000, self.read_initial_config)
        else:
            messagebox.showerror("Error", "Failed to send restart command.")

    def on_closing(self):
        """Handle application closing."""
        if self.is_connected:
            self.data_queue.put(("log", ("info", "Closing application...")))
            self.is_connected = False
            time.sleep(0.2)
            if self.serial_port and self.serial_port.is_open:
                try:
                    disable_frame = CONFIG_HEADER + b'\x02\x00' + struct.pack('<H', CONFIG_CMD_DISABLE) + CONFIG_TAIL
                    self.serial_port.write(disable_frame)
                    self.serial_port.flush()
                    time.sleep(0.1)
                except: pass
                self.serial_port.close()
        self.root.destroy()
        
    def format_hex(self, data):
        return ' '.join(f'{b:02X}' for b in data)

    def log_message(self, message, tag='info'):
        """Add a message to the log window with the specified tag."""
        self.log_text.configure(state='normal')
        timestamp = time.strftime("%H:%M:%S")
        # Handle messages that already have a newline
        end_char = "" if message.endswith('\n') else "\n"
        self.log_text.insert(tk.END, f"[{timestamp}] {message}{end_char}", tag)
        self.log_text.see(tk.END)
        self.log_text.configure(state='disabled')
        
        lines = self.log_text.get("1.0", tk.END).count('\n')
        if lines > 200:
            self.log_text.configure(state='normal')
            self.log_text.delete("1.0", "51.0")
            self.log_text.configure(state='disabled')

if __name__ == "__main__":
    root = tk.Tk()
    app = LD2451_GUI(root)
    root.mainloop()