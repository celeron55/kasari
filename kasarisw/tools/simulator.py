import math
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import time
from typing import List, Tuple, Union
import json
import sys
import numpy as np
import argparse
import matplotlib.patches as patches
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

class RobotSimulator:
    """
    A simulator class for replaying robot events in real-time.
    
    Processes streamed events to update robot rotation and collect LiDAR points.
    Provides methods to update with new events and draw the current plot.
    """
    
    def __init__(self, debug=False):
        """
        Initialize the simulator.
        """
        self.debug = debug
        self.theta = 0.0  # Current angular position (radians)
        self.rpm = 0.0  # Current rotational speed (RPM)
        self.last_ts = None  # Last timestamp (initialize to None)
        self.points: List[Tuple[float, float]] = []  # Accumulated (x, y) points from LiDAR
        self.event_count = 0  # Counter for processed events
        self.last_lidar_theta = 0.0  # Theta at the last LiDAR event
        
        # Scale factor for high-DPI
        self.scale = 2.0
        
        # Calibration
        self.accel_offset = 0.0
        self.calibration_samples = []
        self.calibration_done = False
        self.CALIBRATION_COUNT = 10
        self.CALIBRATION_MIN_G = -8.0
        self.CALIBRATION_MAX_G = 8.0
        
        # Lidar timestamp adjustment
        self.last_lidar_ts = None
        self.last_lidar_adjusted_ts = None
        self.lowpass_interval = 0.0  # in microseconds
        self.lowpass_alpha = 0.1
        self.bias_factor = 0.01  # Bias as fraction of lowpass_interval
        
        # Playback controls
        self.mode = 'play'
        self.speed = {'play': 1.0, 'slow': 0.25, 'pause': 0.0}
        self.virtual_elapsed = 0.0
        
        # Set up interactive plot with Tkinter
        self.root = tk.Tk()
        self.root.title("Robot Simulator")
        self.root.geometry("2000x2000")
        if self.debug:
            print(f"Matplotlib backend: {plt.get_backend()}")
        plt.rcParams.update({'font.size': 12 * self.scale})
        self.fig, self.ax = plt.subplots(figsize=(8 * self.scale, 8 * self.scale))
        self.ax.set_title("Real-Time Robot LiDAR Simulation\nEvents: 0, TS: 0 ms, RPM: 0.00")
        self.ax.set_xlabel("X (units)")
        self.ax.set_ylabel("Y (units)")
        self.ax.set_aspect('equal')
        self.ax.grid(True)
        self.sc = self.ax.scatter([], [], s=50 * self.scale, alpha=1.0, color='blue', marker='o')
        
        # Heading indicator (line from center)
        self.heading_line, = self.ax.plot([0, math.cos(self.theta) * 200 * self.scale], [0, math.sin(self.theta) * 200 * self.scale],
                                          color='r', linewidth=2 * self.scale)
        
        # Rotation direction arrow
        self.rotation_arrow = None
        
        # Fixed axis limits
        self.ax.set_xlim(-1200, 1200)
        self.ax.set_ylim(-1200, 1200)
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)
        
        self.root.bind('<Key>', self.on_key_press)

    def on_key_press(self, event):
        char = event.char.lower()
        if char in ['c', 'x', 'z']:
            modes = {'c': 'play', 'x': 'slow', 'z': 'pause'}
            self.mode = modes[char]
            if self.debug:
                print(f"Mode changed to {self.mode}")

    def accel_to_rpm(self, accel_g: float) -> float:
        """
        Convert acceleration in g to RPM.
        
        Parameters:
        accel_g (float): Acceleration measured by the accelerometer in g.
        
        Returns:
        float: Rotational speed in revolutions per minute (RPM).
        """
        g = 9.81  # Acceleration due to gravity in m/s²
        r = 0.0145  # Radius in meters
        
        # Apply offset
        a_calibrated = accel_g - self.accel_offset
       
        # Convert acceleration to m/s²
        a = a_calibrated * g
        
        # Calculate angular velocity ω in rad/s
        omega = math.sqrt(abs(a) / r)
        if a < 0:
            omega = -omega  # Preserve direction if acceleration is negative
        
        # Convert to RPM
        rpm = (omega * 60) / (2 * math.pi)
        
        return rpm

    def update(self, event: Union[list, dict]):
        """
        Update the simulation state with a new event.
        
        Parameters:
        event (Union[list, dict]): Event as list [type, ts, ...data] or dict equivalent.
        """
        if isinstance(event, dict):
            # Convert dict to list format if needed (e.g., from JSON)
            event_type = event.get('type')
            ts = event.get('ts')
            data = event.get('data', [])
            event = [event_type, ts] + data
        
        event_type = event[0]
        original_ts = ts = event[1]
        
        if event_type == "WifiControl":
            # Skip time/theta update due to different ts base; process if needed in future
            return
        
        if self.last_ts is None:
            self.last_ts = ts
            return  # Initial event, no dt
        
        if event_type == "Lidar":
            if self.debug:
                print(f"Lidar event at original ts={original_ts}, current theta={self.theta}, last_lidar_theta={self.last_lidar_theta}")
            if self.last_lidar_ts is not None:
                actual_delta = original_ts - self.last_lidar_ts
                if self.lowpass_interval == 0.0:
                    self.lowpass_interval = actual_delta
                else:
                    self.lowpass_interval = self.lowpass_alpha * actual_delta + (1 - self.lowpass_alpha) * self.lowpass_interval
                bias = self.lowpass_interval * self.bias_factor
                expected_ts = self.last_lidar_adjusted_ts + self.lowpass_interval - bias
                ts = expected_ts if expected_ts <= original_ts else original_ts
                ts = max(ts, self.last_ts)  # Ensure dt >= 0
                if self.debug:
                    print(f"Lidar adjustment: actual_delta={actual_delta}, lowpass_interval={self.lowpass_interval}, expected_ts={expected_ts}, adjusted_ts={ts}")
            self.last_lidar_ts = original_ts
            self.last_lidar_adjusted_ts = ts
        
        # Calculate time delta (assume ts in microseconds, convert to seconds)
        dt = (ts - self.last_ts) / 1_000_000.0
        if dt < 0:
            return  # Skip if timestamp decreases (though we prevented it for Lidar)
        
        # Update angular position based on current RPM
        self.theta += (self.rpm / 60.0) * 2 * math.pi * dt
        self.theta %= (2 * math.pi)  # Normalize to [0, 2π)
        
        self.last_ts = ts
        
        if event_type == "Accelerometer":
            accel_y = event[2]  # Assume Y is the radial acceleration
            if not self.calibration_done:
                if self.CALIBRATION_MIN_G <= accel_y <= self.CALIBRATION_MAX_G:
                    self.calibration_samples.append(accel_y)
                if len(self.calibration_samples) >= self.CALIBRATION_COUNT:
                    self.accel_offset = sum(self.calibration_samples) / len(self.calibration_samples)
                    self.calibration_done = True
                    print(f"Calibration complete. Offset: {self.accel_offset:.2f} G")
            if self.calibration_done:
                self.rpm = self.accel_to_rpm(accel_y)
        
        elif event_type == "Lidar":
            distances = event[2:6]  # d1, d2, d3, d4
            delta_theta = self.theta - self.last_lidar_theta
            step_theta = delta_theta / len(distances)
            if self.debug:
                print(f"Lidar processing: distances={distances}, delta_theta={delta_theta}, step_theta={step_theta}")
            for i, d in enumerate(distances):
                if d > 0:  # Valid distance (filter out invalid/zero)
                    # Angle for this beam: starting from last_lidar_theta + (i+0.5)*step
                    angle = self.last_lidar_theta + (i + 0.5) * step_theta
                    x = d * math.cos(angle)
                    y = d * math.sin(angle)
                    self.points.append((x, y))
                    if self.debug:
                        print(f"Adding point: i={i}, d={d}, angle={angle}, x={x}, y={y}, total points now={len(self.points)}")
            self.last_lidar_theta = self.theta  # Update for next
        
        self.event_count += 1

    def draw(self):
        """
        Update and redraw the plot.
        """
        if self.debug:
            print(f"Draw called with {len(self.points)} points")
        if self.points:
            offsets = np.array(self.points)
            if self.debug:
                print(f"Drawing {len(offsets)} points, range x: {offsets[:, 0].min():.2f} to {offsets[:, 0].max():.2f}, "
                      f"y: {offsets[:, 1].min():.2f} to {offsets[:, 1].max():.2f}")
            self.sc.set_offsets(offsets)
        else:
            self.sc.set_offsets(np.empty((0, 2)))
            if self.debug:
                print("No points to draw")
        
        # Update heading line
        heading_length = 200 * self.scale  # Fixed length for visibility
        self.heading_line.set_data([0, math.cos(self.theta) * heading_length],
                                   [0, math.sin(self.theta) * heading_length])
        
        # Update rotation direction arrow
        if self.rotation_arrow:
            self.rotation_arrow.remove()
            self.rotation_arrow = None
        
        arrow_length = 200 * self.scale
        y_pos = 1100
        if self.rpm != 0:
            if self.rpm > 0:  # CCW, point left
                self.rotation_arrow = patches.FancyArrowPatch((arrow_length, y_pos), (0, y_pos),
                                                              connectionstyle="arc3,rad=0.5",
                                                              arrowstyle="->", mutation_scale=20 * self.scale,
                                                              color='black')
            else:  # CW, point right
                self.rotation_arrow = patches.FancyArrowPatch((-arrow_length, y_pos), (0, y_pos),
                                                              connectionstyle="arc3,rad=-0.5",
                                                              arrowstyle="->", mutation_scale=20 * self.scale,
                                                              color='black')
            self.ax.add_patch(self.rotation_arrow)
        
        self.ax.set_title(f"Real-Time Robot LiDAR Simulation\nEvents: {self.event_count}, TS: {self.last_ts // 1000 if self.last_ts else 0} ms, RPM: {self.rpm:.2f}")
        self.canvas.draw()
        self.canvas.flush_events()

    def reset(self):
        """Reset the simulation state."""
        self.theta = 0.0
        self.rpm = 0.0
        self.last_ts = None
        self.points = []
        self.event_count = 0
        self.accel_offset = 0.0
        self.calibration_samples = []
        self.calibration_done = False
        self.last_lidar_theta = 0.0
        self.last_lidar_ts = None
        self.last_lidar_adjusted_ts = None
        self.lowpass_interval = 0.0
        self.mode = 'play'
        self.virtual_elapsed = 0.0
        if self.rotation_arrow:
            self.rotation_arrow.remove()
            self.rotation_arrow = None
        self.sc.set_offsets(np.empty((0, 2)))
        self.heading_line.set_data([0, 0], [0, 0])  # Hide line initially
        self.ax.set_title("Real-Time Robot LiDAR Simulation\nEvents: 0, TS: 0 ms, RPM: 0.00")
        self.canvas.draw()
        self.canvas.flush_events()

def process_events(source, sim: RobotSimulator, is_file: bool = True, max_events: int = None, debug=False):
    """
    Process events from a file or stdin in real-time.
    
    Parameters:
    source (str or file-like): Path to log file or '-' for stdin.
    sim (RobotSimulator): Simulator instance.
    is_file (bool): True if source is a file path, False for stdin.
    max_events (int): Optional max number of events to process.
    debug (bool): Enable debug prints.
    """
    sim.reset()
    handle = None
    events = []  # Collect all events if file, for batching
    
    try:
        if is_file:
            if source == '-':
                handle = sys.stdin
            else:
                handle = open(source, 'r')
            for line in handle:
                if line.strip():
                    try:
                        event = json.loads(line)
                        events.append(event)
                    except json.JSONDecodeError:
                        print(f"Skipping invalid JSON line: {line.strip()}")
                        continue
        else:
            handle = source  # Assume file-like object
            for line in handle:
                if line.strip():
                    try:
                        event = json.loads(line)
                        events.append(event)
                    except json.JSONDecodeError:
                        print(f"Skipping invalid JSON line: {line.strip()}")
                        continue
    finally:
        if is_file and source != '-' and handle:
            handle.close()
    
    if not events:
        print("No events to process.")
        return
    
    # Sort events by timestamp
    events.sort(key=lambda e: e[1])
    
    # Initialize simulation time
    first_ts = events[0][1]
    last_real = time.time()
    current_event_idx = 0
    interval_us = 100000  # 100ms in microseconds
    batch_start_sim_ts = first_ts
    batch_end_ts = batch_start_sim_ts + interval_us
    
    def scheduled_update():
        nonlocal current_event_idx, batch_start_sim_ts, batch_end_ts, last_real
        
        current_real = time.time()
        delta_real = current_real - last_real
        last_real = current_real
        
        sim.virtual_elapsed += delta_real * sim.speed[sim.mode]
        current_sim_ts = first_ts + sim.virtual_elapsed * 1_000_000
        
        if debug:
            print(f"Batch: sim_ts={current_sim_ts}, batch_start={batch_start_sim_ts}, batch_end={batch_end_ts}")
        
        if current_sim_ts >= batch_end_ts:
            # Clear points for new batch
            sim.points = []
            
            # Process events in the current 100ms sim interval
            processed = 0
            while current_event_idx < len(events) and events[current_event_idx][1] < batch_end_ts:
                if debug:
                    print(f"Processing event {current_event_idx}: {events[current_event_idx][0]} at ts={events[current_event_idx][1]}")
                sim.update(events[current_event_idx])
                current_event_idx += 1
                processed += 1
            if debug:
                print(f"Processed {processed} events in batch")
            
            sim.draw()
            batch_start_sim_ts = batch_end_ts
            batch_end_ts += interval_us
        
        if current_event_idx < len(events):
            sim.root.after(10, scheduled_update)
    
    sim.root.after(10, scheduled_update)
    sim.root.mainloop()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Robot Simulator")
    parser.add_argument("source", help="Log file path or '-' for stdin")
    parser.add_argument("--debug", action="store_true", help="Enable debug prints")
    args = parser.parse_args()
    
    sim = RobotSimulator(debug=args.debug)
    try:
        process_events(args.source, sim, debug=args.debug)
    except FileNotFoundError:
        print(f"Log file {args.source} not found.")
