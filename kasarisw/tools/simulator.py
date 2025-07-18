import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import time
from typing import List, Tuple
import json
import sys
import numpy as np
import argparse
import matplotlib.patches as patches
import tkinter as tk
import math
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from collections import deque
from algorithm import ObjectDetector

class RobotSimulator:
    """
    A simulator class for replaying robot events in real-time with visualization.
    Interfaces with ObjectDetector for data processing.
    """
    
    def __init__(self, debug=False):
        """
        Initialize the simulator.
        """
        self.debug = debug
        self.detector = ObjectDetector()
        self.event_count = 0  # Counter for processed events
        
        # Scale factor for high-DPI
        self.scale = 2.0
        
        # Playback controls
        self.mode = 'play'
        self.speed = {'play': 1.0, 'slow': 0.25, 'pause': 0.0, 'step': 0.0}
        self.virtual_elapsed = 0.0
        self.step_requested = False
        self.point_history = deque(maxlen=100)  # Store up to 100 batches
        self.current_lidar_points = []
        
        self.running = True
        self.after_id = None
        
        # Set up interactive plot with Tkinter
        self.root = tk.Tk()
        self.root.title("Robot Simulator")
        self.root.geometry("2000x2000")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        if self.debug:
            print(f"Matplotlib backend: {plt.get_backend()}")
        plt.rcParams.update({'font.size': 12 * self.scale})
        self.fig, self.ax = plt.subplots(figsize=(8 * self.scale, 8 * self.scale))
        self.ax.set_title("Real-Time Robot LiDAR Simulation\nEvents: 0, TS: 0 ms, RPM: 0.00")
        self.ax.set_xlabel("X (mm)")
        self.ax.set_ylabel("Y (mm)")
        self.ax.set_aspect('equal')
        self.ax.grid(True)
        self.sc = self.ax.scatter([], [], s=50 * self.scale, alpha=0.6, color='blue', marker='o')
        self.highlight_sc = self.ax.scatter([], [], s=60 * self.scale, alpha=1.0, color='red', marker='o')
        
        # Heading indicator (line from center)
        self.heading_line, = self.ax.plot([0, 0], [0, 0], color='r', linewidth=2 * self.scale)
        
        # Rotation direction arrow
        self.rotation_arrow = None
        
        # Vector patches for closest wall, open space, and object
        self.closest_wall_arrow = None
        self.open_space_arrow = None
        self.object_arrow = None
        
        # Fixed axis limits
        self.ax.set_xlim(-800, 800)
        self.ax.set_ylim(-800, 800)
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)
        
        self.root.bind('<Key>', self.on_key_press)

    def on_close(self):
        self.running = False
        if self.after_id:
            self.root.after_cancel(self.after_id)
        self.root.quit()

    def on_key_press(self, event):
        char = event.char.lower()
        if char in ['c', 'x', 'z']:
            modes = {'c': 'play', 'x': 'slow', 'z': 'pause'}
            self.mode = modes[char]
            self.current_lidar_points = []
            if self.debug:
                print(f"Mode changed to {self.mode}")
        elif char == 'e':
            if self.mode != 'step':
                self.mode = 'step'
                if self.debug:
                    print("Entered event stepping mode. Press 'e' to advance to next LiDAR event.")
            else:
                self.step_requested = True
                self.current_lidar_points = []
                if self.debug:
                    print("Advancing to next LiDAR event.")

    def draw(self):
        """
        Update and redraw the plot, showing vectors for closest wall, open space, and object.
        """
        if self.debug:
            print(f"Draw called with {len(self.detector.points)} points in detector")
        current_ts = self.detector.last_ts if self.detector.last_ts else 0
        fade_time_us = 100_000 if self.detector.rpm == 0.0 else 1.1 * 60 * 1_000_000 / abs(self.detector.rpm)
        # Remove old points from history
        while self.point_history and self.point_history[0][0] < current_ts - fade_time_us:
            self.point_history.popleft()
        
        # Combine points from history for visualization
        all_points = [(x, y) for ts, batch in self.point_history for x, y in batch]
        offsets = np.array(all_points) if all_points else np.empty((0, 2))
        if self.debug and len(offsets) > 0:
            print(f"Drawing {len(offsets)} points, range x: {offsets[:, 0].min():.2f} to {offsets[:, 0].max():.2f}, "
                  f"y: {offsets[:, 1].min():.2f} to {offsets[:, 1].max():.2f}")
        self.sc.set_offsets(offsets)
        
        # Highlight current LiDAR points
        highlight_offsets = np.array(self.current_lidar_points) if self.current_lidar_points else np.empty((0, 2))
        self.highlight_sc.set_offsets(highlight_offsets)
        
        # Update heading line
        heading_length = 200 * self.scale
        self.heading_line.set_data([0, math.cos(self.detector.theta) * heading_length],
                                   [0, math.sin(self.detector.theta) * heading_length])
        
        # Update rotation direction arrow
        if self.rotation_arrow:
            self.rotation_arrow.remove()
            self.rotation_arrow = None
        
        arrow_length = 200 * self.scale
        y_pos = 700
        if self.detector.rpm != 0:
            if self.detector.rpm > 0:  # CCW, point left
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
        
        # Update vector visualizations
        if self.closest_wall_arrow:
            self.closest_wall_arrow.remove()
            self.closest_wall_arrow = None
        if self.open_space_arrow:
            self.open_space_arrow.remove()
            self.open_space_arrow = None
        if self.object_arrow:
            self.object_arrow.remove()
            self.object_arrow = None
        
        # Detect objects
        closest_wall, open_space, object_pos = self.detector.detect_objects(self.detector.points)
        
        # Draw closest wall vector (green)
        if closest_wall != (0.0, 0.0):
            self.closest_wall_arrow = patches.FancyArrowPatch((0, 0), closest_wall,
                                                             arrowstyle="->", mutation_scale=20 * self.scale,
                                                             color='green')
            self.ax.add_patch(self.closest_wall_arrow)
        
        # Draw open space vector (blue)
        if open_space != (0.0, 0.0):
            self.open_space_arrow = patches.FancyArrowPatch((0, 0), open_space,
                                                            arrowstyle="->", mutation_scale=20 * self.scale,
                                                            color='blue')
            self.ax.add_patch(self.open_space_arrow)
        
        # Draw object vector (purple)
        if object_pos != (100.0, 100.0):
            self.object_arrow = patches.FancyArrowPatch((0, 0), object_pos,
                                                       arrowstyle="->", mutation_scale=20 * self.scale,
                                                       color='purple')
            self.ax.add_patch(self.object_arrow)
        
        self.ax.set_title(f"Real-Time Robot LiDAR Simulation\nEvents: {self.event_count}, TS: {self.detector.last_ts // 1000 if self.detector.last_ts else 0} ms, RPM: {self.detector.rpm:.2f}")
        self.canvas.draw()
        self.canvas.flush_events()

    def reset(self):
        """Reset the simulation state."""
        self.detector.reset()
        self.event_count = 0
        self.mode = 'play'
        self.virtual_elapsed = 0.0
        self.step_requested = False
        self.running = True
        self.point_history = deque(maxlen=100)
        self.current_lidar_points = []
        if self.rotation_arrow:
            self.rotation_arrow.remove()
            self.rotation_arrow = None
        if self.closest_wall_arrow:
            self.closest_wall_arrow.remove()
            self.closest_wall_arrow = None
        if self.open_space_arrow:
            self.open_space_arrow.remove()
            self.open_space_arrow = None
        if self.object_arrow:
            self.object_arrow.remove()
            self.object_arrow = None
        self.sc.set_offsets(np.empty((0, 2)))
        self.highlight_sc.set_offsets(np.empty((0, 2)))
        self.heading_line.set_data([0, 0], [0, 0])
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
    events = []
    
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
            handle = source
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
    
    events.sort(key=lambda e: e[1])
    
    first_ts = events[0][1]
    last_real = time.time()
    current_event_idx = 0
    interval_us = 20000  # 20ms batches
    batch_start_sim_ts = first_ts
    batch_end_ts = batch_start_sim_ts + interval_us
    
    def scheduled_update():
        nonlocal current_event_idx, batch_start_sim_ts, batch_end_ts, last_real
        
        if not sim.running:
            return
        
        current_real = time.time()
        delta_real = current_real - last_real
        last_real = current_real
        
        sim.virtual_elapsed += delta_real * sim.speed[sim.mode]
        current_sim_ts = first_ts + sim.virtual_elapsed * 1_000_000
        
        if debug:
            print(f"Batch: sim_ts={current_sim_ts}, batch_start={batch_start_sim_ts}, batch_end={batch_end_ts}")
        
        if sim.mode == 'step':
            if sim.step_requested:
                sim.step_requested = False
                processed = 0
                found_lidar = False
                new_points = []
                while current_event_idx < len(events) and not found_lidar:
                    event = events[current_event_idx]
                    sim.detector.update(event)
                    if event[0] == "Lidar":
                        found_lidar = True
                        new_points = [(x, y) for x, y, _ in sim.detector.points[-len(event[2:6]):]]
                        sim.current_lidar_points = new_points
                    current_event_idx += 1
                    processed += 1
                if debug:
                    print(f"Processed {processed} events in step mode")
                if found_lidar:
                    sim.point_history.append((sim.detector.last_ts, new_points))
                    sim.draw()
                    if current_event_idx > 0:
                        last_processed_ts = events[current_event_idx - 1][1]
                        sim.virtual_elapsed = (last_processed_ts - first_ts) / 1_000_000
                        batch_start_sim_ts = last_processed_ts
                        batch_end_ts = last_processed_ts + interval_us
                else:
                    print("No more LiDAR events.")
        elif sim.mode in ['play', 'slow']:
            if current_sim_ts >= batch_end_ts:
                processed = 0
                new_points = []
                while current_event_idx < len(events) and events[current_event_idx][1] < batch_end_ts:
                    if debug:
                        print(f"Processing event {current_event_idx}: {events[current_event_idx][0]} at ts={events[current_event_idx][1]}")
                    event = events[current_event_idx]
                    sim.detector.update(event)
                    if event[0] == "Lidar":
                        new_points.extend([(x, y) for x, y, _ in sim.detector.points[-len(event[2:6]):]])
                    current_event_idx += 1
                    processed += 1
                if debug:
                    print(f"Processed {processed} events in batch")
                
                if new_points:
                    sim.point_history.append((sim.detector.last_ts, new_points))
                
                sim.draw()
                batch_start_sim_ts = batch_end_ts
                batch_end_ts += interval_us
        
        if current_event_idx < len(events):
            interval_ms = 20 if sim.mode == 'play' else 100
            sim.after_id = sim.root.after(interval_ms, scheduled_update)
    
    sim.after_id = sim.root.after(20, scheduled_update)
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
    except KeyboardInterrupt:
        sys.exit(0)
