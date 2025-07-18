import math
from typing import List, Tuple, Union
import numpy as np

class ObjectDetector:
    """
    A class for processing accelerometer and LiDAR data and detecting objects in a robotic environment.
    Designed for porting to an ESP32 embedded system.
    """
    def __init__(self):
        """
        Initialize the object detector and simulation state.
        """
        self.theta = 0.0  # Current angular position (radians)
        self.rpm = 0.0  # Current rotational speed (RPM)
        self.last_ts = None  # Last timestamp (initialize to None)
        self.points: List[Tuple[float, float]] = []  # Accumulated (x, y) points from LiDAR in current batch
        self.last_lidar_theta = 0.0  # Theta at the last LiDAR event
        
        # Calibration
        self.accel_offset = 0.0
        self.calibration_samples = []
        self.calibration_done = False
        self.CALIBRATION_COUNT = 10
        self.CALIBRATION_MIN_G = -8.0
        self.CALIBRATION_MAX_G = 8.0

        self.smoothed_accel_y = 0.0

    def accel_to_rpm(self, accel_g: float) -> float:
        """
        Convert acceleration in g to RPM.
        
        Parameters:
        accel_g (float): Acceleration measured by the accelerometer in g.
        
        Returns:
        float: Rotational speed in revolutions per minute (RPM).
        """
        g = 9.81  # Acceleration due to gravity in m/s²
        r = 0.0145  # Radius Cipin meters
        
        # Apply offset
        a_calibrated = accel_g - self.accel_offset
        
        # Convert acceleration to m/s²
        a = a_calibrated * g
        
        # Calculate angular velocity ω in rad/s
        omega = math.sqrt(abs(a) / r)
        if a < 0:
            omega = -omega  # Preserve direction
        
        # Convert to RPM
        rpm = (omega * 60) / (2 * math.pi)
        
        return rpm

    def update(self, event: Union[list, dict]) -> None:
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
        ts = event[1]
        
        if event_type == "WifiControl":
            # Skip time/theta update due to different ts base
            return
        
        if self.last_ts is None:
            self.last_ts = ts
            return  # Initial event, no dt
        
        # Calculate time delta (assume ts in microseconds, convert to seconds)
        dt = (ts - self.last_ts) / 1_000_000.0
        if dt < 0:
            return  # Skip if timestamp decreases
        
        # Update angular position based on current RPM
        self.theta += (self.rpm / 60.0) * 2 * math.pi * dt
        self.theta %= (2 * math.pi)
        
        self.last_ts = ts
        
        if event_type == "Accelerometer":
            raw_accel_y = event[2]  # Assume Y is the radial acceleration
            # Apply EMA lowpass filter (alpha=0.2 for smoothing)
            self.smoothed_accel_y = 0.2 * raw_accel_y + 0.8 * self.smoothed_accel_y
            accel_y = self.smoothed_accel_y  # Use smoothed value

            if not self.calibration_done:
                if self.CALIBRATION_MIN_G <= raw_accel_y <= self.CALIBRATION_MAX_G:
                    self.calibration_samples.append(raw_accel_y)
                if len(self.calibration_samples) >= self.CALIBRATION_COUNT:
                    self.accel_offset = sum(self.calibration_samples) / len(self.calibration_samples)
                    self.calibration_done = True
            if self.calibration_done:
                self.rpm = self.accel_to_rpm(accel_y)
        
        elif event_type == "Lidar":
            # Each measurement within the event is spaced by a certain amount of
            # time (1.67ms) so use a speed dependent step_theta
            delta_theta = 0.00167 * ((self.rpm / 60.0) * 2 * math.pi) if self.rpm != 0.0 else 0
            distances = event[2:6]  # d1, d2, d3, d4
            step_theta = delta_theta / len(distances)
            
            points_this_event = []
            for i, d in enumerate(distances):
                if d > 0:  # Valid distance (filter out invalid/zero)
                    # Angle for this beam
                    angle = self.theta - (len(distances) - i - 1.0) * step_theta
                    x = d * math.cos(angle)
                    y = d * math.sin(angle)
                    points_this_event.append((x, y))
            # Add to current batch
            self.points.extend(points_this_event)
            self.last_lidar_theta = self.theta  # Update for next

    def detect_objects(self, points: List[Tuple[float, float]]) -> Tuple[dict, Tuple[float, float]]:
        """
        Detect a rectangular arena and one object based on LiDAR points.
        
        Parameters:
        points (List[Tuple[float, float]]): List of (x, y) coordinates from LiDAR.
        
        Returns:
        Tuple[dict, Tuple[float, float]]: 
            - Dictionary with arena properties: {'size': (width, height), 'angle': float, 'position': (x, y)}
            - Tuple with object position: (x, y)
        """
        # Dummy implementation: returns fixed values within the specified ranges
        # Arena: size between 800mm and 2000mm, arbitrary angle and position
        arena = {
            'size': (1000.0, 1200.0),  # (width, height) in mm
            'angle': 0.0,  # radians
            'position': (0.0, 0.0)  # (x, y) in mm
        }
        
        # Object: size between 80mm and 150mm (size not returned), arbitrary position
        object_position = (100.0, 100.0)  # (x, y) in mm
        
        return arena, object_position

    def reset(self):
        """Reset the detector state."""
        self.theta = 0.0
        self.rpm = 0.0
        self.last_ts = None
        self.points = []
        self.accel_offset = 0.0
        self.calibration_samples = []
        self.calibration_done = False
        self.last_lidar_theta = 0.0
        self.smoothed_accel_y = 0.0
