import math
from typing import List, Tuple, Union
import random

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
        self.points: List[Tuple[float, float, float]] = []  # (x, y, timestamp) points from LiDAR
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
        r = 0.0145  # Radius in meters
        
        a_calibrated = accel_g - self.accel_offset
        a = a_calibrated * g
        omega = math.sqrt(abs(a) / r)
        if a < 0:
            omega = -omega
        rpm = (omega * 60) / (2 * math.pi)
        return rpm

    def prune_points(self, current_ts: float, fade_time_us: float):
        """
        Remove points older than fade_time_us from self.points.
        
        Parameters:
        current_ts (float): Current timestamp in microseconds.
        fade_time_us (float): Time window for keeping points (microseconds).
        """
        self.points = [pt for pt in self.points if current_ts - pt[2] <= fade_time_us]
        if len(self.points) > 1000:
            self.points = self.points[-1000:]
        print(f"[DEBUG] prune_points: Kept {len(self.points)} points after pruning")

    def update(self, event: Union[list, dict]) -> None:
        """
        Update the simulation state with a new event.
        
        Parameters:
        event (Union[list, dict]): Event as list [type, ts, ...data] or dict equivalent.
        """
        if isinstance(event, dict):
            event_type = event.get('type')
            ts = event.get('ts')
            data = event.get('data', [])
            event = [event_type, ts] + data
        
        event_type = event[0]
        ts = event[1]
        
        if event_type == "WifiControl" or event_type == "Vbat":
            return
        
        if self.last_ts is None:
            self.last_ts = ts
            return
        
        dt = (ts - self.last_ts) / 1_000_000.0
        if dt < 0:
            return
        
        self.theta += (self.rpm / 60.0) * 2 * math.pi * dt
        self.theta %= (2 * math.pi)
        
        fade_time_us = 100_000 if self.rpm == 0.0 else 1.1 * 60 * 1_000_000 / abs(self.rpm)
        self.prune_points(ts, fade_time_us)
        
        self.last_ts = ts
        
        if event_type == "Accelerometer":
            raw_accel_y = event[2]
            self.smoothed_accel_y = 0.2 * raw_accel_y + 0.8 * self.smoothed_accel_y
            accel_y = self.smoothed_accel_y

            if not self.calibration_done:
                if self.CALIBRATION_MIN_G <= raw_accel_y <= self.CALIBRATION_MAX_G:
                    self.calibration_samples.append(raw_accel_y)
                if len(self.calibration_samples) >= self.CALIBRATION_COUNT:
                    self.accel_offset = sum(self.calibration_samples) / len(self.calibration_samples)
                    self.calibration_done = True
            if self.calibration_done:
                self.rpm = self.accel_to_rpm(accel_y)
        
        elif event_type == "Lidar":
            delta_theta = 0.00167 * ((self.rpm / 60.0) * 2 * math.pi) if self.rpm != 0.0 else 0
            distances = event[2:6]
            step_theta = delta_theta / len(distances)
            
            points_this_event = []
            for i, d in enumerate(distances):
                if d > 0:
                    angle = self.theta - (len(distances) - i - 1.0) * step_theta
                    x = d * math.cos(angle)
                    y = d * math.sin(angle)
                    points_this_event.append((x, y, ts))
            self.points.extend(points_this_event)
            self.last_lidar_theta = self.theta
            print(f"[DEBUG] update: Added {len(points_this_event)} LiDAR points")

    def _fit_line(self, points: List[Tuple[float, ...]], n_samples: int = 2, n_iterations: int = 100, threshold: float = 30.0) -> Tuple[Tuple[float, float], Tuple[float, float], List[Tuple[float, ...]]]:
        """
        Fit a line to points using a RANSAC-like approach.
        
        Parameters:
        points: List of (x, y) or (x, y, timestamp) coordinates.
        n_samples: Number of points to sample for line fitting.
        n_iterations: Number of RANSAC iterations.
        threshold: Distance threshold for inliers (mm).
        
        Returns:
        Tuple of (point on line, direction vector, inlier points).
        """
        if len(points) < n_samples:
            print(f"[DEBUG] _fit_line: Not enough points ({len(points)} < {n_samples})")
            return None, None, []
        
        best_inliers = []
        best_line = None
        best_direction = None
        
        for _ in range(n_iterations):
            sample = random.sample(points, n_samples)
            x1, y1 = sample[0][:2]
            x2, y2 = sample[1][:2]
            
            point = (x1, y1)
            dx, dy = x2 - x1, y2 - y1
            length = math.sqrt(dx**2 + dy**2)
            if length < 1e-6:
                continue
            direction = (dx / length, dy / length)
            
            inliers = []
            for p in points:
                px, py = p[:2]
                t = ((px - x1) * dx + (py - y1) * dy) / length**2
                proj_x = x1 + t * dx
                proj_y = y1 + t * dy
                dist = math.sqrt((px - proj_x)**2 + (py - proj_y)**2)
                if dist < threshold:
                    inliers.append(p)
            
            if len(inliers) > len(best_inliers):
                best_inliers = inliers
                best_line = point
                best_direction = direction
        
        print(f"[DEBUG] _fit_line: Found {len(best_inliers)} inliers, direction=({best_direction[0] if best_direction else None}, {best_direction[1] if best_direction else None})")
        return best_line, best_direction, best_inliers

    def _points_to_rectangle(self, points: List[Tuple[float, float, float]]) -> Tuple[Tuple[float, float], float, Tuple[float, float]]:
        """
        Fit a rectangle to points, ensuring it surrounds the robot at (0, 0).
        
        Returns:
        Tuple of (size (width, height), angle, position (x, y)).
        """
        print(f"[DEBUG] _points_to_rectangle: Processing {len(points)} points")
        remaining_points = points[:]
        lines = []
        max_lines = 4
        
        for i in range(max_lines):
            line_point, direction, inliers = self._fit_line(remaining_points, n_iterations=50, threshold=30.0)
            if not line_point or len(inliers) < 3:
                print(f"[DEBUG] _points_to_rectangle: Stopped at {len(lines)} lines, insufficient inliers")
                break
            lines.append((line_point, direction, inliers))
            remaining_points = [p for p in remaining_points if p not in inliers]
        
        if len(lines) < 2:
            print(f"[DEBUG] _points_to_rectangle: Only found {len(lines)} lines, returning fallback")
            if points:
                x_vals = [p[0] for p in points]
                y_vals = [p[1] for p in points]
                width = min(max(max(x_vals) - min(x_vals), 800.0), 2000.0)
                height = min(max(max(y_vals) - min(y_vals), 800.0), 2000.0)
                print(f"[DEBUG] _points_to_rectangle: Fallback arena size=({width}, {height}), angle=0.0, position=(0.0, 0.0)")
                return (width, height), 0.0, (0.0, 0.0)  # Center at robot
            return (800.0, 800.0), 0.0, (0.0, 0.0)
        
        # Pair lines to form rectangle sides
        pairs = []
        used = set()
        for i, (p1, d1, inliers1) in enumerate(lines):
            if i in used:
                continue
            for j, (p2, d2, inliers2) in enumerate(lines[i+1:], i+1):
                if j in used:
                    continue
                dot = abs(d1[0] * d2[0] + d1[1] * d2[1])
                if dot < 0.6:  # Relaxed perpendicularity
                    pairs.append((i, j))
                    used.add(i)
                    used.add(j)
                    break
        
        corners = []
        if pairs:
            for i, j in pairs:
                p1, d1, _ = lines[i]
                p2, d2, _ = lines[j]
                denom = d1[0] * d2[1] - d1[1] * d2[0]
                if abs(denom) < 1e-2:
                    print(f"[DEBUG] _points_to_rectangle: Skipping intersection due to near-parallel lines (denom={denom})")
                    continue
                t = ((p2[0] - p1[0]) * d2[1] - (p2[1] - p1[1]) * d2[0]) / denom
                s = ((p2[0] - p1[0]) * d1[1] - (p2[1] - p1[1]) * d1[0]) / denom
                if -300 <= t <= 300 and -300 <= s <= 300:  # Wider bounds
                    corner = (p1[0] + t * d1[0], p1[1] + t * d1[1])
                    x_vals = [p[0] for p in points]
                    y_vals = [p[1] for p in points]
                    if (min(x_vals) - 100 <= corner[0] <= max(x_vals) + 100 and
                        min(y_vals) - 100 <= corner[1] <= max(y_vals) + 100):
                        corners.append(corner)
                    else:
                        print(f"[DEBUG] _points_to_rectangle: Discarded corner {corner} outside bounds")
        
        if len(corners) >= 4:
            cx = sum(c[0] for c in corners) / len(corners)
            cy = sum(c[1] for c in corners) / len(corners)
            # Bias position toward (0, 0)
            cx = cx * 0.5  # Move halfway toward origin
            cy = cy * 0.5
            width = (math.sqrt((corners[0][0] - corners[2][0])**2 + (corners[0][1] - corners[2][1])**2) +
                     math.sqrt((corners[1][0] - corners[3][0])**2 + (corners[1][1] - corners[3][1])**2)) / 2
            height = (math.sqrt((corners[1][0] - corners[0][0])**2 + (corners[1][1] - corners[0][1])**2) +
                      math.sqrt((corners[3][0] - corners[2][0])**2 + (corners[3][1] - corners[2][1])**2)) / 2
            width = min(max(width, 800.0), 2000.0)
            height = min(max(height, 800.0), 2000.0)
            angle = math.atan2(lines[0][1][1], lines[0][1][0])
            print(f"[DEBUG] _points_to_rectangle: Detected arena size=({width}, {height}), angle={angle}, position=({cx}, {cy})")
            return (width, height), angle, (cx, cy)
        
        print(f"[DEBUG] _points_to_rectangle: Only found {len(corners)} corners, estimating rectangle from {len(lines)} lines")
        if len(lines) >= 2:
            if pairs:
                line_idx1, line_idx2 = pairs[0]
                (p1, d1, inliers1), (p2, d2, inliers2) = lines[line_idx1], lines[line_idx2]
            else:
                lines_sorted = sorted(lines, key=lambda x: len(x[2]), reverse=True)
                (p1, d1, inliers1), (p2, d2, inliers2) = lines_sorted[:2]
            
            t1_min = min((p[0] - p1[0]) * d1[0] + (p[1] - p1[1]) * d1[1] for p in inliers1)
            t1_max = max((p[0] - p1[0]) * d1[0] + (p[1] - p1[1]) * d1[1] for p in inliers1)
            t2_min = min((p[0] - p2[0]) * d2[0] + (p[1] - p2[1]) * d2[1] for p in inliers2)
            t2_max = max((p[0] - p2[0]) * d2[0] + (p[1] - p2[1]) * d2[1] for p in inliers2)
            
            c1 = (p1[0] + t1_min * d1[0], p1[1] + t1_min * d1[1])
            c2 = (p1[0] + t1_max * d1[0], p1[1] + t1_max * d1[1])
            c3 = (p2[0] + t2_min * d2[0], p2[1] + t2_min * d2[1])
            c4 = (p2[0] + t2_max * d2[0], p2[1] + t2_max * d2[1])
            corners = [c1, c2, c3, c4]
            
            cx = sum(c[0] for c in corners) / len(corners)
            cy = sum(c[1] for c in corners) / len(corners)
            cx = cx * 0.5  # Bias toward (0, 0)
            cy = cy * 0.5
            width = min(max(abs(t1_max - t1_min), 800.0), 2000.0)
            height = min(max(abs(t2_max - t2_min), 800.0), 2000.0)
            angle = math.atan2(d1[1], d1[0])
            print(f"[DEBUG] _points_to_rectangle: Estimated arena size=({width}, {height}), angle={angle}, position=({cx}, {cy}) from {len(lines)} lines")
            return (width, height), angle, (cx, cy)
        
        x_vals = [p[0] for p in points]
        y_vals = [p[1] for p in points]
        width = min(max(max(x_vals) - min(x_vals), 800.0), 2000.0)
        height = min(max(max(y_vals) - min(y_vals), 800.0), 2000.0)
        print(f"[DEBUG] _points_to_rectangle: Fallback arena size=({width}, {height}), angle=0.0, position=(0.0, 0.0)")
        return (width, height), 0.0, (0.0, 0.0)  # Center at robot

    def _find_object(self, points: List[Tuple[float, float, float]], arena: dict) -> Tuple[float, float]:
        """
        Find an L-shaped object (~5 points) by clustering or line fitting.
        
        Parameters:
        points: List of (x, y, timestamp) coordinates.
        arena: Dictionary with keys 'size' (width, height), 'angle' (radians), 'position' (x, y).
        
        Returns:
        Tuple of (x, y) for object position.
        """
        aw, ah = arena['size']
        a_angle = arena['angle']
        ax, ay = arena['position']
        
        non_arena_points = []
        cos_a = math.cos(a_angle)
        sin_a = math.sin(a_angle)
        
        for x, y, _ in points:
            x_rel = (x - ax) * cos_a + (y - ay) * sin_a
            y_rel = -(x - ax) * sin_a + (y - ay) * cos_a
            margin = 250.0  # Increased margin
            if abs(x_rel) > aw/2 + margin or abs(y_rel) > ah/2 + margin:
                non_arena_points.append((x, y))
        
        print(f"[DEBUG] _find_object: Found {len(non_arena_points)} non-arena points")
        
        if len(non_arena_points) < 2:
            print(f"[DEBUG] _find_object: Too few non-arena points, returning fallback")
            return (100.0, 100.0)
        
        line1_point, line1_dir, line1_inliers = self._fit_line(non_arena_points, threshold=15.0, n_samples=2, n_iterations=20)
        if not line1_point or len(line1_inliers) < 2:
            print(f"[DEBUG] _find_object: No first line found, inliers={len(line1_inliers) if line1_inliers else 0}")
            cx = sum(p[0] for p in non_arena_points) / len(non_arena_points)
            cy = sum(p[1] for p in non_arena_points) / len(non_arena_points)
            print(f"[DEBUG] _find_object: Fallback to centroid at ({cx}, {cy})")
            return (cx, cy)
        
        remaining = [p for p in non_arena_points if p not in line1_inliers]
        line2_point, line2_dir, line2_inliers = self._fit_line(remaining, threshold=15.0, n_samples=2, n_iterations=20)
        
        if not line2_point or len(line2_inliers) < 1:  # Reduced to 1 inlier
            print(f"[DEBUG] _find_object: No second line found, inliers={len(line2_inliers) if line2_inliers else 0}")
            cx = sum(p[0] for p in line1_inliers) / len(line1_inliers)
            cy = sum(p[1] for p in line1_inliers) / len(line1_inliers)
            print(f"[DEBUG] _find_object: Using first line centroid at ({cx}, {cy})")
            return (cx, cy)
        
        dot = abs(line1_dir[0] * line2_dir[0] + line1_dir[1] * line2_dir[1])
        if dot > 0.6:
            print(f"[DEBUG] _find_object: Lines not perpendicular (dot={dot}), using first line")
            cx = sum(p[0] for p in line1_inliers) / len(line1_inliers)
            cy = sum(p[1] for p in line1_inliers) / len(line1_inliers)
            print(f"[DEBUG] _find_object: Using first line centroid at ({cx}, {cy})")
            return (cx, cy)
        
        denom = line1_dir[0] * line2_dir[1] - line1_dir[1] * line2_dir[0]
        if abs(denom) < 1e-2:
            print(f"[DEBUG] _find_object: Lines near-parallel (denom={denom}), using inlier centroid")
            cx = sum(p[0] for p in line1_inliers + line2_inliers) / len(line1_inliers + line2_inliers)
            cy = sum(p[1] for p in line1_inliers + line2_inliers) / len(line1_inliers + line2_inliers)
            return (cx, cy)
        
        t = ((line2_point[0] - line1_point[0]) * line2_dir[1] - (line2_point[1] - line1_point[1]) * line2_dir[0]) / denom
        cx = line1_point[0] + t * line1_dir[0]
        cy = line1_point[1] + t * line1_dir[1]
        
        print(f"[DEBUG] _find_object: Detected L-shape object at ({cx}, {cy}) with {len(line1_inliers)} + {len(line2_inliers)} inliers")
        return (cx, cy)

    def detect_objects(self, points: List[Tuple[float, float, float]]) -> Tuple[dict, Tuple[float, float]]:
        """
        Detect a rectangular arena and one object based on LiDAR points.
        
        Parameters:
        points: List of (x, y, timestamp) coordinates from LiDAR.
        
        Returns:
        Tuple[dict, Tuple[float, float]]: 
            - Dictionary with arena properties: {'size': (width, height), 'angle': float, 'position': (x, y)}
            - Tuple with object position: (x, y)
        """
        print(f"[DEBUG] detect_objects: Received {len(points)} points")
        if len(points) < 10:
            print("[DEBUG] detect_objects: Too few points, returning fallback")
            return {'size': (800.0, 800.0), 'angle': 0.0, 'position': (0.0, 0.0)}, (100.0, 100.0)
        
        size, angle, position = self._points_to_rectangle(points)
        arena = {
            'size': size,
            'angle': angle,
            'position': position
        }
        object_position = self._find_object(points, arena)
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
