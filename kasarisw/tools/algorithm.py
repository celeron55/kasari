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

    def prune_points(self, current_ts: float, fade_time_us: float):
        """
        Remove points older than fade_time_us from self.points.
        
        Parameters:
        current_ts (float): Current timestamp in microseconds.
        fade_time_us (float): Time window for keeping points (microseconds).
        """
        self.points = [pt for pt in self.points if current_ts - pt[2] <= fade_time_us]
        if len(self.points) > 1000:  # Additional safety limit for memory
            self.points = self.points[-1000:]
        print(f"[DEBUG] prune_points: Kept {len(self.points)} points after pruning")

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
        
        # Prune old points before adding new ones
        fade_time_us = 100_000 if self.rpm == 0.0 else 1.1 * 60 * 1_000_000 / abs(self.rpm)
        self.prune_points(ts, fade_time_us)
        
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
            # time (1.67ms) so use a speed-dependent step_theta
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
                    points_this_event.append((x, y, ts))  # Store with timestamp
            # Add to current batch
            self.points.extend(points_this_event)
            self.last_lidar_theta = self.theta  # Update for next
            print(f"[DEBUG] update: Added {len(points_this_event)} LiDAR points")

    def _fit_line(self, points: List[Tuple[float, float, float]], n_samples: int = 2, n_iterations: int = 100, threshold: float = 30.0) -> Tuple[Tuple[float, float], Tuple[float, float], List[Tuple[float, float, float]]]:
        """
        Fit a line to points using a RANSAC-like approach.
        
        Parameters:
        points: List of (x, y, timestamp) coordinates.
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
            # Sample two points
            sample = random.sample(points, n_samples)
            x1, y1, _ = sample[0]
            x2, y2, _ = sample[1]
            
            # Define line: point and direction
            point = (x1, y1)
            dx, dy = x2 - x1, y2 - y1
            length = math.sqrt(dx**2 + dy**2)
            if length < 1e-6:  # Avoid division by zero
                continue
            direction = (dx / length, dy / length)
            
            # Count inliers
            inliers = []
            for p in points:
                px, py, _ = p
                # Distance from point to line
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
        Fit a rectangle to points by finding four dominant lines.
        
        Returns:
        Tuple of (size (width, height), angle, position (x, y)).
        """
        print(f"[DEBUG] _points_to_rectangle: Processing {len(points)} points")
        remaining_points = points[:]
        lines = []
        max_lines = 4  # We need four lines for a rectangle
        
        for i in range(max_lines):
            line_point, direction, inliers = self._fit_line(remaining_points, n_iterations=50, threshold=30.0)
            if not line_point or len(inliers) < 3:  # Minimum 3 inliers
                print(f"[DEBUG] _points_to_rectangle: Stopped at {len(lines)} lines, insufficient inliers")
                break
            lines.append((line_point, direction, inliers))
            # Remove inliers from remaining points
            remaining_points = [p for p in remaining_points if p not in inliers]
        
        if len(lines) < 4:
            print(f"[DEBUG] _points_to_rectangle: Only found {len(lines)} lines, returning fallback")
            # Estimate from point cloud
            if points:
                x_vals = [p[0] for p in points]
                y_vals = [p[1] for p in points]
                width = min(max(max(x_vals) - min(x_vals), 800.0), 2000.0)
                height = min(max(max(y_vals) - min(y_vals), 800.0), 2000.0)
                cx = sum(x_vals) / len(x_vals)
                cy = sum(y_vals) / len(y_vals)
                return (width, height), 0.0, (cx, cy)
            return (1000.0, 1200.0), 0.0, (0.0, 0.0)  # Fallback
        
        # Pair lines to form rectangle sides
        pairs = []
        used = set()
        for i, (p1, d1, _) in enumerate(lines):
            if i in used:
                continue
            for j, (p2, d2, _) in enumerate(lines[i+1:], i+1):
                if j in used:
                    continue
                # Dot product close to 0 indicates perpendicular lines
                dot = abs(d1[0] * d2[0] + d1[1] * d2[1])
                if dot < 0.3:  # Relaxed perpendicularity
                    pairs.append((i, j))
                    used.add(i)
                    used.add(j)
                    break
        
        if len(pairs) < 2:
            print(f"[DEBUG] _points_to_rectangle: Only found {len(pairs)} perpendicular pairs, returning fallback")
            if points:
                x_vals = [p[0] for p in points]
                y_vals = [p[1] for p in points]
                width = min(max(max(x_vals) - min(x_vals), 800.0), 2000.0)
                height = min(max(max(y_vals) - min(y_vals), 800.0), 2000.0)
                cx = sum(x_vals) / len(x_vals)
                cy = sum(y_vals) / len(y_vals)
                return (width, height), 0.0, (cx, cy)
            return (1000.0, 1200.0), 0.0, (0.0, 0.0)  # Fallback
        
        # Compute intersections to find rectangle corners
        corners = []
        for (i, j) in pairs:
            p1, d1, _ = lines[i]
            p2, d2, _ = lines[j]
            # Line equations: p = p1 + t*d1, p = p2 + s*d2
            denom = d1[0] * d2[1] - d1[1] * d2[0]
            if abs(denom) < 1e-3:  # Relaxed threshold for near-parallel lines
                print(f"[DEBUG] _points_to_rectangle: Skipping intersection due to near-parallel lines (denom={denom})")
                continue
            t = ((p2[0] - p1[0]) * d2[1] - (p2[1] - p1[1]) * d2[0]) / denom
            s = ((p2[0] - p1[0]) * d1[1] - (p2[1] - p1[1]) * d1[0]) / denom
            # Check if intersection is within line segments (approximate)
            if -50 <= t <= 50 and -50 <= s <= 50:  # Allow reasonable extrapolation
                corner = (p1[0] + t * d1[0], p1[1] + t * d1[1])
                # Check if corner is within point cloud bounds
                x_vals = [p[0] for p in points]
                y_vals = [p[1] for p in points]
                if (min(x_vals) - 50 <= corner[0] <= max(x_vals) + 50 and
                    min(y_vals) - 50 <= corner[1] <= max(y_vals) + 50):
                    corners.append(corner)
                else:
                    print(f"[DEBUG] _points_to_rectangle: Discarded corner {corner} outside bounds")
        
        if len(corners) < 4:
            print(f"[DEBUG] _points_to_rectangle: Only found {len(corners)} corners, returning fallback")
            if points:
                x_vals = [p[0] for p in points]
                y_vals = [p[1] for p in points]
                width = min(max(max(x_vals) - min(x_vals), 800.0), 2000.0)
                height = min(max(max(y_vals) - min(y_vals), 800.0), 2000.0)
                cx = sum(x_vals) / len(x_vals)
                cy = sum(y_vals) / len(y_vals)
                return (width, height), 0.0, (cx, cy)
            return (1000.0, 1200.0), 0.0, (0.0, 0.0)  # Fallback
        
        # Compute rectangle properties
        cx = sum(c[0] for c in corners) / len(corners)
        cy = sum(c[1] for c in corners) / len(corners)
        
        # Estimate width and height (average distances between opposite corners)
        width = (math.sqrt((corners[0][0] - corners[2][0])**2 + (corners[0][1] - corners[2][1])**2) +
                 math.sqrt((corners[1][0] - corners[3][0])**2 + (corners[1][1] - corners[3][1])**2)) / 2
        height = (math.sqrt((corners[1][0] - corners[0][0])**2 + (corners[1][1] - corners[0][1])**2) +
                  math.sqrt((corners[3][0] - corners[2][0])**2 + (corners[3][1] - corners[2][1])**2)) / 2
        
        # Ensure size is within bounds
        width = min(max(width, 800.0), 2000.0)
        height = min(max(height, 800.0), 2000.0)
        
        # Estimate angle from first line
        angle = math.atan2(lines[0][1][1], lines[0][1][0])
        
        print(f"[DEBUG] _points_to_rectangle: Detected arena size=({width}, {height}), angle={angle}, position=({cx}, {cy})")
        return (width, height), angle, (cx, cy)

    def _find_object(self, points: List[Tuple[float, float, float]], arena: dict) -> Tuple[float, float]:
        """
        Find an object (80–150mm in size) by clustering points not belonging to the arena.
        
        Parameters:
        points: List of (x, y, timestamp) coordinates.
        arena: Dictionary with keys 'size' (width, height), 'angle' (radians), 'position' (x, y).
        
        Returns:
        Tuple of (x, y) for object position.
        """
        aw, ah = arena['size']
        a_angle = arena['angle']
        ax, ay = arena['position']
        
        # Transform points to arena's frame to filter out arena points
        non_arena_points = []
        cos_a = math.cos(a_angle)
        sin_a = math.sin(a_angle)
        
        for x, y, _ in points:
            # Transform to arena-aligned coordinates
            x_rel = (x - ax) * cos_a + (y - ay) * sin_a
            y_rel = -(x - ax) * sin_a + (y - ay) * cos_a
            # Check if point is inside arena bounds (with margin)
            margin = 150.0
            if abs(x_rel) > aw/2 + margin or abs(y_rel) > ah/2 + margin:
                non_arena_points.append((x, y))
        
        print(f"[DEBUG] _find_object: Found {len(non_arena_points)} non-arena points")
        
        if not non_arena_points:
            print("[DEBUG] _find_object: No non-arena points, returning fallback")
            return (100.0, 100.0)  # Fallback
        
        # Simple centroid-based clustering for object
        clusters = []
        remaining = non_arena_points[:]
        eps = 250.0  # Clustering radius
        
        while remaining:
            cluster = [remaining[0]]
            remaining = remaining[1:]
            i = 0
            while i < len(remaining):
                px, py = remaining[i]
                # Check distance to cluster centroid
                cx = sum(p[0] for p in cluster) / len(cluster)
                cy = sum(p[1] for p in cluster) / len(cluster)
                dist = math.sqrt((px - cx)**2 + (py - cy)**2)
                if dist < eps:
                    cluster.append(remaining.pop(i))
                else:
                    i += 1
            # Check cluster size (in mm)
            if len(cluster) >= 2:  # Require at least 2 points
                max_dist = max(math.sqrt((p[0] - cx)**2 + (p[1] - cy)**2) for p in cluster)
                if 30.0 <= max_dist <= 125.0:  # Relaxed bounds (60–250mm diameter)
                    clusters.append(cluster)
        
        print(f"[DEBUG] _find_object: Found {len(clusters)} valid clusters")
        
        if not clusters:
            print("[DEBUG] _find_object: No valid clusters, returning fallback")
            return (100.0, 100.0)  # Fallback
        
        # Pick largest cluster
        largest_cluster = max(clusters, key=len)
        cx = sum(p[0] for p in largest_cluster) / len(largest_cluster)
        cy = sum(p[1] for p in largest_cluster) / len(largest_cluster)
        
        print(f"[DEBUG] _find_object: Detected object at ({cx}, {cy}) with {len(largest_cluster)} points")
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
        if len(points) < 10:  # Need enough points for reliable detection
            print("[DEBUG] detect_objects: Too few points, returning fallback")
            return {'size': (1000.0, 1200.0), 'angle': 0.0, 'position': (0.0, 0.0)}, (100.0, 100.0)
        
        # Detect arena
        size, angle, position = self._points_to_rectangle(points)
        
        arena = {
            'size': size,  # (width, height) in mm
            'angle': angle,  # radians
            'position': position  # (x, y) in mm
        }
        
        # Detect object
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
