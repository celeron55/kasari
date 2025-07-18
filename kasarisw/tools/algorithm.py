import math
from typing import List, Tuple, Union

class ObjectDetector:
    """
    A class for detecting closest wall, biggest open space, and object using window-based distance changes,
    with a point-based fallback for high-speed sparse data.
    Designed for robustness and ESP32 compatibility.
    """
    def __init__(self):
        """
        Initialize the detector state.
        """
        self.theta = 0.0  # Current angular position (radians)
        self.rpm = 0.0  # Current rotational speed (RPM)
        self.last_ts = None  # Last timestamp
        self.points: List[Tuple[float, float, float]] = []  # (x, y, timestamp) points from LiDAR
        self.last_lidar_theta = 0.0  # Theta at last LiDAR event
        self.last_vectors = None  # Store last vectors for smoothing (closest_wall, open_space, object)
        
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
        accel_g (float): Acceleration in g.
        
        Returns:
        float: Rotational speed in RPM.
        """
        g = 9.81  # m/s²
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
        if len(self.points) > 0:  # Only print if points exist
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
        
        fade_time_us = 100_000 if self.rpm == 0.0 else 1.5 * 60 * 1_000_000 / abs(self.rpm)  # 1.5 rotations
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
                if 50.0 < d < 1600.0:  # Filter outliers
                    angle = self.theta - (len(distances) - i - 1.0) * step_theta
                    x = d * math.cos(angle)
                    y = d * math.sin(angle)
                    points_this_event.append((x, y, ts))
            # Replace old points with similar angles
            angular_tolerance = 0.01  # radians ~0.57 degrees
            for new_pt in points_this_event:
                a = math.atan2(new_pt[1], new_pt[0]) % (2 * math.pi)
                self.points = [pt for pt in self.points if min((math.atan2(pt[1], pt[0]) % (2 * math.pi) - a) % (2 * math.pi), (a - math.atan2(pt[1], pt[0]) % (2 * math.pi)) % (2 * math.pi)) > angular_tolerance]
            self.points.extend(points_this_event)
            self.last_lidar_theta = self.theta
            print(f"[DEBUG] update: Added {len(points_this_event)} LiDAR points")

    def _compute_convexity_score(self, points: List[Tuple[float, float]]) -> float:
        """
        Compute convexity score for a set of points using cross products.
        
        Parameters:
        points: List of (x, y) coordinates.
        
        Returns:
        float: Convexity score (positive for convex, negative for concave, near-zero for straight).
        """
        if len(points) < 3:
            return 0.0
        
        sorted_points = sorted(points, key=lambda p: math.atan2(p[1], p[0]) % (2 * math.pi))
        convexity = 0.0
        for i in range(len(sorted_points)):
            p1 = sorted_points[i]
            p2 = sorted_points[(i + 1) % len(sorted_points)]
            p3 = sorted_points[(i + 2) % len(sorted_points)]
            v1x, v1y = p2[0] - p1[0], p2[1] - p1[1]
            v2x, v2y = p3[0] - p2[0], p3[1] - p2[1]
            cross = v1x * v2y - v1y * v2x
            convexity += cross
        return convexity / len(sorted_points)

    def detect_objects(self, points: List[Tuple[float, float, float]], debug: bool = False) -> Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float]]:
        """
        Detect closest wall, biggest open space, and object using window-based distance changes,
        with a point-based fallback for high-speed sparse data.
        
        Parameters:
        points: List of (x, y, timestamp) coordinates from LiDAR.
        debug: Enable detailed debug prints.
        
        Returns:
        Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float]]: 
            - Closest wall vector (x, y)
            - Biggest open space vector (x, y)
            - Object position (x, y)
        """
        print(f"[DEBUG] detect_objects: Received {len(points)} points")
        if len(points) < 10:
            print(f"[DEBUG] detect_objects: Too few points, returning fallback")
            return (0.0, 0.0), (0.0, 0.0), (100.0, 100.0)
        
        # Compute distances and angles
        points_with_data = [(x, y, math.sqrt(x**2 + y**2), math.atan2(y, x) % (2 * math.pi)) 
                            for x, y, _ in points]
        points_with_data.sort(key=lambda p: p[3])
        
        # Wall detection: larger window
        wall_window_size = max(5, len(points) // 12)  # ~30° for walls
        windows = []
        min_dist = float('inf')
        max_dist = -float('inf')
        closest_wall_points = []
        open_space_points = []
        
        for i in range(len(points)):
            window = []
            for j in range(i, i + wall_window_size):
                idx = j % len(points)
                window.append((points_with_data[idx][0], points_with_data[idx][1]))
            
            convexity = self._compute_convexity_score(window)
            distances = [points_with_data[j % len(points)][2] for j in range(i, i + wall_window_size)]
            median_dist = sorted(distances)[len(distances) // 2]
            window_points = [(points_with_data[j % len(points)][0], points_with_data[j % len(points)][1]) 
                             for j in range(i, i + wall_window_size)]
            
            if debug:
                print(f"[DEBUG] detect_objects: Wall window {i}, points={len(window)}, convexity={convexity}, median_dist={median_dist}")
            
            windows.append((median_dist, window_points, convexity))
            
            if convexity <= 0.0 and 120.0 <= median_dist <= 1500.0:  # Wall: straight/concave
                if median_dist < min_dist:
                    min_dist = median_dist
                    closest_wall_points = window_points
                if median_dist > max_dist:
                    max_dist = median_dist
                    open_space_points = window_points
        
        # Primary method: find two most sudden distance changes
        distance_changes = []
        for i in range(len(windows) - 1):
            dist_diff = abs(windows[i + 1][0] - windows[i][0])
            distance_changes.append((dist_diff, i))
        
        # Sort by magnitude of distance change
        distance_changes.sort(key=lambda x: x[0], reverse=True)
        
        # Find the best pair of transitions (4 to 30 windows apart)
        best_score = -float('inf')
        best_region = None
        object_points = []
        
        for i in range(len(distance_changes)):
            for j in range(i + 1, len(distance_changes)):
                idx1, idx2 = distance_changes[i][1], distance_changes[j][1]
                if idx1 > idx2:
                    idx1, idx2 = idx2, idx1
                if 4 <= idx2 - idx1 <= 30:  # Window separation constraint
                    # Check if middle region is closer
                    middle_dists = [windows[k][0] for k in range(idx1 + 1, idx2)]
                    if not middle_dists:
                        continue
                    avg_middle_dist = sum(middle_dists) / len(middle_dists)
                    start_dist = windows[idx1][0]
                    end_dist = windows[idx2][0]
                    if avg_middle_dist < min(start_dist, end_dist) - 50.0:  # Middle must be significantly closer
                        # Compute depth
                        depth = min(start_dist, end_dist) - avg_middle_dist
                        # Collect points in the middle region
                        region_points = []
                        for k in range(idx1 + 1, idx2):
                            region_points.extend(windows[k][1])
                        if len(region_points) < 5:
                            continue
                        # Compute convexity of the region
                        convexity = self._compute_convexity_score(region_points)
                        # Score based on depth and number of points
                        score = depth * len(region_points)
                        if debug:
                            print(f"[DEBUG] detect_objects: Candidate region {idx1+1} to {idx2-1}, points={len(region_points)}, avg_dist={avg_middle_dist:.1f}, depth={depth:.1f}, convexity={convexity:.1f}, score={score:.4f}")
                        if depth > 50.0 and convexity > -100.0 and 120.0 <= avg_middle_dist <= 1200.0:  # Valid object
                            if score > best_score:
                                best_score = score
                                best_region = (idx1 + 1, idx2, region_points, avg_middle_dist)
        
        # Fallback method: point-based protrusion check
        if not best_region:
            max_protrusion = -float('inf')
            best_point = None
            for point in points_with_data:
                x, y, dist, angle = point
                # Find the wall window containing this point
                window_idx = None
                for i, (median_dist, window_points, _) in enumerate(windows):
                    for wp in window_points:
                        if abs(wp[0] - x) < 1e-6 and abs(wp[1] - y) < 1e-6:
                            window_idx = i
                            break
                    if window_idx is not None:
                        break
                if window_idx is None:
                    continue
                # Compute protrusion
                protrusion = windows[window_idx][0] - dist
                if debug:
                    print(f"[DEBUG] detect_objects: Point at ({x:.1f}, {y:.1f}), dist={dist:.1f}, window={window_idx}, window_dist={windows[window_idx][0]:.1f}, protrusion={protrusion:.1f}")
                if protrusion > 30.0 and 120.0 <= dist <= 1200.0:  # Valid protrusion
                    if protrusion > max_protrusion:
                        max_protrusion = protrusion
                        best_point = (x, y)
            if best_point:
                object_points = [(best_point[0], best_point[1])]
                if debug:
                    print(f"[DEBUG] detect_objects: Selected fallback point at ({best_point[0]:.1f}, {best_point[1]:.1f}), protrusion={max_protrusion:.1f}")
            elif debug:
                print("[DEBUG] detect_objects: No valid fallback point found")
        
        if best_region:
            object_points = best_region[2]
            if debug:
                print(f"[DEBUG] detect_objects: Selected object region from window {best_region[0]} to {best_region[1]-1}, points={len(object_points)}, avg_dist={best_region[3]:.1f}")
        elif not object_points and debug:
            print("[DEBUG] detect_objects: No valid object region or point found")
        
        print(f"[DEBUG] detect_objects: Found {len(closest_wall_points)} closest wall points, "
              f"{len(open_space_points)} open space points, {len(object_points)} object points")
        
        # Compute vectors
        closest_wall = (0.0, 0.0)
        open_space = (0.0, 0.0)
        object_pos = (100.0, 100.0)
        
        if closest_wall_points:
            cx = sum(p[0] for p in closest_wall_points) / len(closest_wall_points)
            cy = sum(p[1] for p in closest_wall_points) / len(closest_wall_points)
            closest_wall = (cx, cy)
        
        if open_space_points:
            cx = sum(p[0] for p in open_space_points) / len(open_space_points)
            cy = sum(p[1] for p in open_space_points) / len(open_space_points)
            open_space = (cx, cy)
        
        if object_points:
            cx = sum(p[0] for p in object_points) / len(object_points)
            cy = sum(p[1] for p in object_points) / len(object_points)
            object_pos = (cx, cy)
        
        # Smooth vectors
        if self.last_vectors:
            closest_wall = (0.98 * closest_wall[0] + 0.02 * self.last_vectors[0][0],
                            0.98 * closest_wall[1] + 0.02 * self.last_vectors[0][1])
            open_space = (0.98 * open_space[0] + 0.02 * self.last_vectors[1][0],
                          0.98 * open_space[1] + 0.02 * self.last_vectors[1][1])
            object_pos = (0.98 * object_pos[0] + 0.02 * self.last_vectors[2][0],
                          0.98 * object_pos[1] + 0.02 * self.last_vectors[2][1])
        
        self.last_vectors = (closest_wall, open_space, object_pos)
        print(f"[DEBUG] detect_objects: Closest wall at {closest_wall}, open space at {open_space}, object at {object_pos}")
        
        return closest_wall, open_space, object_pos

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
        self.last_vectors = None
        self.smoothed_accel_y = 0.0
