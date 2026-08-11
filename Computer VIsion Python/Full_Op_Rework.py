'''
Created using Deep Seek and Claude.ai

Robust spooler automation system integrating:
1. Camera-based filament diameter measurement
2. Motor speed control via hardware PWM
3. Manual/Automatic mode switching
4. PID-based control for maintaining target diameter

Usage:
- Place in same directory as Wrapper_HardwarePWM.py and Wrapper_UI.py
- Run with: sudo -E ~/motor-env/bin/python Full_Op.py
'''

import cv2 as cv
import numpy as np
import time
import sys
from Wrapper_HardwarePWM import HardwarePWM
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple
from Wrapper_UI import UIWrapper

# ──────────────────────────────────────────────────────────────────────────────
# CONSTANTS & CONFIGURATION
# ──────────────────────────────────────────────────────────────────────────────

class OperatingMode(Enum):
    MANUAL = "manual"
    AUTOMATIC = "automatic"
    FAILSAFE = "failsafe"

@dataclass
class FilamentConfig:
    """
    Configuration for filament measurement and control
    Edit here and they'll propogate automatically
    """
    # Object detection polarity
    bright_on_dark: bool = True  # True = filament is lighter than background
    calibration_pixels_per_mm:  Optional[float] = 138  # calibrated July 24th
    history_retention_s: float = 1.5 # measurements held for averaging
    camera_fps: int = 30

    target_diameter_mm:         float = 1.750   # Target filament diameter
    tolerance_min_mm:           float = 1.650   # Minimum acceptable diameter
    tolerance_max_mm:           float = 1.850   # Maximum acceptable diameter
    rapid_change_threshold_mm:  float = 0.1     # Rapid change detection (per second)

    plausible_diameter_min_mm:  float = 1.2
    plausible_diameter_max_mm:  float = 2.2
    
    display_width:  int = 1280
    display_height: int = 720
    roi_x:          int = 1270
    roi_y:          int = 380
    roi_w:          int = 1300
    roi_h:          int = 1400
    
@dataclass
class ControlConfig:
    """Configuration for motor control and PID"""
    # Motor PWM settings
    led_pin:            int = 17 # GPIO17
    chip_:              int = 0
    pwm_in1_channel:    int = 0  # GPIO12
    pwm_in2_channel:    int = 1  # GPIO13
    pwm_frequency_hz:   int = 50_000

    # Speed limits
    duty_min_percent:           float = 52.5  # Minimum duty cycle (%)
    duty_max_percent:           float = 65.0  # Maximum duty cycle (%) # 100 is overkill
    
    # Rescaling duty cycle 52.5 to 85 to 1 to 100 instead ######################################
    speed_pct_min:              float = 1.0     # Normalized speed floor
    speed_pct_max:              float = 100.0   # Normalized speed ceiling
    speed_manual_default:       float = 10.0     # Default manual speed
    significant_speed_change:   float = 0.0005
    
    # PID controller gains
    ################################# CHANGE TO INTEGRAL CONTROL ################################
    pid_kp:                     float = 0.9  # Proportional gain, set to 1 Aug 8
    pid_ki:                     float = 0.02  # Integral gain
    pid_kd:                     float = 0  # Derivative gain
    
    # Control timing
    measurement_interval_s:     float = 1 / 10.0  # Time between measurements
    response_delay_s:           float = 30.0  # Delay after motor adjustment actual ~ 45 seconds
    
    # Safety limits
    max_integral_windup:        float = 0.20 / pid_ki  # Anti-windup limit
    max_step_speed_pct:         float = 1 # UNITS? Max speed change allowed per control update
    emergency_stop_timeout_s:   float = 30.0  # Time without valid measurement before stop

# ──────────────────────────────────────────────────────────────────────────────
# CAMERA & MEASUREMENT COMPONENTS (adapted from Test_Operation_with_Webcam.py)
# ──────────────────────────────────────────────────────────────────────────────

class FilamentMeasurement:
    """Handles filament diameter measurement using camera"""
    
    def __init__(self, config: FilamentConfig):
        self.config         = config
        self.display_width  = config.display_width
        self.display_height = config.display_height

        # Timing and history tracking
        self.start_time      =  time.time()  # When measurement started
        self.elapsed         =  0.0
        self.last_status_t   =  0.0       # Last time status was printed

        # Cache of the most recent measurement's frame/edges, so display
        # code can draw the exact data that was measured instead of
        # re-reading the camera
        self.last_frame         = None
        self.last_gray_full     = None
        self.last_roi_bgr       = None
        self.last_top_edges     = None
        self.last_bot_edges     = None
        self.last_diameter_px   = float('nan')
        self.last_sobel         = None
        self.last_roi_bounds    = None  # (ry1, ry2, rx1, rx2)
        self.cap = None
        self._init_camera()
        self.measurement_history: list = []
        
    def _init_camera(self):
        """Initialize camera with optimal settings"""
        self.cap = cv.VideoCapture(0, cv.CAP_V4L2)
        for _ in range(15):
            self.cap.read()

        # Set camera properties for high-quality measurement
        self.cap.set(cv.CAP_PROP_FOURCC,        cv.VideoWriter.fourcc(*'MJPG'))
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH,   3840)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT,  2160)
        self.cap.set(cv.CAP_PROP_FPS,           self.config.camera_fps)
        self.cap.set(cv.CAP_PROP_ZOOM,          400)
        self.cap.set(cv.CAP_PROP_AUTOFOCUS,     0.0)
        self.cap.set(cv.CAP_PROP_FOCUS,         255.0)
        
        for _ in range(30): # Allow camera to stabilize
            self.cap.read()
        self._print_camera_state(self.cap, "LOCKED BASELINE")    
        
        print(f"Camera initialized: {int(self.cap.get(cv.CAP_PROP_FRAME_WIDTH))}x"
              f"{int(self.cap.get(cv.CAP_PROP_FRAME_HEIGHT))}")
    
    def _print_camera_state(self, cap, label="Camera State"):
        """Print detailed camera properties"""
        props = {
            "Backend":              cap.getBackendName(),
            "Frame Width":          cap.get(cv.CAP_PROP_FRAME_WIDTH),
            "Frame Height":         cap.get(cv.CAP_PROP_FRAME_HEIGHT),
            "FPS":                  cap.get(cv.CAP_PROP_FPS),
            "Zoom":                 cap.get(cv.CAP_PROP_ZOOM),
            "Focus (Auto)":         cap.get(cv.CAP_PROP_AUTOFOCUS),
            "Focus (Manual)":       cap.get(cv.CAP_PROP_FOCUS),
        }
        print(f"\n{'='*40}\n  {label}\n{'='*40}")
        for name, val in props.items():
            print(f"  {name:<22}: {val:>10}")
        print(f"{'='*40}\n")
        return props
    
    def get_display_frame(self, frame):
        """    Resize frame for display while preserving aspect ratio.
        Processing always uses the original full-res frame."""
        h, w = frame.shape[:2]
        scale = min(self.display_width/ w, self.display_height / h)
        new_w, new_h = int(w * scale), int(h * scale)
        resized = cv.resize(frame, (new_w, new_h), interpolation=cv.INTER_AREA)

        canvas = np.zeros((self.display_height, self.display_width), dtype=np.uint8) \
                if len(frame.shape) == 2 \
                else np.zeros((self.display_height, self.display_width, 3), dtype=np.uint8)

        y_off = (self.display_height - new_h) // 2
        x_off = (self.display_width - new_w) // 2
        canvas[y_off:y_off+new_h, x_off:x_off+new_w] = resized
        return canvas, scale, x_off, y_off
    
    def draw_edges_on_roi(self, roi_bgr, top_edges, bot_edges, diameter_px):
        """Overlay sub-pixel edge positions as colored lines on a GRAY ROI copy."""
        out = roi_bgr.copy()
        h, w = out.shape[:2]
        for col in range(w):
            if not np.isnan(top_edges[col]):
                y = int(round(top_edges[col]))
                if 0 <= y < h:
                    out[max(0, y-1):y+2, col] = (0, 255, 0)    # green = top edge
            if not np.isnan(bot_edges[col]):
                y = int(round(bot_edges[col]))
                if 0 <= y < h:
                    out[max(0, y-1):y+2, col] = (0, 100, 255)  # orange = bottom edge
        # Solid median lines so they're visible even on a busy ROI
        if not np.isnan(top_edges).all():
            med_top = int(round(np.nanmedian(top_edges)))
            cv.line(out, (0, med_top), (w-1, med_top), (0, 255, 0), 2)
        if not np.isnan(bot_edges).all():
            med_bot = int(round(np.nanmedian(bot_edges)))
            cv.line(out, (0, med_bot), (w-1, med_bot), (0, 120, 255), 2)
        if not np.isnan(diameter_px):
            cv.putText(out, f"{diameter_px:.1f} px",
                       (10, 40), cv.FONT_HERSHEY_SIMPLEX, 0.65, (200, 200, 200), 2)
        return out

    def get_full_frame_visualization(self):
        """Full-frame view with the ROI box and median edge lines projected
        into display space, built from the most recent measurement."""
        if self.last_gray_full is None or self.last_roi_bounds is None:
            return None

        ry1, ry2, rx1, rx2 = self.last_roi_bounds
        display_gray, scale, x_off, y_off = self.get_display_frame(self.last_gray_full)
        display_bgr = cv.cvtColor(display_gray, cv.COLOR_GRAY2BGR)
        dx = int(rx1 * scale) + x_off
        dy = int(ry1 * scale) + y_off
        dw = int((rx2 - rx1) * scale)
        dh = int((ry2 - ry1) * scale)
        cv.rectangle(display_bgr, (dx, dy), (dx + dw, dy + dh), (255, 120, 0), 2)

        if self.last_top_edges is not None and not np.isnan(self.last_top_edges).all():
            mt = ry1 + int(round(np.nanmedian(self.last_top_edges)))
            mt_d = int(mt * scale) + y_off
            cv.line(display_bgr, (dx, mt_d), (dx + dw, mt_d), (0, 255, 0), 1)
        if self.last_bot_edges is not None and not np.isnan(self.last_bot_edges).all():
            mb = ry1 + int(round(np.nanmedian(self.last_bot_edges)))
            mb_d = int(mb * scale) + y_off
            cv.line(display_bgr, (dx, mb_d), (dx + dw, mb_d), (0, 120, 255), 1)
                    
        if self.config.calibration_pixels_per_mm is not None and not np.isnan(self.last_diameter_px):
            diameter_mm = self.last_diameter_px / self.config.calibration_pixels_per_mm
            diam_str = f"{diameter_mm:.3f} mm"
        else:
            diameter_mm = None
            diam_str = (f"{self.last_diameter_px:.1f} px  (not calibrated)"
                    if not np.isnan(self.last_diameter_px) else "no edges detected")
        cv.putText(display_bgr, f"Diameter: {diam_str}", (20, 28),
                   cv.FONT_HERSHEY_SIMPLEX, 0.55, (255, 117, 24), 2)
        return display_bgr

    def get_sobel_debug_visualization(self):
        """Normalized |Sobel dy| view for debugging edge detection, built
        from the most recent measurement."""
        if self.last_sobel is None:
            return None
        sobel_disp = cv.normalize(np.abs(self.last_sobel), None, 0, 255, cv.NORM_MINMAX).astype(np.uint8)
        h, w = sobel_disp.shape[:2]
        s = min(960 / w, 500 / h, 1.0)
        return cv.resize(sobel_disp, (int(w * s), int(h * s)), interpolation=cv.INTER_CUBIC)

    def find_edges_subpixel(self, roi_gray: np.ndarray, blur_ksize: int = 5,
                           sobel_ksize: int = 5) -> Tuple[np.ndarray, np.ndarray, float, np.ndarray]:
        """Find top and bottom edges with sub-pixel precision"""
        blurred     = cv.GaussianBlur(roi_gray, (blur_ksize, blur_ksize), 0)
        sobel       = cv.Sobel(blurred, cv.CV_64F, 0, 1, ksize=sobel_ksize)
        h, w        = sobel.shape
        top_edges   = np.full(w, np.nan)
        bot_edges   = np.full(w, np.nan)

        # Edge detection polarity
        if self.config.bright_on_dark:
            top_finder, top_positive = np.argmax, True
            bot_finder, bot_positive = np.argmin, False
        else:
            top_finder, top_positive = np.argmin, False
            bot_finder, bot_positive = np.argmax, True
        
        # Top edge detection
        cols = np.arange(w)
        top_idx = top_finder(sobel, axis=0)
        top_val = sobel[top_idx, cols]
        top_sign_ok = (top_val > 0) if top_positive else (top_val < 0)
        top_bounds_ok = (top_idx >= 1) & (top_idx <= h - 2)
        top_valid = top_sign_ok & top_bounds_ok
        
        top_idx_c = np.clip(top_idx, 1, h - 2)
        y0 = sobel[top_idx_c - 1,   cols]
        y1 = sobel[top_idx_c,       cols]
        y2 = sobel[top_idx_c + 1,   cols]
        denom = y0 - 2 * y1 + y2
        safe_denom = np.where(denom == 0, 1, denom)
        top_subpix = top_idx_c - 0.5 * (y2 - y0) / safe_denom
        top_subpix = np.where(denom == 0, top_idx_c.astype(float), top_subpix)
        top_edges = np.where(top_valid, top_subpix, np.nan)
        
        # ── BOTTOM EDGE: search window starts below the top edge per column ────
        safe_top = np.nan_to_num(top_edges, nan=(h // 2) - 5) # dummy fill
        search_start = safe_top.astype(int) + 100 # how far below to top to start searching for bottom edge
        window_ok = search_start <= h - 3
        
        rows = np.arange(h)[:, None]
        mask = rows < search_start[None, :]
        sobel_search = sobel.copy()
        sobel_search[mask] = np.inf if bot_finder is np.argmin else -np.inf
        
        bot_idx = bot_finder(sobel_search, axis=0)
        bot_val = sobel[bot_idx, cols]
        bot_sign_ok = (bot_val < 0) if not bot_positive else (bot_val > 0)
        bot_bounds_ok = (bot_idx >= 1) & (bot_idx <= h - 2)
        bot_valid = bot_sign_ok & bot_bounds_ok & window_ok
        
        bot_idx_c = np.clip(bot_idx, 1, h - 2)
        y0b     = sobel[bot_idx_c - 1,  cols]
        y1b     = sobel[bot_idx_c,      cols]
        y2b     = sobel[bot_idx_c + 1,  cols]
        denomb  = y0b - 2 * y1b + y2b
        safe_denomb = np.where(denomb == 0, 1, denomb)
        bot_subpix  = bot_idx_c - 0.5 * (y2b - y0b) / safe_denomb
        bot_subpix  = np.where(denomb == 0, bot_idx_c.astype(float), bot_subpix)
        bot_edges   = np.where(bot_valid, bot_subpix, np.nan)
        
        # Calculate median diameter
        valid = ~(np.isnan(top_edges) | np.isnan(bot_edges))
        if valid.sum() < 10:
            return top_edges, bot_edges, np.nan, sobel
            # Reject outliers: columns where diameter deviates >15% from the median
        diameters   = bot_edges[valid] - top_edges[valid]
        med         = np.median(diameters)
        inliers     = np.abs(diameters - med) < 0.15 * med
        diameter_px = np.median(diameters[inliers]) if inliers.sum() > 0 else med
    # ''' --------------- do YOU really want to use MEDIAN VALUE ------------------ ''' 
        return top_edges, bot_edges, diameter_px, sobel

    def measure_diameter(self) -> Optional[float]:
        ret, frame = self.cap.read()
        if not ret:
            print("Warning: Could not read camera frame")
            return None
        
        self.elapsed = time.time() - self.start_time
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        
        # Extract ROI
        h, w = gray.shape
        ry1 = max(0, self.config.roi_y)
        ry2 = min(h, self.config.roi_y + self.config.roi_h)
        rx1 = max(0, self.config.roi_x)
        rx2 = min(w, self.config.roi_x + self.config.roi_w)
        roi_gray = gray[ry1:ry2, rx1:rx2]
        roi_bgr  = frame[ry1:ry2, rx1:rx2] 

        # Find edges
        top_edges, bot_edges, diameter_px, sobel = self.find_edges_subpixel(roi_gray)
        
        # Cache everything needed for visualization from *this* frame, so a display step can draw the edges on the same frame they were
        # measured from rather than reading a new (mismatched) frame.
        self.last_frame         = frame
        self.last_gray_full     = gray
        self.last_roi_bgr       = roi_bgr
        self.last_top_edges     = top_edges
        self.last_bot_edges     = bot_edges
        self.last_diameter_px   = diameter_px
        self.last_sobel         = sobel
        self.last_roi_bounds    = (ry1, ry2, rx1, rx2)

        if np.isnan(diameter_px):
            return None
        
        # Convert to mm if calibrated
        if self.config.calibration_pixels_per_mm is not None:
            diameter_mm = diameter_px / self.config.calibration_pixels_per_mm
            if not (self.config.plausible_diameter_min_mm <= diameter_mm <= self.config.plausible_diameter_max_mm):
                return None
        else:
            # If not calibrated, return pixels
            diameter_mm = diameter_px
        
        # Print status every 2 seconds
        if self.elapsed - self.last_status_t >= 3.0:
            if self.config.calibration_pixels_per_mm is not None:
                print(f"t={self.elapsed:.1f}s | diameter: {diameter_px:.2f} px ({diameter_mm:.3f} mm)")
            else:
                print(f"t={self.elapsed:.1f}s | diameter: {diameter_px:.2f} px (not calibrated)")
            self.last_status_t = self.elapsed

        # Store in history for averaging
        now = time.time()
        self.measurement_history.append((now, diameter_mm))
        cutoff = now - self.config.history_retention_s
        self.measurement_history = [(t, d) for t, d in self.measurement_history 
                                  if t > cutoff]
        return diameter_mm
    
    def get_average_diameter(self) -> Optional[float]:
        """Get average diameter over recent measurements"""
        if not self.measurement_history:
            return None
        cutoff = time.time() - self.config.history_retention_s
        recent = [d for t, d in self.measurement_history if t > cutoff]  # time based retention
        return np.mean(recent) if recent else None
    
    def check_for_rapid_change(self) -> bool:
        """Check if diameter is changing too rapidly"""
        if not self.measurement_history:
            return False
        cutoff = time.time() - 3 # 3 seconds
        recent = [d for t, d in self.measurement_history if t > cutoff]
        if len(recent) < 15: # min number of samples
            return False
        return np.mean(np.abs(np.diff(recent))) > self.config.rapid_change_threshold_mm
    
    def cleanup(self):
        if self.cap:
            self.cap.release()

# ──────────────────────────────────────────────────────────────────────────────
# MOTOR CONTROL COMPONENTS (adapted from Pot_Motor_Ctrl.py and Wrapper_HardwarePWM)
# ──────────────────────────────────────────────────────────────────────────────

class MotorController:
    """Controls motor speed via hardware PWM"""
    
    def __init__(self, config: ControlConfig):
        self.config         = config
        self.motor_in1      = None
        self.motor_in2      = None
        self.current_speed  = 0.0
        self.is_running     = False
        self._init_motors()
    
    def _init_motors(self):
        """Initialize PWM hardware"""
        try:
            self.motor_in1 = HardwarePWM(
                chip=self.config.chip_,
                channel=self.config.pwm_in1_channel,
                frequency=self.config.pwm_frequency_hz)
            self.motor_in2 = HardwarePWM(
                chip=self.config.chip_,
                channel=self.config.pwm_in2_channel,
                frequency=self.config.pwm_frequency_hz)
            self.motor_in1.start(0)
            self.motor_in2.start(0)
            print("Motor controller initialized")
        except Exception as e:
            print(f"Error initializing motor controller: {e}")
            raise

    def _speed_pct_to_duty(self, speed_pct: float) -> float:
        """Map a normalized speed (config.speed_pct_min-config.speed_pct_max)
        onto the hardware's clamped duty-cycle range (duty_min_percent-duty_max_percent)."""
        pct_span = self.config.speed_pct_max - self.config.speed_pct_min
        duty_span = self.config.duty_max_percent - self.config.duty_min_percent
        fraction = (speed_pct - self.config.speed_pct_min) / pct_span
        return self.config.duty_min_percent + fraction * duty_span

    # def _duty_to_speed_pct(self, duty_percent: float) -> float:
    #     """Inverse of _speed_pct_to_duty."""
    #     pct_span = self.config.speed_pct_max - self.config.speed_pct_min
    #     duty_span = self.config.duty_max_percent - self.config.duty_min_percent
    #     fraction = (duty_percent - self.config.duty_min_percent) / duty_span
    #     return self.config.speed_pct_min + fraction * pct_span

    def set_speed(self, speed_pct: float):
        speed_pct = max(self.config.speed_pct_min, 
                           min(self.config.speed_pct_max, speed_pct))
        duty_percent = self._speed_pct_to_duty(speed_pct)
        self.current_speed = speed_pct
        self.motor_in1.ChangeDutyCycle(duty_percent) # Forward direction only
        self.motor_in2.ChangeDutyCycle(0)
    
    def stop(self):
        self.motor_in1.ChangeDutyCycle(0)
        self.motor_in2.ChangeDutyCycle(0)
        self.current_speed  = 0.0
        self.is_running     = False
    
    def start(self, initial_speed: float = None):
        if initial_speed is None:
            initial_speed = self.config.speed_manual_default
        self.set_speed(initial_speed)
        self.is_running = True

    # def debug_duty(self):
    #     """ Print exactly what values are being computed and sent """
    #     for speed in [1.0, 50.0, 100.0]:
    #         duty = self._speed_pct_to_duty(speed)
    #         print(f"  speed {speed:6.1f} → duty {duty:.4f}%")
    #     print(f"  current_speed={self.current_speed}, is_running={self.is_running}")

    def cleanup(self):
        self.stop()
        if self.motor_in1: self.motor_in1.stop()
        if self.motor_in2: self.motor_in2.stop()

# ──────────────────────────────────────────────────────────────────────────────
# PID CONTROLLER
# ──────────────────────────────────────────────────────────────────────────────
##################################### UPDATE FOR INTEGRAL ONLY ADJUSTMENT #########################################
class PIDController:
    """PID controller for motor speed adjustment"""
    
    def __init__(self, config: ControlConfig):
        self.kp = config.pid_kp
        self.ki = config.pid_ki
        self.kd = config.pid_kd
        self.max_integral = config.max_integral_windup

        self.reset()
    
    def compute(self, setpoint: float, measurement: float, 
                current_speed: float) -> float:
        """Compute new speed based on error"""
        current_time = time.time()
        dt = current_time - self.previous_time
        if dt <= 0:
            return current_speed
        
        # Calculate error
        self.error = setpoint - measurement
        
        # Proportional
        self.proportional = self.kp * self.error

        # Integral with anti-windup
        self.integral += self.error * dt
        self.integral = max(-self.max_integral,
                            min( self.max_integral, self.integral))
        self.integral_term = self.ki * self.integral

        # Derivative
        self.derivative = self.kd * (self.error - self.previous_error) / dt

        self.previous_error = self.error
        self.previous_time  = current_time

        return self.proportional + self.integral_term + self.derivative

    def reset(self):
        self.integral       = 0.0
        self.previous_error = 0.0
        self.previous_time  = time.time()
        self.error          = 0.0
        self.proportional   = 0.0
        self.integral_term  = 0.0   # named differently to avoid shadowing self.integral
        self.derivative     = 0.0


class SpoolerAutomation:
    """Main spooler automation system"""
    
    def __init__(self, filament_config: FilamentConfig, 
                 control_config: ControlConfig):
        self.filament_config = filament_config
        self.control_config = control_config
        
        # Initialize components
        self.ui            = UIWrapper(control_config, status_lines=17)
        self.led           = self.ui.led
        self.keyboard      = self.ui.keyboard
        self.logger        = self.ui.logger
        self.measurement   = FilamentMeasurement(filament_config)
        self.motor         = MotorController(control_config)
        self.pid           = PIDController(control_config)
        
        # State variables
        self.mode                       = OperatingMode.MANUAL
        self.last_measurement_time      = 0.0
        self.new_measurement_available  = False
        self.last_emergency_check_time  = time.time()
        self.last_control_update        = 0.0
        self.last_status_display_time   = 0.0
        self.last_motor_adjustment_time = 0.0
        self.emergency_stop_timer       = 0.0
        self.user_intervention_required = False
        self.display_enabled            = True

        # Sanity check: measurement rate can never exceed the camera's actual
        # capture rate - if it does, cap.read() would just be re-reading the
        # same buffered frame faster than a new one arrives.
        measurement_fps = 1.0 / control_config.measurement_interval_s
        if measurement_fps > filament_config.camera_fps:
            print(f"WARNING: measurement rate ({measurement_fps:.1f} FPS) exceeds "
                  f"camera_fps ({filament_config.camera_fps} FPS) - lower "
                  f"measurement_interval_s or raise camera_fps.")
        
        print("Spooler Automation System with LED Initialized")
        print(f"Target diameter: {filament_config.target_diameter_mm} mm | "
            f"Tolerance: {filament_config.tolerance_min_mm} - "
            f"{filament_config.tolerance_max_mm} mm")
        print(f"Measurement rate: {measurement_fps:.1f} FPS (camera: {filament_config.camera_fps} FPS)")

    def run(self):
        self.ui.startup()
        self._display_status()

        try:
            # Initial motor check
            self._check_camera_and_initialize()
            
            while True:
                # Check for user input
                self._process_user_commands()
                current_time = time.time()

                # Perform measurement periodically
                if current_time - self.last_measurement_time >= self.control_config.measurement_interval_s: 
                    self._perform_measurement()
                    self.last_measurement_time = current_time
                    self.new_measurement_available = True

                    if self.display_enabled:
                        full_view = self.measurement.get_full_frame_visualization()
                        if full_view is not None:
                            cv.imshow("Full frame + ROI box", full_view)
                        sobel_view = self.measurement.get_sobel_debug_visualization()
                        if sobel_view is not None:
                            cv.imshow("Sobel magnitude |dy|", sobel_view)

                # Control logic based on mode
                if self.mode == OperatingMode.AUTOMATIC:
                    self._automatic_mode_loop(current_time)
                elif self.mode == OperatingMode.FAILSAFE:
                    self._failsafe_mode_loop()
                
                # Display status
                self._display_status()
                time.sleep(0.005)                
        except KeyboardInterrupt:
            print("\nShutting down...")
        finally:
            self.cleanup()
        
    # --- Startup -----------------------------------------
    def _check_camera_and_initialize(self):
        """Check camera status and initialize motor if ready"""
        ret, frame = self.measurement.cap.read()
        if not ret:
            print("ERROR: Camera not working. Motor not initialized.")
            return False
        print("Camera is working. Initializing motor...")
        print("Beging feeding filament... ")
        self.motor.start(self.control_config.speed_manual_default)
        # self.motor.debug_duty()
        return True
    
    def _perform_measurement(self):
        """Perform diameter measurement and check for failures"""
        diameter = self.measurement.measure_diameter()
        if diameter is not None:
            self.logger.add_entry(
                self.measurement,
                self.filament_config,
                self.motor,
                self.pid,
                self.mode
            )
        now = time.time()
        if diameter is None:
            self.emergency_stop_timer += now - self.last_emergency_check_time
            self.last_emergency_check_time = now
            if self.emergency_stop_timer > self.control_config.emergency_stop_timeout_s:
                print("ERROR: No valid measurement for extended period")
                self._enter_failsafe_mode()
            return
        # Reset emergency timer on valid measurement
        self.emergency_stop_timer = 0.0
        self.last_emergency_check_time = now
        
        # Check for out-of-tolerance conditions ####################################### ADD BACK LATER
        # if (diameter < self.filament_config.tolerance_min_mm or 
        #     diameter > self.filament_config.tolerance_max_mm):
        #     print(f"WARNING: Diameter {diameter:.3f} mm out of tolerance!")
        #     self._enter_failsafe_mode()
        #     return
        
        # Check for rapid changes
        if self.measurement.check_for_rapid_change():
            print("WARNING: Rapid diameter change detected! - switchng to MANUAL")
            self.user_intervention_required = True
            self.mode = OperatingMode.MANUAL
            self.led.stop_blink()
            self.led.on()
    
    def _automatic_mode_loop(self, current_time: float):
        """Automatic mode control logic with PID"""
        if not self.new_measurement_available:
            return
        self.new_measurement_available = False

        in_response_delay = (current_time - self.last_motor_adjustment_time < 
                             self.control_config.response_delay_s)
        if in_response_delay:
            self.last_control_update = current_time
            return

        # Get average diameter for control
        avg_diameter = self.measurement.get_average_diameter()
        if avg_diameter is None:
            return
        
        # Compute new speed using PID
        raw_adjustment = self.pid.compute(self.filament_config.target_diameter_mm,
            avg_diameter, self.motor.current_speed)
        raw_adjustment = -raw_adjustment
        
        # Clamp to the max allowed step size
        step = self.control_config.max_step_speed_pct
        adjustment = max(-step, min(step, raw_adjustment))

        new_speed = max(self.control_config.speed_pct_min,
                        min(self.control_config.speed_pct_max,
                            self.motor.current_speed + adjustment))
        
        # Apply change if significant
        if abs(new_speed - self.motor.current_speed) > self.control_config.significant_speed_change:
            print(f"PID (speed): {self.motor.current_speed:.4f} -> "
                  f"{new_speed:.4f} /100 (Diameter: {avg_diameter:.3f} mm)")
            self.motor.set_speed(new_speed)
            self.last_motor_adjustment_time = current_time
            self.led.blink(interval=0.3, duration=1.0)  # Flash LED
        self.last_control_update = current_time
        return avg_diameter
    
    def _failsafe_mode_loop(self):
        """Failsafe mode - motor stopped, waiting for user"""
        if self.motor.is_running:
            print("ENTERING FAILSAFE MODE - Motor stopped")
            self.motor.stop()

    def _enter_failsafe_mode(self):
        """Enter failsafe mode with motor stop"""
        self.mode = OperatingMode.FAILSAFE
        self.user_intervention_required = True
        if self.motor.is_running:
            self.motor.stop()
        self.led.blink(interval=0.15)
        print("FAILSAFE ACTIVATED - Check filament and press 'M' to acknowledge")
    
    def _process_user_commands(self):
        """Process keyboard commands"""
        # Pump the OpenCV GUI event loop so any visualization windows actually redraw. We do NOT rely on its return value for key
        # capture -- cv.waitKey() only sees a keypress when a HighGUI window has OS focus, which is unreliable for a script run from
        # a terminal/SSH session. Actual key capture below comes from KeyboardReader, which reads stdin directly and works regardless
        # of window focus.
        if self.display_enabled:
            cv.waitKey(1)

        key_char = self.keyboard.get_key(timeout=0)
        if not key_char:
            return
        key_char = key_char.lower()
        
        if key_char == 'q':
            self.logger.stop_logging()
            raise KeyboardInterrupt
        elif key_char == 'm':
            self.mode = OperatingMode.MANUAL
            if self.user_intervention_required:
                print("Failsafe acknowledged. Verify filament before resuming.")
                self.user_intervention_required = False
            self.led.stop_blink()  # Stop any failsafe/auto blinking cleanly
            self.led.on()  # Turn LED solid for manual mode
            print("Mode: MANUAL (LED ON)")
        elif key_char == 'a':
            if self.user_intervention_required:
                print("User intervention required. Fix issue first.")
                self.led.pulse(count=3, speed=0.1)  # Error indication
            else:
                self.mode = OperatingMode.AUTOMATIC
                self.pid.reset()
                self.led.pulse(count=2, speed=0.3)  # Quick blink for auto mode
                print("Mode: AUTOMATIC (PID reset)")
                if not self.logger.logging:
                    self.logger.start_logging()
                    print(f"Data logging STARTED -> {self.logger.file_path}")
        elif key_char == 's':
            if self.motor.is_running:
                self.motor.stop()
                self.led.pulse(count=2, speed=0.2)  # Stop indication
                print("Motor STOPPED")
            else:
                self.motor.start()
                self.led.on()  # Turn LED on when motor starts
                print("Motor STARTED")
        elif key_char == '+':
            if self.mode != OperatingMode.FAILSAFE:
                self.mode = OperatingMode.MANUAL
                self.pid.reset()
                new_speed = min(self.control_config.speed_pct_max,
                              self.motor.current_speed + self.control_config.max_step_speed_pct)
                self.motor.set_speed(new_speed)
                self.led.pulse(count=1, speed=0.1)
                print(f"Manual speed: {new_speed:.4f} /100")
        elif key_char == '-':
            if self.mode != OperatingMode.FAILSAFE:
                self.mode = OperatingMode.MANUAL
                self.pid.reset()
                new_speed = max(self.control_config.speed_pct_min,
                              self.motor.current_speed - self.control_config.max_step_speed_pct)
                self.motor.set_speed(new_speed)
                self.led.pulse(count=1, speed=0.1)
                print(f"Manual speed: {new_speed:.4f} /100")
        elif key_char == 'r':
            if self.logger.logging:
                print("Already logging.")
            else:
                self.logger.start_logging()
                self.led.pulse(count=1, speed=0.15)
                print(f"Data logging STARTED -> {self.logger.file_path}")

    def _display_status(self):
        """Display current system status"""
        if not self.display_enabled:
            return
        current_time = time.time()
        if current_time - self.last_status_display_time < 0.5:
            return
        self.last_status_display_time = current_time

        lines = []
        lines.append("="*40)
        lines.append("SPOOLER AUTOMATION SYSTEM - STATUS")
        lines.append("-"*40)
        lines.append("Commands: A=Auto,     M=Manual,       +/-=Adjust")
        lines.append("          R=Record,   S=Start/Stop,   Q=Quit")
        lines.append("-"*40)
        lines.append("LED Status Indicator Guide:")
        lines.append("SOLID - Manual | SLOW BLINK - Automatic")
        lines.append("FAST BLINK - System adjusting | RAPID BLINK - Failsafe")
        lines.append("-"*40)
        
        mode_str = self.mode.value.upper()
        motor_str = "RUNNING" if self.motor.is_running else "STOPPED"
        if self.user_intervention_required:
            mode_str += " - INTERVENTION REQUIRED"
        lines.append(f"Mode    : {mode_str}")
        lines.append(f"Motor   : {motor_str} at {self.motor.current_speed:.4f} / 100")

        # Measurement status
        avg = self.measurement.get_average_diameter()
        if avg is not None:
            error = avg - self.filament_config.target_diameter_mm
            lines.append(f"Diameter: {avg:.3f} mm (Target: {self.filament_config.target_diameter_mm} mm)")
            lines.append(f"Error: {error:+.3f} mm")
            bar_pos = int((error + 0.1) / 0.2 * 40)
            bar_pos = max(0, min(40, bar_pos))
            bar = "[" + "─" * bar_pos + "│" + "─" * (40 - bar_pos) + "]"
            lines.append(f"         {bar}")
        else:
            lines.append("Diameter: No valid measurement")
            lines.append("")
            lines.append("") # placeholder for bar row

        lines.append("-"*40)
        self.ui.status.draw_status(lines)
    
    def cleanup(self):
        print("Cleaning up resources...")
        self.logger.stop_logging()
        self.keyboard.stop()
        self.motor.cleanup()
        self.measurement.cleanup()
        self.led.cleanup()
        cv.destroyAllWindows()
        self.ui.status.stop()
        print("Cleanup complete.")

# ──────────────────────────────────────────────────────────────────────────────
# MAIN ENTRY POINT
# ──────────────────────────────────────────────────────────────────────────────

def main():
    try:
        # Create and run automation system
        SpoolerAutomation(FilamentConfig(), ControlConfig()).run()
    except Exception as e:
        print(f"Fatal error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    return 0

if __name__ == "__main__":
    sys.exit(main())
