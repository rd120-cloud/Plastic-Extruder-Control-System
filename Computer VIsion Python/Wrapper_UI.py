"""
UI_Wrapper: consolidates the spooler's UI / IO peripheral classes
(LED status light, keyboard input, CSV logging, and the scrolling
terminal status panel) behind a single UIWrapper facade.

Pulled out of Full_Op_Rework.py so the main file only has to deal with
one object (`UIWrapper`) instead of four separate ones.

`from __future__ import annotations` is used so the type hints in
LogData.add_entry() (FilamentMeasurement, FilamentConfig, MotorController,
PIDController, OperatingMode) don't need to be imported here -- they stay
as plain strings and are never evaluated at runtime. This avoids a
circular import with the main file.
"""
from __future__ import annotations

import csv
import os
import select
import sys
import termios
import threading
import time
import tty
from datetime import datetime
from pathlib import Path
from typing import List, TYPE_CHECKING

import lgpio

if TYPE_CHECKING:
    # Only imported for type-checkers/IDEs; not needed at runtime because
    # of the `from __future__ import annotations` above.
    from Full_Op_Rework import (
        ControlConfig,
        FilamentConfig,
        FilamentMeasurement,
        MotorController,
        OperatingMode,
        PIDController,
    )

# ──────────────────────────────────────────────────────────────────────────────
# LED STATUS INDICATOR
# ──────────────────────────────────────────────────────────────────────────────

class LedController:
    """Controls LED on GPIO 17 for status indication"""

    def __init__(self, pin: int, gpiochip: int):
        self.pin            = pin
        self.gpiochip       = gpiochip
        self.chip_handle    = None
        self.blink_thread   = None
        self.blink_running  = False
        self._init_gpio()

    def _init_gpio(self):
        """Initialize GPIO for LED control"""
        self.chip_handle = lgpio.gpiochip_open(self.gpiochip)
        lgpio.gpio_claim_output(self.chip_handle, self.pin, 0)
        print(f"LED initialized on GPIO {self.pin} (gpiochip{self.gpiochip})")

    def on(self):
        """Turn LED on"""
        lgpio.gpio_write(self.chip_handle, self.pin, 1)

    def off(self):
        """Turn LED off"""
        lgpio.gpio_write(self.chip_handle, self.pin, 0)

    def blink(self, interval: float = 0.5, duration: float = None):
        """
        Blink LED at specified interval
        interval: seconds between on/off cycles
        duration: total blink duration (None = until stop_blink() called)
        """
        self.stop_blink()  # Stop any existing blink thread
        self.blink_running = True
        self.blink_thread = threading.Thread(
            target=self._blink_worker, args=(interval, duration), daemon=True)
        self.blink_thread.start()

    def _blink_worker(self, interval: float, duration: float):
        """Worker thread for blinking LED"""
        start_time = time.time()
        while self.blink_running:
            # Check duration limit
            if duration and (time.time() - start_time) >= duration:
                self.blink_running = False
                self.off()
                break
            # Blink cycle
            self.on()
            time.sleep(interval / 2)
            if not self.blink_running:
                break
            self.off()
            time.sleep(interval / 2)

    def stop_blink(self):
        """Stop blinking and turn LED off"""
        self.blink_running = False
        if self.blink_thread and self.blink_thread.is_alive():
            self.blink_thread.join(timeout=1.0)
        self.off()

    def pulse(self, count: int = 1, speed: float = 0.2):
        """
        Create a pulse effect (quick on/off)
        count: number of pulses
        speed: time for each pulse (seconds)
        """
        for i in range(count):
            self.on()
            time.sleep(speed)
            self.off()
            if i < count - 1:  # Don't sleep after last pulse
                time.sleep(speed)

    def cleanup(self):
        """Clean up GPIO resources"""
        self.stop_blink()
        if self.chip_handle is not None:
            lgpio.gpio_free(self.chip_handle, self.pin)
            lgpio.gpiochip_close(self.chip_handle)
            self.chip_handle = None

# ──────────────────────────────────────────────────────────────────────────────
# CSV DATA LOGGING
# ──────────────────────────────────────────────────────────────────────────────

class LogData:
    def __init__(self):
        self.folder = Path.home() / "Camera_Vis_Filament_Data"
        self.folder.mkdir(exist_ok=True)
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M")
        self.file_path = self.folder / f"{timestamp}_extrusion_data_log.csv"
        self.logging = False
        self.data = []

    def start_logging(self):
        self.logging = True

    def stop_logging(self):
        if not self.logging:
            return
        self.logging = False
        self.save()

    def add_entry(self, meas: FilamentMeasurement, config: FilamentConfig,
                  motor: MotorController, pid: PIDController, mode: OperatingMode):
        if not self.logging:
            return
        diameter_mm = (meas.last_diameter_px / config.calibration_pixels_per_mm
                       if config.calibration_pixels_per_mm else meas.last_diameter_px)
        error = config.target_diameter_mm - diameter_mm
        
        self.data.append([
            time.time(),
            meas.elapsed,
            config.target_diameter_mm,
            diameter_mm,
            error,
            meas.last_diameter_px,
            motor.current_speed,
            mode.value,
            pid.proportional,
            pid.integral_term,
            pid.derivative
        ])

    def save(self):
        with open(self.file_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["Time", "Elapsed_s", "Target_mm", "Diameter_mm", "Error_mm", "Diameter_px",
                             "MotorSpeed_pct", "Mode", "P-term", "I-term", "D-term"])
            writer.writerows(self.data)
        print(f"Saved {len(self.data)} measurements to")
        print(self.file_path)

# ──────────────────────────────────────────────────────────────────────────────
# KEYBOARD INPUT
# ──────────────────────────────────────────────────────────────────────────────

class KeyboardReader:
    """Non-blocking single-character terminal keyboard reader.

    Reads keypresses directly from stdin using cbreak mode, so it works from a plain SSH/terminal session and does NOT require any OpenCV
    GUI window to have OS focus -- unlike cv.waitKey(), which only sees keypresses when a HighGUI window is the focused window. cbreak mode
    (not raw mode) is used so Ctrl-C still raises KeyboardInterrupt normally.

    If stdin isn't an interactive TTY (e.g. running as a systemd service, or redirected input), this disables itself gracefully instead of
    crashing -- in that case, keyboard control simply isn't available and the system will need another input method (GUI window focus, a
    physical button on GPIO, etc).
    """

    def __init__(self):
        self.fd             = None
        self.old_settings   = None
        self.enabled        = False

    def start(self):
        try:
            if not sys.stdin.isatty():
                print("Warning: stdin is not an interactive terminal -- "
                      "keyboard control is disabled.")
                return
            self.fd           = sys.stdin.fileno()
            self.old_settings = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
            self.enabled = True
            # print("Terminal keyboard input ready (works even without a GUI window focused).")
        except Exception as e:
            print(f"Warning: could not set up terminal keyboard input: {e}")
            self.enabled = False

    def stop(self):
        """Restore normal terminal settings (line-buffered, echoing input)."""
        if self.enabled and self.old_settings is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
        self.enabled = False

    def get_key(self, timeout: float = 0.0):
        """Return a single character if one is waiting within `timeout`
        seconds, else None. Non-blocking when timeout=0."""
        if not self.enabled:
            return None
        ready, _, _ = select.select([sys.stdin], [], [], timeout)
        if ready:
            return sys.stdin.read(1)
        return None

# ──────────────────────────────────────────────────────────────────────────────
# SCROLLING TERMINAL STATUS PANEL
# ──────────────────────────────────────────────────────────────────────────────

class ScrollingStatusUI:
    """Splits the terminal into two regions using ANSI escape codes:

      - a SCROLLING region (top of the terminal) where ordinary print()
        calls keep behaving exactly as before -- they scroll up and are
        retained/scrollable, nothing gets erased.
      - a FIXED status panel (bottom `status_lines` rows) that is redrawn
        in place on top of itself, without touching the scroll region.

    This replaces the old approach of clearing the whole screen
    (\\033[2J\\033[H) every couple seconds to draw the status block, which
    destroyed all prior terminal output.

    If stdout isn't a real terminal (e.g. redirected to a log file, or run
    under a service manager), this disables itself and draw_status() just
    falls back to plain print() so nothing crashes and nothing is lost.
    """

    def __init__(self, status_lines: int = 17):
        self.status_lines = status_lines
        self.enabled       = sys.stdout.isatty()
        self.term_rows     = 24
        self.term_cols     = 80
        self.scroll_bottom = 0

    def _update_size(self):
        try:
            size = os.get_terminal_size(sys.stdout.fileno())
            self.term_cols, self.term_rows = size.columns, size.lines
        except OSError:
            pass

    def start(self):
        """Reserve the bottom `status_lines` rows for the status panel and
        confine normal scrolling output to the rows above it."""
        if not self.enabled:
            return
        self._update_size()
        self.scroll_bottom = max(1, self.term_rows - self.status_lines)
        # Set the scrolling region to rows [1, scroll_bottom]. Anything
        # printed via normal print()/stdout now scrolls only within that
        # band, leaving the rows below it (the status panel) untouched.
        sys.stdout.write(f"\033[1;{self.scroll_bottom}r")
        # Park the cursor at the bottom of the scroll region so the next
        # print() call appears right above the status panel, like normal.
        sys.stdout.write(f"\033[{self.scroll_bottom};1H")
        sys.stdout.flush()

    def draw_status(self, lines: List[str]):
        """Redraw the fixed status panel in place without disturbing the
        scrolled log history above it."""
        if not self.enabled:
            # No real terminal to do fancy positioning in -- just print
            # normally so the information isn't lost.
            print("\n".join(lines))
            return
        out = ["\0337"]  # save cursor position
        for i, line in enumerate(lines[: self.status_lines]):
            row = self.scroll_bottom + 1 + i
            # \033[2K clears that single row before rewriting it
            out.append(f"\033[{row};1H\033[2K{line}")
        out.append("\0338")  # restore cursor position (back in the scroll region)
        sys.stdout.write("".join(out))
        sys.stdout.flush()

    def stop(self):
        """Undo the scroll-region restriction so the terminal behaves
        normally again after the program exits."""
        if not self.enabled:
            return
        sys.stdout.write("\033[r")  # reset scroll region to full screen
        sys.stdout.write(f"\033[{self.term_rows};1H\n")
        sys.stdout.flush()

# ──────────────────────────────────────────────────────────────────────────────
# UI_WRAPPER FACADE
# ──────────────────────────────────────────────────────────────────────────────

class UIWrapper:
    """Single entry point for all UI/IO peripherals: LED, keyboard, CSV
    logger, and the scrolling terminal status panel.

    Sub-components stay reachable via attributes (`.led`, `.keyboard`,
    `.logger`, `.status`) so calling code reads the same as before
    (e.g. `ui.led.on()`, `ui.keyboard.get_key()`), just through one object
    instead of four.
    """

    def __init__(self, control_config: ControlConfig, status_lines: int = 14):
        self.led      = LedController(pin=control_config.led_pin,
                                       gpiochip=control_config.chip_)
        self.keyboard = KeyboardReader()
        self.logger   = LogData()
        self.status   = ScrollingStatusUI(status_lines=status_lines)

    def startup(self):
        """Bring up all UI/IO subsystems at the start of the run loop."""
        self.led.on()
        self.keyboard.start()
        self.status.start()

    def cleanup(self):
        """Tear down all UI/IO subsystems. Call after any other hardware
        (motor, camera) has already been cleaned up -- LED/status teardown
        is safe to do last."""
        self.logger.stop_logging()
        self.keyboard.stop()
        self.led.cleanup()
        self.status.stop()
