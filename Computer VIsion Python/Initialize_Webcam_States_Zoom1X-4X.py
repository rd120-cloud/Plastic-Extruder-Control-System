import cv2 as cv
import numpy as np
import time

def print_camera_state(cap, label="Camera State"):
    props = {
        "Backend":              cap.getBackendName(),
        "Frame Width":          cap.get(cv.CAP_PROP_FRAME_WIDTH),
        "Frame Height":         cap.get(cv.CAP_PROP_FRAME_HEIGHT),
        "FPS":                  cap.get(cv.CAP_PROP_FPS),
        "Zoom":                 cap.get(cv.CAP_PROP_ZOOM),
        "Focus (Auto)":         cap.get(cv.CAP_PROP_AUTOFOCUS),
        "Focus (Manual)":       cap.get(cv.CAP_PROP_FOCUS),
        "Exposure (Auto)":      cap.get(cv.CAP_PROP_AUTO_EXPOSURE),
        "Exposure":             cap.get(cv.CAP_PROP_EXPOSURE),
        "Brightness":           cap.get(cv.CAP_PROP_BRIGHTNESS),
        "Contrast":             cap.get(cv.CAP_PROP_CONTRAST),
        "Saturation":           cap.get(cv.CAP_PROP_SATURATION),
        "Sharpness":            cap.get(cv.CAP_PROP_SHARPNESS),
        "Gain":                 cap.get(cv.CAP_PROP_GAIN),
        "White Balance (Auto)": cap.get(cv.CAP_PROP_AUTO_WB),
        "White Balance":        cap.get(cv.CAP_PROP_WB_TEMPERATURE),
        "Pan":                  cap.get(cv.CAP_PROP_PAN),
        "Tilt":                 cap.get(cv.CAP_PROP_TILT),
    }
    print(f"\n{'='*40}\n  {label}\n{'='*40}")
    for name, val in props.items():
        print(f"  {name:<22}: {val:>10}")
    print(f"{'='*40}\n")
    return props

# --- Display config ---
DISPLAY_WIDTH  = 1280   # Change to fit your monitor
DISPLAY_HEIGHT = 720

def get_display_frame(frame, display_w=DISPLAY_WIDTH, display_h=DISPLAY_HEIGHT):
    """
    Resize frame for display while preserving aspect ratio.
    Adds letterbox/pillarbox padding if needed.
    Processing always uses the original full-res frame.
    """
    h, w = frame.shape[:2]
    scale = min(display_w / w, display_h / h)
    new_w, new_h = int(w * scale), int(h * scale)

    resized = cv.resize(frame, (new_w, new_h), interpolation=cv.INTER_AREA)

    # Center in a black canvas of exact display size
    canvas = np.zeros((display_h, display_w), dtype=np.uint8) \
             if len(frame.shape) == 2 \
             else np.zeros((display_h, display_w, 3), dtype=np.uint8)

    y_off = (display_h - new_h) // 2
    x_off = (display_w - new_w) // 2
    canvas[y_off:y_off+new_h, x_off:x_off+new_w] = resized

    return canvas, scale, x_off, y_off

# --- Init ---
cap = cv.VideoCapture(1, cv.CAP_DSHOW)
cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
cap.set(cv.CAP_PROP_FRAME_WIDTH, 3840)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 2160)
cap.set(cv.CAP_PROP_ZOOM, 100)
cap.set(cv.CAP_PROP_AUTOFOCUS, 0.0) # 0 turns off autofocus?
cap.set(cv.CAP_PROP_FOCUS, 255.0) # 190.0 is the highest it seems

for _ in range(30):
    cap.read()

baseline = print_camera_state(cap, "LOCKED BASELINE")

# Confirm actual capture resolution
cap_w = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
cap_h = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))
print(f"Capture resolution: {cap_w}x{cap_h}")

# Size the window once explicitly
cv.namedWindow("Zoom Test", cv.WINDOW_NORMAL)
cv.resizeWindow("Zoom Test", DISPLAY_WIDTH, DISPLAY_HEIGHT)

start_time = time.time()
zoom_changed = False
last_zoom_print = 0

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame")
        break

    elapsed = time.time() - start_time

    # --- All processing on full-res frame ---
    gray_frame = cap.get(cv.CAP_PROP_FRAME_WIDTH)   # placeholder for your pipeline
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    if elapsed - last_zoom_print >= 1.0:
        print(f"Zoom: {cap.get(cv.CAP_PROP_ZOOM)}")
        last_zoom_print = elapsed

    if elapsed >= 10 and not zoom_changed:
        print("Switching to 4X Zoom...")
        success = cap.set(cv.CAP_PROP_ZOOM, 400)
        print(f"set() returned: {success}")
        print_camera_state(cap, "AFTER set ZOOM=400")
        zoom_changed = True

    current_zoom = cap.get(cv.CAP_PROP_ZOOM)

    # --- Downsample only for display ---
    display_frame, scale, x_off, y_off = get_display_frame(gray_frame)

    # Overlay text on the display frame (not the processing frame)
    cv.putText(display_frame,
               f"Capture: {cap_w}x{cap_h} | Display: {DISPLAY_WIDTH}x{DISPLAY_HEIGHT} | Scale: {scale:.2f}x",
               (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, 255, 1)
    cv.putText(display_frame,
               f"Time: {elapsed:.1f}s  Zoom: {current_zoom}",
               (20, 60), cv.FONT_HERSHEY_SIMPLEX, 0.6, 255, 1)

    cv.imshow("Zoom Test", display_frame)

    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()
