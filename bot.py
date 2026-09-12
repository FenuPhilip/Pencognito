"""
laptop_brain.py
Runs on the laptop (Windows 11) - NOT on the ESP32-CAM, NOT on any Pi.
Pulls the MJPEG video stream from the ESP32-CAM over WiFi, runs YOLOv8
locally for display, tracks a red object with plain color detection for
autonomous chasing, and sends drive commands back to the ESP32-CAM.

Install once (any terminal, cmd/PowerShell is fine, venv optional):
    pip install opencv-python requests numpy ultralytics

Run:
    python laptop_brain.py

MANUAL controls: W/A/S/D to drive, SPACE to stop, Q to quit.
Movement now truly follows key state: the car drives ONLY while a key is
physically held down, and stops the instant it's released. This uses
Windows' GetAsyncKeyState (via ctypes, no extra install needed) instead of
OpenCV's keydown-only events, because OpenCV has no "key released" signal.

SAFETY NOTE: GetAsyncKeyState reports the real keyboard state regardless of
which window is focused, so this script only acts on keys while the
"Pencognito - laptop brain" window is the foreground window. If you click
away to another app, all driving stops and stays stopped until you click
back - otherwise a stray W press somewhere else could drive the car.

SPEED: there's no known speed/PWM API on the ESP32 side, only discrete
forward/left/right/reverse/stop routes. To make it drive slower than "full
send" without that API, both manual and autonomous driving now pulse the
command on/off rapidly (a software duty cycle) instead of holding it on
continuously. Tune MANUAL_THROTTLE / TURN_* / FORWARD_* below. If your
ESP32 firmware DOES support a speed parameter (e.g. /forward?speed=150),
tell me the route shape and I'll switch this to true continuous speed
control instead of pulsing.

AUTONOMOUS control: press P to toggle "chase the red object" mode.
- While ON, steering and forward-driving are now pulsed proportionally to
  how far off-center / how far away the target is (a simple P+D
  controller expressed as pulse duration, since we can't send a
  continuous speed/turn-rate value) instead of just slamming "left" or
  "right" at full power until the next frame.
- If it loses the target, it stops rather than drive blind.
- Holding any manual key immediately hands control back to you and turns
  autonomous mode off.

DIRECTION NOTE (fix for "pen on right -> car spins left"): this was almost
certainly the camera frame being mirrored relative to how the car
physically turns (very common with these camera mounts) - manual driving
never looked at the video frame at all, so it was unaffected. INVERT_STEERING
below fixes this. It defaults to True since you confirmed it was reversed.
If it's now backwards the OTHER way, flip it back to False.
"""

import time
import ctypes
import cv2
import requests
import urllib.request
import numpy as np
from ultralytics import YOLO

# ---- Update this to match the IP your ESP32-CAM is actually running at ----
ESP32_IP = "10.44.41.200"
STREAM_URL = f"http://{ESP32_IP}:81/stream"  # video is on port 81
CONTROL_PORT = 80                             # drive commands are on port 80
CONTROL_TIMEOUT = 0.15                        # http timeout per drive command

WINDOW_TITLE = "Pencognito - laptop brain"

model = YOLO("yolov8n.pt")

# ---- Red-blob detection tuning (unchanged) ----
RED_LOWER1 = np.array([0, 120, 70])
RED_UPPER1 = np.array([10, 255, 255])
RED_LOWER2 = np.array([170, 120, 70])
RED_UPPER2 = np.array([180, 255, 255])
MIN_TARGET_AREA = 500        # ignore tiny red specks/noise, raise if it's twitchy
CENTER_TOLERANCE_PX = 60     # deadband: within this many px of center counts as "aligned"
                             # wider = less hair-trigger turning on small errors
STOP_WIDTH_RATIO = 0.55      # stop once target width fills this fraction of frame width

# ---- Manual drive throttle (fixes "too fast" + works with real key-hold now) ----
MANUAL_PULSE_PERIOD = 0.08   # seconds per on/off cycle while a drive key is held
MANUAL_THROTTLE = 0.35       # fraction of each cycle actively driving; lower = slower

# ---- Autonomous PID steering/approach tuning ----
INVERT_STEERING = True       # flip this if left/right comes out backwards (see note above)

# Full PID on the normalized horizontal error (-1 left edge .. +1 right edge).
PID_KP = 0.15                # reacts to how far off-center it currently is
PID_KI = 0.0             # reacts to persistent bias (e.g. one motor weaker/pulling one way)
PID_KD = 0.28            # reacts to how fast the error is changing, damps overshoot
INTEGRAL_CLAMP = 0.8         # cap on accumulated integral term (anti-windup)

LARGE_ERROR_THRESH = 0.80    # |error| beyond this = badly misaligned -> pure rotate
                             # raised high so bot almost always arc-drives instead of spinning;
                             # pure rotate causes over-turning because WiFi lag >> pulse duration
# Between CENTER_TOLERANCE_PX and LARGE_ERROR_THRESH: "arc drive" - nudge steering for a short
# pulse, then keep advancing, so it corrects its aim WHILE approaching instead of stop/turn/stop/go.
ARC_TURN_MIN_PULSE = 0.005
ARC_TURN_MAX_PULSE = 0.006   # ~6ms arc nudge
PURE_TURN_MIN_PULSE = 0.005
PURE_TURN_MAX_PULSE = 0.008  # ~8ms rotate burst - minimal physical turn

FORWARD_MIN_PULSE = 0.005
FORWARD_MAX_PULSE = 0.010    # ~10ms forward burst
SLOWDOWN_ZONE = 0.15         # width-ratio headroom before STOP_WIDTH_RATIO where forward eases off
POST_CMD_SETTLE = 0.30       # seconds to wait (stopped) after each pulse before next decision;
                             # compensates for WiFi lag - bot has time to physically stop & ESP32
                             # has time to receive/execute the stop before the next command arrives

# ---- Slash motor (second L298N, OUT3/OUT4) ----
# Route name is a placeholder - rename to match whatever you add to the ESP32 firmware.
SLASH_ROUTE = "slash"
# Once triggered on an approach, don't re-fire every settle-cycle while still parked at the
# target - only re-arm once the target is lost or backs off well clear of the stop distance.
SLASH_REARM_WIDTH_RATIO = STOP_WIDTH_RATIO * 0.7
VK_T = 0x54  # manual test-fire key, for bench-testing the slash motor without full auto-chase

# ---- Windows virtual-key codes ----
VK_W, VK_A, VK_S, VK_D = 0x57, 0x41, 0x53, 0x44
VK_SPACE, VK_P, VK_Q = 0x20, 0x50, 0x51

user32 = ctypes.windll.user32
user32.GetAsyncKeyState.restype = ctypes.c_short
user32.GetAsyncKeyState.argtypes = [ctypes.c_int]


def key_down(vk):
    return user32.GetAsyncKeyState(vk) < 0


def window_focused():
    hwnd = user32.FindWindowW(None, WINDOW_TITLE)
    return bool(hwnd) and user32.GetForegroundWindow() == hwnd


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def send_car_cmd(route: str):
    try:
        requests.get(f"http://{ESP32_IP}:{CONTROL_PORT}/{route}", timeout=CONTROL_TIMEOUT)
    except requests.exceptions.RequestException:
        pass  # dropped command is fine, next cycle will resend


def open_stream():
    return urllib.request.urlopen(STREAM_URL, timeout=5)


def find_red_object(frame):
    """Returns (cx, cy, x, y, w, h, area) for the largest red blob, or None."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, RED_LOWER1, RED_UPPER1)
    mask2 = cv2.inRange(hsv, RED_LOWER2, RED_UPPER2)
    mask = cv2.bitwise_or(mask1, mask2)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)
    if area < MIN_TARGET_AREA:
        return None
    x, y, w, h = cv2.boundingRect(largest)
    cx, cy = x + w // 2, y + h // 2
    return cx, cy, x, y, w, h, area


def main():
    print("Laptop Brain connected.")
    print("Manual: hold W/A/S/D to drive, SPACE to stop, Q to quit.")
    print("(Movement only works while the video window is focused.)")
    print("Autonomous: press P to toggle 'chase the red object' mode.")

    stream = open_stream()
    bytes_buffer = b""

    auto_mode = False
    p_was_down = False
    q_was_down = False

    # -- manual pulse state --
    manual_route = None          # currently held drive route, or None
    manual_pulse_ref = 0.0       # time reference for the current on/off cycle
    manual_pulse_on = False

    # -- autonomous pulse state --
    auto_pulse_end = 0.0         # time when the current auto pulse finishes
    auto_settle_end = 0.0        # time when the post-command settle pause finishes
    in_settle = False            # True = stop sent, waiting for settle before next decision
    prev_error = 0.0
    integral_error = 0.0
    last_decision_time = time.time()
    slash_armed = True    # ready to fire; goes False right after firing until re-armed
    t_was_down = False    # manual test-fire key edge detection

    manual_vks = {VK_W: "forward", VK_A: "left", VK_S: "reverse", VK_D: "right"}

    while True:
        try:
            chunk = stream.read(1024)
        except Exception:
            print("Stream dropped, reconnecting...")
            try:
                stream = open_stream()
            except Exception:
                cv2.waitKey(500)
            continue

        if not chunk:
            continue
        bytes_buffer += chunk

        start = bytes_buffer.find(b"\xff\xd8")
        end = bytes_buffer.find(b"\xff\xd9")
        if start == -1 or end == -1 or end < start:
            continue

        jpg = bytes_buffer[start:end + 2]
        bytes_buffer = bytes_buffer[end + 2:]

        frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            continue
        frame_h, frame_w = frame.shape[:2]

        # ---- YOLO detections (display only) ----
        results = model(frame, imgsz=320, verbose=False)[0]
        for box in results.boxes:
            conf = float(box.conf[0])
            if conf <= 0.4:
                continue
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            label = model.names[int(box.cls[0])]
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"{label} {conf:.2f}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # ---- Red-object detection ----
        target = find_red_object(frame)
        cv2.line(frame, (frame_w // 2, 0), (frame_w // 2, frame_h), (255, 255, 0), 1)
        if target:
            cx, cy, x, y, w, h, area = target
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)

        mode_label = "AUTO - chasing red" if auto_mode else "MANUAL"
        mode_color = (0, 255, 255) if auto_mode else (200, 200, 200)
        cv2.putText(frame, mode_label, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, mode_color, 2)
        if not window_focused():
            cv2.putText(frame, "(window not focused - driving disabled)", (10, 42),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        cv2.imshow(WINDOW_TITLE, frame)
        cv2.waitKey(1)  # just pumps the GUI event loop; input comes from GetAsyncKeyState below

        focused = window_focused()
        now = time.time()

        # ---- Edge-triggered P (toggle auto) / Q (quit) ----
        p_down = focused and key_down(VK_P)
        q_down = focused and key_down(VK_Q)
        if p_down and not p_was_down:
            auto_mode = not auto_mode
            print("Autonomous mode:", "ON" if auto_mode else "OFF")
            send_car_cmd("stop")
            manual_route = None
            manual_pulse_on = False
            auto_pulse_end = 0.0
            integral_error = 0.0
            last_decision_time = now
            in_settle = False
            auto_settle_end = 0.0
        p_was_down = p_down
        if q_down and not q_was_down:
            send_car_cmd("stop")
            break
        q_was_down = q_down

        t_down = focused and key_down(VK_T)
        if t_down and not t_was_down:
            print("Manual slash test-fire")
            send_car_cmd(SLASH_ROUTE)
        t_was_down = t_down

        # ---- Manual driving: real hold-to-move, pulsed for speed control ----
        new_manual_route = None
        if focused:
            if key_down(VK_SPACE):
                new_manual_route = "stop"
            else:
                for vk, route in manual_vks.items():
                    if key_down(vk):
                        new_manual_route = route
                        break  # first match wins if multiple keys held

        if new_manual_route is not None:
            auto_mode = False  # any manual input takes back control

        if new_manual_route != manual_route:
            # key changed (pressed, released, or switched direction) - restart pulse cycle
            manual_route = new_manual_route
            manual_pulse_ref = now
            manual_pulse_on = False
            if manual_route is None or manual_route == "stop":
                send_car_cmd("stop")
                manual_pulse_on = False

        if manual_route and manual_route != "stop":
            phase = (now - manual_pulse_ref) % MANUAL_PULSE_PERIOD
            want_on = phase < (MANUAL_PULSE_PERIOD * MANUAL_THROTTLE)
            if want_on and not manual_pulse_on:
                send_car_cmd(manual_route)
                manual_pulse_on = True
            elif not want_on and manual_pulse_on:
                send_car_cmd("stop")
                manual_pulse_on = False

        # ---- Autonomous chase: real PID steering, arc-drives instead of stop/turn/stop/go ----
        # Two-phase timing to compensate for WiFi lag:
        #   Phase 1 (pulse): send move command, wait pulse duration
        #   Phase 2 (settle): send stop, wait POST_CMD_SETTLE for bot to physically stop
        #                     and ESP32 to receive/ack before we read the new error
        if auto_mode:
            if not in_settle and now >= auto_pulse_end:
                # --- Phase 1 end: pulse is done, send stop and start settle ---
                send_car_cmd("stop")
                in_settle = True
                auto_settle_end = now + POST_CMD_SETTLE

            elif in_settle and now >= auto_settle_end:
                # --- Phase 2 end: bot has settled, now make the next decision ---
                in_settle = False

                if target is None:
                    prev_error = 0.0
                    integral_error = 0.0
                    last_decision_time = now
                    auto_pulse_end = now  # immediately re-enter settle next loop
                    in_settle = True
                    auto_settle_end = now + POST_CMD_SETTLE
                    slash_armed = True  # lost the target - safe to re-arm for next approach
                else:
                    cx, _, _, _, w, _, _ = target
                    half_w = frame_w / 2
                    error = (cx - half_w) / half_w  # normalized, -1 (left) .. +1 (right)
                    if INVERT_STEERING:
                        error = -error
                    width_ratio = w / frame_w

                    dt = max(now - last_decision_time, 0.001)
                    last_decision_time = now

                    if width_ratio < SLASH_REARM_WIDTH_RATIO:
                        slash_armed = True  # backed off well clear of the target - re-arm

                    if width_ratio >= STOP_WIDTH_RATIO:
                        # arrived - stay stopped, reset PID
                        prev_error = error
                        integral_error = 0.0
                        auto_pulse_end = now
                        in_settle = True
                        auto_settle_end = now + POST_CMD_SETTLE
                        if slash_armed:
                            print("Target reached - firing slash motor")
                            send_car_cmd(SLASH_ROUTE)
                            slash_armed = False
                    else:
                        integral_error = clamp(integral_error + error * dt, -INTEGRAL_CLAMP, INTEGRAL_CLAMP)
                        derivative = (error - prev_error) / dt
                        pid_out = PID_KP * error + PID_KI * integral_error + PID_KD * derivative
                        prev_error = error

                        remaining = max(STOP_WIDTH_RATIO - width_ratio, 0.0)
                        ease = clamp(remaining / SLOWDOWN_ZONE, 0.15, 1.0)
                        fwd_pulse = FORWARD_MIN_PULSE + (FORWARD_MAX_PULSE - FORWARD_MIN_PULSE) * ease
                        route = "left" if pid_out < 0 else "right"

                        if abs(error) > LARGE_ERROR_THRESH:
                            # badly misaligned - rotate in place, then settle
                            pulse = clamp(abs(pid_out), PURE_TURN_MIN_PULSE, PURE_TURN_MAX_PULSE)
                            send_car_cmd(route)
                            auto_pulse_end = now + pulse
                        elif abs(cx - half_w) > CENTER_TOLERANCE_PX:
                            # moderately off - short turn nudge then forward, then settle
                            turn_pulse = clamp(abs(pid_out), ARC_TURN_MIN_PULSE, ARC_TURN_MAX_PULSE)
                            send_car_cmd(route)
                            time.sleep(turn_pulse)
                            send_car_cmd("forward")
                            auto_pulse_end = time.time() + fwd_pulse
                        else:
                            # aligned - ease forward, then settle
                            send_car_cmd("forward")
                            auto_pulse_end = now + fwd_pulse

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
