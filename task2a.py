"""
===================================================
  eLSI Sprint 1 - Task 2A : PID Line Following + Pick & Place
===================================================

Participant template.

HOW TO RUN
  1. Open the Task 2A scene in CoppeliaSim.
  2. Start the bridge:   python3 bridge_v1_2a.py --eval
  3. Run this file:      python3 task2a.py

WHAT YOU IMPLEMENT
  control_loop()  — PID controller that returns (left_speed, right_speed).
  detect_color()  — identify the box color from RGB sensor values.
  should_pick()   — decide when to stop and pick the box.
  should_drop()   — decide when to drop the carried box.

Everything else (connecting, receiving sensors, sending motor/pick/drop
commands) is handled by CoppeliaClient.
Don't edit this file except the marked TODO sections.
You may add helper functions.

SENSOR PROTOCOL  (from bridge_v1_2a.py):
  Line sensors:  'left_corner', 'left', 'middle', 'right', 'right_corner'
                  — float [0.0, 1.0];  higher = line detected.
  Proximity:     'proximity'  — metres to nearest object; 1.0 = nothing in range.
  Color sensor:  'color_r', 'color_g', 'color_b'  — float [0.0, 1.0].

TASK FLOW
  1. Robot drives the line following the PID controller.
  2. When the robot is close to the box (proximity low), read the color,
     stop, and send a PICK command.
  3. Robot carries the box and continues following the line.
  4. At the correct drop zone, send a DROP command.

Team ID: [ XXX ]
"""

import time

from connector import CoppeliaClient

# The five line sensors, ordered left → right across the robot ([0.0, 1.0]).
SENSOR_ORDER = ['left_corner', 'left', 'middle', 'right', 'right_corner']

def control_loop(sensors):
    """
    Return (left_speed, right_speed).
    Count 1: All 5 sensors must be black -> Stops briefly then turns.
    Count 2: Middle black + (Left or Right black) -> Stops completely then drops.
    """
    Kp = 0.85   
    Kd = 0.50   
    
    time.sleep(0.02)

    # --- Persistent States ---
    if not hasattr(control_loop, '_prev_error'):
        control_loop._prev_error = 0.0
        control_loop._prev_position = 0.0
        
        # States: "LINE_FOLLOW", "STOP_AT_JUNCTION", "AGGRESSIVE_TURN", "FINDING_LINE", "PASSING_STRAIGHT", "STOP_AT_DROP"
        control_loop._junction_state = "LINE_FOLLOW"  
        control_loop._turn_direction = None
        control_loop._state_start_time = 0.0
        control_loop._last_display_time = 0.0         
        
        control_loop._black_area_count = 0  
        control_loop._last_count_time = 0.0  

    # --- Get Raw Sensor Readings ---
    s = [sensors[name] for name in SENSOR_ORDER]

    # --- Get Carried Box Color ---
    r = sensors.get('color_r', 0.0)
    g = sensors.get('color_g', 0.0)
    b = sensors.get('color_b', 0.0)
    
    current_color = None
    if (r + g + b) > 0.05:
        if r > g and r > b: current_color = "red"
        elif g > r and g > b: current_color = "green"
        elif b > r and b > g: current_color = "blue"

    current_time = time.time()

    # --- SPLIT COUNT LOGIC SYSTEM ---
    if current_time - control_loop._last_count_time > 0.80:
        
        # LOOKING FOR COUNT 1: All 5 sensors must hit black at the crossroads junction
        if control_loop._black_area_count == 0:
            if s[0] < 0.20 and s[1] < 0.20 and s[2] < 0.20 and s[3] < 0.20 and s[4] < 0.20:
                control_loop._black_area_count = 1
                control_loop._last_count_time = current_time
                print(f"\n[COUNT 1 DETECTED] All 5 sensors triggered black junction!")
                
                control_loop._state_start_time = current_time
                control_loop._junction_state = "STOP_AT_JUNCTION"
                
                if current_color == "red":
                    control_loop._turn_direction = "LEFT"
                elif current_color == "green":
                    control_loop._turn_direction = "RIGHT"
                else:
                    control_loop._turn_direction = "STRAIGHT"

        # LOOKING FOR COUNT 2: Fixed to trigger reliably if middle is black + either left or right is black
        elif control_loop._black_area_count == 1:
            if s[2] < 0.20 and (s[1] < 0.20 or s[3] < 0.20):
                control_loop._black_area_count = 2
                control_loop._last_count_time = current_time
                print(f"\n[COUNT 2 DETECTED] Center tracking sensors hit drop zone line!")
                
                control_loop._state_start_time = current_time
                control_loop._junction_state = "STOP_AT_DROP"

    # --- Turning & Stopping State Machine ---
    
    # COUNT 2 STOP ACTION: Halt wheels completely for 0.15 seconds at destination before dropping
    if control_loop._junction_state == "STOP_AT_DROP":
        return 0.0, 0.0

    # COUNT 1 STOP ACTION: Brief stop before executing junction branch turns
    if control_loop._junction_state == "STOP_AT_JUNCTION":
        if current_time - control_loop._state_start_time > 0.10: 
            if control_loop._turn_direction == "STRAIGHT":
                control_loop._junction_state = "PASSING_STRAIGHT"
            else:
                control_loop._junction_state = "AGGRESSIVE_TURN"
                control_loop._state_start_time = current_time
        return 0.0, 0.0

    if control_loop._junction_state == "PASSING_STRAIGHT":
        if s[1] > 0.4 and s[3] > 0.4:
            control_loop._junction_state = "LINE_FOLLOW"
        return 2.5, 2.5

    if control_loop._junction_state == "AGGRESSIVE_TURN":
        if current_time - control_loop._state_start_time > 0.25: 
            control_loop._junction_state = "FINDING_LINE"
            
        if control_loop._turn_direction == "LEFT":
            return -4.0, 4.0  
        elif control_loop._turn_direction == "RIGHT":
            return 4.0, -4.0  

    if control_loop._junction_state == "FINDING_LINE":
        if control_loop._turn_direction == "LEFT":
            if s[2] < 0.25 or s[3] < 0.25:
                control_loop._junction_state = "LINE_FOLLOW"
            return -1.5, 1.8
        elif control_loop._turn_direction == "RIGHT":
            if s[2] < 0.25 or s[1] < 0.25:
                control_loop._junction_state = "LINE_FOLLOW"
            return 1.8, -1.5

    # --- Curve Line Following ---
    line_signals = [1.0 - v for v in s]
    weights = [-2.0, -1.0, 0.0, 1.0, 2.0]
    total = sum(line_signals)
    
    if total > 0.15:
        position = sum(w * v for w, v in zip(weights, line_signals)) / total
        control_loop._prev_position = position
    else:
        position = control_loop._prev_position

    error = position
    derivative = error - control_loop._prev_error
    pid = (Kp * error) + (Kd * derivative)
    control_loop._prev_error = error

    base_speed = 2.8 - 1.5 * abs(error)
    base_speed = max(0.8, base_speed)

    left = base_speed + pid
    right = base_speed - pid

    left = max(-5.0, min(left, 5.0))
    right = max(-5.0, min(right, 5.0))

    if current_time - control_loop._last_display_time > 1.0:
        print(f"[TELEMETRY] Black Count: {control_loop._black_area_count} | Mode: {control_loop._junction_state}")
        control_loop._last_display_time = current_time

    return round(left, 3), round(right, 3)


def detect_color(sensors):
    r = sensors.get('color_r', 0.0)
    g = sensors.get('color_g', 0.0)
    b = sensors.get('color_b', 0.0)
    if (r + g + b) > 0.05:
        if r > g and r > b: return "red"
        elif g > r and g > b: return "green"
        elif b > r and b > g: return "blue"
    return None


def should_pick(sensors, carrying_box):
    if carrying_box:
        return False
        
    r = sensors.get('color_r', 0.0)
    g = sensors.get('color_g', 0.0)
    b = sensors.get('color_b', 0.0)
    proximity = sensors.get('proximity', 1.0)
    f
    if (r > g and r > b) or (g > r and g > b) or (b > r and b > g) or proximity < 0.12:
        # Stop briefly or 0.1 seconds to perfectly grab the box
        time.sleep(0.1)
        if hasattr(control_loop, '_black_area_count'):
            control_loop._black_area_count = 0
            control_loop._junction_state = "LINE_FOLLOW"
        print(f"\n[ACTION] Box Stopped & Picked. Counter Reset.")
        return True
    return False


def should_drop(sensors, carrying_box, detected_color):
    if not carrying_box:
        return False

    current_count = getattr(control_loop, '_black_area_count', 0)
    proximity = sensors.get('proximity', 1.0)
    current_state = getattr(control_loop, '_junction_state', "LINE_FOLLOW")

    # Drop triggers when count is 2 and wheels have completely come to a halt
    if (current_count >= 2 and current_state == "STOP_AT_DROP") or proximity < 0.11:
        print(f"\n[DROP EXECUTION] Target achieved. Releasing box.")
        control_loop._black_area_count = 0
        control_loop._junction_state = "LINE_FOLLOW"  
        return True
            
    return False
# =============================================================================
def main():
    client = CoppeliaClient(host="127.0.0.1", port=50002)
    client.connect()
    print("Connected to bridge_v1_2a. Running... (Ctrl+C to stop)")

    last_sensors   = None
    carrying_box   = False
    detected_color = None

    try:
        while True:
            sensors = client.receive_sensor_data()
            if sensors is not None:
                last_sensors = sensors
            if last_sensors is None:
                time.sleep(0.02)
                continue

            # --- Color detection (once, before picking) ---
            if detected_color is None and not carrying_box:
                color = detect_color(last_sensors)
                if color is not None:
                    detected_color = color
                    print(f"Color detected: {color!r}")

            # --- Pick ---
            if not carrying_box and should_pick(last_sensors, carrying_box):
                success = client.send_pick()
                print(f"PICK attempted  — success={success}")
                if success:
                    carrying_box = True

            # --- Drop ---
            if carrying_box and should_drop(last_sensors, carrying_box, detected_color):
                success = client.send_drop()
                print(f"DROP attempted  — success={success}")
                if success:
                    carrying_box = False

            # --- Motor command ---
            left, right = control_loop(last_sensors)
            client.send_motor_command(left, right)

            time.sleep(0.05)   # ~20 Hz control loop

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        try:
            client.send_motor_command(0.0, 0.0)
        except Exception:
            pass
        client.close()


if __name__ == "__main__":
    main()
