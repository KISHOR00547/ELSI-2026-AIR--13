"""
===================================================
    eLSI Sprint 1 - Task 1A : PID Line Following
===================================================

Participant template.

HOW TO RUN
  1. Open the Task 1A scene in CoppeliaSim.
  2. Start the bridge:   python3 bridge_task1a.py --eval
  3. Run this file:      python3 task1a_template.py

WHAT YOU IMPLEMENT
  Only control_loop(). Everything else (connecting, receiving sensors,
  sending motor commands) is handled for you by CoppeliaClient.
  Don't Edit this file except control_loop().
  You can add helper functions if you like.

Team ID: [ 862 ]
"""

import time

from connector_task1a import CoppeliaClient

# The five line sensors, ordered left -> right across the robot.
# Each value is in [0.0, 1.0]; a higher value means the line is detected.
SENSOR_ORDER = ['left_corner', 'left', 'middle', 'right', 'right_corner']

def control_loop(sensors):

    # --- PID tuning parameters ---
    Kp = 2.2
    Ki = 0.0
    Kd = 2.5
    alpha = 0.5  # Derivative filter

    # --- Persistent state ---
    if not hasattr(control_loop, '_prev_error'):
        control_loop._prev_error = 0.0
        control_loop._integral = 0.0
        control_loop._prev_position = 0.0
        control_loop._prev_derivative = 0.0
        control_loop._is_white_bg = False

    # --- Step 1: Read sensor values ---
    s = [sensors[name] for name in SENSOR_ORDER]

    # --- Background Color Hysteresis Detection ---
    num_white = sum(1 for v in s if v > 0.6)
    num_black = sum(1 for v in s if v < 0.4)

    if num_white >= 4:
        control_loop._is_white_bg = True
    elif num_black >= 4:
        control_loop._is_white_bg = False

    if control_loop._is_white_bg:
        s = [1.0 - v for v in s]

    # --- Display detected line/background type ---
    if control_loop._is_white_bg:
        bg_status = "BLACK LINE | WHITE BACKGROUND"
    else:
        bg_status = "WHITE LINE | BLACK BACKGROUND"

    # --- Step 2: Line position estimation ---
    weights = [-2.0, -1.0, 0.0, 1.0, 2.0]
    total = sum(s)

    if total > 0.2:
        position = sum(w * v for w, v in zip(weights, s)) / total
        control_loop._prev_position = position
    else:
        if control_loop._prev_position > 1.2:
            position = 2.5
        elif control_loop._prev_position < -1.2:
            position = -2.5
        else:
            position = control_loop._prev_position

    # --- Step 3: Error computation ---
    error = position

    # --- Step 4: PID computation ---
    control_loop._integral += error
    control_loop._integral = max(
        -50.0,
        min(50.0, control_loop._integral)
    )

    raw_derivative = error - control_loop._prev_error

    derivative = (
        alpha * raw_derivative
        + (1.0 - alpha) * control_loop._prev_derivative
    )

    pid = (
        Kp * error
        + Ki * control_loop._integral
        + Kd * derivative
    )

    control_loop._prev_error = error
    control_loop._prev_derivative = derivative

    # --- Step 5: Adaptive speed & Differential drive ---
    base_speed = 3.5 - 1.0 * abs(error)
    base_speed = max(1.2, base_speed)

    left = base_speed + pid
    right = base_speed - pid

    # Clamp to valid range
    left = max(-15.0, min(left, 15.0))
    right = max(-15.0, min(right, 15.0))

    # --- Debug Output ---
    print(
        f"{bg_status} | "
        f"White={num_white} Black={num_black} | "
        f"Pos={position:+.2f} | "
        f"Err={error:+.2f} | "
        f"PID={pid:+.2f} | "
        f"L={left:+.2f} | "
        f"R={right:+.2f}"
    )

    print(
        f"Sensors => "
        f"LC:{sensors['left_corner']:.2f} "
        f"L:{sensors['left']:.2f} "
        f"M:{sensors['middle']:.2f} "
        f"R:{sensors['right']:.2f} "
        f"RC:{sensors['right_corner']:.2f}"
    )

    print("-" * 80)

    return left, right

def main():
    client = CoppeliaClient(host="127.0.0.1", port=50002)
    client.connect()
    print("Connected to bridge_task1a. Running... (Ctrl+C to stop)")

    last_sensors = None
    try:
        while True:
            # Pull the freshest sensor packet; reuse the last one between packets.
            sensors = client.receive_sensor_data()
            if sensors is not None:
                last_sensors = sensors
            if last_sensors is None:
                time.sleep(0.02)
                continue

            left, right = control_loop (last_sensors)
            client.send_motor_command(left, right)

            time.sleep(0.05)   # ~20 Hz control loop
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        try:
            client.send_motor_command(0.0, 0.0)   # stop the robot
        except Exception:
            pass
        client.close()


if __name__ == "__main__":
    main()
