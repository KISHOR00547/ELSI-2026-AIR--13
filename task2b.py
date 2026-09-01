import time

from connector_2b import CoppeliaClient

# The five line sensors, ordered left -> right across the robot ([0.0, 1.0]).
SENSOR_ORDER = ['left_corner', 'left', 'middle', 'right', 'right_corner']


# =============================================================================
#  IMPLEMENTATION: High-Speed State-Based Routing and Node Management
# =============================================================================
def control_loop(sensors):

    # --- Retuned PID for Higher Base Speeds ---
    Kp = 12 # Increased to snap back faster at high speed
    Ki = 0.0
    Kd = 8   # Higher derivative gain to dampen high-speed oscillations
    alpha = 0.85

    # --- Persistent state ---
    if not hasattr(control_loop, '_prev_error'):
        control_loop._prev_error = 0.0
        control_loop._integral = 0.0
        control_loop._prev_position = 0.0
        control_loop._prev_derivative = 0.0
        control_loop._is_white_bg = True
        control_loop._start_time = time.time()
        control_loop._bg_debounce_counter = 0 
        
        # State tracking for nodes
        control_loop._node_count = 0
        control_loop._node_cooldown = 0
        control_loop._action_type = None
        control_loop._action_timer = 0
        control_loop._initial_timer_val = 0  # To track sequence progression

        # Timestamp to prevent picking up immediately after a drop
        control_loop._drop_cooldown_until = 0.0

    # Decrement cooldowns and timers
    if control_loop._node_cooldown > 0:
        control_loop._node_cooldown -= 1
    if control_loop._action_timer > 0:
        control_loop._action_timer -= 1
    else:
        control_loop._action_type = None

    # --- Step 1: Read sensor values ---
    s = [sensors[name] for name in SENSOR_ORDER]

    # --- Robust Background Color Detection ---
    outer_brightness = (s[0] + s[4]) / 2.0
    
    if time.time() - control_loop._start_time < 1.0:
        control_loop._is_white_bg = True
        control_loop._bg_debounce_counter = 0
    else:
        target_is_white = outer_brightness > 0.50
        if target_is_white != control_loop._is_white_bg:
            control_loop._bg_debounce_counter += 1
            if control_loop._bg_debounce_counter >= 4:  # Reduced debounce frames for high speed
                control_loop._is_white_bg = target_is_white
                control_loop._bg_debounce_counter = 0
        else:
            control_loop._bg_debounce_counter = max(0, control_loop._bg_debounce_counter - 1)

    # Invert readings if on a white background so the line math stays consistent
    if control_loop._is_white_bg:
        s = [1.0 - v for v in s]

    # --- Node/Intersection Detection ---
    if time.time() - control_loop._start_time > 1.0:
        is_at_node = (s[0] > 0.45 and s[4] > 0.45) or (s[1] > 0.6 and s[2] > 0.6 and s[3] > 0.6)
    else:
        is_at_node = False
    
    if is_at_node and control_loop._node_cooldown == 0:
        control_loop._node_count += 1
        # Node cooldown reduced from 20 to 12 frames to prevent missing nodes at high speed
        control_loop._node_cooldown = 12  
        nc = control_loop._node_count
        print(f"--- Node {nc} Detected! ---")

        # Set specific configuration actions for nodes (Timers scaled down for faster turn speeds)
        if nc == 1:
            control_loop._action_type = 'right'
            control_loop._action_timer = 10
        elif nc == 2:
            control_loop._action_type = 'left'
            control_loop._action_timer = 10
        elif nc == 3:
            control_loop._action_type = 'left'
            control_loop._action_timer = 10
        elif nc == 4:
            control_loop._action_type = 'right'
            control_loop._action_timer = 10
        elif nc in [5, 6]:
            pass
        elif nc == 7:
            control_loop._action_type = 'slight_right'
            control_loop._action_timer = 9
        elif nc == 8:
            pass  # Skip
        elif nc == 9:
            # Red Drop: STOP -> DROP -> TURN 180
            control_loop._action_type = 'stop_drop_180'
            control_loop._action_timer = 22
        elif nc == 10:
            pass
        elif nc == 11:
            control_loop._action_type = 'left'
            control_loop._action_timer = 10
        elif nc in [12, 13]:
            pass
        elif nc == 14:
            control_loop._action_type = 'left'
            control_loop._action_timer = 10
        elif nc in [15, 16, 17]:
            control_loop._action_type = 'right'
            control_loop._action_timer = 10
        elif nc in [18, 19]:
            pass  
        elif nc == 20:
            control_loop._action_type = 'slight_left'
            control_loop._action_timer = 9
        elif nc in [21, 22]:
            pass
        elif nc == 23:
            # Blue Drop Zone: Drive in -> STOP -> DROP (No turn)
            control_loop._action_type = 'stop_drop'
            control_loop._action_timer = 18
        elif nc == 24:
            control_loop._action_type = 'left'
            control_loop._action_timer = 10
        elif nc == 25:
            control_loop._action_type = 'left'
            control_loop._action_timer = 10
        elif nc == 26:
            # Final Finish Node: Complete 10-second stop (200 control cycles @ 20 Hz)
            control_loop._action_type = 'stop_10sec'
            control_loop._action_timer = 200

        control_loop._initial_timer_val = control_loop._action_timer

    # --- Execute Phased Action Overrides if active ---
    if control_loop._action_type is not None:
        elapsed = control_loop._initial_timer_val - control_loop._action_timer
        
        # Complete 10-second Stop
        if control_loop._action_type == 'stop_10sec':
            return 0.0, 0.0

        # Phase 1: Move forward briefly (~150ms) to center into the junction
        if elapsed < 3:
            return 8.0, 8.0

        # Phase 2: Execute discrete maneuvers (Increased wheel differential speeds)
        if control_loop._action_type == 'right':
            return 12.0, 2.0
        elif control_loop._action_type == 'left':
            return 2.0, 12.0
        elif control_loop._action_type == 'slight_right':
            return 11.0, 3.5
        elif control_loop._action_type == 'slight_left':
            return 3.5, 11.0
        elif control_loop._action_type == 'stop_drop_180':
            if elapsed < 5:  # Pause for red drop before turning
                return 0.0, 0.0
            return -8.0, 8.0  # Faster 180 turn in place
        elif control_loop._action_type == 'stop_drop':
            # Straight stop without rotation for blue box
            return 6.0, 6.0
        elif control_loop._action_type == 'left_end':
            if elapsed < 3:
                return 0.0, 0.0
            return 0.0, 10.0

    # --- Step 2: Line position estimation & Junction Handling ---
    weights = [-2.0, -1.0, 0.0, 1.0, 2.0]
    
    # Increase priority for the center sensor 
    if s[2] > 0.6:
        s[0] *= 0.4  
        s[4] *= 0.4  

    total = sum(s)
    recovering = False

    if total > 0.15:
        position = sum(w * v for w, v in zip(weights, s)) / total
        control_loop._prev_position = position
    else:
        recovering = True
        if control_loop._prev_position > 0.1:
            position = 2.5
        elif control_loop._prev_position < -0.1:
            position = -2.5
        else:
            position = 0.0

    # --- Step 3: Error computation ---
    error = position

    # --- Step 4: PID computation ---
    control_loop._integral += error
    control_loop._integral = max(-5.0, min(5.0, control_loop._integral))

    raw_derivative = error - control_loop._prev_error
    derivative = (alpha * raw_derivative) + ((1.0 - alpha) * control_loop._prev_derivative)
    pid = (Kp * error) + (Ki * control_loop._integral) + (Kd * derivative)

    control_loop._prev_error = error
    control_loop._prev_derivative = derivative

    # --- Step 5: High-Speed Adaptive Curve ---
    if recovering:
        base_speed = 15.0
    else:
        # Boosted base line-following speed (18.0 max vs 11.5 previously)
        base_speed = 20.0 - 5.0 * abs(error)
        base_speed = max(4.0, base_speed) 

    left = base_speed + pid
    right = base_speed - pid

    # Expanded motor limits for higher top speed and faster turn response
    left = max(-16.0, min(left, 20.0))
    right = max(-16.0, min(right, 20.0))

    return left, right


def detect_color(sensors):
    r = sensors.get('color_r', 0.0)
    g = sensors.get('color_g', 0.0)
    b = sensors.get('color_b', 0.0)
    
    if (r + g + b) > 0.10:
        if r > g and r > b: 
            return "red"
        elif g > r and g > b: 
            return "green"
        elif b > r and b > g: 
            return "blue"
    return None


def should_pick(sensors, carrying_color):
    if carrying_color is not None:
        return False

    # Block pickup if cooldown period has not elapsed
    if time.time() < getattr(control_loop, '_drop_cooldown_until', 0.0):
        return False

    proximity = sensors.get('proximity', 1.0)
    
    # Increased proximity threshold to catch items earlier at higher speed
    if proximity < 0.25:  
        colour_seen = detect_color(sensors)
        if colour_seen in ["red", "blue"]:
            return True

    return False


def should_drop(sensors, carrying_color):
    if carrying_color is None:
        return False

    action_type = getattr(control_loop, '_action_type', None)
    current_node = getattr(control_loop, '_node_count', 0)
    
    initial_timer = getattr(control_loop, '_initial_timer_val', 0)
    current_timer = getattr(control_loop, '_action_timer', 0)
    elapsed = initial_timer - current_timer

    dropped = False

    # 1. RED BOX (Node 9): Drop AFTER stopping, BEFORE turning
    if action_type == 'stop_drop_180' and current_node == 9 and carrying_color == "red":
        if elapsed >= 3:
            dropped = True

    # 2. BLUE BOX (Node 23): Drop AFTER forward drive phase ends
    if action_type == 'stop_drop' and current_node == 23 and carrying_color == "blue":
        if elapsed >= 1:
            dropped = True

    if dropped:
        control_loop._drop_cooldown_until = time.time() + 3.0

    return dropped


# =============================================================================
#  Main loop
# =============================================================================
def main():
    client = CoppeliaClient(host="127.0.0.1", port=50002)
    client.connect()
    print("Connected to bridge_v1_2b. Running High-Speed Mode...")

    last_sensors   = None
    carrying_color = None 
    delivered      = 0 

    try:
        while True:
            sensors = client.receive_sensor_data()
            if sensors is not None:
                last_sensors = sensors
            if last_sensors is None:
                time.sleep(0.01)
                continue

            # --- Pick ---
            if carrying_color is None and should_pick(last_sensors, carrying_color):
                colour_seen = detect_color(last_sensors)
                success = client.send_pick()
                print(f"PICK attempted (saw {colour_seen!r}) — success={success}")
                if success:
                    carrying_color = colour_seen

            # --- Drop ---
            if carrying_color is not None and should_drop(last_sensors, carrying_color):
                success = client.send_drop()
                print(f"DROP attempted ({carrying_color!r}) — success={success}")
                if success:
                    delivered += 1
                    carrying_color = None
                    print(f"Delivered {delivered} box(es) so far.")

            # --- Motor command ---
            left, right = control_loop(last_sensors)
            client.send_motor_command(left, right)

            time.sleep(0.02)   # ~50 Hz control loop speed for fast response

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
