"""
===================================================
    eLSI Sprint 1 - Task 1B : Q-Learning
===================================================

Participant template.

HOW TO RUN
  1. Open the Task 1B scene in CoppeliaSim.
  2. Start the bridge:   python3 bridge_task1b.py --eval
  3. Train:              python3 task1b_template.py --mode train
     Test (no learning): python3 task1b_template.py --mode test

MODES
  train : choose actions with exploration AND update the Q-table.
ex          The Q-table is saved to disk on exit.
  test  : load the saved Q-table, act greedily, and DO NOT update it.

WHAT YOU IMPLEMENT
  get_state()     - how to turn the 5 sensor values into a discrete state.
  get_reward()    - how good the latest reading is.
  choose_action() - which action to take in a given state (the policy).

Team ID: [ 862 ]
"""

import time
import os
import pickle
import random
import argparse

from connector_task1b import CoppeliaClient

# The five line sensors, ordered left -> right across the robot ([0.0, 1.0]).
SENSOR_ORDER = ['left_corner', 'left', 'middle', 'right', 'right_corner']

# Action set: index -> (left_speed, right_speed). 
ACTIONS = [
    (8.0, 8.0),   # 0: Forward
    (7.5, 8.0),   # 1: Slight Left (Super wide arc)
    (8.0, 7.5),   # 2: Slight Right
    (6.0, 8.0),   # 3: Mild Left (Gentle curve)
    (8.0, 6.0),   # 4: Mild Right
    (4.0, 8.0),   # 5: Sharp Left (Wider hairpin)
    (8.0, 4.0),   # 6: Sharp Right
    (0.0, 8.0),   # 7: Pivot Left (Gentle pivot around inner wheel)
    (8.0, 0.0),   # 8: Pivot Right
]

# Hyper parameter for tuning
ALPHA = 0.2
GAMMA = 0.9
EPSILON = 0.1

# Saved next to this script, so it doesn't depend on the launch directory.
Q_TABLE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "q_table.pkl")

# Global variables for state memory and dynamic epsilon
last_seen_side = "CENTER"
last_action_taken = 0
training_steps = 0


# =============================================================================
#  TODO (participants): implement get_state(), get_reward() and choose_action().
#  You may also add your own helper functions in this section.
# =============================================================================
def get_state(sensors):
    """Convert the sensor reading into a discrete Q-table state."""
    global last_seen_side, last_action_taken
    
    # Convert sensors to a binary tuple (1 if sensor reading > 0.5 else 0)
    raw = [1 if sensors[k] > 0.5 else 0 for k in SENSOR_ORDER]
    
    # Invert if the background is detected instead of line
    if sum(raw) > 2:
        raw_state = tuple(1 - x for x in raw)
    else:
        raw_state = tuple(raw)
        
    # Memory logic
    if raw_state[0] == 1 or raw_state[1] == 1:
        last_seen_side = "LEFT"
    elif raw_state[3] == 1 or raw_state[4] == 1:
        last_seen_side = "RIGHT"
    elif raw_state[2] == 1:
        last_seen_side = "CENTER"
        
    state = raw_state + (last_seen_side, last_action_taken)
    return state


def get_reward(sensors, state):
    """Compute the reward for the latest reading (the result of the last action)."""
    sensor_state = state[:5]
    last_seen_side = state[5]
    last_action = state[6]
    
    # 1. Perfectly centered
    if sensor_state == (0, 0, 1, 0, 0):
        return 10.0
        
    # 2. Slightly off-center (still safe)
    elif sensor_state in [(0, 1, 1, 0, 0), (0, 0, 1, 1, 0)]:
        return 5.0
        
    # 3. Moderately off-center
    elif sensor_state in [(0, 1, 0, 0, 0), (0, 0, 0, 1, 0)]:
        return 2.0
        
    # 4. Far edge (danger)
    elif sensor_state in [(1, 1, 0, 0, 0), (0, 0, 0, 1, 1), (1, 0, 0, 0, 0), (0, 0, 0, 0, 1)]:
        return -5.0
        
    # 5. Completely lost (rely on memory to penalize wrong turns)
    elif sensor_state == (0, 0, 0, 0, 0):
        left_actions = [1, 3, 5, 7]
        right_actions = [2, 4, 6, 8]
        if last_seen_side == "LEFT" and last_action in left_actions:
            return -2.0 # Good guess, keep turning left
        elif last_seen_side == "RIGHT" and last_action in right_actions:
            return -2.0 # Good guess, keep turning right
        else:
            return -20.0 # Wrong way!
            
    # Default fallback
    return 0.0


def choose_action(agent, state, training):
    """Pick an action index for the current state (the policy)."""
    global last_action_taken
    
    agent._ensure(state)
    
    if training:
        current_epsilon = agent.epsilon
    else:
        current_epsilon = 0.0
        
    if training and random.random() < current_epsilon:
        action = random.randint(0, agent.n_actions - 1)
    else:
        q_values = agent.q_table[state]
        max_q = max(q_values)
        best_actions = [i for i, q in enumerate(q_values) if q == max_q]
        action = random.choice(best_actions)
        
    last_action_taken = action
    return action


# =============================================================================
#  Q-learning agent (Don't Edit this)
# =============================================================================
class QLearningAgent:
    def __init__(self, n_actions, alpha, gamma, epsilon, path):
        self.n_actions = n_actions
        self.alpha = alpha
        self.gamma = gamma
        self.epsilon = epsilon
        self.path = path
        self.q_table = {}   

    def _ensure(self, state):
        if state not in self.q_table:
            self.q_table[state] = [0.0] * self.n_actions

    def update(self, state, action, reward, next_state):
        """Q-learning update. Called only in train mode."""
        self._ensure(state)
        self._ensure(next_state)
        best_next = max(self.q_table[next_state])
        td_target = reward + self.gamma * best_next
        self.q_table[state][action] += self.alpha * (td_target - self.q_table[state][action])

    def load(self):
        if os.path.exists(self.path):
            with open(self.path, "rb") as f:
                self.q_table = pickle.load(f)
            print(f"Loaded Q-table ({len(self.q_table)} states) from {self.path}")
            return True
        return False

    def save(self):
        with open(self.path, "wb") as f:
            pickle.dump(self.q_table, f)
        print(f"Saved Q-table ({len(self.q_table)} states) to {self.path}")


# =============================================================================
#  Main loop
# =============================================================================
def run(mode):
    training = (mode == "train")

    agent = QLearningAgent(len(ACTIONS), ALPHA, GAMMA, EPSILON, Q_TABLE_PATH)
    loaded = agent.load()
    if not training and not loaded:
        print("ERROR: test mode needs a trained Q-table. Run --mode train first.")
        return

    client = CoppeliaClient(host="127.0.0.1", port=50002)
    client.connect()
    print(f"Connected to bridge_task1b. Mode = {mode}. (Ctrl+C to stop)")

    last_sensors = None
    prev_state = None
    prev_action = None
    reward = 0.0

    try:
        while True:
            sensors = client.receive_sensor_data()
            if sensors is not None:
                last_sensors = sensors
            if last_sensors is None:
                time.sleep(0.02)
                continue

            state = get_state(last_sensors)
            reward = get_reward(last_sensors, state)
            if training and prev_state is not None:
                agent.update(prev_state, prev_action, reward, state)

            action = choose_action(agent, state, training)
            left, right = ACTIONS[action]
            client.send_motor_command(
                left, right,
                state=list(state),  
                reward=reward,
                action=action,
            )

            prev_state, prev_action = state, action
            time.sleep(0.05)   # ~20 Hz
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        try:
            client.send_motor_command(0.0, 0.0, state=0, reward=0.0, action=0)
        except Exception:
            pass
        client.close()
        if training:
            agent.save()   # persist what was learned


def main():
    parser = argparse.ArgumentParser(description="Task 1B - Q-Learning")
    parser.add_argument("--mode", choices=["train", "test"], default="train",
                        help="train: explore + update Q-table; test: greedy, no update")
    args = parser.parse_args()
    run(args.mode)


if __name__ == "__main__":
    main()
