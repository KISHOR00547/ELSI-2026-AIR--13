# ELSI-2026-AIR--13
there is the simulations project in eyantra Elsi 2026


# e-Yantra eLSI 2026-27 – Robotics Control Project

<p align="center">
  <b>Team ID: 862</b><br>
  PID Line Following • Q-Learning • Pick & Place • Node-Based Navigation
</p>

---

## 📌 Project Overview

This repository contains our implementation for the **e-Yantra eLSI 2026-27** robotics tasks.

The project focuses on controlling a differential-drive robot in **CoppeliaSim** using Python. The robot receives sensor data from the simulator through a communication bridge, processes the data, and generates motor commands.

The project includes:

- PID-based line following
- Q-learning based line following
- Sensor data processing
- Node/intersection detection
- Pick and place using proximity sensing
- RGB-based box colour detection
- Route-based navigation
- TCP socket communication
- Python virtual environment setup

---

## 🧠 System Architecture

```text
                 ┌──────────────────────┐
                 │     CoppeliaSim      │
                 │                      │
                 │  Robot + Sensors     │
                 └──────────┬───────────┘
                            │
                     Sensor Data
                            │
                            ▼
                 ┌──────────────────────┐
                 │        Bridge        │
                 │                      │
                 │   TCP Communication  │
                 └──────────┬───────────┘
                            │
                            ▼
                 ┌──────────────────────┐
                 │     Python Code      │
                 │                      │
                 │ Sensor Processing    │
                 │ PID / Q-Learning     │
                 │ Node Detection       │
                 │ Pick & Place Logic   │
                 └──────────┬───────────┘
                            │
                     Motor Commands
                            │
                            ▼
                 ┌──────────────────────┐
                 │       Robot          │
                 │ Left / Right Motors  │
                 └──────────────────────┘



🚗 Task 1A – PID Line Following

Task 1A implements a PID controller for line following.

The robot uses five line sensors:

Left Corner | Left | Middle | Right | Right Corner

The sensor values are converted into a weighted line position.

Sensor weights
Left Corner    → -2
Left           → -1
Middle         →  0
Right          → +1
Right Corner   → +2

The calculated position is used as the PID error.

error = position

The controller calculates:

PID = Kp × error
    + Ki × integral
    + Kd × derivative

The motor speeds are then calculated using differential drive:

Left Motor  = Base Speed + PID
Right Motor = Base Speed - PID
PID features
Proportional control
Integral accumulation
Derivative control
Derivative filtering
Adaptive base speed
Line-loss recovery
Background detection
Sensor inversion
Hysteresis/debounce
🤖 Task 1B – Q-Learning

Task 1B uses reinforcement learning for line following.

The five sensor readings are converted into a discrete state.

Example:

(0, 0, 1, 0, 0)

means the middle sensor is detecting the line.

State

A state is represented by five binary sensor values:

[LC, L, M, R, RC]

Because each sensor has two possible states:

0 = line not detected
1 = line detected

there can theoretically be:

2^5 = 32

different sensor states.

Actions

The robot uses different motor commands as actions.

Example:

Action 0 → Forward
Action 1 → Slight Left
Action 2 → Slight Right
Action 3 → Sharp Left
Action 4 → Sharp Right
Action 5 → Pivot Left
Action 6 → Pivot Right
Action 7 → Stop
Reward Function

The reward function encourages the robot to stay centered on the line.

Example:

Perfect center      → high positive reward
Near center         → positive reward
Outer sensor        → penalty
Line completely lost → large negative reward

The objective is for the robot to learn actions that maximize cumulative reward.

Q-Learning Update

The Q-value is updated using:

Q(s,a) ← Q(s,a)
        + α [r + γ max Q(s',a') - Q(s,a)]

Where:

Parameter	Meaning
α	Learning rate
γ	Discount factor
ε	Exploration probability
s	Current state
a	Current action
r	Reward
s'	Next state
Training
Read sensors
     ↓
Determine state
     ↓
Choose action
     ↓
Move robot
     ↓
Receive reward
     ↓
Determine next state
     ↓
Update Q-table
     ↓
Repeat

The learned Q-table is saved using Python pickle.

📦 Task 2A – Pick and Place

Task 2A combines line following with object manipulation.

The robot must:

Follow the line.
Detect the box.
Identify its colour.
Pick the box.
Navigate to the correct drop zone.
Drop the box.
Proximity Sensor

The proximity sensor determines the distance to the nearest object.

Example logic:

if proximity < 0.25:
    # Object is close

The proximity sensor is different from the five line sensors.

Line sensors
    ↓
Line following

Proximity sensor
    ↓
Object detection

RGB sensor
    ↓
Box colour detection
🎨 Colour Detection

The RGB sensor provides:

color_r
color_g
color_b

The largest RGB channel is used to determine the box colour.

Example:

R > G and R > B → Red

G > R and G > B → Green

B > R and B > G → Blue

A confidence threshold is also used to avoid detecting very weak readings.

🧭 Task 2B – Node-Based Navigation

Task 2B combines:

PID line following
Node detection
Route management
Pick and place
Timed turning
Drop-zone detection

The robot maintains a node counter:

Node 1
Node 2
Node 3
...
Node 26

Different nodes trigger different actions.

Example:

Node 1 → Right turn
Node 2 → Left turn
Node 3 → Left turn
Node 4 → Right turn
...

This creates a predefined route through the track.

Node Detection

The five line sensors are examined together.

Example conditions can detect an intersection when:

Left Corner + Right Corner

or

Left + Middle + Right

detect the line simultaneously.

A cooldown is used so the same node is not counted multiple times.

Node detected
      ↓
Increase node counter
      ↓
Start cooldown
      ↓
Execute maneuver
      ↓
Resume line following
🔄 Persistent State

The PID controller needs information from previous control cycles.

Variables such as:

control_loop._prev_error
control_loop._prev_position
control_loop._prev_derivative

store previous information.

This allows the controller to calculate:

Derivative = Current Error - Previous Error

and recover the line using the previous position.

Persistent variables are also used for:

Node count
Action type
Action timer
Node cooldown
Drop cooldown
⚡ Control Loop Frequency

The controller repeatedly performs:

Sensor Read
     ↓
Processing
     ↓
Control Calculation
     ↓
Motor Command

For example:

time.sleep(0.05)

gives approximately:

1 / 0.05 = 20 Hz

Therefore:

20 control cycles / second

For faster control:

time.sleep(0.02)

gives approximately:

1 / 0.02 = 50 Hz

Higher frequency provides more frequent control updates, but the communication bridge and simulator must be able to handle the increased rate.

🔌 Communication

The Python controller communicates with CoppeliaSim through a bridge.

The connection uses TCP.

Example:

CoppeliaClient(
    host="127.0.0.1",
    port=50002
)
Host
127.0.0.1

means the local computer.

Port
50002

is the TCP communication endpoint used by the bridge.

🐍 Python 3 Environment

The project is developed using Python 3.

A virtual environment can be created using:

python -m venv venv

Activate it on Windows:

venv\Scripts\activate

Activate it on Linux:

source venv/bin/activate

Then install required packages:

pip install -r requirements.txt
▶️ Running the Project
Task 1A

Start CoppeliaSim and open the Task 1A scene.

Start the bridge:

python3 bridge_task1a.py --eval

Run:

python3 task1a_template.py
Task 1B

Start the Task 1B bridge.

Training:

python3 task1b_template.py --mode train

Testing:

python3 task1b_template.py --mode test

Training generates a Q-table:

q_table.pkl
Task 2A

Start the Task 2A scene.

Run the bridge:

./bridge_v1_2a

Then:

python task2a.py

On Windows, use the provided Windows bridge executable.

Task 2B

Start the Task 2B scene and bridge.

Then run:

python task2b.py
🗂️ Suggested Repository Structure
eLSI-2026-27/
│
├── README.md
│
├── Task0/
│   └── task0.py
│
├── Task1A/
│   ├── task1a_template.py
│   └── connector_task1a.py
│
├── Task1B/
│   ├── task1b_template.py
│   ├── connector_task1b.py
│   └── q_table.pkl
│
├── Task2A/
│   ├── task2a.py
│   └── connector.py
│
├── Task2B/
│   ├── task2b.py
│   └── connector_2b.py
│
├── requirements.txt
│
└── docs/
    └── presentation.pptx
🛠️ Technologies Used
Python 3
CoppeliaSim
TCP/IP Socket Communication
PID Control
Q-Learning
Differential Drive
Line Sensors
Proximity Sensor
RGB Color Sensor
Virtual Environments
Git & GitHub
🎯 Key Learning Outcomes

Through this project, I gained practical experience in:

Closed-loop control systems
PID controller implementation
Sensor data processing
Differential-drive robotics
Reinforcement learning
Q-table implementation
Exploration vs exploitation
TCP client-server communication
Real-time control loops
Persistent program state
Node/intersection detection
Pick-and-place automation
Debugging robotic behaviour
Python virtual environments
💡 Future Improvements

Possible improvements include:

Automatic PID parameter tuning
Better sensor calibration
More robust node detection
Dynamic route planning
Improved Q-learning reward shaping
Larger and more expressive state representations
Sensor fusion
Real hardware implementation using STM32/ESP32
Moving from simulation to a physical robot
👨‍💻 Author

Team 862

e-Yantra eLSI 2026-27

⭐ Project Focus

Sense → Process → Decide → Act

The overall robotics pipeline can be summarized as:

Sensors
   ↓
Sensor Processing
   ↓
State / Error
   ↓
PID / Q-Learning / Routing
   ↓
Decision
   ↓
Motor Commands
   ↓
Robot Motion
   ↓
New Sensor Feedback



