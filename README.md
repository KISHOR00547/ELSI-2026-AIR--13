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



