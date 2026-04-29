# Angad Humanoid — Stable Deep Crouch in MuJoCo

<p align="center">
  <b>A 65 kg, 21-DOF humanoid robot achieving a stable 12 cm deep crouch</b><br>
  <i>using 6-DOF inverse kinematics with oblique hip-axis compensation</i>
</p>

---

## 🤖 What Is This?

This repository contains a **complete, self-contained** MuJoCo simulation of the **Angad humanoid robot** performing a stable deep-crouch pose. The robot weighs 65 kg, has 21 degrees of freedom, and features a non-standard kinematic design where the hip-pitch joints are mounted at a **15-degree oblique angle** — a challenging engineering constraint that most off-the-shelf controllers cannot handle.

This project solves that problem using custom **6-DOF Inverse Kinematics** with axis-compensation and a **PD balance controller** with IMU feedback.

---

## 🎯 The Problem

Most bipedal humanoid robots (ASIMO, NAO, Atlas) have hip-pitch joints aligned perfectly horizontal (`axis = [1, 0, 0]`). Angad's hip-pitch joints are tilted:

| Joint | Global Axis |
|---|---|
| Left Hip Pitch  | `[0.966, 0.000, -0.259]` |
| Right Hip Pitch | `[0.966, 0.000, +0.259]` |

This 15° oblique mounting means that when the robot bends its knees to crouch:
1. The legs **twist outward** (yaw coupling from the skewed axis)
2. The feet **splay laterally**, scrubbing against the floor
3. The Center of Mass (COM) traces a **3D S-curve** instead of dropping straight down
4. The robot **rolls violently** and falls over

Standard crouch controllers that only use hip-pitch + knee + ankle-pitch fail catastrophically on this robot.

---

## 💡 The Solution

### 1. 6-DOF Inverse Kinematics (`angad_oblique_ik.py`)

Instead of using only 3 joints (hip, knee, ankle) per leg, we use **all 6 joints** simultaneously:

| Joint | Role in Crouch |
|---|---|
| **Hip Pitch** (`-0.5543 rad`) | Primary: lowers the body |
| **Hip Roll** (`+0.0425 rad`) | Compensates lateral swing from oblique axis |
| **Thigh Yaw** (`-0.0032 rad`) | Counter-twist for the 15° skew coupling |
| **Knee** (`-1.1537 rad` / 66°) | Primary: deep knee bend |
| **Ankle Pitch** (`-0.6024 rad`) | Keeps foot flat on the ground |
| **Ankle Roll** (`0.0 rad`) | No correction needed |

These angles were computed by a **numerical optimizer** (`scipy.optimize.minimize` with SLSQP) that minimizes foot position error + foot orientation error while respecting all physical joint limits.

### 2. Left↔Right Mirroring Rules

Because the joint axes are not symmetrically oriented, standard sign-flipping doesn't work. We empirically determined the correct mirroring:

```
hip_pitch_r  =  hip_pitch_l      (SAME sign — NOT mirrored!)
hip_roll_r   = -hip_roll_l       (mirrored)
thigh_yaw_r  = -thigh_yaw_l      (mirrored)
knee_r       =  knee_l           (SAME sign)
ankle_pitch_r=  ankle_pitch_l    (SAME sign)
ankle_roll_r = -ankle_roll_l     (mirrored)
```

### 3. IMU Balance Controller

A PD controller continuously corrects pitch and roll deviations using simulated IMU data:

```
Pitch correction → 100% to ankle pitch actuators
Roll correction  → 40% to ankle roll + 60% to hip roll
```

Balance gains:
- **Pitch:** Kp=2000, Kd=300
- **Roll:** Kp=1500, Kd=200

### 4. Stability Verification

We ran a systematic sweep across all crouch depths (1–20 cm) and found the stability envelope:

| Drop | Knee Angle | Knee Actuator Usage | Result |
|------|-----------|---------------------|--------|
| 5 cm | 42° | 100% (saturated) | ❌ Falls |
| 7 cm | 50° | 100% (barely) | ✅ Stable |
| 10 cm | 60° | 77% | ✅ Stable |
| **12 cm** | **66°** | **67%** | **✅ Stable (recommended)** |
| 15 cm | 74° | 100% (saturated) | ❌ Falls |

**12 cm** is the optimal operating point — deepest crouch with sufficient actuator headroom (33% margin).

---

## 📁 Repository Structure

```
angad_crouch_final/
├── angad_crouch_final.py      # Main script — run this to see the crouch
├── angad_oblique_ik.py        # 6-DOF IK solver for computing crouch angles
├── angad_robot_params.py      # All robot parameters (importable Python module)
├── angad_inertial_data.txt    # Human-readable inertial data reference
├── XP_robot_walking.xml       # MuJoCo MJCF robot model
├── meshes/                    # 22 STL mesh files for the robot body
│   ├── pelvis.stl
│   ├── torso.stl
│   ├── hip_pitch_l.stl
│   ├── hip_pitch_r.stl
│   ├── hip_roll_l.stl
│   ├── hip_roll_r.stl
│   ├── thigh_l.stl
│   ├── thigh_r.stl
│   ├── leg_shank_l.stl
│   ├── leg_shank_r.stl
│   ├── uj_l.stl
│   ├── uj_r.stl
│   ├── foot_l.stl
│   ├── foot_r.stl
│   ├── shoulder_pitch_l.stl
│   ├── shoulder_pitch_r.stl
│   ├── shoulder_roll_l.stl
│   ├── shoulder_roll_r.stl
│   ├── bicep_l.stl
│   ├── bicep_r.stl
│   ├── forearm_l.stl
│   └── forearm_r.stl
└── README.md
```

---

## 🚀 How to Run

### Prerequisites

- Python 3.8+
- MuJoCo (via pip)
- NumPy
- SciPy (only needed for `angad_oblique_ik.py`)

### Installation

```bash
# Clone the repository
git clone https://github.com/Charan-mdnl/angad_crouch_final.git
cd angad_crouch_final

# Install dependencies
pip install mujoco numpy scipy
```

### Run the Crouch Pose

```bash
python angad_crouch_final.py
```

This will:
1. Load the MuJoCo model
2. Spawn the robot directly into the verified 12 cm crouch pose
3. Open a 3D viewer with the robot holding a stable crouch
4. Print real-time balance metrics (roll, pitch, COM height, knee actuator usage)

Expected output:
```
============================================================
  ANGAD CROUCH POSE — 12cm drop, 66° knee bend
  Verified stable. Watching balance metrics...
============================================================
t=   0.5s | Roll:  -1.6° | Pitch:  +0.6° | COM Z: 0.880m | Knee L: 64% R: 42%
t=   1.0s | Roll:  -1.6° | Pitch:  +0.4° | COM Z: 0.881m | Knee L: 58% R: 43%
t=   1.5s | Roll:  -1.5° | Pitch:  +0.5° | COM Z: 0.880m | Knee L: 60% R: 42%
...
```

### Run the IK Solver (optional)

To compute crouch angles for a different depth:

```bash
python angad_oblique_ik.py
```

---

## 🔧 Robot Specifications

| Parameter | Value |
|---|---|
| Total Mass | 64.94 kg |
| Total DOF | 21 (6 per leg + 4 per arm + 1 torso) |
| Standing Height | 0.746 m (pelvis Z) |
| Track Width | 0.299 m (distance between feet) |
| Knee Gear Ratio | 120:1 (max 120 Nm) |
| Hip Pitch Gear Ratio | 80:1 (max 80 Nm) |
| Ankle Pitch Gear Ratio | 80:1 (max 80 Nm) |
| Hip Pitch Axis Tilt | 15° oblique |
| Ankle Pitch Range | ±40° (hard limit) |
| Knee Range | -90° to 0° |

---

## 🧠 Techniques Used

1. **Numerical Inverse Kinematics** — SciPy SLSQP optimizer minimizing foot position + orientation error
2. **Minimum Jerk Trajectory** — Smooth 5th-order polynomial interpolation for transitions
3. **PD Joint Control** — High-gain position tracking with velocity damping
4. **IMU Balance Feedback** — Continuous pitch/roll correction via quaternion decomposition
5. **Gear-Normalized Torque** — `ctrl = torque / gear_ratio` to respect actuator limits
6. **Systematic Stability Sweep** — Automated headless testing across all depths to find the stability envelope
7. **Global Axis Analysis** — Computed joint axes in world frame to determine correct L↔R mirroring

---

## 📊 Key Findings

- **Ankle-only crouching fails** — Bending only the ankles (0–10 cm range) is unstable because the COM moves too far forward
- **Deep crouching requires 6-DOF coordination** — All 6 leg joints must work together to compensate for the oblique hip axis
- **The hip-roll compensation is critical** — Even a tiny 0.04 rad hip-roll offset prevents the lateral COM drift that causes roll failure
- **12 cm is the sweet spot** — Deepest crouch with 33% actuator torque margin
- **15 cm crouch is physically possible but dynamically unstable** — The knee actuator saturates at 100%, leaving no margin for balance corrections

---

## 📄 License

This project is for educational and research purposes.

---

## 👤 Author

**Charan** — XP Robotics  
Working on humanoid locomotion control for the Angad bipedal platform.
