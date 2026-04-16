<div align="center">

#  VTOL System
### Vertical Take-Off and Landing — 3-DOF Helicopter Rig



> A real-time, hardware-in-the-loop control system for a 3 Degree-of-Freedom VTOL rig, featuring a full state-feedback **LQR-Integral** controller designed in MATLAB, executed in LabVIEW, and implemented on embedded Arduino hardware.

</div>

---

##  System Overview

> *A 3-DOF helicopter rig mounted on a mechanical support, driven by two brushless motors and instrumented with an IMU and two potentiometers.*

![VTOL System Architecture](diagramvtol.png)

---

##  Demo Video

> ** [Watch the live demonstration →](https://drive.google.com/file/d/1X5AhMbgtWYZdPLM-NShv_fOlvziu0eXS/view)**
>
> The demo showcases the rig stabilizing from a perturbed initial condition, tracking elevation and yaw setpoints in real-time while rejecting roll disturbances all driven by the closed-loop LQRI controller running across the Arduino–LabVIEW pipeline.

---

##  Table of Contents

- [Project Motivation](#-project-motivation)
- [Physical Model & Dynamics](#-physical-model--dynamics)
- [State-Space Linearization](#-state-space-linearization)
- [Control Strategy — LQR with Integral Action](#-control-strategy--lqr-with-integral-action)
- [Embedded Programming — Arduino](#-embedded-programming--arduino)
- [LabVIEW Real-Time Interface](#-labview-real-time-interface)
- [MATLAB Design Workflow](#-matlab-design-workflow)
- [Hardware Stack](#-hardware-stack)
- [Results](#-results)
- [Team](#-team)

---

##  Project Motivation

**Vertical Take-Off and Landing (VTOL)** aircraft represent one of the most demanding challenges in modern control engineering. Unlike conventional fixed-wing aircraft, VTOL platforms must simultaneously manage lift, attitude, and position without the luxury of aerodynamic surfaces relying entirely on thrust vectoring and active control.

This project implements a lab-scale VTOL rig with **3 Degrees of Freedom** (Elevation `e`, Roll `θ`, Yaw `ψ`) to study and validate advanced control strategies on a real physical platform. The system is intentionally designed around accessible, cost-efficient components to demonstrate that high-performance optimal control is achievable without industrial-grade equipment.

**Key engineering challenges addressed:**

- **Strong Input Coupling** : thrust intended for lift simultaneously induces lateral forces and yaw moments due to thruster geometry (ε ≠ 0)
- **Underactuated Dynamics** : 3 DOF are controlled with only 2 inputs (F₁, F₂), demanding a carefully designed control law
- **Nonlinear Equations of Motion** : trigonometric coupling between axes creates sensitivity to operating-point deviations and external disturbances
- **Real-Time Embedded Execution** : the control loop must close at 100 Hz with deterministic timing across a heterogeneous hardware pipeline

---

##  Physical Model & Dynamics

The rig transitions from the classical free-flying VTOL benchmark to a **3-DOF Helicopter Rig** model mechanically constrained on a pivot support, capturing the same complex coupled dynamics in a safe, lab-testable form.

### Nonlinear Equations of Motion

The system dynamics derive from Newton-Euler mechanics applied to the three rotational axes:

$$J_e \ddot{e} = l(u\cos\theta + \epsilon v \sin\theta) - mgl\cos e$$

$$J_\theta \ddot{\theta} = d \cdot v$$

$$J_\psi \ddot{\psi} = l(u\sin\theta + \epsilon v \cos\theta)\cos e$$

Where:
- `u = F₁ + F₂`  total thrust (lift input)
- `v = F₁ − F₂`  differential thrust (torque input)
- `ε = 0.2` **input coupling coefficient** (the source of the nonlinearity)

### Physical Parameters

| Parameter | Symbol | Value | Unit |
|-----------|--------|-------|------|
| Arm length | `l` | 0.56 | m |
| Motor spacing | `d` | 0.14 | m |
| Payload mass | `m` | 0.03 | kg |
| Elevation inertia | `Jₑ` | 0.21 | kg·m² |
| Roll inertia | `J_θ` | 0.0032 | kg·m² |
| Yaw inertia | `J_ψ` | 0.23 | kg·m² |
| Coupling coefficient | `ε` | 0.2 | — |
| Hover thrust | `u₀ = mg` | 0.2943 | N |

---

##  State-Space Linearization

The nonlinear system is linearized around the **hover equilibrium** (e = θ = ψ = 0, u = u₀ = mg) to obtain the standard LTI form `ẋ = Ax + Bτ`.

### State Vector

$$x = \begin{bmatrix} e & \theta & \psi & \dot{e} & \dot{\theta} & \dot{\psi} \end{bmatrix}^T$$

### System Matrices

The A matrix captures the natural coupling between yaw and roll angle via the `lu₀/J_ψ` term, a direct consequence of the ε ≠ 0 strong input coupling:

```
A (6×6):                              B (6×2):
⎡ 0    0         0    1    0    0 ⎤   ⎡   0       0      ⎤
⎢ 0    0         0    0    1    0 ⎥   ⎢   0       0      ⎥
⎢ 0    0         0    0    0    1 ⎥   ⎢   0       0      ⎥
⎢ 0    0         0    0    0    0 ⎥   ⎢ l/Jₑ     0      ⎥
⎢ 0    0         0    0    0    0 ⎥   ⎢   0     d/J_θ   ⎥
⎣ 0  lu₀/J_ψ    0    0    0    0 ⎦   ⎣   0    lε/J_ψ   ⎦
```

---

##  Control Strategy : LQR with Integral Action

### Why LQR?

The **Linear Quadratic Regulator** is an optimal state-feedback controller that minimizes the infinite-horizon cost functional:

$$J = \int_0^{\infty} \left( x^T Q x + u^T R u \right) dt$$

This gives a principled, systematic way to trade off **tracking accuracy** (Q) against **control effort** (R), and produces provably stable closed-loop poles by solving the algebraic Riccati equation.

### Why Integral Action?

Pure LQR guarantees regulation to zero but cannot reject constant disturbances or track non-zero setpoints without steady-state error. **Integral states** are appended for elevation `e` and yaw `ψ`, forming an **8-state augmented system** (LQRI):

```
Augmented state: x_aug = [e, θ, ψ, ė, θ̇, ψ̇, ∫e, ∫ψ]ᵀ
```

The augmented matrices become:

```
A_aug (8×8) = ⎡  A    | 0₆ₓ₂ ⎤     B_aug (8×2) = ⎡  B   ⎤
              ⎣ -C_int | 0₂ₓ₂ ⎦                   ⎣ 0₂ₓ₂ ⎦

where C_int selects states 1 (e) and 3 (ψ) for integration.
```

### Tuned Weighting Matrices

After systematic tuning, the optimal weights were found to be:

```
Q = diag([100, 300, 150, 1, 100, 5, 30, 20])
R = diag([300, 300])
```

Higher weight on θ (300) reflects the strong coupling sensitivity of the roll axis. Higher R penalizes large motor commands, protecting the ESCs and motors.

### Resulting Gain Matrix K

Solved via MATLAB's `lqr()` function (Riccati equation):

```
K = ⎡ 0.9287    0       0      0.8366    0       0     −0.3162    0     ⎤
    ⎣   0     1.7954  1.3396     0     0.6171  2.5070     0     −0.2582 ⎦
```

The control law applied in real-time: **`τ = −K · x_aug`**

### Controllability Verification

The augmented system was verified to be **fully controllable** (rank of controllability matrix = 8) before proceeding to LQR synthesis.

---

##  Embedded Programming — Arduino

The Arduino layer handles all **real-time sensor acquisition**, signal conditioning, and actuator driving designed for deterministic 100 Hz execution.

### Key Implementation Details

**Timing**  A `micros()`-based loop with a 10 ms period ensures consistent sampling without blocking delays:
```cpp
if (currentMicros - lastMicros >= 10000) { /* 100 Hz control tick */ }
```

**IMU Interface (I²C)**  Direct register-level communication with the ADXL345 accelerometer (0x53) and L3G4200D gyroscope (0x68) via inline I²C writes to minimize function call overhead on the hot path.

**Sensor Fusion** Elevation (pitch) is derived from accelerometer data using `atan2f()`. Roll and Yaw are read from two potentiometers via analog inputs and scaled to radians:
```cpp
float ADC_SCALE = 4.7123f / 1024.0f; // Maps 0–1023 → 0–3π/2 rad
```

**Low-Pass Filtering** Angular velocities (roll speed, yaw speed) are filtered with a first-order IIR filter (α = 0.15) with deadband suppression to reduce noise at rest:
```cpp
filteredRollSpeed = 0.15f * (dRoll * dt_inv) + 0.85f * filteredRollSpeed;
```

**Auto-Calibration** On startup, 100 samples are averaged to compute per-axis offsets for pitch, roll, yaw, and both gyro axes.

**ESC Control** Two brushless motors are driven via PWM signals in the 1000–1700 μs range using the Arduino `Servo` library on pins 9 and 10.

**Serial Protocol** Bidirectional 230400 baud UART:
- **Outgoing** (Arduino → LabVIEW): `e,θ,ψ,ė,θ̇,ψ̇\n` at 100 Hz
- **Incoming** (LabVIEW → Arduino): `ESC1_μs,ESC2_μs\n`

---

##  LabVIEW Real-Time Interface

LabVIEW serves as the **real-time control host**  receiving the 6-state vector from Arduino, computing the control law, and dispatching motor commands within the same loop iteration.

### LabVIEW Functions

- **Serial Acquisition** : Parse the incoming comma-delimited state vector at 100 Hz
- **State Augmentation** : Numerically integrate error on `e` and `ψ` to build `x_aug`
- **Control Law** : Matrix multiplication: `τ = −K · x_aug` using the imported K matrix
- **Input Mixing** : Convert to motor commands: `ESC₁ = u + v`, `ESC₂ = u − v`, then map to PWM microseconds
- **Real-Time Monitoring** : Live plots of all 6 states, control inputs, and setpoints for operator oversight

---

##  MATLAB Design Workflow

```matlab
% 1. Define physical parameters
l=0.56; d=0.14; m=0.03; u0=m*9.81; J_psi=0.23; J_e=0.21; J_theta=0.0032; epsilon=0.2;

% 2. Build linearized (A, B) matrices
% 3. Augment with integral states on e and ψ
% 4. Verify controllability — rank(ctrb(A_aug, B_aug)) == 8
% 5. Tune Q and R weights
Q = diag([100, 300, 150, 1, 100, 5, 30, 20]);
R = diag([300, 300]);

% 6. Solve Riccati equation
[K, S, e] = lqr(A_aug, B_aug, Q, R);

% 7. Export gain matrix for LabVIEW
writematrix(K, 'export_data.csv');
```

Run [`LQRI.m`](LQRI.m) to reproduce the full design and export a fresh K matrix.

---

##  Hardware Stack

| Component | Role | Interface |
|-----------|------|-----------|
| **Arduino UNO** | Embedded control node, sensor I/O | USB / UART |
| **ADXL345 Accelerometer** | Elevation (pitch) angle measurement | I²C (0x53) |
| **L3G4200D Gyroscope** | Angular velocity measurement | I²C (0x68) |
| **Potentiometer × 2** | Roll (θ) and Yaw (ψ) angle measurement | Analog A0, A1 |
| **Brushless Motor × 2** | Thrust generation at M1 and M2 | ESC PWM (pins 9, 10) |
| **PC + LabVIEW** | Real-time control host and HMI | Serial @ 230400 baud |
| **MATLAB** | Offline LQR design and gain export | CSV |

---

##  Repository Structure

```
VTOL-Control/
├── arduinoVTOL.ino        # Embedded firmware — sensor fusion, ESC driver, serial protocol
├── LQRI.m                 # MATLAB LQR-Integral design script → exports K matrix
├── LabVIEW_VTOL.vi        # LabVIEW VI — real-time control loop and HMI
├── export_data.csv        # Pre-computed K matrix (auto-generated by LQRI.m)
├── diagramvtol.png        # System architecture diagram
├── vtol.pdf               # Full project presentation
└── README.md
```

---

##  Results

The closed-loop LQRI controller achieves:

- ✅ **Zero steady-state error** on elevation and yaw setpoints (guaranteed by integral action)
- ✅ **Stable roll regulation** at θ = 0 despite strong input coupling (ε = 0.2)
- ✅ **100 Hz real-time loop** with sub-millisecond Arduino cycle time (verified via `micros()` profiling)
- ✅ **Disturbance rejection**  manual perturbations in roll recover within ~2 seconds
- ✅ **Full controllability** confirmed: rank(C_aug) = 8

---

##  Team

**IIA4 — Institut National des Sciences Appliquées et de Technologie (INSAT)**

| Name |
|------|
| **GRATI Elyes** | 
| **NJEH Oussema** | 
| **SNOUN Ferid** | 
| **KHELIL Souheib** | 

---

##  License

This project is released under the [MIT License](LICENSE). Academic use and adaptation are welcome — please cite this repository if you build upon it.

---

<div align="center">

**Built with 🔧 hardware, 📐 math, and ☕ late nights.**

*Control Systems · Nonlinear Dynamics · Embedded Programming · Optimal Control · Real-Time Systems*

</div>
