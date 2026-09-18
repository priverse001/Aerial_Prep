# Part 2: PX4 Motor Effectiveness Characterization

## 1. Overview
This module explores quadrotor underactuation dynamics by characterizing the flight stack's response to single-rotor thrust degradation across 5 distinct levels: **100%, 75%, 50%, 25%, and 0%**.

Rather than overriding raw PWM outputs or killing the simulated actuator model directly, this implementation operates directly on PX4's **Control Allocation Model** (`ActuatorEffectivenessRotors`), modifying the effectiveness column representing the affected rotor during active flight without simulation restarts.

---

## 2. Technical Formulation
In PX4's modern control allocation architecture:
$$\mathbf{v} = \mathbf{B} \cdot \mathbf{u}$$
where:
- $\mathbf{v} \in \mathbb{R}^6$ represents the desired torque and thrust vector $[\tau_x, \tau_y, \tau_z, F_x, F_y, F_z]^T$.
- $\mathbf{u} \in [0, 1]^N$ is the normalized actuator setpoint vector.
- $\mathbf{B}$ is the Control Effectiveness Matrix computed in `ActuatorEffectivenessRotors.cpp`.

For rotor $i$:
$$\mathbf{B}_{:, i} = \begin{bmatrix} c_{t, i} (\mathbf{p}_i \times \mathbf{a}_i) - c_{t, i} k_{m, i} \mathbf{a}_i \\ c_{t, i} \mathbf{a}_i \end{bmatrix}$$

By dynamically scaling $c_{t, i} = \alpha_i \cdot c_{t, \text{nominal}}$ with $\alpha_i \in \{1.0, 0.75, 0.50, 0.25, 0.0\}$, the allocator dynamically computes the adjusted pseudo-inverse / sequential desaturation solution, redistributing effort across remaining healthy motors until physical saturation occurs.

---

## 3. Comparison: Built-In Failure Injection vs. Control Allocation Scaling

| Feature | PX4 Built-In Failure (`failure motor off`) | Runtime Allocation Degradation (`CA_ROTOR_CT`) |
| :--- | :--- | :--- |
| **Layer of Action** | Output layer (zeroes PWM / actuator output) | Control Allocation Layer ($\mathbf{B}$ matrix column scaling) |
| **Allocator Awareness** | ❌ Unaware (assumes rotor is 100% active) | ✅ Aware (updates rotor authority model) |
| **Control Redistribution** | No redistribution; PID integrators wind up rapidly | Allocator redistributes load to diagonal/adjacent motors |
| **Flight Dynamics** | Sudden, violent uncontrolled yaw/roll divergence | Progressive degradation; stable at moderate loss, gradual loss of control at severe underactuation |

---

## 4. Test Execution Instructions

### Step 1: Launch Gazebo SITL
```bash
cd /home/priverse/px4/PX4-Autopilot
make px4_sitl gazebo-classic_iris
```

### Step 2: Run Phase 1 Reference Test (Built-In Failure)
```bash
python3 /home/priverse/Documents/Aerial_Prep/PART_2/motor_effectiveness_test_runner.py --phase1
```

### Step 3: Run Phase 2 Characterization Test (Allocation Scaling across 5 Levels)
```bash
python3 /home/priverse/Documents/Aerial_Prep/PART_2/motor_effectiveness_test_runner.py
```
