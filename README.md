# Tensorflow_PID

Neural-network-augmented PID control for robotic systems. Each script pairs a
TensorFlow/Keras **forward-dynamics model** (trained on simulator rollouts)
with a classical **PID controller**, closing the loop by feeding the NN's
force/acceleration estimates back into the controller as a virtual sensor.

Both scripts are self-contained: they generate training data from a physics
simulator, train the forward-dynamics network, then run a closed-loop
simulation and save diagnostic plots. They are plain Python files without a
`.py` extension — run them with `python <filename>`.

## Contents

| File | System | Description |
|---|---|---|
| [`NN_PID_omnidirectional`](./NN_PID_omnidirectional) | 3-wheel omnidirectional robot | Velocity + force control in the plane (vx, vy) |
| [`NN+PID_twolink`](./NN+PID_twolink) | 2-link planar manipulator | Joint position + torque control (q1, q2) |

## Architecture

```
 Desired setpoint
        │
        ▼
 [ PID Controller ] ←── error from (robot state + NN predictions)
        │
        ▼
 [ control signal ] ──────────────────────→ [ Robot Simulator ]
        │                                            │
        ▼                                            ▼
 [ Forward Dynamics NN ] ←──────────── [ state feedback ]
   predicts forces / accelerations
        │
        └───────────────── fed back to PID ──────────┘
```

The NN acts as a learned virtual sensor: it predicts quantities (contact
forces, joint torques, accelerations) that the PID's secondary control loop
uses as feedback, alongside the directly-measured state from the simulator.

### `NN_PID_omnidirectional`

Simulates a symmetric 3-wheel omnidirectional robot driven by motor voltages.

- **Simulator** (`OmniRobotSimulator`): motor electrical dynamics (exact
  exponential solution, stable for any timestep), wheel Jacobian mapping
  wheel/body velocities, body drag, and Euler-integrated pose/velocity.
- **NN inputs (10)**: `u1, u2, u3, w1, w2, w3, vx, vy, omega, theta`
  **NN outputs (5)**: `Fx, Fy, ax, ay, alpha`
- **Network**: `Dense(128) → BN → ReLU → Dropout → Dense(64) → BN → ReLU →
  Dropout → Dense(5, linear)`, trained with MSE loss, early stopping, and
  LR-on-plateau.
- **Controller** (`OmniRobotPIDController`): four PID loops — velocity
  tracking (`vx`, `vy`) plus a secondary force-tracking correction (`Fx`,
  `Fy`) using the NN's force estimates — summed, rotated into the body
  frame, and mapped to motor voltages via the wheel Jacobian.
- **Closed-loop demo**: tracks a `(vx*, vy*, Fx*, Fy*)` setpoint that steps
  to a second setpoint halfway through the run.

### `NN+PID_twolink`

Simulates a 2-link planar manipulator with full rigid-body dynamics
(inertia, Coriolis/centrifugal, gravity, and joint damping terms), plus
joint limits with velocity-reflecting collisions.

- **NN inputs (6)**: `tau1, tau2, q1, q2, dq1, dq2`
  **NN outputs (4)**: `tau1_nn, tau2_nn, ddq1, ddq2`
- **Network**: `Dense(128) → BN → ReLU → Dropout → Dense(64) → BN → ReLU →
  Dropout → Dense(32) → BN → ReLU → Dense(4, linear)`, with L2
  regularization on each dense layer for stability.
- **Controller** (`TwoLinkRobotPIDController`): conservative-gain PID on
  joint position (`q1`, `q2`) plus a low-gain torque-correction loop using
  the NN's torque estimates, with derivative low-pass filtering and
  anti-windup.
- **Closed-loop demo**: tracks a `(q1*, q2*, tau1*, tau2*)` setpoint with a
  step change halfway through the run, and reports stability-violation
  counts (excess joint accelerations) during the run.

## Requirements

```
numpy
tensorflow
matplotlib
scikit-learn
```

Install with:

```bash
pip install numpy tensorflow matplotlib scikit-learn
```

## Usage

Each script runs end-to-end: generate data → train the forward-dynamics NN
→ run the closed-loop PID simulation → save plots.

```bash
python NN_PID_omnidirectional
python NN+PID_twolink
```

Outputs (trained model checkpoints, training curves, and closed-loop result
plots) are written to `saved_model/` and `saved_model_2link_stable/`
respectively.
