# Vehicle-Trajectory-Estimation-using-EKF
Vehicle Trajectory Estimation using Extended Kalman Filter

## Extended Kalman Filter for Vehicle Localization with LIDAR Measurements

This repository implements an Extended Kalman Filter (EKF) for real-time vehicle localization using a LIDAR sensor. The filter estimates the vehicle's position (x, y) and orientation (theta) in a 2D environment based on odometry measurements (linear and angular velocities) and LIDAR range and bearing measurements of known landmarks.

### Problem Description

The goal is to estimate the trajectory of a vehicle equipped with a LIDAR sensor. We assume knowledge of the global positions of landmarks and data association (i.e., which measurement belongs to which landmark).

### Models

**Motion Model:**

The motion model relates the previous state and control inputs (odometry) to the current state:


$$X_k = X_{k-1} + T \begin{bmatrix}
\cos(\theta_{k-1}) & 0 \\
\sin(\theta_{k-1}) & 0 \\
0 & 1
\end{bmatrix}
(\begin{bmatrix}
v_k \\
\omega_k \\
\end{bmatrix} + W_k)
\ \ \, \ \ \ \mathbf{W}_k = \mathcal{N}\left(\mathbf{0}, \mathbf{Q}\right)$$

where:
- `X_k` = current state vector = (x, y, theta)
- `X_(k-1)` = previous state vector
- `T` = timestep
- `v_k` = linear velocity
- `ω_k` = angular velocity
- `W_k` = process noise (zero-mean Gaussian with covariance `Q`)

**Measurement Model:**

The measurement model relates the current state to the LIDAR range and bearing measurements of a landmark:

$$
\mathbf{y}^l_k =
\begin{bmatrix}
\sqrt{(x_l - x_k - d\cos\theta_{k})^2 + (y_l - y_k - d\sin\theta_{k})^2} \\
atan2\left(y_l - y_k - d\sin\theta_{k},x_l - x_k - d\cos\theta_{k}\right) - \theta_k
\end{bmatrix}
+
\mathbf{n}^l_k
\  \ \, \ \ \ \mathbf{n}^l_k = \mathcal{N}\left(\mathbf{0}, \mathbf{R}\right)
$$

where:

- `y_k^l` = measurement vector (range, bearing) for landmark l
- `x_l, y_l` = known coordinates of landmark l
- `d` = distance between robot center and LIDAR
- `n_k^l` = measurement noise (zero-mean Gaussian with covariance R)

### Kalman Filter Implementation

**Explanation:**

1. **Prediction:**


    - Uses the motion model to predict the state and covariance at the current timestep based on the previous state and control inputs.
    - Calculates the Jacobian matrices of the motion model.


2. **Correction:**
    - For each received landmark measurement:
        - Calculates the Jacobian matrices of the measurement model.
        - Computes the Kalman gain.
        - Updates the state estimate and covariance using the measurement.


The filter consists of two main steps:

**PREDICTION**

1. **State Prediction:**

```
X_k = f(hat(X_(k-1)), u_(k-1), 0)
```

2. **Covariance Prediction:**

```
P_k = F_(k-1) * P_(k-1) * F_(k-1)^T + L_(k-1) * Q_(k-1) * L_(k-1)^T
```
- `X_k`: Predicted state vector at timestep k (including x, y, and theta)
- `hat(X_(k-1))`: Estimated state vector at the previous timestep (k-1)
- `u_(k-1)`: Control input vector (e.g., linear and angular velocities) at timestep k-1
- `f`: Motion model function
- `P_k`: Predicted covariance matrix at timestep k
- `F_(k-1)`: Jacobian of the motion model with respect to the state vector
- `L_(k-1)`: Jacobian of the motion model with respect to the process noise
- `Q_(k-1)`: Process noise covariance matrix
- The `hat` symbol (^) is often used to denote an estimated value in Kalman filter notation. Here, `hat(x_(k-1))` represents the estimated state at the previous timestep.


**CORRECTION**

The correction step of the EKF refines the predicted state estimate (`X_k`) and its covariance (`P_k`) based on a received landmark measurement.

1. **Measurement Jacobians:**

```
y^l_k = h(x_k, n_k)   (Measurement Model)

H_k = ∂h / ∂x_k (Jacobian of measurement model w.r.t. state) | evaluated at x_k, 0

M_k = ∂h / ∂n_k (Jacobian of measurement model w.r.t. noise) | evaluated at x_k, 0
```

- `y^l_k`: Actual measurement of landmark `l` at timestep `k`
- `h`: Measurement model function
- `H_k`: Jacobian of the measurement model with respect to the state. This tells us how sensitive the measurement is to changes in the robot's position and orientation.
- `M_k`: Jacobian of the measurement model with respect to the measurement noise. This reflects how the measurement changes with variations in noise.
- We evaluate both Jacobians at the predicted state (`x_k`) and zero noise (`0`) to determine their influence on the measurement.

2. **Kalman Gain:**

```
K_k = P_k * H_k^T * (H_k * P_k * H_k^T + M_k * R_k * M_k^T)^(-1)
```


- `K_k`: Kalman Gain. This matrix determines how much weight to give to the new measurement in updating the state estimate.
- `P_k`: Predicted covariance matrix. This reflects the uncertainty in the predicted state.
- `H_k^T`: Transpose of the measurement model Jacobian.
- `R_k`: Measurement noise covariance matrix. This represents the expected variance of the measurement noise.

3. **State Correction:**

```
y_k^l = h(x_k, 0)  (Predicted measurement)

hat(x_k) = x_k + K_k * (y^l_k - y_k^l)  (Corrected state estimate)
```

- `y_k^l`: Predicted measurement of landmark `l` at timestep `k`. This is calculated using the predicted state (`x_k`) and assuming no noise (`0`).
- `hat(x_k)`: Corrected state estimate. This combines the predicted state (`x_k`) with the Kalman Gain (`K_k`) and the difference between the actual measurement (`y^l_k`) and the predicted measurement (`y_k^l`).
- The Kalman Gain determines how much of the difference between the actual and predicted measurement is incorporated into the corrected state estimate.

4. **Covariance Correction:**

```
hat(P_k) = (I - K_k * H_k) * P_k  (Corrected covariance)
```
- `hat(P_k)`: Corrected covariance matrix. This reflects the updated uncertainty in the state estimate after incorporating the measurement.
- `I`: Identity matrix
- `The Kalman Gain (`K_k`) and the measurement model Jacobian (`H_k`) are used to adjust the predicted covariance (`P_k`) into the corrected covariance (`hat(P_k)`). This reduces the uncertainty in the state estimate as the measurement provides new information.

**Output Trajectory**

<div>
    <img src="https://github.com/Abhi-0212000/Vehicle-Trajectory-Estimation-using-EKF/assets/70425157/1af6131d-e35f-4596-862c-d1e6dd151b95" alt="trajectory_plot" style="width:45%; float:left; margin-right:5%">
    <img src="https://github.com/Abhi-0212000/Vehicle-Trajectory-Estimation-using-EKF/assets/70425157/c2b8c84f-3881-4c50-839d-39ddbcc02f6c" alt="theta_plot" style="width:45%; float:left;">
</div>
