

# Autonomous RC Car  (This is an unfinished project that is still in progress!)
## Introduction

In this project, I used a 4-wheel car model to simulate an RC car navigating through a corridor, using Sequential Convex Programming to find the most fuel-efficient path offline.  
I then built an RC car with a Jetson Orin AGX and a ZED2i camera. The ZED2i camera has built-in localization SLAM algorithms that, with visual sensing, can accurately estimate the car's position. 
Combined with the Jetson computer and ROS 2, the Jetson can send signals to an Adafruit controller, which then drives the servos and motor of the car using PWM to track the offline-calculated trajectory.

![RC Car Build](RCCAR.png)

*Figure: The physical build of the RC car is complete. I am currently addressing some software challenges.*


## 🚗 4-Wheel Vehicle Dynamics Model

This repository implements a **4-wheel vehicle dynamics model** for an RC car, simulating its motion and stability while navigating real-world environments. It calculates the vehicle’s position, orientation, and velocity, taking into account the forces acting on **all four wheels** and realistic front steering input.

---

### 📐 Key Aspects

 **Four wheels**: The model explicitly considers all four tires, providing higher accuracy than simpler models (e.g., the bicycle model).  
 **Front steering**: Only the front wheels are actively steered (via the front steering angle). The rear wheels passively roll.  
 **Longitudinal and lateral forces**: Both forward (longitudinal) and sideways (lateral) tire forces are included.  
 **Yaw moment**: Accounts for how the car rotates around its center of mass (yaw motion).  

---

### 🛠 Model Inputs

- Front steering angle (\(\delta\))
- Vehicle speed (longitudinal velocity)
- Tire slip angles (difference between wheel orientation and actual path)
- Vehicle mass and inertia
- Tire-road interaction (simple linear tire model or more advanced models)

---

###  Model Outputs

- Vehicle global position and orientation (\(X, Y, \psi\))
- Velocities (longitudinal, lateral, and yaw rate)
- Forces at each wheel (traction and lateral forces)

---

###  Formulation

#### Coordinate Frames & Variables
- **Global frame**: \((X, Y)\) – fixed world coordinates.
- **Body frame**: \((x, y)\) – attached to vehicle CoM.
- **Yaw angle**: \(\psi\) – vehicle heading angle.
- **Longitudinal velocity**: \(u\) – forward speed.
- **Lateral velocity**: \(v\) – sideways speed.
- **Yaw rate**: \(r = \dot{\psi}\).

#### Vehicle Equations of Motion
1️ **Longitudinal dynamics**  
\[
m(\dot{u} - vr) = \sum F_{x}
\]

2️ **Lateral dynamics**  
\[
m(\dot{v} + ur) = \sum F_{y}
\]

3️ **Yaw motion**  
\[
I_{z} \dot{r} = a(F_{y_{fL}} + F_{y_{fR}}) - b(F_{y_{rL}} + F_{y_{rR}})
\]

where:  
- \(m\): vehicle mass  
- \(I_{z}\): yaw moment of inertia  
- \(a, b\): distances from CoM to front/rear axles  
- \(F_{y_{fL}}, F_{y_{fR}}\): lateral forces at front-left/right wheels  
- \(F_{y_{rL}}, F_{y_{rR}}\): lateral forces at rear-left/right wheels  

#### Tire Slip Angles
- **Front wheels**:  
\[
\alpha_{f} = \delta - \arctan\left(\frac{v + a r}{u}\right)
\]
- **Rear wheels**:  
\[
\alpha_{r} = -\arctan\left(\frac{v - b r}{u}\right)
\]

#### Lateral Tire Forces (Linear Model)
\[
F_{y} = -C_{\alpha} \alpha
\]

where:  
- \(C_{\alpha}\): cornering stiffness  
- \(\alpha\): slip angle

#### Global Position Update
\[
\dot{X} = u \cos{\psi} - v \sin{\psi}
\]
\[
\dot{Y} = u \sin{\psi} + v \cos{\psi}
\]
\[
\dot{\psi} = r
\]

---

###  Simulation Steps
At each time step:
1️ Compute slip angles.  
2️ Calculate lateral forces from slip angles.  
3️ Update accelerations (\(\dot{u}, \dot{v}, \dot{r}\)).  
4️ Integrate to get new velocities and yaw rate.  
5️ Update global position and orientation (\(X, Y, \psi\)).

---

### ⚠️ Assumptions & Limits
- Valid for **low to moderate speeds** (linear tire behavior).  
- Does not account for load transfer, roll, or pitch.  
- No advanced tire models (like Pacejka) in the basic version.

---


### Sequential Convex Programming (Trajectory Planning)

We use Sequential Convex Programming (SCP) to compute the most fuel-efficient trajectory for the RC car within a corridor.  
SCP iteratively solves convexified subproblems around a trust region:

- **Convexified subproblems:**  
  We linearize the nonlinear vehicle dynamics around the current trajectory estimate.  
- **Trust region:**  
  A penalty term constrains how far the new trajectory can move from the previous one to ensure convergence.
- **Optimization goal:**  
  Minimize the total motor input (fuel usage) while ensuring the following constraints:
  - Motor and steering maximum/minimum values.
  - Vehicle speed limits ($v_{\min}$, $v_{\max}$).
  - Start and end position constraints.
  - Convex half-space (linear) constraints representing the corridor walls.
  - Maximum allowable centripetal acceleration (limits turning speed).

This formulation provides a computationally efficient and robust way to generate safe, dynamically feasible trajectories for the RC car.

## Controlling the Vehicle

To control the autonomous RC car along the precomputed trajectory, we plan to use either a **PID controller** or an **LQR (Linear Quadratic Regulator)**.

**PID Controller** – A straightforward approach that minimizes error based on proportional, integral, and derivative gains.  
**LQR Controller** – An optimal control approach that minimizes a quadratic cost function of state and control effort, providing a balance between tracking accuracy and control input magnitude.

For the LQR approach, we linearize the vehicle model around the current reference trajectory and compute time-varying feedback gains. This enables more robust tracking, especially in scenarios with high curvature turns or fast vehicle dynamics.

The chosen controller generates steering and motor commands (PWM) to track the precomputed path in real-time.  
This control algorithm will be implemented as a ROS 2 node, subscribing to localization data from the ZED 2i SLAM module and publishing PWM commands to the Adafruit PWM driver for steering and motor actuation.


### Results from Sequential Convex Programming (Trajectory Planning)



![RC Car Trajectory](TrajectoryofRCCAR.png)

*Figure: The planned trajectory of the RC car within the corridor. The corridor walls are visible, and the red dots represent trajectory nodes at each discrete time step.*


![RC Car Speed vs Time](CarSpeedVSTime.png)

*Figure: The speed profile of the RC car, showing acceleration to terminal velocity, cruising, and deceleration phases, along with speed dips during tight turns.*


![RC Car Steering Angle vs Time](SteeringanglevsTime.png)

*Figure: The steering angle vs. time plot shows how the car should steer in an ideal scenario. However, in the real world, actual steering inputs may differ due to disturbances and actuator limits.*


### Building an Autonomous RC Car

To build an autonomous RC car, you’ll need the following hardware components:

 **Jetson computer** (e.g., Jetson Orin AGX) – for real-time processing and control.  
 **ZED 2i camera** – for SLAM-based localization, enabling the car to understand its position in the environment.  
 **DC-DC converter** – to safely power the Jetson from a 4S LiPo battery.  
 **Adafruit 16-Channel 12-bit PWM/Servo Driver (I2C interface)** – generates precise PWM signals from the Jetson, which control the servos and brushless motor.  
 **I2C interface** – a communication protocol that allows multiple devices (sensors, PWM driver, etc.) to exchange data using just two wires (SDA and SCL).  
 **RC car hardware** – a chassis with a brushless motor, electronic speed controller (ESC), steering servo, and a battery to power the drivetrain.

This hardware setup provides a powerful and flexible platform to track and execute the precomputed trajectory, combining SLAM-based localization, real-time control, and hardware interfacing through I2C.


