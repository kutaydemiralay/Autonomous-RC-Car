

# Autonomous RC Car

In this project, I used a 4-wheel car model to simulate an RC car navigating through a corridor, using Sequential Convex Programming to find the most fuel-efficient path offline.  
I then built an RC car with a Jetson Orin AGX and a ZED2i camera. The ZED2i camera has built-in localization SLAM algorithms that, with visual sensing, can accurately estimate the car's position. 
Combined with the Jetson computer and ROS 2, the Jetson can send signals to an Adafruit controller, which then drives the servos and motor of the car using PWM to track the offline-calculated trajectory.


\section*{4-Wheel Car Model (Kinematic Bicycle Approximation)}

The 4-wheel car model captures how a vehicle’s four wheels contribute to its motion. To simplify the complexity, it is often represented as a kinematic bicycle model, where the left and right wheels on each axle are combined into a single effective wheel:

\begin{itemize}
  \item \textbf{Front axle:} Responsible for steering.
  \item \textbf{Rear axle:} Primarily provides traction.
\end{itemize}

\noindent
The vehicle’s motion can be described by:

\[
\begin{aligned}
\dot{x} &= v \cos(\psi) \\
\dot{y} &= v \sin(\psi) \\
\dot{\psi} &= \frac{v}{L} \tan(\delta) \\
\dot{v} &= a
\end{aligned}
\]

\noindent
where:
\begin{itemize}
  \item $x, y$ are the vehicle’s position coordinates.
  \item $\psi$ is the yaw angle (heading).
  \item $v$ is the longitudinal velocity.
  \item $\delta$ is the front wheel steering angle.
  \item $L$ is the wheelbase (distance between front and rear axles).
  \item $a$ is the longitudinal acceleration.
\end{itemize}

This model captures essential lateral and longitudinal dynamics while remaining computationally efficient for real-time trajectory planning and control.
