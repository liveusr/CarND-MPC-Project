# **Model Predictive Control**

The goal of this project is to implement Model Predictive Control to drive the car around the track in simulation. The simulator provides reference trajectory points and vehicle state in order to compute the appropriate steering angle and throttle.

## Implementation

Model Predictive Control involves simulating different actuator inputs, predicting the resulting trajectory and selecting the trajectory with a minimum cost. Following sections explain selected model and cost function.

### The Model

The Model used for this project is simple Kinematic Vehicle Model. This model is less complex and runs faster as compared to Dynamic Model. This model doesn't consider other forces, interaction of tires with road, etc.

The vehicle state is defined by following variables:
*x*, *y* = position of vehicle
*ψ* = orientation of vehicle
*v* = velocity of vehicle
*cte* = cross track error
*eψ* = orientation error

Actuator outputs are:
*δ* = steering angle
*a* = acceleration

State update equations for the model are defined as follow:
*x<sub>t+1</sub> = x<sub>t</sub> + v<sub>t</sub>cos(ψ<sub>t</sub>) * dt*
*y<sub>t+1</sub> = y<sub>t</sub> + v<sub>t</sub>sin(ψ<sub>t</sub>) * dt*
*ψ<sub>t+1</sub> = ψ<sub>t</sub> + (v<sub>t</sub>/L<sub>f</sub>)δ<sub>t</sub> * dt*
*v<sub>t+1</sub> = v<sub>t</sub> + a<sub>t</sub> * dt*
*cte<sub>t+1</sub> = f(x<sub>t</sub>) - y<sub>t</sub> + v<sub>t</sub>sin(eψ<sub>t</sub>) * dt*
*eψ<sub>t+1</sub> = ψ<sub>t</sub> - ψdes<sub>t</sub> + (v<sub>t</sub>/L<sub>f</sub>)δ<sub>t</sub> * dt*

where, 
*ψdes = arctan(f'(x))*
*f(x) = a<sub>0</sub> + a<sub>1</sub> * x*
*f'(x) = a<sub>1</sub>*

### Timestep Length and Elapsed Duration (N & dt)

The prediction horizon is the duration over which future predictions are made. *N* is number of timesteps in the horizon and *dt* is how much time elapses between actuations. With high speeds, prediction horizon should only be a few seconds. Because beyond that, environment will change a lot and predictions will be incorrect. *N* determines number of variables optimized by MPC. So, higher value of *N* involves higher computational cost. Similarly, large value of *dt* results in less frequent actuations making it harder to approximate continuous reference trajectory. After multiple trials *N* is set to 10 with *dt* equal to 0.1 giving duration of trajectory *T* as 1 second.

### Polynomial Fitting and MPC Preprocessing

First, waypoints provided by simulator are transformed to car coordinate system. Reference trajectory is passed to control block as a polynomial. A 3rd order polynomial is fitted to these transformed waypoints. Usually 3rd order polynomials fit trajectories for most roads.

### Model Predictive Control with Latency

In a real car, actuation command won't execute instantly, there will be a delay as the command propagates through the system. For this project, delay - also called as Latency - is assumed as *100ms*. Kinematic equations calculate actuations based on previous actuations. In my implementation, *dt* is 0.1 seconds which is equal to Latency. So, to compensate for Latency, I'm using actuations values at timestep *t-2* instead of timestep *t-1*. Another way to handle this could be predicting its next state based on current state and latency. But this approach did not work well, so I used one timestep older actuation values as explained.   

## Results

Following GIF shows snippet from actual test run in the simulator:

![alt text](writeup_data/output_video.gif "output_video")

Complete output video can be found here: [output_video.mp4](output_video.mp4)
