# Vehicle and Obstacle Position Estimation using Extended Kalman Filters and Particle Filters

The project, presents three cases to solve the problem of the estimation of vehicle and obstacles position. In the first case, I used **Extended Kalman Filters** considering that the obstacles were stable and I tried to predict the position of the vehicle and the position of the obstacles. In the second case, I used **Particle Filters** to estimate only the position of the vehicle. The obstacles were motionless and as coordinates I used the values that Extended Kalman Filter had predicted in the first case. Last, in the third case, I assumed that the second obstacle was moving and I tried to predict the position of the vehicle and the position of the second obstacle.

# Description of the repository

The repository consists of the following 3 folders.

- dataset: 
    - 4 csv files, 2 files with measurements from vehicle and 2 files with measurements from radar
    - control1.csv and control2.csv consists of 100 samples. The first column is the speed velocity and the second column is the angular velocity
    - radar1.csv and radar2.csv consists 100 samples. There are 4 columns for distance and angle of each obstacle. 
- src: 3 files, one for each case study
    - ExtendedKalmanFilter.py, the first case study
    - ParticleFilter2.py, the second case study
    - ParticleFilter3.py, the third case study
- images: 
    - math formulas
    - results

## Extended Kalman Filter

The Kalman filter is an algorithm for calculating the optimal state of a linear system, which is subject to noises. These noises make the conclusions we draw unreliable. The Extended Kalman Filter is used for non-linear problems by transforming them into linear ones. Linearizes the functions f (x, u) and h (x) by taking the partial derivatives of each at the points x and u. This is called Jacobian matrix. Below are the equations of the motion model, the measurement model and the corresponding Jacobian ones. Obstacles are considered stable. There are two obstacles.

**Motion Model**
![Equations of Motion Model](https://github.com/AngelikiTsintzira/Vehicle-and-Obstacle-Position-Estimation-using-Extended-Kalman-and-Particle-Filters/blob/master/images/motionModelEquations.png?raw=true)

**Measurement Model**
![Equations of Measurement Model](https://github.com/AngelikiTsintzira/Vehicle-and-Obstacle-Position-Estimation-using-Extended-Kalman-and-Particle-Filters/blob/master/images/measurementModelEquations.png?raw=true)

To implement the filter I followed the formulas given on Wikipedia. After loading the csv files, we need to convert the radius to the range [-π, π].
Then, the tables and the filter are initialized and an iteration structure is started for each measurement. For each measurement the predict and update step is implemented. At the end, we have the final estimate of the 7 situations and the diagram showing the evolution of prediction and uncertainty. 

In the predict step, I try to estimate the 7 situations according to the knowledge I have, ie the equations that describe the motion model, the noise, the uncertainty and the speeds. First, the next state is estimated based on the motion model and then the estimate is adjusted according to the uncertainty. In the update step, I use the measurement model to correct the estimates I made in the previous step. The measurement is taken and its accuracy is checked, the distance of the measurement value from the prediction value is calculated, the Kalman Gain is modified depending on whether the measurement or prediction is more accurate and finally the state is updated based on the measurement uncertainty.

Noise tables are very important to be properly defined for the problem as they have a great impact on the accuracy of estimates and the overall performance of the model. Kalman filters combine noisy and limited knowledge of how a system behaves with noisy and limited sensor measurements to produce the best possible assessment of system condition. The principle that follows is not to reject information. However, due to the noise of the measurements, a percentage of information is lost in each prediction.

Below, is the result of an execution of EKF where all the above are perceived. The assessment of the position - movement of the vehicle follows a smooth course. Initially, we observe that the uncertainty is almost zero while as we go into the long run it grows. The symbol(+) is the position and the ellipse indicates the uncertainty area. This is expected because in every estimation we have the effect / addition of noise and as we move forward in time, with each prediction we lose a percentage of our original information. The uncertainty of estimating the position of the obstacles follows the opposite course from that of the vehicle, ie the uncertainty is constantly decreasing. This conclusion is to be expected because the obstacles are stable so in any assessment the system is able to improve its belief / knowledge about prediction.

**Results of Extended Kalman Filter**
![Results of Extended Kalman Filter](https://github.com/AngelikiTsintzira/Vehicle-and-Obstacle-Position-Estimation-using-Extended-Kalman-and-Particle-Filters/blob/master/images/ExtendedKalmanFilterResults.png?raw=true)

## Particle Filter 2

The idea of ​​the Particle Filter is based on Monte Carlo methods, which use sets of particles that represent probabilities (possible states of the system - space exploration). A weight is calculated for each probability and as we move forward in time the model converges to a specific situation. The main advantage of the Particle Filter is the ability to handle any non-linearity and any noise distribution. However, it has a high computational complexity as the variables and complexity of the problem increase. 

The algorithm is based on 5 steps. 
1. The first step is to randomly generate a set of particles. The particles consist of 3 values ​​(x, y, θ) which are the states to be estimated. Each has a weight (probability) that indicates how likely it is to match the actual state of the system. In initialization, everything has the same weight (1 / N). The uniform and normal distribution (Gauss) was tested and the normal one was chosen as it had better results because the particles were initialized close to the initial state (0.0.0) and not uniform in space. 
2. The second step is to predict the next state of the particles. This way should reflect the behavior of the real system. Therefore, I use the given state equations (in the previous position of the vehicle) by introducing noise and multiplying by the displacement (dt = 0.1). The measurements are noisy so this must be taken into account during the predict step. 
3. The third step is update. The update is based on radar measurements. Particles that match the measurements score higher than particles that do not match as well. This step also included measuring distance and angle to make a more accurate prediction. 
4. The fourth step is sampling. A common problem is that very few particles contribute to the estimation process and the rest have very small weights. In this case the sampling is performed. The sampling algorithm rejects very low probability particles and replaces them with new particles with a higher probability. This is done by reproducing particles with a relatively high probability. The copies are slightly scattered by the noise added to the prediction step. This results in a set of points at which a large majority of the particles accurately represent the probability distribution. The threshold selected for sampling is N * 0.7 (N: number of particles).
5. The fifth and final step is to assess the final situation. But because we have many states, the weighted average and the variability of the set of particles is calculated.

The figure below, presents the final solutions. On the left we can see the final solution. On the right, a zoom of particles distribution. We notice that in the beginning the particles spread in space and slowly converge with the repetitions / corrections. In the end, the particles converge in the final solution.

**Results of Particle Filter 2**
![Results of Particle Filter 2](https://github.com/AngelikiTsintzira/Vehicle-and-Obstacle-Position-Estimation-using-Extended-Kalman-and-Particle-Filters/blob/master/images/ParticleFilters2Results.png?raw=true)

## Particle Filter 3

The 3rd case is similar to the 2nd with the difference that the 2nd obstacle moves at a constant and unknown speed by x. This means that 2 new states are added to the equation, the x position of the obstacle and its velocity. The equations of the motion model are 5 (x, y, θ, xobst, uobst) and are shown below.

**Particle Filter 3 Equations**
![Results of Particle Filter 2](https://github.com/AngelikiTsintzira/Vehicle-and-Obstacle-Position-Estimation-using-Extended-Kalman-and-Particle-Filters/blob/master/images/particleFilter3Equations.png?raw=true)

To model the x-position of the obstacle I used the above functions. From the velocity equation it follows that the displacement is x = x + u * dt. This measurement has the noise added to the speed. The other options (N, Q, R, Resample method) remained the same as in case 2. Finally, for the selection of the initial speed and movement of the obstacle, the values ​​from the radar and control files were observed. Initially, the vehicle speed sign confirms that the vehicle is moving correctly (positive speed right and negative left). Then, taking into account the above with the distance of the obstacle from the vehicle, it results that the obstacle moves to the right (as when the vehicle goes to the left the distance decreases and the speed is constant). The initial value I gave to the obstacle is 
0.6 which results from observing the values ​​of distances and angles.

In figure below, on the left we see the vehicle following the same movement as the previous 2 cases and the obstacle moving to the right. To the right are the magnified particles, 6 repetitions up and 1 down showing their convergence which took place in the first repetitions. 

**Results of Particle Filter 3**
![Results of Particle Filter 3](https://github.com/AngelikiTsintzira/Vehicle-and-Obstacle-Position-Estimation-using-Extended-Kalman-and-Particle-Filters/blob/master/images/ParticleFilters3Results1.png?raw=true)

 In figure below we see in magnification the route of the vehicle. At the beginning there are many values ​​added and then they dilute. This is because the obstacle at first went right to left until it stabilized to the right.

**Results of Particle Filter 3**
![Results of Particle Filter 3](https://github.com/AngelikiTsintzira/Vehicle-and-Obstacle-Position-Estimation-using-Extended-Kalman-and-Particle-Filters/blob/master/images/ParticleFilters3Results2.png?raw=true)


