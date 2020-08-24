# Vehicle and Obstacle Position Estimation using Extended Kalman and Particle Filters

The project, presents three cases to solve the problem of the estimation of vehicle and obstacles position. In the first case, I used Extended Kalman Filters considering that the obstacles were motionless. In the second case, I used Particle Filters. I tried to estimate the position of the vehicle. The obstacles were motionless and as coordinates I used the values that Extended Kalman Filter had predicted in the first case. Last, in the third case, I assumed that the second obstacle was moving and i tried to predict the position of the vehicle and the position of the obstacle.

# Description of the repository

The repository consists of the following 3 folders.

- dataset: 
    - 4 csv files, 2 files with measurements from vehicle and 2 files with measurements from radar
    - control1.csv and control2.csv consists of 100 samples. The first column is the veloxity an the second column is the angular velocity
    - radar1.csv and radar2.csv consists 100 samples. There are 4 columns for distance and angle of each obstacle. 
- src: 3 files, one for each case study
    - ExtendedKalmanFilter.py, the first case study
    - ParticleFilter2.py, the second case study
    - ParticleFilter3.py, the third case study
- images: 
    - math formulas
    - results

## Extended Kalman Filter

The Kalman filter is an algorithm for calculating the optimal state of a linear system, which is subject to noises. That noises make the conclusions we draw unreliable. The Extended Kalman Filter is used for non-linear problems by transforming them into linear ones. Linearizes the functions f (x, u) and h (x) by taking the partial derivatives of each at the points x and u. This is called Jacobian matrix. Below are the equations of the motion model, the measurement model and the corresponding Jacobian ones. Obstacles are considered stable. There are two obstacles.

**Motion Model**
![Equations of Motion Model](https://github.com/AngelikiTsintzira/Vehicle-and-Obstacle-Position-Estimation-using-Extended-Kalman-and-Particle-Filters/blob/master/images/motionModelEquations.png?raw=true)

**Measurement Model**
![Equations of Measurement Model](https://github.com/AngelikiTsintzira/Vehicle-and-Obstacle-Position-Estimation-using-Extended-Kalman-and-Particle-Filters/blob/master/images/measurementModelEquations.png?raw=true)

To implement the filter I followed the formulas given on Wikipedia. After loading the csv files, we need to convert the radius to the range [-π, π].
Then, the tables and the filter are initialized and an iteration structure is started for each measurement. For each measurement the predict and update step is implemented. At the end we have the final estimate of the 7 situations and the diagram showing the evolution of prediction and uncertainty. 

In the predict step, I try to estimate the 7 situations according to the knowledge I have, ie the equations that describe the motion model, the noise, the uncertainty and the speeds. First, the next state is estimated based on the motion model and then the estimate is adjusted according to the uncertainty. In the update step, I use the measurement model to correct the estimates I made in the previous step. The measurement is taken and its accuracy is checked, the distance of the measurement value from the prediction value is calculated, the Kalman Gain is modified depending on whether the measurement or prediction is more accurate and finally the state is updated based on the measurement uncertainty.

Noise tables are very important to be properly defined for the problem as they have a great impact on the accuracy of estimates and the overall performance of the model. Kalman filters combine noisy and limited knowledge of how a system behaves with noisy and limited sensor measurements to produce the best possible assessment of system condition. The principle that follows is not to reject information. However, due to the noise of the measurements, a percentage of information is lost in each prediction.

Below, is the result of an execution of EKF where all the above are perceived. The assessment of the position - movement of the vehicle follows a smooth course. Initially, we observe that the uncertainty is almost zero while as we go into the long run it grows. (+) Is the position and the lack indicates the uncertainty. This is expected because in every estimate we have the effect / addition of noise and as we move forward in time, with each prediction we lose a percentage of our original information. The uncertainty of estimating the position of the obstacles follows the opposite course from that of the vehicle, ie the uncertainty is constantly decreasing. This conclusion is to be expected because the obstacles are constant so in any assessment the system is able to improve its belief / knowledge about prediction.

![Results of Extended Kalman Filter](https://github.com/AngelikiTsintzira/Vehicle-and-Obstacle-Position-Estimation-using-Extended-Kalman-and-Particle-Filters/blob/master/images/ExtendedKalmanFilterResults.png?raw=true)

## Particle Filter 2

## Particle Filter 3






