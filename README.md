### Vehicle and Obstacle Position Estimation using Extended Kalman and Particle Filters

## Extended Kalman Filter

***The datasets of this project are ommited. We need two csv datasets, one with the sensor measurements and the other with the radar measurements***
***control.csv consists of 2 columns for the velocity and angular velocity***
***radar.csv consists of 4 columns for distance and angle of each obstacle***

The Kalman filter is an algorithm for calculating the optimal state of a linear system, which is subject to noises. That noises make the conclusions we draw unreliable. The Extended Kalman Filter is used for non-linear problems by transforming them into linear ones. Linearizes the functions f (x, u) and h (x) by taking the partial derivatives of each at the points x and u. This is called Jacobian matrix. Below are the equations of the motion model, the measurement model and the corresponding Jacobian ones. Obstacles are considered stable. There are two obstacles.

![Equations of Motion Model](https://github.com/AngelikiTsintzira/Vehicle-and-Obstacle-Position-Estimation-using-Extended-Kalman-and-Particle-Filters/blob/master/images/motionModelEquations.png?raw=true)

![Equations of Measurement Model](https://github.com/AngelikiTsintzira/Vehicle-and-Obstacle-Position-Estimation-using-Extended-Kalman-and-Particle-Filters/blob/master/images/measurementModelEquations.png?raw=true)

To implement the filter I followed the formulas given on Wikipedia. After loading the csv files, we need to convert the radius to the range [-π, π].
Then, the tables and the filter are initialized and an iteration structure is started for each measurement. For each measurement the predict and update step is implemented. At the end we have the final estimate of the 7 situations and the diagram showing the evolution of prediction and uncertainty. 

In the predict step, I try to estimate the 7 situations according to the knowledge I have, ie the equations that describe the motion model, the noise, the uncertainty and the speeds. First, the next state is estimated based on the motion model and then the estimate is adjusted according to the uncertainty. In the update step, I use the measurement model to correct the estimates I made in the previous step. The measurement is taken and its accuracy is checked, the distance of the measurement value from the prediction value is calculated, the Kalman Gain is modified depending on whether the measurement or prediction is more accurate and finally the state is updated based on the measurement uncertainty.

Noise tables are very important to be properly defined for the problem as they have a great impact on the accuracy of estimates and the overall performance of the model. Kalman filters combine noisy and limited knowledge of how a system behaves with noisy and limited sensor measurements to produce the best possible assessment of system condition. The principle that follows is not to reject information. However, due to the noise of the measurements, a percentage of information is lost in each prediction.

*****To be continued*






