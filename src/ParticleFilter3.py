#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Import libraries
import numpy as np
import pandas as pd
import scipy.stats
import statistics
import random 
import matplotlib.pyplot as plt
from numpy.linalg import norm
from numpy.random import randn
from numpy.random import uniform
from numpy.random import rand
from scipy.stats import multivariate_normal
from scipy.stats import norm
from filterpy.monte_carlo import systematic_resample

def create_uniform_particles(x_range, y_range, theta_range, N, velocity_range):
    particles = np.empty((N, 5))
    particles[:, 0] = uniform(x_range[0], x_range[1], size = N)
    particles[:, 1] = uniform(y_range[0], y_range[1], size = N)
    particles[:, 2] = uniform(theta_range[0], theta_range[1], size = N)
    for i in range(0, N):
        particles[i, 2] = wrapToPi(particles[i, 2])

    particles[:, 3] = uniform(x_range[0], x_range[1], size = N)
    particles[:, 4] = uniform(velocity_range[0], velocity_range[1], size = N)
  
    return particles

def create_gaussian_particles(mean, std, N):
    particles = np.empty((N, 5))
    particles[:, 0] = mean[0] + (randn(N) * std[0])
    particles[:, 1] = mean[1] + (randn(N) * std[1])
    particles[:, 2] = mean[2] + (randn(N) * std[2])
    for i in range(0, N):
        particles[i, 2] = wrapToPi(particles[i, 2])

    particles[:, 3] = mean[3] + (randn(N) * std[3])
    particles[:, 4] = mean[4] + (randn(N) * std[4])

    return particles

# Wrap radians to [−pi pi]
def wrapToPi(d):
    # Wrap to [0..2*pi]
    d = d % (2 * np.pi)
    # Wrap to [-pi..pi]
    if d > np.pi:             
        d -= 2 * np.pi   
    return d

# Wrap radians to [−pi pi]
def wrapToPiN(d, N):
    # Wrap to [0..2*pi]
    for i in range(0, N):
        d[i] = d[i] % (2 * np.pi)
        if d[i] > np.pi:             
            d[i] -= 2 * np.pi  
    return d

def neff(weights):
    return 1. / np.sum(np.square(weights))

class ParticleFilter:

    def __init__(self, N, landmarks, Q, R,  state, dt):
        # N : Number of particles
        # landmarks : Landmarks positions
        # particles : Particles
        # weights : Weights 
        # Q : Noise std
        # R : Noise
        
        self.dt = dt
        self.N = N 
        self.landmarks = landmarks
        self.state = state
        self.weights = np.ones(N)/N
        self.Q = Q
        self.R = R 
        # Randomly generate N particles
        self.particles = create_gaussian_particles(self.state, np.array([0.1, 0.1, 0.1, 0.1, 0.1]), N)
        for i in range(0, N):
            self.particles[i, 2] = wrapToPi(self.particles[i, 2])

    def predict(self, u, dt):

        N = self.N
        std = self.Q

        # Move in the (noisy) commanded direction
        self.particles[:,0] = self.particles[:, 0] + np.cos(self.particles[:, 2]) * (u[0]+ (randn(N) * std[0])) * dt 
        self.particles[:,1] = self.particles[:, 1] + np.sin(self.particles[:, 2]) * (u[0]+ (randn(N) * std[1])) * dt 
        self.particles[:,2] = self.particles[:, 2] + dt * (u[1] + randn(N) * std[2])
        for i in range(0, N):
            self.particles[i, 2] = wrapToPi(self.particles[i, 2])

        # x= x + u*Dt
        self.particles[:,3] = self.particles[:,3] + (self.particles[:,4]) * dt
        #self.particles[:,3] = self.particles[:,3] + (self.particles[:,4]+ (randn(N)*std[3]))*dt
        self.particles[:,4] = self.particles[:,4] + random.gauss(0.0, std[4])
  
    
    def update(self, z):
        for i in range(0,self.landmarks.shape[0]) :

            landmark= self.landmarks[i]
            
            distance = np.linalg.norm(self.particles[:, 0:2] - landmark, axis = 1)
            angle = np.array(wrapToPiN(np.arctan2( self.particles[:,1] - landmark[1],  self.particles[:,0] - landmark[0]) - self.particles[:,2], self.N)) 
            distance = np.array((np.array([distance, angle])).T)

            self.weights *= [normpdf(distance[j], self.R, z[0,i]) for j in range(0, np.size(distance,0))]

        self.weights += 1.e-300      # avoid round-off to zero
        self.weights /= sum(self.weights) # Normalize to sum to 1
        
    def estimate(self):
        robotPosition = self.particles[:, 0:5]
        mean = np.average(robotPosition, weights = self.weights, axis = 0)
        var  = np.average((robotPosition - mean)**2, weights = self.weights, axis = 0)
        return mean, var

    def multinomial_resample(self):
        N = self.N
        cumulative_sum = np.cumsum(self.weights)
        cumulative_sum[-1] = 1. # avoid round-off error
        indexes = np.searchsorted(cumulative_sum, rand(len(self.weights)))

        # resample according to indexes
        self.particles[:] = self.particles[indexes]
        self.weights[:] = self.weights[indexes]
        self.weights.fill(1.0 / N)

    def resample_from_index(self, indexes):
        self.particles[:] = self.particles[indexes]
        self.weights[:] = self.weights[indexes]
        self.weights.fill(1.0 / len(self.weights))

    def residual_resample(self):
        N = self.N
        indexes = np.zeros(self.N, 'i')

        # take int(N*w) copies of each weight
        num_copies = (N*np.asarray(self.weights)).astype(int)
        k = 0
        for i in range(N):
            for _ in range(num_copies[i]): # make n copies
                indexes[k] = i
                k += 1

        # use multinormial resample on the residual to fill up the rest.
        residual = self.weights - num_copies     # get fractional part
        residual /= sum(residual)     # normalize
        cumulative_sum = np.cumsum(residual)
        cumulative_sum[-1] = 1. # ensures sum is exactly one
        indexes[k:N] = np.searchsorted(cumulative_sum, rand(N-k))

        self.particles[:] = self.particles[indexes]
        self.weights[:] = self.weights[indexes]
        self.weights.fill(1.0 / len(self.weights))

    def stratified_resample(self):
        N = self.N
        # make N subdivisions, chose a random position within each one
        positions = (rand(N) + range(N)) / N

        indexes = np.zeros(N, 'i')
        cumulative_sum = np.cumsum(self.weights)
        i, j = 0, 0
        while i < N:
            if positions[i] < cumulative_sum[j]:
                indexes[i] = j
                i += 1
            else:
                j += 1

        self.particles[:] = self.particles[indexes]
        self.weights[:] = self.weights[indexes]
        self.weights.fill(1.0 / len(self.weights))

    def systematic_resample(self):
        N = self.N

        # make N subdivisions, choose positions 
        # with a consistent random offset
        positions = (np.arange(N) + rand()) / N

        indexes = np.zeros(N, 'i')
        cumulative_sum = np.cumsum(self.weights)
        i, j = 0, 0
        while i < N:
            if positions[i] < cumulative_sum[j]:
                indexes[i] = j
                i += 1
            else:
                j += 1

        self.particles[:] = self.particles[indexes]
        self.weights[:] = self.weights[indexes]
        self.weights.fill(1.0 / len(self.weights))

def normpdf(x, mu, std):

    cov = np.diag((1,1))*std
    part1 = 1 / ( ((2* np.pi)**(len(mu)/2)) * (np.linalg.det(cov)**(1/2)) )
    part2 = (-1/2) * ((x-mu).T.dot(np.linalg.inv(cov))).dot((x-mu))
    return float(part1 * np.exp(part2))

def mainFunction():
    # Read Datasets 
    control2 = np.array(pd.read_csv("control2.csv", header = None, names=['u', 'theta']))
    radar2 = np.array(pd.read_csv("radar2.csv", header = None, names=['d1', 'f1', 'd2', 'f2']))
    
    # Convert radians to [-pi, pi]
    for i in range(0,100):
        radar2[i][1] = wrapToPi(radar2[i][1])
        radar2[i][3] = wrapToPi(radar2[i][3])

    # Best positions of Extended Kalman Filter
    landmarks = np.array([[ 4.30069035, 3.55923413], [-2.52430552, 3.69715365]])
    # First position of state
    state = np.array([0, 0, 0, landmarks[1,0], 0.6])
    print(state)
    # Number of particles
    N = 100
    dt = 0.1
    # Noise matricies
    Q = np.array([0.001, 0.001, 0.001, 0.001, 0.001])
    R = np.array([0.5, 0.3])

    plt.figure()
    pf = ParticleFilter (N = N, landmarks = landmarks, Q = Q, R = R, state = state, dt = dt)

    resampling = 0
    resampleIndex = []
    
    for i in range(0,100):
        ut = control2[i,:] 

        z = np.matrix([radar2[i,0],radar2[i,1], radar2[i,2], radar2[i,3]])

        pf.predict(ut,dt)
        pf.update(z)

        if neff(pf.weights) < N * 0.7 :
            # Multinomial resample
            #pf.multinomial_resample()
            # Resample from index
            #indexes = systematic_resample(pf.weights)
            #pf.resample_from_index(indexes)
            # Residual Resampling
            pf.residual_resample()
            # Stratified Resampling
            #pf.stratified_resample()
            # Systematic Resampling
            #pf.systematic_resample()
            resampling = resampling + 1
            resampleIndex.append(i)
            
        mean, var = pf.estimate()

        y_val = [landmarks[1,1] for i in range(0,N) ]

        plt.scatter(landmarks[0,0], landmarks[0,1], c = 'red') 
        p1 = plt.scatter(pf.particles[:, 0], pf.particles[:, 1], marker = '+', color = 'k', s = 4)
        plt.scatter(pf.particles[:, 3], y_val, marker = 'o', color = 'b', s = 3)
        p2 = plt.scatter(mean[0], mean[1], marker = 's', color = 'g', s = 4)

    plt.legend([p1, p2], ['Particles', 'Estimation'], loc=4, numpoints=1)

    print('Final position error, variance:\n\t', mean, var)
    print("Resampling: ", resampling)
    print("Indexes of resampling: ", resampleIndex)
    print("Velocity: ", pf.particles[N-1,:])
    plt.show()

np.random.seed(2020)
mainFunction()