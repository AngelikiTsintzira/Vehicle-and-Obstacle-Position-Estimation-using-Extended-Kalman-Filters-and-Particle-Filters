#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Import libraries
import numpy as np
import os
import sys
import pandas as pd
import matplotlib.pyplot as plt
from sympy import Symbol,nsolve,sqrt,atan2
from filterpy.stats import plot_covariance
#from sympy.abc import alpha, x, y, v, w, R, theta

# Clear console screen
def cls():
    os.system('cls' if os.name=='nt' else 'clear')
cls()

class ExtendedKalmanFilter(object):
        
    def __init__(self, F = None, H = None, Q = None, R = None, P = None, x0 = None, dt = None):
        # x : State estimate vector
        # P : Uncertainty Covariance
        # R : Measurement noise matrix - State Uncertainty
        # Q : Process noise matrix - Process Uncertainty
        # F : State Transition matrix - Jacobian
        # H : Measurement function - Jacobian
        # K : Kalman gain of the update step
   
        self.n = F.shape[1]
        self.m = H.shape[1]

        self.F = F
        self.H = H

        self.P = P
        self.Q = Q
        self.R = R

        self.xt = x0 
        self.dt = dt


    def predict(self, ut, wt):

        self.xt = self.xt + motionEquation(self.xt, ut, self.dt, wt)
        self.xt[2,0] = wrapToPi(self.xt[2,0])
        print("State after predict: \n", self.xt)

        self.F = jacobian_F_Matrix(self.dt, ut, self.xt[2,0])
        print("F after predict: \n", self.F)
        '''
            As we don’t know exactly how the vehicle has behaved in this last time step, 
            it can accelerate or change the direction of movement, our uncertainty increases:
        '''       
        self.P = self.F*self.P*self.F.T + self.Q    
        print("P after predict: \n", self.P)
        return self.P, self.xt

    def update(self, z):

        # Calculate d and phi and subtract them from radar measuremets
        d1, phi1 = calculateDF(self.xt[0,0], self.xt[1,0], self.xt[3,0], self.xt[4,0], self.xt[2,0])
        d2, phi2 = calculateDF(self.xt[0,0], self.xt[1,0], self.xt[5,0], self.xt[6,0], self.xt[2,0])
        hx = np.matrix([[d1], [phi1], [d2], [phi2]])
        y = (z.T - hx).astype(np.float)
        y[1,0] = wrapToPi(y[1,0])
        y[3,0] = wrapToPi(y[3,0])
        print("Difference Radar from Maths: ", y)

        self.H = jacobian_H_Matrix([self.xt[0,0], self.xt[1,0]], [self.xt[3,0], self.xt[4,0]], [self.xt[5,0], self.xt[6,0]]    )

        self.S = self.R + self.H * self.P *self.H.T
        
        self.K = (self.P*self.H.T) * np.linalg.inv(self.S)

        self.xt = self.xt + self.K * y
        self.xt[2,0] = wrapToPi(self.xt[2,0])
        print("State after update: ", self.xt)
        
        I = np.eye(self.n)
        
        self.P = (I - self.K * self.H) * self.P
        return self.P, self.xt
        
def mainFunction():
    # Read Datasets 
    control1 = np.array(pd.read_csv("control1.csv", header = None, names=['u', 'theta']))
    radar1 = np.array(pd.read_csv("radar1.csv", header = None, names=['d1', 'f1', 'd2', 'f2']))
  
    # Convert negative rad to positive rad 
    # Then make sure the values are in [0,2pi]
    #radar1[:,1] = ((radar1[:,1] + 2 * np.pi) % (2 * np.pi))
    #radar1[:,3] = ((radar1[:,3] + 2 * np.pi) % (2 * np.pi))    
    #radar1[:,1] = ((radar1[:,1] + 2 * np.pi) % (2 * np.pi))
    #radar1[:,3] = ((radar1[:,3] + 2 * np.pi) % (2 * np.pi))
    
    # Convert radians to [-pi, pi]
    for i in range(0,100):
        radar1[i][1] = wrapToPi(radar1[i][1])
        radar1[i][3] = wrapToPi(radar1[i][3])

    # Initialization - First State
    dt = 0.1 # Sampling Rate 10Hz
    ut = control1[0,0] # First value of dataset control
    wt = control1[0,1]

    # First measurements of distance and degree of landmarks compared with vehicle
    d1 = radar1[0,0]
    phi1 = radar1[0,1]
    d2 = radar1[0,2]
    phi2 = radar1[0,3]

    # Values I want to find by solving the measurement model
    Xt = Symbol('Xt')
    Yt = Symbol('Yt')
    
    eq1 = sqrt(Xt**2 + Yt**2) - d1
    # θ = 0 for initialization
    eq2 = atan2(Yt,Xt) - phi1
    landmark1 = nsolve((eq1, eq2), (Xt, Yt), (-1, 1))
    landmark1 = np.array(landmark1).astype(float)
    
    eq1 = sqrt(Xt**2 + Yt**2) - d2
    # θ = 0 for initialization
    eq2 = atan2(Yt,Xt) - phi2
    landmark2 = nsolve((eq1, eq2), (Xt, Yt), (-1, 1))
    landmark2 = np.array(landmark2).astype(float)

     # Started State - initialization 
    state = np.matrix([0.0, 0.0, 0.0, landmark1[0,0], landmark1[1,0], landmark2[0,0], landmark2[1,0]]).T
    print(" -------- First State -------- ")
    print(state)
    H = jacobian_H_Matrix([0.0, 0.0], landmark1, landmark2)
    print(" -------- Jacobian H (Landmark) -------- ")
    print(H)
    F = jacobian_F_Matrix(dt, ut, 0)
    print(" -------- Jacobian F (State) -------- ")
    print(F)

    Q = np.matrix([[0.01**2, 0, 0, 0, 0, 0, 0],
                   [0, 0.01**2, 0, 0, 0, 0, 0],
                   [0, 0, 0.01**2, 0, 0, 0, 0],
                   [0, 0, 0, 0.001, 0, 0, 0],
                   [0, 0, 0, 0, 0.001, 0, 0],
                   [0, 0, 0, 0, 0, 0.001, 0],
                   [0, 0, 0, 0, 0, 0, 0.001]])
    
    # Covariance matrix R where var(theta1) = var(theta2) = 0.09 
    # and var(d1) = var(d2) = 0.25
    R = np.matrix([[0.25, 0.0, 0.0,0.0], 
                   [0.0, 0.09, 0.0,0.0], 
                   [0.0, 0.0, 0.25,0.0],
                   [0.0, 0.0, 0.0,0.09]])
    
    P = np.matrix([[0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
                  [0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0], 
                  [0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0],
                  [0.0, 0.0, 0.0, 1, 0.0, 0.0, 0.0],
                  [0.0, 0.0, 0.0, 0.0, 1, 0.0, 0.0],
                  [0.0, 0.0, 0.0, 0.0, 0.0, 1, 0.0],
                  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1]])
    
   
    ek = ExtendedKalmanFilter(F = F, H = H, Q = Q, R = R, P = P, x0 = state, dt = dt)
    temp_x = []
    temp_y = []
    for i in range(1, 100):
        
        ut = control1[i,0] 
        wt = control1[i,1]
        print(" ----------- Iteration: ------------", i)
        z = np.matrix([radar1[i,0],radar1[i,1], radar1[i,2], radar1[i,3]])
        print("Radar Measurements: ", z)
        ek.predict(ut,wt)
        temp_x.append(ek.xt.item(0))
        temp_y.append(ek.xt.item(1))
        ek.update(z)
        
        print("Final Position:")
        print(ek.xt)    
        
        plot_covariance((ek.xt[0,0], ek.xt[1,0]), ek.P[0:2, 0:2],std=1, facecolor='#0057e7', alpha=0.05)
        plot_covariance((ek.xt[3,0], ek.xt[4,0]), ek.P[3:5, 3:5],std=1, facecolor='#ae0001', alpha=0.05)
        plot_covariance((ek.xt[5,0], ek.xt[6,0]), ek.P[5:7, 5:7],std=1, facecolor='#d3a625', alpha=0.05)
        
    

def calculateDF(X, Y, x, y, theta):
    d = np.sqrt((X - x)**2 + (Y - y)**2)
    f = np.arctan2(y - Y, x - X) - theta
    f = wrapToPi(f)
    return d, f   

# Wrap radians to [−pi pi]
def wrapToPi(d):
    # Wrap to [0..2*pi]
    d = d % (2 * np.pi)
    
    # Wrap to [-pi..pi]
    if d > np.pi:             
        d -= 2 * np.pi   
    return d

# This fuction returns the motion Equations
def motionEquation(xt, ut, dt, wt):
    #Xt = xt[0,0]
    #Yt = xt[1,0]
    Tht = xt[2,0]
    x1 = xt[3,0]
    y1 = xt[4,0]
    x2 = xt[5,0]
    y2 = xt[6,0]

    #dt1 = np.sqrt( (Xt - x1)**2 + (Yt -y1)**2 )
    #ft1 = np.arctan2(y1 - Yt, x1 - Xt) - Tht

    #dt2 = np.sqrt( (Xt -x2)**2 + (Yt -y2)**2 )
    #ft2 = np.arctan2(y2 - Yt, x2 - Xt) - Tht

    motionState = np.matrix([[np.cos(Tht)*ut*dt],
                   [np.sin(Tht)*ut*dt],
                   [wt*dt],
                   [0],
                   [0],
                   [0],
                   [0]])
    print("---- Motion Model ----")
    print(motionState)
    return motionState

def jacobian_H_Matrix(vehicle, landmark_pos1, landmark_pos2):
    # Linearize Measurement Model
    # The Maths to calculate H Jacobian Matrix
    '''sympy.init_printing(use_latex="mathjax", fontsize='16pt')
    X1t, X2t, Y1t, Y2t, x, y = symbols('X1t, X2t, Y1t, Y2t, x, y')
    
    z = Matrix([[sympy.sqrt((X1t-x)**2 + (Y1t-y)**2)],
                [sympy.atan2(y-Y1t, x-X1t) - theta],
                [sympy.sqrt((X2t-x)**2 + (Y2t-y)**2)],
                [sympy.atan2(y-Y2t, x-X2t) - theta]])
    z.jacobian(Matrix([x, y, theta, X1t, Y1t, X2t, Y2t]))'''
    x1t = float(landmark_pos1[0])
    y1t = float(landmark_pos1[1])
    x2t = float(landmark_pos2[0])
    y2t = float(landmark_pos2[1])
    X = float(vehicle[0])
    Y = float(vehicle[1])  
    
    hyp1 = float((x1t -X)**2 + (y1t - Y)**2)
    dist1 = float(np.sqrt(hyp1))
    
    hyp2 = float((x2t -X)**2 + (y2t - Y)**2)
    dist2 = float(np.sqrt(hyp2))
    
    nosqr1 = (-x1t + X)**2 + (-y1t + Y)**2
    nosqr2 = (-x2t + X)**2 + (-y2t + Y)**2
    
    arrayJ = np.zeros((4,7))
    arrayJ[1,2] = -1
    arrayJ[3,2] = -1
    
    arrayJ[0,0] = (-x1t + X) / dist1
    arrayJ[0,1] = (-y1t + Y) / dist1
    
    arrayJ[0,3] = (x1t - X) / dist1
    arrayJ[0,4] = (y1t - Y) / dist1
    
    arrayJ[1,0] = (y1t - Y) / nosqr1
    arrayJ[1,1] = (-x1t + X) / nosqr1
    
    arrayJ[1,3] = - (y1t - Y) / nosqr1
    arrayJ[1,4] = - (-x1t + X) / nosqr1

    arrayJ[2,0] = (-x2t + X) / dist2
    arrayJ[2,1] = (-y2t + Y) / dist2
    
    arrayJ[2,5] = (x2t - X) / dist2
    arrayJ[2,6] = (y2t - Y) / dist2 
    
    arrayJ[3,0] = (y2t - Y) / nosqr2
    arrayJ[3,1] = (-x2t + X) / nosqr2
    
    arrayJ[3,5] = - (y2t - Y) / nosqr2
    arrayJ[3,6] = - (-x2t + X) / nosqr2

    H = np.matrix(arrayJ).astype(np.float)
    
    return H
    
def jacobian_F_Matrix(dt, ut, theta):
    # Linearize Motion Model
    # The Maths to calculate F Jacobian Matrix
    
    '''sympy.init_printing(use_latex="mathjax", fontsize='16pt')
    X1t, X2t, Y1t, Y2t, wt, dt, x, y, ut = symbols('X1t, X2t, Y1t, Y2t, wt, dt, x, y, ut')
    
    fxu = Matrix([[x + cos(theta)*ut*dt],
                  [y + sin(theta)*ut*dt],
                  [theta + wt*dt],
                  [X1t],
                  [Y1t],
                  [X2t],
                  [Y2t]])
    
    fxu.jacobian(Matrix([x, y, theta, X1t, Y1t, X2t, Y2t]))'''
    
    F = np.eye(7, dtype=float)
    f1 = dt * ut * np.sin(theta)
    f2 = dt * ut * np.cos(theta)
    F[0,2] = -f1
    F[1,2] = f2
    
    return np.matrix(F).astype(np.float)
       
# Execute the main function
mainFunction()
plt.show()

         
    
        
