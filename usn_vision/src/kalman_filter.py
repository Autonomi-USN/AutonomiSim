import numpy as np
import matplotlib.pyplot as plt

class KalmanDir:
    # Generic implementation of 2D Kalman filter
    # [x,y,vx,vy]
    def __init__(self, x0, P, R, Q=np.eye(4)*5, dtype=float, dt=1.0):
        self.x = x0  # Initial state
        self.P = P   # State covariance matrix 
        self.R = R   # Measurment noise matrix
        self.Q = Q   # Process noise matrix
        self.F = np.array([[1,0,dt,0],          # Transition model matrix
                           [0,1,0,dt],
                           [0,0,1,0],
                           [0,0,0,1]], dtype)
        self.H = np.array([[1,0,0,0],          # Observation model matrix
                           [0,1,0,0]], dtype)
                
    def predict(self, motion = np.zeros((4,1))):
        # Predict next state (a priori estimate of state and state covariance)
        self.x = self.F @ self.x + motion
        self.P = self.F @ self.P @ self.F.T + self.Q
        
    def update(self, z):
        # Update the state (a posteriori estimate of state and state covariance)
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.identity(4) - K@self.H) @ self.P
        return z - self.H@self.x # Return post-fit residual

# Initialize the Kalman filter
x0 = np.array([0, 0, 0, 0]).reshape(4,1)  # Initial state
P0 = np.eye(4)*1             # initial state uncertainty
Q  = np.eye(4)*1                # Model noise
R  = np.eye(2)*10000**2          # Measurment noise
KD = KalmanDir(x0, P0, R, Q=Q)

# Run it
N = 1201
true_x = np.linspace(0.0, 10.0, N)
true_y = true_x**2
observed_x = true_x + 5*np.random.random(N)#*true_x
observed_y = true_y + 5*np.random.random(N)#*true_y
result = []

for ind, (xi, yi) in enumerate(zip(observed_x, observed_y)):
    meas = np.array([xi, yi]).reshape(2,1)
   
    KD.update(meas)
   
    KD.predict()    
        
    result.append(KD.x[0:2].tolist())

result = result[1:]
# Plot result
result = result[1:] # Remove initial value
kalman_x, kalman_y = zip(*result)
plt.plot(observed_x, observed_y, 'ro')
plt.plot(kalman_x, kalman_y, 'g-')


plt.show()