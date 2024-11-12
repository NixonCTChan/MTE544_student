import numpy as np

class kalman_filter:
    def __init__(self, P, Q, R, x, dt):
        """
        Initialize Extended Kalman Filter
        
        Parameters:
        P: Initial state covariance matrix (6x6)
        Q: Process noise covariance matrix (6x6)
        R: Measurement noise covariance matrix (4x4)
        x: Initial state vector [x, y, th, w, v, vdot]
        dt: Time step
        """
        self.P = P  # State covariance matrix
        self.Q = Q  # Process noise covariance
        self.R = R  # Measurement noise covariance
        self.x = x  # State vector [x, y, th, w, v, vdot]
        self.dt = dt  # Time step
        
    def predict(self):
        """
        EKF Prediction step using motion model
        Updates state estimate and covariance using system dynamics
        """
        # Get Jacobians of motion model
        self.A = self.jacobian_A()  # State transition Jacobian
        self.C = self.jacobian_H()  # Measurement Jacobian
        
        # Propagate state using nonlinear motion model
        self.motion_model()
        
        # Update covariance using linearized system
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
        
    def update(self, z):
        """
        EKF Update step using measurement model
        
        Parameters:
        z: Measurement vector [v, w, ax, ay]
        """
        # Innovation covariance
        S = np.dot(np.dot(self.C, self.P), self.C.T) + self.R
        
        # Kalman gain
        kalman_gain = np.dot(np.dot(self.P, self.C.T), np.linalg.inv(S))
        
        # Innovation (measurement residual)
        surprise_error = z - self.measurement_model()
        
        # Update state and covariance
        self.x = self.x + np.dot(kalman_gain, surprise_error)
        self.P = np.dot((np.eye(self.A.shape[0]) - np.dot(kalman_gain, self.C)), self.P)
    
    def measurement_model(self):
        """
        Measurement model h(x)
        Returns expected measurements given current state
        """
        x, y, th, w, v, vdot = self.x
        return np.array([
            v,      # Linear velocity
            w,      # Angular velocity
            vdot,   # Linear acceleration x
            w * v   # Linear acceleration y (centripetal)
        ])
        
    def motion_model(self):
        """
        Motion model f(x,u)
        Propagates state forward in time using system dynamics
        """
        x, y, th, w, v, vdot = self.x
        dt = self.dt
        
        self.x = np.array([
            x + v * np.cos(th) * dt,  # x position update
            y + v * np.sin(th) * dt,  # y position update
            th + w * dt,              # heading update
            w,                        # angular velocity (constant)
            v + vdot * dt,           # linear velocity update
            vdot,                    # acceleration (constant)
        ])
    
    def jacobian_A(self):
        """
        Compute Jacobian of motion model with respect to state
        """
        x, y, th, w, v, vdot = self.x
        dt = self.dt
        
        return np.array([
            #x, y, th,                       w,  v,                    vdot
            [1, 0, -v * np.sin(th) * dt,    0,  np.cos(th) * dt,     0],    # dx/d(state)
            [0, 1, v * np.cos(th) * dt,     0,  np.sin(th) * dt,     0],    # dy/d(state)
            [0, 0, 1,                       dt, 0,                    0],    # dth/d(state)
            [0, 0, 0,                       1,  0,                    0],    # dw/d(state)
            [0, 0, 0,                       0,  1,                    dt],   # dv/d(state)
            [0, 0, 0,                       0,  0,                    1]     # dvdot/d(state)
        ])
    
    def jacobian_H(self):
        """
        Compute Jacobian of measurement model with respect to state
        """
        x, y, th, w, v, vdot = self.x
        return np.array([
            #x, y, th, w,  v, vdot
            [0, 0, 0,  0,  1, 0],    # dv/d(state)
            [0, 0, 0,  1,  0, 0],    # dw/d(state)
            [0, 0, 0,  0,  0, 1],    # dax/d(state)
            [0, 0, 0,  v,  w, 0],    # day/d(state)
        ])
        
    def get_states(self):
        """Return current state estimate"""
        return self.x