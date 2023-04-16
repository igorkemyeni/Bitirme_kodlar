import numpy as np
import sympy as sp

class EKF():
    def __init__(self, x0_n, P0_n, Q_n, R_n, state:sp.Matrix, control:sp.Matrix, f:sp.Matrix, h:sp.Matrix):
        """
        Initializes the Extended Kalman Filter with the initial state and state covariance, process noise covariance,
        measurement noise covariance, and time step.

        :param x0: Initial state vector dictionary for symbol. 
        :param P0: Initial state covariance matrix.
        :param Q: Process noise covariance matrix.
        :param R: Measurement noise covariance matrix.
        :param state: Symbol State Vector
        :param control: Symbol Control Vector
        :param f: Symbol State Transition Function
        :param h: Symbol Measurement Function
        """
        self.x = x0_n
        self.P = P0_n
        self.Q = Q_n
        self.R = R_n
        self.f = f
        self.state = state
        self.control = control
        self.h = h

        dt = sp.symbols("dt")
        # Compile symbolic functions to callable functions for numerical computation
        self.f_x = sp.lambdify([state, control, dt], f, 'numpy')
        self.h_x = sp.lambdify([state], h, 'numpy')

        self.f_jacob = sp.lambdify([state, control, dt], f.jacobian(state), 'numpy')
        self.h_jacob = sp.lambdify([state], h.jacobian(state), 'numpy')
      
    def predict(self, u, dt_n):
      """
      Predicts the next state using the control input.

      :param u: Control input.
      """
      # Predict the state and state covariance using the control input and the state transition function
      x_pred = self.f_x(self.x.flatten().tolist(), u.flatten().tolist(), dt_n)
      F = self.f_jacob(self.x.flatten().tolist(), u.flatten().tolist(), dt_n)
      P_pred = F @ self.P @ F.T + self.Q

      # Update the state and state covariance
      self.x = x_pred
      self.P = P_pred

    def update(self, z):
      """
      Updates the state estimate using the measurement.

      :param z: Measurement vector (GPS).
      """
      # Calculate the innovation covariance and the Kalman gain
      y_pred = self.h_x(self.x.flatten().tolist())
      H = self.h_jacob(self.x.flatten().tolist())
      S = H @ self.P @ H.T + self.R
      K = self.P @ H.T @ np.linalg.inv(S)

      # Update the state and state covariance using the measurement residual and the Kalman gain
      self.x = self.x + K @ (z - y_pred)
      self.P = (np.eye(len(self.x)) - K @ H) @ self.P