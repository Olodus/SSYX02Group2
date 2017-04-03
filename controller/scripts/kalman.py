import numpy as np


class Kalman(object):

    def __init__(self):

        self.p = 0.1*np.eye(6) # P- the error covariance matrix
#        self.p = 0*np.eye(6) # P- the error covariance matrix
        self.q = np.zeros((6, 6))   # Q- convariance matrix

        self.std_meas = 0.05   # Standard deviation for UWB measurements, NOT percentage
        self.std_dev_x = 0.05  # Standard deviation for speed, percentage
        self.std_dev_z = 0.025  # Standard deviation for rotation, percentage
#        self.std_meas = 0
#        self.std_dev_x = 0
#        self.std_dev_z = 0



    def predict(self, x_k1_k1, v, omega, dt):
        '''
        Returns the estimated state (prediction) X_k|k-1 from the equation
        X_k|k-1 = F_k*X_k-1|k-1 + B_k*u_k
        '''

        dt = np.abs(dt)
        x_pos = x_k1_k1[0, 0]
        y_pos = x_k1_k1[2, 0]
        theta = x_k1_k1[4, 0]

        x_k_k1 = np.array([[x_pos + v*np.cos(theta)*dt],
                           [v*np.cos(theta)],
                           [y_pos + v*np.sin(theta)*dt],
                           [v*np.sin(theta)],
                           [theta + omega*dt],
                           [omega]]) # Updates the state to a new prediction


        return x_k_k1

    def uncertainty(self, f):
        '''
        Retruns P  uncertainty estimation (predictin) from equation
        P = F_k*P_k_k1*F_k^T + Q_k
        '''

        self.q = np.array([[self.std_dev_x**2, 0, 0, 0, 0, 0],
                      [0, self.std_dev_z**2, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0]])

        self.p += np.dot(np.dot(f, self.p), f.T) + self.q
        return self.p


    def update(self, x_k1_k1, v, omega, dt, measured):
        '''
        1. -Predict the next state from the old state
           - Predict the new uncertainty from the old uncertainty
        '''

        dt = np.abs(dt)
        x_pos = x_k1_k1[0, 0]
        y_pos = x_k1_k1[2, 0]
        theta = x_k1_k1[4, 0]


        #derivatives in jacobian
        dx_dxv = dt
        dxv_dtheta = -v*np.sin(theta)
        dy_dyv = dt
        dyv_dtheta = v*np.cos(theta)
        dtheta_domega = dt


        f = np.array([[1, dx_dxv, 0, 0, 0, 0],
                      [0, 1, 0, 0, dxv_dtheta, 0],
                      [0, 0, 1, dy_dyv, 0, 0],
                      [0, 0, 0, 1, dyv_dtheta, 0],
                      [0, 0, 0, 0, 1, dtheta_domega],
                      [0, 0, 0, 0, 0, 1]])




        x_k_k1 = self.predict(x_k1_k1, v, omega, dt)
        p = self.uncertainty(f);


        '''
        2. -Calculate residual using
        y_k = z_k - H_k*X_k|k-1
        '''
        z_k = np.array([[measured[0]],
                        [0],
                        [measured[1]],
                        [0],
                        [0],
                        [0]])

                        # measured state

        h = np.array([[1, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0],
                      [0, 0, 1, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0]])
                      # observation matrix

        y_k = z_k - np.dot(h, x_k_k1)
              # residual
        '''
        3. Calculate residual covariance using
        S_k = H_k * P_k|k-1 * H_k^T + Rk
        '''

#self.std_meas**2*
        r = self.std_meas**2*np.array([[1, 0, 0, 0, 0, 0],
                                       [0, 1, 0, 0, 0, 0],
                                       [0, 0, 1, 0, 0, 0],
                                       [0, 0, 0, 1, 0, 0],
                                       [0, 0, 0, 0, 1, 0],
                                       [0, 0, 0, 0, 0, 1]])

#        print  (h)
#        print  (self.p)
#        print  (r)

        s = np.add(np.dot(np.dot(h, p), h.T), r)
        #not sure if this is correct
        '''
        4. Calculate the Kalman gain using
        K = P_k|k-1 * H^T * S_k^-1
        '''

        kalman_gain = np.dot(np.dot(self.p, h.T), np.linalg.inv(s))  # Kalman gain

        '''
        5. Calculate new state and state covariance with
        following equations
        - X_k|k = X_k|k-1 + K*(z_k - F*X_k|k-1)
        - P_k|k = P_k|k-1 - K*F*P_k|k-1
        '''
        x_k_k = x_k_k1 + np.dot(kalman_gain, y_k)  # current state
            # current covariance
        self.p = np.dot(np.eye(6) - np.dot(kalman_gain, h), self.p)
        self.q = np.zeros((6, 6))  # reset control noise after correction

        print (self.p)

        return x_k_k
