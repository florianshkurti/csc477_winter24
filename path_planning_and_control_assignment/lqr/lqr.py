import scipy
import numpy as np

class LQR(object):
    def __init__(self, A, B, Q, R):
        self.A = A
        self.B = B
        self.Q = Q
        self.R = R
        
    def compute_policy_gains(self, T, dt):
        # Need to stabilize the system around error = 0, command = 0

        if type(self.A) != type([]):
            self.A = T*[self.A] 

        if type(self.B) != type([]):
            self.B = T*[self.B] 
            
        
        self.P = (T+1)*[self.Q]
        self.K = (T+1)*[0]
        
        for t in range(1, T + 1):

            # TODO: compute the LQR gains K_t and the covariance P_t
            
            
        return self.K
