import numpy as np

class KalmanFilter(object):
    def __init__(self, A, B, G, H, Q, R, x_init, Sigma_init):
        self.x = x_init
        self.Sigma = Sigma_init
        self.A = A
        self.B = B
        self.G = G
        self.H = H
        self.Q = Q
        self.R = R

    def predict(self, u=None):
        if u is None:
            u = np.zeros((self.B.shape[1], ))

        # TODO: implement the prediciton step    
    
    def update(self, z):

        # TODO: implement the update step
        
