import numpy as np 

class Obstacle:
    def __init__(self,x0,y0,R,safedisatnce):
        self.x0 = x0
        self.y0 = y0
        self.R = R
        self.safedistance =safedisatnce
        self.R_final = self.R+self.safedistance
    def ObstacleMatrices(self,MPCparameter):
        if self.x0*self.x0+self.y0*self.y0 < self.R_final*self.R_final:
            try:
                sys.exit(0)
            finally:
                print('Too close to obstacle !!!!!!!!!!!!!!!!!')
        A1 = np.hstack((np.zeros((MPCparameter.N_steps,MPCparameter.N)),np.identity(MPCparameter.N_steps)))
        A1 = np.vstack((A1,np.zeros((MPCparameter.N_steps,MPCparameter.N+MPCparameter.N_steps))))
        A2 = np.hstack((np.zeros((MPCparameter.N_steps,MPCparameter.N)),np.identity(MPCparameter.N_steps)))
        A2 = np.vstack((np.zeros((MPCparameter.N_steps,MPCparameter.N+MPCparameter.N_steps)),A2))
        A = np.hstack((A1,A2))
        step = []
        for i in range(MPCparameter.N_steps):
            Step = np.zeros((1,MPCparameter.N_steps))
            Step[0,0:i+1] = np.ones((1,i+1))
            Step = np.kron(np.identity(2),Step)
            step.append(np.dot(Step,A))
        self.distance = np.array([[self.x0],[self.y0]])
        self.step = step