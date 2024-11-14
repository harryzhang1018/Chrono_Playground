import numpy as np

class simplifiedVehModel():
    def __init__(self, state, control ,dt):
        # state = [x, y, theta,v]
        self.x = state[0]
        self.y = state[1]
        self.theta = state[2]
        self.v = state[3]
        # control = [alpha, beta]
        self.alpha = control[0]
        self.beta = control[1]
        self.dt = dt # time step

    def update(self, control):
        # car parameters
        delta = 0.667
        l = 2.5
        tau0 = 130
        omega0 = 1600
        gamma = 1/3
        r_wheel = 0.3
        i_wheel = 0.6
        c0,c1 = 0.01, 0.02

        self.alpha = control[0]
        self.beta = control[1]

        omega_m = self.v/(r_wheel*gamma)
        helpfun1 = - tau0 * omega_m / omega0 + tau0
        helpfunT = self.alpha * helpfun1 - c1*omega_m - c0

        self.x += self.v*np.cos(self.theta)*self.dt
        self.y += self.v*np.sin(self.theta)*self.dt
        self.theta += (self.v*np.tan(self.beta * delta)/l)*self.dt
        self.v += helpfunT * gamma / i_wheel * r_wheel * self.dt
    
    def set_state(self, state):
        self.x = state[0]
        self.y = state[1]
        self.theta = state[2]
        self.v = state[3]
    
    def get_state(self):
        return [self.x, self.y, self.theta, self.v] 

# vehicle = simplifiedVehModel([0,0,0,0],[0,0],0.05)
# vehicle.set_state([1,1,0,0])
# for i in range(100):
#     vehicle.update([0.1,0.2])
#     print(vehicle.x, vehicle.y, vehicle.theta, vehicle.v)        
            
