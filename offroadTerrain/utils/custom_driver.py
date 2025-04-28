import pychrono.vehicle as veh
import math

class MyDriver (veh.ChDriver):
    def __init__(self, vehicle:veh.ChWheeledVehicle):
        veh.ChDriver.__init__(self, vehicle)
        self.vehicle = vehicle
        self.t_range_1 = 5.0 # ramp increase throttle
        self.t_range_2 = 10.0 # ramp increase steering
        self.t_range_3 = 15.0 # ramp decrease steering
        self.t_range_4 = 20.0 # sine steering 1
        self.t_range_5 = 30.0 # sine steering 2
        self.target_speed = 10.0 # target speed in m/s
    
    def simple_pid(self):
        current_speed = self.vehicle.GetSpeed()
        throttle = (self.target_speed - current_speed)
        return throttle
    
    def Synchronize(self, time):
        
        # ramp increase throttle
        if (0<time<self.t_range_1):
            thorttle = self.simple_pid()
            self.SetThrottle(thorttle)
                  
        elif (self.t_range_1<time<self.t_range_2): # sine steering 1
            throttle = self.simple_pid()
            steering = 0.5 * math.sin(2 * math.pi / (self.t_range_2 - self.t_range_1) * (time - self.t_range_1))
            self.SetSteering(steering)
            self.SetThrottle(throttle)    
        
        elif (self.t_range_2<time<self.t_range_3): # sine steering 1
            throttle = self.simple_pid()
            steering = -0.5 * math.sin(2 * math.pi / (self.t_range_3 - self.t_range_2) * (time - self.t_range_2))
            self.SetSteering(steering)
            self.SetThrottle(throttle)

        elif (self.t_range_3<time<self.t_range_4): # sine steering 3
            throttle = self.simple_pid()
            steering = -1 * math.sin(2 * math.pi / (self.t_range_4 - self.t_range_3) * (time - self.t_range_3))
            self.SetSteering(steering)
            self.SetThrottle(throttle)
        
        elif (self.t_range_4<time<self.t_range_5): # sine steering 4
            throttle = self.simple_pid()
            steering = math.sin(2 * math.pi / (self.t_range_5 - self.t_range_4) * (time - self.t_range_4))
            self.SetSteering(steering)
            self.SetThrottle(throttle)