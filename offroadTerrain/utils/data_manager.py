import pychrono.vehicle as veh
import pychrono.core as chrono
import numpy as np
import csv
import os, sys

class DataManager:
    def __init__(self, vehicle: veh.ChWheeledVehicle, terrain: veh.SCMTerrain):
        self.vehicle = vehicle
        self.terrain = terrain
        self.data = []
        self.Lf, self.Lr = 1.745, 1.633
        self.written_header = False
        self.root_dir = os.path.dirname(os.path.abspath(__file__))
        print(self.root_dir)

    def GetNNTrainDataHeader(self):
        return [ "Time","Fy_f", "Fy_r" , "Fz_f", "Fz_r", "alpha_f", "alpha_r", "front_steer_rate", "rear_steer_rate", "u_w_f", "u_w_r"]
    

    def WriteNNTrainData(self, file_path=None):
        if file_path is None:
            file_path = self.root_dir + '/../data/train_data.csv'

        if not self.written_header:
            self.written_header = True
            with open(file_path, 'w') as f:
                writer = csv.writer(f)
                writer.writerow(self.GetNNTrainDataHeader())
        else:
            with open(file_path, 'a') as f:
                writer = csv.writer(f)
                writer.writerow(self.GetNNTraindata())
        
        

    def GetNNTraindata(self):
        ### time
        time = self.vehicle.GetSystem().GetChTime()

        ### longitudinal speed
        u_x = self.vehicle.GetSpeed()
        #print(f"u_x: {u_x}")

        ### lateral speed
        global_vel = self.vehicle.GetPointVelocity(chrono.ChVector3d(0, 0, 0))
        local_vel = self.vehicle.GetTransform().TransformDirectionParentToLocal(global_vel)
        v = local_vel.y
        #print(f"v: {v}")

        ### front steering angle
        front_steer_left = self.vehicle.GetSteeringAngle(axle=0,side=0)
        front_steer_right = self.vehicle.GetSteeringAngle(axle=0,side=1)
        front_steer = (front_steer_left + front_steer_right) / 2
        #print(f"front_steer: {front_steer}")

        ### yaw rate
        r = self.vehicle.GetYawRate()
        
        ### front steering rate
        left_global_vel = self.vehicle.GetSpindleAngVel(axle=0,side=0)
        right_global_vel = self.vehicle.GetSpindleAngVel(axle=0,side=1)
        left_local_vel = self.vehicle.GetTransform().TransformDirectionParentToLocal(left_global_vel)
        right_local_vel = self.vehicle.GetTransform().TransformDirectionParentToLocal(right_global_vel)
        front_steer_rate_left = left_local_vel.z
        front_steer_rate_right = right_local_vel.z
        delta_dot_f = (front_steer_rate_left + front_steer_rate_right) / 2
        #print(f"front_steer_rate: {front_steer_rate}")

        ### rear steering rate
        left_global_vel = self.vehicle.GetSpindleAngVel(axle=1,side=0)
        right_global_vel = self.vehicle.GetSpindleAngVel(axle=1,side=1)
        left_local_vel = self.vehicle.GetTransform().TransformDirectionParentToLocal(left_global_vel)
        right_local_vel = self.vehicle.GetTransform().TransformDirectionParentToLocal(right_global_vel)
        rear_steer_rate_left = left_local_vel.z
        rear_steer_rate_right = right_local_vel.z
        delta_dot_r = (rear_steer_rate_left + rear_steer_rate_right) / 2

        ### wheel rotation speed
        front_left_vel = self.vehicle.GetSpindleOmega(axle=0,side=0)
        front_right_vel = self.vehicle.GetSpindleOmega(axle=0,side=1)
        rear_left_vel = self.vehicle.GetSpindleOmega(axle=1,side=0)
        rear_right_vel = self.vehicle.GetSpindleOmega(axle=1,side=1)
        u_w_f = (front_left_vel + front_right_vel) / 2
        u_w_r = (rear_left_vel + rear_right_vel) / 2

        ### tire slip angle
        alpha_f = np.arctan( (v + self.Lf * r) / (u_x + 1e-6)) - self.Lf
        alpha_r = np.arctan( (v - self.Lr * r) / (u_x + 1e-6)) 
        
        ## force
        F_z_f_left = self.vehicle.GetTire(0,0).ReportTireForce(self.terrain).force.z
        F_z_f_right = self.vehicle.GetTire(0,1).ReportTireForce(self.terrain).force.z
        F_z_r_left = self.vehicle.GetTire(1,0).ReportTireForce(self.terrain).force.z
        F_z_r_right = self.vehicle.GetTire(1,1).ReportTireForce(self.terrain).force.z
        F_z_f = (F_z_f_left + F_z_f_right) / 2
        F_z_r = (F_z_r_left + F_z_r_right) / 2

        F_y_f_left = self.vehicle.GetTire(0,0).ReportTireForce(self.terrain).force.y
        F_y_f_right = self.vehicle.GetTire(0,1).ReportTireForce(self.terrain).force.y
        F_y_r_left = self.vehicle.GetTire(1,0).ReportTireForce(self.terrain).force.y
        F_y_r_right = self.vehicle.GetTire(1,1).ReportTireForce(self.terrain).force.y
        F_y_f = (F_y_f_left + F_y_f_right) / 2
        F_y_r = (F_y_r_left + F_y_r_right) / 2

        return [time, F_y_f, F_y_r, F_z_f, F_z_r, alpha_f, alpha_r, delta_dot_f, delta_dot_r, u_w_f, u_w_r]
    

        
        
