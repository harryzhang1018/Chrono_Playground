import numpy as np
import matplotlib.pyplot as plt
import time
from scipy.interpolate import CubicSpline,splprep, splev
# Load the trajectory from the CSV file
# trajectory = np.loadtxt('trajectory.csv', delimiter=',')

def find_closest_point(x_ref,y_ref, truck_x, truck_y):
    
    # Calculate the Euclidean distance between the truck and each point on the trajectory
    distances = np.sqrt((x_ref - truck_x)**2 + (y_ref - truck_y)**2)
    # Find the index of the closest point
    closest_index = np.argmin(distances)
    return closest_index
def merging_scenario(trajectory, truck_x, truck_y, from_left_right='left',vel=31,merge_time=3.8,freq=20):
    x_ref = trajectory[:, 0]
    y_ref = trajectory[:, 1]
    heading_ref = trajectory[:, 2]
    if from_left_right == 'left':
        deviate_theta = np.pi/2
    else:
        deviate_theta = -np.pi/2
    closest_index = find_closest_point(x_ref, y_ref, truck_x, truck_y)
    closest_point_x = x_ref[closest_index]
    closest_point_y = y_ref[closest_index]
    closest_point_theta = heading_ref[closest_index]
    deviation_distance = 3.7
    start_x = closest_point_x + deviation_distance * np.cos(closest_point_theta+deviate_theta)
    start_y = closest_point_y + deviation_distance * np.sin(closest_point_theta+deviate_theta)
    x_disp_to = closest_point_x + vel*merge_time*np.cos(closest_point_theta)
    y_disp_to = closest_point_y + vel*merge_time*np.sin(closest_point_theta)
    end_x,end_y,end_theta = trajectory[find_closest_point(x_ref,y_ref,x_disp_to,y_disp_to)]
    # choose four points to do curve fitting
    length_car = 5 # take average lenth of the car
    point_1 = [start_x,start_y]
    point_2 = [start_x+length_car*np.cos(closest_point_theta),start_y+length_car*np.sin(closest_point_theta)]
    point_3 = [end_x-length_car*np.cos(end_theta),end_y-length_car*np.sin(end_theta)]
    point_4 = [end_x,end_y]
    keypoints = np.array([point_1,point_2,point_3,point_4])
    # do bspline curve fitting
    num_points = int(merge_time*freq)
    t = np.linspace(0,1,num_points)
    tck,u = splprep(keypoints.T,s=0)
    traj_x,traj_y = splev(t,tck)
    merging_traj = np.array([traj_x,traj_y]).T
    return merging_traj

def parallel_scenario(trajectory, truck_x, truck_y, from_left_right='left',vel=31,run_time=3.8,freq=20):
    x_ref = trajectory[:, 0]
    y_ref = trajectory[:, 1]
    heading_ref = trajectory[:, 2]
    if from_left_right == 'left':
        deviate_theta = np.pi/2
    else:
        deviate_theta = -np.pi/2
    closest_index = find_closest_point(x_ref, y_ref, truck_x, truck_y)
    closest_point_x = x_ref[closest_index]
    closest_point_y = y_ref[closest_index]
    closest_point_theta = heading_ref[closest_index]
    deviation_distance = 3.7
    start_x = closest_point_x + deviation_distance * np.cos(closest_point_theta+deviate_theta)
    start_y = closest_point_y + deviation_distance * np.sin(closest_point_theta+deviate_theta)
    disp_to_x = closest_point_x + vel*run_time*np.cos(closest_point_theta)
    disp_to_y = closest_point_y + vel*run_time*np.sin(closest_point_theta)
    end_x,end_y,end_theta = trajectory[find_closest_point(x_ref,y_ref,disp_to_x,disp_to_y)]
    end_x = end_x + deviation_distance*np.cos(end_theta+deviate_theta)
    end_y = end_y + deviation_distance*np.sin(end_theta+deviate_theta)
    # choose four points to do curve fitting
    length_car = 5 # take average lenth of the car
    point_1 = [start_x,start_y]
    point_2 = [start_x+length_car*np.cos(closest_point_theta),start_y+length_car*np.sin(closest_point_theta)]
    point_3 = [end_x-length_car*np.cos(end_theta),end_y-length_car*np.sin(end_theta)]
    point_4 = [end_x,end_y]
    keypoints = np.array([point_1,point_2,point_3,point_4])
    # do bspline curve fitting
    num_points = int(run_time*freq)
    t = np.linspace(0,1,num_points)
    tck,u = splprep(keypoints.T,s=0)
    traj_x,traj_y = splev(t,tck)
    merging_traj = np.array([traj_x,traj_y]).T
    return merging_traj
# trajectory = np.loadtxt('trajectory.csv', delimiter=',')
# for i in range(10):
#     # given location of the truck
#     random_index = np.random.randint(0, len(trajectory))
#     truck_x, truck_y = trajectory[random_index, 0], trajectory[random_index, 1]
#     #keypoints, traj_x, traj_y = merging_scenario(trajectory, truck_x, truck_y, from_left_right='right')
#     parallel_traj = parallel_scenario(trajectory, truck_x, truck_y, from_left_right='left')
#     # Plot the trajectory
#     plt.figure(figsize=(10, 6))
#     plt.plot(trajectory[:,0], trajectory[:,1], label='Reference Trajectory')
#     plt.plot(truck_x, truck_y, 'bo', label='Truck')
#     plt.plot(parallel_traj[:,0],parallel_traj[:,1],'g-',label='Fitted Curve')
#     plt.title('Reference Trajectory')
#     plt.xlabel('X Position')
#     plt.ylabel('Y Position')
#     plt.legend()
#     plt.grid(True)
#     plt.show()