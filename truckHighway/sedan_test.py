# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2014 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =============================================================================

import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens

import math
import numpy as np
# import time
from controller import error_state ,vir_veh_controller
from merge_scenario import merging_scenario,parallel_scenario,serial_scenario
from rom_vehicle import simplifiedVehModel
import csv
"""
!!!! Set this path before running the demo!
"""
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


import sys,os
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(project_root)
# Add the parent directory of 'models' to the Python path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))


# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(-500, 0, 0.3)
initRot = chrono.QuatFromAngleZ(0)

# initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH
# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain
# terrain_model = veh.RigidTerrain.BOX
terrainHeight = 0      # terrain height
terrainLength = 1200.0  # size in X direction
terrainWidth = 1200.0   # size in Y direction

# Poon chassis tracked by the camera
trackPoint = chrono.ChVector3d(-5.0, 0.0, 1.8)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 20  # FPS = 50
control_step_size = 1.0 / 20 # FPS to run control = 20

# =============================================================================

# --------------
# Create systems
# --------------
# Create the sedan vehicle, set parameters, and initialize
sedan = veh.BMW_E90()
sedan.SetContactMethod(contact_method)
sedan.SetChassisCollisionType(chassis_collision_type)
sedan.SetChassisFixed(False)
sedan.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
sedan.SetTireType(tire_model)
sedan.SetTireStepSize(tire_step_size)


sedan.Initialize()

sedan.SetChassisVisualizationType(vis_type)
sedan.SetSuspensionVisualizationType(vis_type)
sedan.SetSteeringVisualizationType(vis_type)
sedan.SetWheelVisualizationType(vis_type)
sedan.SetTireVisualizationType(vis_type)



#sedan.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the terrain

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(sedan.GetSystem())
patch = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
    terrainLength, terrainWidth)

patch.SetTexture(veh.GetDataFile("terrain/textures/Concrete002_2K-JPG/Concrete002_2K_Color.jpg"), 100, 100)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# adding visualization for the reference trajectory
# Create visualization material for the path
vis_mat_path = chrono.ChVisualMaterial()
vis_mat_path.SetDiffuseColor(chrono.ChColor(0.0, 1.0, 0.0))
road_vis_mat = chrono.ChVisualMaterial()
road_vis_mat.SetDiffuseColor(chrono.ChColor(1.0, 1.0, 1.0))
# Create ChBodyEasyBox objects at specified positions
# reference centerline of the sedan
# reference_trajectory = np.loadtxt('./data/reference_traj/trajectory.csv', delimiter=',')
reference_trajectory = np.loadtxt(project_root+'/truckHighway/data/reference_traj/trajectory_complex.csv', delimiter=',')
# only show sparse points not all points
visual_reference_trajectory = reference_trajectory[::50]
for pos in visual_reference_trajectory:
    center_x,center_y,center_heading = pos
    
    right_lane_x = center_x + 1.85 * np.cos(center_heading+np.pi/2)
    right_lane_y = center_y + 1.85 * np.sin(center_heading+np.pi/2)
    left_lane_x = center_x + 1.85 * np.cos(center_heading-np.pi/2)
    left_lane_y = center_y + 1.85 * np.sin(center_heading-np.pi/2)
    lane_rot = chrono.QuatFromAngleZ(center_heading)
    # add right lane and left lane
    right_lane_body = chrono.ChBodyEasyBox(1.5, 0.1, 0.05, 1000, True, False)
    right_lane_body.SetPos(chrono.ChVector3d(right_lane_x, right_lane_y, 0.5))
    right_lane_body.SetRot(lane_rot)
    right_lane_body.SetFixed(True)
    right_lane_shape = right_lane_body.GetVisualModel().GetShape(0)
    right_lane_shape.AddMaterial(road_vis_mat)
    left_lane_body = chrono.ChBodyEasyBox(1.5, 0.1, 0.05, 1000, True, False)
    left_lane_body.SetPos(chrono.ChVector3d(left_lane_x, left_lane_y, 0.5))
    left_lane_body.SetRot(lane_rot)
    left_lane_body.SetFixed(True)
    left_lane_shape = left_lane_body.GetVisualModel().GetShape(0)
    left_lane_shape.AddMaterial(road_vis_mat)
    sedan.GetSystem().Add(right_lane_body)
    sedan.GetSystem().Add(left_lane_body)
    
    #sedan.GetSystem().Add(box_body)

manager = sens.ChSensorManager(sedan.GetSystem())

intensity = 1.0
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)

# Update rate in Hz
update_rate = 15
# Image width and height
image_width = 1280
image_height = 720
# Camera's horizontal field of view
fov = 2.8
# Lag (in seconds) between sensing and when data becomes accessible
lag = 0
# Exposure (in seconds) of each image
exposure_time = 0
offset_pose = chrono.ChFramed(
        chrono.ChVector3d(-1.35, -0.25, 0.8), chrono.QuatFromAngleAxis(-0.1, chrono.ChVector3d(0, 1, 0)))
cam = sens.ChCameraSensor(
        sedan.GetChassisBody(),              # body camera is attached to
        update_rate,            # update rate in Hz
        offset_pose,            # offset pose
        image_width,            # image width
        image_height,           # image height
        fov)                    # camera's horizontal field of view
cam.SetName("Camera Sensor")
cam.SetLag(lag)
cam.SetCollectionWindow(exposure_time)
cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Before Grayscale Filter"))
# add sensor to manager
manager.AddSensor(cam)


# initialize the virtual vehicles
sur_veh1 = simplifiedVehModel(sedan.GetSystem(),[0,0,0,0],[0,0],control_step_size)
sur_veh2 = simplifiedVehModel(sedan.GetSystem(),[0,0,0,0],[0,0],control_step_size)
sur_veh3 = simplifiedVehModel(sedan.GetSystem(),[0,0,0,0],[0,0],control_step_size)
sur_veh4 = simplifiedVehModel(sedan.GetSystem(),[0,0,0,0],[0,0],control_step_size)
sur_veh5 = simplifiedVehModel(sedan.GetSystem(),[0,0,0,0],[0,0],control_step_size)
# -------------------------------------
# Create the vehicle Irrlicht interface
# Create the driver system
# -------------------------------------

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('sedan vehicle simulation')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
# vis.AddTypicalLights()
vis.AddSkyBox()
vis.AttachVehicle(sedan.GetVehicle())


# Create the driver system
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetJoystickConfigFile(chrono.GetChronoDataFile('vehicle/joystick/controller_WheelPedalsAndShifters.json'))
# Set the time response for steering and throttle keyboard inputs.
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()
# ---------------
# Simulation loop
# ---------------

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)
control_steps = math.ceil(control_step_size / step_size)

# Initialize simulation frame counter s
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
control_number = 0
render_frame = 0
is_merging = False
is_merging_left = False
merge_trajectory = []
merge_trajectory_left = []
merge_traj_ind = 0
merge_traj_ind_left = 0

is_parallel = False
is_parallel_left = False
parallel_trajectory = []
parallel_trajectory_left = []
parallel_traj_ind = 0
parallel_traj_ind_left = 0

is_serial = False
serial_trajectory = []
serial_traj_ind = 0

# Constants
merge_start_time = 5.0
min_pause = 1.0  # Minimum pause between trajectories
max_pause = 2.0  # Maximum pause between trajectories

# Variables to track last trajectory generation time
last_trajectory_time = 0
next_trajectory_time = merge_start_time

# Variables to track the last side used for each scenario type
last_merge_side = None
last_parallel_side = None

ind_s1,ind_s2,ind_s3,ind_s4,ind_s5 = 0,0,0,0,0
log_s1,log_s2,log_s3,log_s4,log_s5 = [],[],[],[],[]
train_data = {}
center_veh_log = []




while vis.Run() :
    time = sedan.GetSystem().GetChTime()

    manager.Update()

    # control sedan vehicle 
    # driver_sedan = straight() # lanechange
    # get trailer and tractor position
    veh_pos = sedan.GetChassisBody().GetPos()
    veh_heading = sedan.GetChassisBody().GetRot().GetCardanAnglesZYX().z
    veh_x = veh_pos.x
    veh_y = veh_pos.y
    veh_heading = veh_heading
    veh_back_x = veh_x - 5.74 * np.cos(veh_heading)
    veh_back_y = veh_y - 5.74 * np.sin(veh_heading)
    

    if time > next_trajectory_time:

        # Randomly choose between tractor and trailer position
        if np.random.rand() < 0.5:
            sedan_x, sedan_y = veh_x, veh_y
            position_used = "parallel"
        else:
            sedan_x, sedan_y = veh_back_x, veh_back_y
            position_used = "behind"

        # Randomly choose between merge, parallel, and serial scenarios
        scenario_choice = np.random.rand()
        
        if scenario_choice < 0.4:  # 40% chance for merge
            # Merge scenario
            from_side = 'left' if last_merge_side != 'left' else 'right'
            vel = np.random.uniform(26, 30)
            merge_time = np.random.uniform(2.5, 4.5)
            
            new_trajectory = merging_scenario(reference_trajectory, sedan_x, sedan_y, 
                                              from_left_right=from_side,
                                              vel=vel,
                                              merge_time=merge_time,
                                              freq=25)
            print(f"Merging from {from_side}, velocity: {vel:.2f}, merge time: {merge_time:.2f}, using {position_used} position")
            print(new_trajectory.shape)
            
            if from_side == 'left':
                merge_trajectory_left = new_trajectory
                is_merging_left = True
                merge_traj_ind_left = 0
                sur_veh2.set_state([merge_trajectory_left[0][0],merge_trajectory_left[0][1],merge_trajectory_left[0][2],vel])
            else:
                merge_trajectory = new_trajectory
                is_merging = True
                merge_traj_ind = 0
                # add virtual vehicle
                sur_veh1.set_state([merge_trajectory[0][0],merge_trajectory[0][1],merge_trajectory[0][2],vel])
            
            last_merge_side = from_side
        elif scenario_choice < 0.8:  # 40% chance for parallel
            # Parallel scenario
            from_side = 'left' if last_parallel_side != 'left' else 'right'
            driver_vel = np.random.uniform(30, 40)
            run_time = np.random.uniform(3.5, 5.5)
            
            new_trajectory = parallel_scenario(reference_trajectory, sedan_x, sedan_y, 
                                               from_left_right=from_side,
                                               vel=driver_vel,
                                               run_time=run_time,
                                               freq=25)
            print(f"Parallel from {from_side}, velocity: {driver_vel:.2f}, run time: {run_time:.2f}, using {position_used} position")
            print(new_trajectory.shape)
            
            if from_side == 'left':
                parallel_trajectory_left = new_trajectory
                is_parallel_left = True
                parallel_traj_ind_left = 0
                sur_veh3.set_state([parallel_trajectory_left[0][0],parallel_trajectory_left[0][1],parallel_trajectory_left[0][2],driver_vel])
            else:
                parallel_trajectory = new_trajectory
                is_parallel = True
                parallel_traj_ind = 0
                sur_veh4.set_state([parallel_trajectory[0][0],parallel_trajectory[0][1],parallel_trajectory[0][2],driver_vel])
            
            last_parallel_side = from_side
        else:  # 20% chance for serial, but only if no merging is active
            if not is_merging and not is_merging_left:
                # Serial scenario
                driver_vel = np.random.uniform(26, 40)
                run_time = np.random.uniform(3.5, 5.5)
                
                serial_trajectory = serial_scenario(reference_trajectory, sedan_x, sedan_y, 
                                                    vel=driver_vel,
                                                    run_time=run_time,
                                                    freq=25)
                print(f"Serial scenario, velocity: {driver_vel:.2f}, run time: {run_time:.2f}, using {position_used} position")
                print(serial_trajectory.shape)
                
                is_serial = True
                serial_traj_ind = 0
                sur_veh5.set_state([serial_trajectory[0][0],serial_trajectory[0][1],serial_trajectory[0][2],driver_vel])
            else:
                print("Serial scenario skipped due to active merging")

        # Set the next trajectory generation time
        last_trajectory_time = time
        next_trajectory_time = time + np.random.uniform(min_pause, max_pause)

    if (step_number % control_steps == 0):
        # for sedan controller now it's a simple PID controller
        error = error_state(veh_state=[veh_x,veh_y,veh_heading],ref_traj=reference_trajectory,lookahead=3.0)
        steering = sum([x * y for x, y in zip(error, [0.02176878 , 0.72672704 , 0.78409284 ,-0.0105355 ])]) # @hang here is the place to add controller
        ref_vel = 20
        current_vel = sedan.GetVehicle().GetSpeed()
        vel_error = ref_vel - current_vel
        throttle = 0.5 * vel_error + 0.5
        driver.SetSteering(steering)
        driver.SetThrottle(throttle)

        if is_merging and merge_traj_ind < len(merge_trajectory):
            control1 = vir_veh_controller(veh_state=[sur_veh1.x,sur_veh1.y, sur_veh1.theta,sur_veh1.v],ref_traj=merge_trajectory,lookahead=1.0)
            sur_veh1.update(control1)
            print("updating vir veh 1")
        
        if is_merging_left and merge_traj_ind_left < len(merge_trajectory_left):
            control2 = vir_veh_controller(veh_state=[sur_veh2.x,sur_veh2.y, sur_veh2.theta,sur_veh2.v],ref_traj=merge_trajectory_left,lookahead=1.0)
            sur_veh2.update(control2)
        
        if is_parallel and parallel_traj_ind < len(parallel_trajectory):
            control3 = vir_veh_controller(veh_state=[sur_veh4.x,sur_veh4.y, sur_veh4.theta,sur_veh4.v],ref_traj=parallel_trajectory,lookahead=1.0)
            sur_veh4.update(control3)
        
        if is_parallel_left and parallel_traj_ind_left < len(parallel_trajectory_left):
            control4 = vir_veh_controller(veh_state=[sur_veh3.x,sur_veh3.y, sur_veh3.theta,sur_veh3.v],ref_traj=parallel_trajectory_left,lookahead=1.0)
            sur_veh3.update(control4)
        
        if is_serial and serial_traj_ind < len(serial_trajectory):
            control5 = vir_veh_controller(veh_state=[sur_veh5.x,sur_veh5.y, sur_veh5.theta,sur_veh5.v],ref_traj=serial_trajectory,lookahead=1.0)
            sur_veh5.update(control5)
            
   
    
    # Render scene and output POV-Ray data
    if (step_number % render_steps == 0) :
        print('sur veh 3:',sur_veh3.get_state())
        center_veh_log.append([time,veh_x,veh_y,veh_heading])
        train_data['center_vehicle'] = center_veh_log
        # print vir veh 1 
        if is_merging and merge_traj_ind < len(merge_trajectory):
            dis_2_end_1 = np.sqrt((merge_trajectory[-1][0]-sur_veh1.x)**2 + (merge_trajectory[-1][1]-sur_veh1.y)**2)
            if dis_2_end_1 < 1:
                is_merging = False
                merge_traj_ind = 0
                train_data['s1_'+str(ind_s1)] = log_s1
                log_s1 = []
                ind_s1 += 1
                print("vehicle 1 reached the end, turn off the visualization")
                sur_veh1.pause_visualization()
            else:
                print('vehicle 1:',sur_veh1.get_state())
                log_s1.append([time,sur_veh1.x,sur_veh1.y,sur_veh1.theta])
                merge_traj_ind += 1

        elif is_merging:
            is_merging = False
            merge_traj_ind = 0
            train_data['s1_'+str(ind_s1)] = log_s1
            log_s1 = []
            ind_s1 += 1
            print("vehicle 1 reached the end at 2nd condition")
            
            
        
        # Left merge
        if is_merging_left and merge_traj_ind_left < len(merge_trajectory_left):
            dis_2_end_2 = np.sqrt((merge_trajectory_left[-1][0]-sur_veh2.x)**2 + (merge_trajectory_left[-1][1]-sur_veh2.y)**2)
            if dis_2_end_2 < 1:
                is_merging_left = False
                merge_traj_ind_left = 0
                train_data['s2_'+str(ind_s2)] = log_s2
                log_s2 = []
                ind_s2 += 1
                sur_veh2.pause_visualization()
            else:
                log_s2.append([time,sur_veh2.x,sur_veh2.y,sur_veh2.theta])
                merge_traj_ind_left += 1

        elif is_merging_left:
            is_merging_left = False
            merge_traj_ind_left = 0
            train_data['s2_'+str(ind_s2)] = log_s2
            log_s2 = []
            ind_s2 += 1
            
        # Right parallel
        if is_parallel and parallel_traj_ind < len(parallel_trajectory):
            dis_2_end_3 = np.sqrt((parallel_trajectory[-1][0]-sur_veh4.x)**2 + (parallel_trajectory[-1][1]-sur_veh4.y)**2)
            if dis_2_end_3 < 3:
                is_parallel = False
                parallel_traj_ind = 0
                train_data['s3_'+str(ind_s3)] = log_s3
                log_s3 = []
                ind_s3 += 1
                sur_veh4.pause_visualization()
                print("vehicle 3 reached the end, exit at condition 1")
            else:
                log_s3.append([time,sur_veh4.x,sur_veh4.y,sur_veh4.theta])
                parallel_traj_ind += 1
            
        elif is_parallel:
            is_parallel = False
            parallel_traj_ind = 0
            train_data['s3_'+str(ind_s3)] = log_s3
            log_s3 = []
            ind_s3 += 1
            sur_veh4.pause_visualization()
            print("vehicle 3 reached the end, exit at condition 2")
        
        # Left parallel
        if is_parallel_left and parallel_traj_ind_left < len(parallel_trajectory_left): 
            dis_2_end_4 = np.sqrt((parallel_trajectory_left[-1][0]-sur_veh3.x)**2 + (parallel_trajectory_left[-1][1]-sur_veh3.y)**2)
            if dis_2_end_4 < 3:
                is_parallel_left = False
                parallel_traj_ind_left = 0
                train_data['s4_'+str(ind_s4)] = log_s4
                log_s4 = []
                ind_s4 += 1
                sur_veh3.pause_visualization()
            else:
                log_s4.append([time,sur_veh3.x,sur_veh3.y,sur_veh3.theta])
                parallel_traj_ind_left += 1

        elif is_parallel_left:
            is_parallel_left = False
            parallel_traj_ind_left = 0
            train_data['s4_'+str(ind_s4)] = log_s4
            log_s4 = []
            ind_s4 += 1
            sur_veh3.pause_visualization()

        # Serial
        if is_serial and serial_traj_ind < len(serial_trajectory):
            dis_2_end_5 = np.sqrt((serial_trajectory[-1][0]-sur_veh5.x)**2 + (serial_trajectory[-1][1]-sur_veh5.y)**2)
            if dis_2_end_5 < 1:
                is_serial = False
                serial_traj_ind = 0
                train_data['s5_'+str(ind_s5)] = log_s5
                log_s5 = []
                ind_s5 += 1
                sur_veh5.pause_visualization()
            else:
                log_s5.append([time,sur_veh5.x,sur_veh5.y,sur_veh5.theta])

                serial_traj_ind += 1

        elif is_serial:
            is_serial = False
            serial_traj_ind = 0
            train_data['s5_'+str(ind_s5)] = log_s5
            log_s5 = []
            ind_s5 += 1
            sur_veh5.pause_visualization()

        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        # filename = './prototype/img_' + str(render_frame) +'.jpg' 
        # vis.WriteImageToFile(filename)
        render_frame += 1


        

    # Get driver inputs
    driver_inputs = driver.GetInputs()
    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    sedan.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    sedan.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)

else:
    list_of_traindata = list(train_data.values())
    print('number of element: ',len(list_of_traindata))
    # this is the center vehicle
    print('number of steps: ', len(list_of_traindata[0]))
    np.save('./data/training_data/sedan_train_data.npy', np.array(list_of_traindata, dtype=object))
    with open('./data/training_data/sedan_train_data.txt', 'w') as f:
        for item in list_of_traindata:
            np.savetxt(f, item, fmt='%f')
            f.write('\n')

    loaded_data = list(np.load('./data/training_data/sedan_train_data.npy', allow_pickle=True))
    print('number of element',len(loaded_data))
    print('number of steps: ', len(loaded_data[0])) # center vehicle information
    