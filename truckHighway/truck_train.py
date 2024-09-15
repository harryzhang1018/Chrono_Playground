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
import math
import numpy as np
import time
from controller import error_state
from merge_scenario import merging_scenario,parallel_scenario,serial_scenario
import csv
"""
!!!! Set this path before running the demo!
"""
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(500, 0, 0.5)
initRot = chrono.QuatFromAngleZ(1.57)

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
trackPoint = chrono.ChVector3d(7.0, 1.0, 7.75)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 25  # FPS = 50
control_step_size = 1.0 / 20 # FPS to run control = 20

# =============================================================================

# --------------
# Create systems
# --------------
# Create the truck vehicle, set parameters, and initialize
truck = veh.Kraz()
truck.SetContactMethod(contact_method)
truck.SetChassisCollisionType(chassis_collision_type)
truck.SetChassisFixed(False)
truck.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
truck.Initialize()
truck.SetChassisVisualizationType(vis_type, vis_type)
truck.SetSteeringVisualizationType(vis_type)
truck.SetSuspensionVisualizationType(vis_type, vis_type)
truck.SetWheelVisualizationType(vis_type, vis_type)
truck.SetTireVisualizationType(vis_type, vis_type)



#truck.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the terrain

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(truck.GetSystem())
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
# reference centerline of the truck
reference_trajectory = np.loadtxt('./data/reference_traj/trajectory.csv', delimiter=',')
# only show sparse points not all points
visual_reference_trajectory = reference_trajectory[::50]
for pos in visual_reference_trajectory:
    center_x,center_y,center_heading = pos
    box_body = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, 0.2, 0.05, 1000, True, False)
    box_body.SetPos(chrono.ChVector3d(center_x, center_y, 0.5))
    box_body.SetFixed(True)
    # Set visual material for the box
    shape = box_body.GetVisualModel().GetShape(0)
    if shape.GetNumMaterials() == 0:
        shape.AddMaterial(vis_mat_path)
    else:
        shape.GetMaterials()[0] = vis_mat_path
        
        
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
    truck.GetSystem().Add(right_lane_body)
    truck.GetSystem().Add(left_lane_body)
    
    #truck.GetSystem().Add(box_body)

# initialize the ball to represent the ball from right hand merging
ball_merge_right = chrono.ChBodyEasySphere(0.4, 1000, True, False)
vis_merge = chrono.ChVisualMaterial()
vis_merge.SetDiffuseColor(chrono.ChColor(1,0,0))
ball_merge_right_shape = ball_merge_right.GetVisualModel().GetShape(0)
ball_merge_right_shape.AddMaterial(vis_merge)
ball_merge_right.SetPos(chrono.ChVector3d(0,0,0))
ball_merge_right.SetFixed(False)
truck.GetSystem().Add(ball_merge_right)
# initialize the ball to represent the ball from left hand merging
ball_merge_left = chrono.ChBodyEasySphere(0.4, 1000, True, False)
vis_merge = chrono.ChVisualMaterial()
vis_merge.SetDiffuseColor(chrono.ChColor(1,0,0))
ball_merge_left_shape = ball_merge_left.GetVisualModel().GetShape(0)
ball_merge_left_shape.AddMaterial(vis_merge)
ball_merge_left.SetPos(chrono.ChVector3d(0,0,0))
ball_merge_left.SetFixed(False)
truck.GetSystem().Add(ball_merge_left)
# initialize the ball to represent the ball from right hand side parallel case
ball_paral_right = chrono.ChBodyEasySphere(0.4, 1000, True, False)
vis_paral = chrono.ChVisualMaterial()
vis_paral.SetDiffuseColor(chrono.ChColor(0,0,1))
ball_paral_right_shape = ball_paral_right.GetVisualModel().GetShape(0)
ball_paral_right_shape.AddMaterial(vis_paral)
ball_paral_right.SetPos(chrono.ChVector3d(0,0,0))
ball_paral_right.SetFixed(False)
truck.GetSystem().Add(ball_paral_right)
# initialize the ball to represent the ball from left hand side parallel case
ball_paral_left = chrono.ChBodyEasySphere(0.4, 1000, True, False)
vis_paral = chrono.ChVisualMaterial()
vis_paral.SetDiffuseColor(chrono.ChColor(0,0,1))
ball_paral_left_shape = ball_paral_left.GetVisualModel().GetShape(0)
ball_paral_left_shape.AddMaterial(vis_paral)
ball_paral_left.SetPos(chrono.ChVector3d(0,0,0))
ball_paral_left.SetFixed(False)
truck.GetSystem().Add(ball_paral_left)
# initialize the ball to represent the ball from serial case
ball_serial = chrono.ChBodyEasySphere(0.4, 1000, True, False)
vis_serial = chrono.ChVisualMaterial()
vis_serial.SetDiffuseColor(chrono.ChColor(0,1,0))
ball_serial_shape = ball_serial.GetVisualModel().GetShape(0)
ball_serial_shape.AddMaterial(vis_serial)
ball_serial.SetPos(chrono.ChVector3d(0,0,0))
ball_serial.SetFixed(False)
truck.GetSystem().Add(ball_serial)
# -------------------------------------
# Create the vehicle Irrlicht interface
# Create the driver system
# -------------------------------------

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('truck vehicle simulation')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 25.0, 10.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional(
    60,
    60,
    chrono.ChColor(0.5,0.5,0.5),
    chrono.ChColor(0.6,0.6,0.6),
    chrono.ChColor(0.5,0.5,0.5),
)
# vis.AddTypicalLights()
vis.AddSkyBox()
vis.AttachVehicle(truck.GetTractor())


# Create the driver system
driver = veh.ChInteractiveDriverIRR(vis)
# Set the time response for steering and throttle keyboard inputs.
steering_time = 5.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()
# ---------------
# Simulation loop
# ---------------

# output vehicle mass
print( "VEHICLE MASS: ",  truck.GetTractor().GetMass())

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

while vis.Run() :
    time = truck.GetSystem().GetChTime()
    # control sedan vehicle 
    # driver_sedan = straight() # lanechange
    # get trailer and tractor position
    tractor_pos = truck.GetTractorChassisBody().GetPos()
    trailer_pos = truck.GetTrailer().GetChassis().GetBody().GetPos()
    tractor_heading = truck.GetTractorChassisBody().GetRot().GetCardanAnglesZYX().z
    trailer_heading = truck.GetTrailer().GetChassis().GetBody().GetRot().GetCardanAnglesZYX().z

    if time > next_trajectory_time:
        tractor_pos = truck.GetTractorChassisBody().GetPos()
        trailer_pos = truck.GetTrailer().GetChassis().GetBody().GetPos()
        
        # Randomly choose between tractor and trailer position
        if np.random.rand() < 0.5:
            truck_x, truck_y = tractor_pos.x, tractor_pos.y
            position_used = "tractor"
        else:
            truck_x, truck_y = trailer_pos.x, trailer_pos.y
            position_used = "trailer"

        # Randomly choose between merge, parallel, and serial scenarios
        scenario_choice = np.random.rand()
        
        if scenario_choice < 0.4:  # 40% chance for merge
            # Merge scenario
            from_side = 'left' if last_merge_side != 'left' else 'right'
            vel = np.random.uniform(26, 30)
            merge_time = np.random.uniform(2.5, 4.5)
            
            new_trajectory = merging_scenario(reference_trajectory, truck_x, truck_y, 
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
            else:
                merge_trajectory = new_trajectory
                is_merging = True
                merge_traj_ind = 0
            
            last_merge_side = from_side
        elif scenario_choice < 0.8:  # 40% chance for parallel
            # Parallel scenario
            from_side = 'left' if last_parallel_side != 'left' else 'right'
            driver_vel = np.random.uniform(30, 40)
            run_time = np.random.uniform(3.5, 5.5)
            
            new_trajectory = parallel_scenario(reference_trajectory, truck_x, truck_y, 
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
            else:
                parallel_trajectory = new_trajectory
                is_parallel = True
                parallel_traj_ind = 0
            
            last_parallel_side = from_side
        else:  # 20% chance for serial, but only if no merging is active
            if not is_merging and not is_merging_left:
                # Serial scenario
                driver_vel = np.random.uniform(26, 40)
                run_time = np.random.uniform(3.5, 5.5)
                
                serial_trajectory = serial_scenario(reference_trajectory, truck_x, truck_y, 
                                                    vel=driver_vel,
                                                    run_time=run_time,
                                                    freq=25)
                print(f"Serial scenario, velocity: {driver_vel:.2f}, run time: {run_time:.2f}, using {position_used} position")
                print(serial_trajectory.shape)
                
                is_serial = True
                serial_traj_ind = 0
            else:
                print("Serial scenario skipped due to active merging")

        # Set the next trajectory generation time
        last_trajectory_time = time
        next_trajectory_time = time + np.random.uniform(min_pause, max_pause)

    if (step_number % control_steps == 0):
        # for truck controller
        error = error_state(veh_state=[tractor_pos.x,tractor_pos.y,tractor_heading],ref_traj=reference_trajectory,lookahead=3.0)
        steering = sum([x * y for x, y in zip(error, [0.02176878 , 0.72672704 , 0.78409284 ,-0.0105355 ])]) # @hang here is the place to add controller
        driver.SetSteering(steering)
        driver.SetThrottle(1.0)
    
    # Render scene and output POV-Ray data
    if (step_number % render_steps == 0) :
        vis.BeginScene()
        
        # Right merge
        if is_merging and merge_traj_ind < len(merge_trajectory):
            ball_merge_right.SetPos(chrono.ChVector3d(merge_trajectory[merge_traj_ind][0], merge_trajectory[merge_traj_ind][1], 0.5))
            merge_traj_ind += 1
        elif is_merging:
            is_merging = False
            merge_traj_ind = 0
            
        vir_veh_1 = [ball_merge_right.GetPos().x, ball_merge_right.GetPos().y] if is_merging else [0,0]

        # Left merge
        if is_merging_left and merge_traj_ind_left < len(merge_trajectory_left):
            ball_merge_left.SetPos(chrono.ChVector3d(merge_trajectory_left[merge_traj_ind_left][0], merge_trajectory_left[merge_traj_ind_left][1], 0.5))
            merge_traj_ind_left += 1
        elif is_merging_left:
            is_merging_left = False
            merge_traj_ind_left = 0
            
        vir_veh_2 = [ball_merge_left.GetPos().x, ball_merge_left.GetPos().y] if is_merging_left else [0,0]
        # Right parallel
        if is_parallel and parallel_traj_ind < len(parallel_trajectory):
            ball_paral_right.SetPos(chrono.ChVector3d(parallel_trajectory[parallel_traj_ind][0], parallel_trajectory[parallel_traj_ind][1], 0.5))
            parallel_traj_ind += 1
        elif is_parallel:
            is_parallel = False
            parallel_traj_ind = 0
        
        vir_veh_3 = [ball_paral_right.GetPos().x, ball_paral_right.GetPos().y] if is_parallel else [0,0]

        # Left parallel
        if is_parallel_left and parallel_traj_ind_left < len(parallel_trajectory_left):
            ball_paral_left.SetPos(chrono.ChVector3d(parallel_trajectory_left[parallel_traj_ind_left][0], parallel_trajectory_left[parallel_traj_ind_left][1], 0.5))
            parallel_traj_ind_left += 1
        elif is_parallel_left:
            is_parallel_left = False
            parallel_traj_ind_left = 0

        vir_veh_4 = [ball_paral_left.GetPos().x, ball_paral_left.GetPos().y] if is_parallel_left else [0,0]
        # Serial
        if is_serial and serial_traj_ind < len(serial_trajectory):
            ball_serial.SetPos(chrono.ChVector3d(serial_trajectory[serial_traj_ind][0], serial_trajectory[serial_traj_ind][1], 0.5))
            serial_traj_ind += 1
        elif is_serial:
            is_serial = False
            serial_traj_ind = 0
            
        vir_veh_5 = [ball_serial.GetPos().x, ball_serial.GetPos().y] if is_serial else [0,0]
        # write virtual vehicles position and truck state into csv file
        with open('./data/training_data/train_data.csv', 'a') as f:
            writer = csv.writer(f)
            writer.writerow([time, tractor_pos.x, tractor_pos.y, tractor_heading,tractor_heading-trailer_heading, vir_veh_1[0], vir_veh_1[1], vir_veh_2[0], vir_veh_2[1], vir_veh_3[0], vir_veh_3[1], vir_veh_4[0], vir_veh_4[1], vir_veh_5[0], vir_veh_5[1]])
        
        

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
    truck.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    truck.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)

