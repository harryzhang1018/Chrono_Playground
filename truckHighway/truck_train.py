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
from merge_scenario import merging_scenario,parallel_scenario
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
reference_trajectory = np.loadtxt('../trajectory.csv', delimiter=',')
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

# initialize the ball to represent the truck
ball_merge = chrono.ChBodyEasySphere(0.4, 1000, True, False)
vis_merge = chrono.ChVisualMaterial()
vis_merge.SetDiffuseColor(chrono.ChColor(1,0,0))
ball_merge_shape = ball_merge.GetVisualModel().GetShape(0)
ball_merge_shape.AddMaterial(vis_merge)
ball_merge.SetPos(chrono.ChVector3d(0,0,0))
ball_merge.SetFixed(False)
truck.GetSystem().Add(ball_merge)

ball_paral = chrono.ChBodyEasySphere(0.4, 1000, True, False)
vis_paral = chrono.ChVisualMaterial()
vis_paral.SetDiffuseColor(chrono.ChColor(0,0,1))
ball_paral_shape = ball_paral.GetVisualModel().GetShape(0)
ball_paral_shape.AddMaterial(vis_paral)
ball_paral.SetPos(chrono.ChVector3d(0,0,0))
ball_paral.SetFixed(False)
truck.GetSystem().Add(ball_paral)
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
merge_trajectory = []
merge_traj_ind = 0

is_parallel = False
parallel_trajectory = []
parallel_traj_ind = 0

while vis.Run() :
    time = truck.GetSystem().GetChTime()
    # control sedan vehicle 
    # driver_sedan = straight() # lanechange
    # get trailer and tractor position
    tractor_pos = truck.GetTractorChassisBody().GetPos()
    trailer_pos = truck.GetTrailer().GetChassis().GetBody().GetPos()
    tractor_heading = truck.GetTractorChassisBody().GetRot().GetCardanAnglesZYX().z
    trailer_heading = truck.GetTrailer().GetChassis().GetBody().GetRot().GetCardanAnglesZYX().z
    if (not is_merging) and time > 10.0:
        # merging case start after 10 seconds at certain probability
        # generate random number between 0 and 1
        random_number = np.random.rand()
        print(random_number)
        if random_number < 0.05:
            print("merging")
            truck_x = tractor_pos.x
            truck_y = tractor_pos.y
            
            # Randomize parameters
            from_left_right = np.random.choice(['left', 'right'])
            vel = np.random.uniform(26, 40)
            merge_time = np.random.uniform(2.5, 7.5)
            
            merge_trajectory = merging_scenario(reference_trajectory, truck_x, truck_y, 
                                                from_left_right=from_left_right,
                                                vel=vel,
                                                merge_time=merge_time,
                                                freq=25)
            print(f"Merging from {from_left_right}, velocity: {vel:.2f}, merge time: {merge_time:.2f}")
            print(merge_trajectory.shape)
            is_merging = True
    # add parallel case
    if (not is_parallel):
        random_number = np.random.rand()
        if random_number < 0.01:
            print("parallel")
            truck_x = tractor_pos.x
            truck_y = tractor_pos.y
            left_or_right = np.random.choice(['left', 'right'])
            driver_vel = np.random.uniform(26, 40)
            run_time = np.random.uniform(3.5, 8.5)
            parallel_trajectory = parallel_scenario(reference_trajectory, truck_x, truck_y, from_left_right=left_or_right,vel=driver_vel,run_time=run_time,freq=25)
            print(f"Parallel from {left_or_right}, velocity: {driver_vel:.2f}, merge time: {run_time:.2f}")
            print(parallel_trajectory.shape)
            is_parallel = True
            
    if (step_number % control_steps == 0):
        # for truck controller
        error = error_state(veh_state=[tractor_pos.x,tractor_pos.y,tractor_heading],ref_traj=reference_trajectory,lookahead=3.0)
        steering = sum([x * y for x, y in zip(error, [0.02176878 , 0.72672704 , 0.78409284 ,-0.0105355 ])]) 
        driver.SetSteering(steering)
        driver.SetThrottle(1.0)
    
    # Render scene and output POV-Ray data
    if (step_number % render_steps == 0) :
        vis.BeginScene()
        if merge_traj_ind < len(merge_trajectory) and is_merging:
            #print('modify the position of the ball')
            ball_merge.SetPos(chrono.ChVector3d(merge_trajectory[merge_traj_ind][0],merge_trajectory[merge_traj_ind][1],0.5))
            merge_traj_ind += 1
        else:
            is_merging = False
            merge_traj_ind = 0
        if parallel_traj_ind < len(parallel_trajectory) and is_parallel:
            ball_paral.SetPos(chrono.ChVector3d(parallel_trajectory[parallel_traj_ind][0],parallel_trajectory[parallel_traj_ind][1],0.5))
            parallel_traj_ind += 1
        else:
            is_parallel = False
            parallel_traj_ind = 0
        vis.Render()
        vis.EndScene()
        filename = './prototype/img_' + str(render_frame) +'.jpg' 
        vis.WriteImageToFile(filename)
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

