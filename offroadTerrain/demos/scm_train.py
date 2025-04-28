import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math

import sys,os
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(project_root)
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))
# =============================================================================

from utils.custom_driver import MyDriver


def main():
    # Simulation step sizes
    step_size = 2e-3
    # SCM patch dimensions
    terrainLength = 250.0  # size in X direction
    terrainWidth = 150.0    # size in Y direction
    # vehicle initial position
    init_x , init_y = -terrainLength/2+5, 0
    # SCM grid spacing
    delta = 0.05
    #  Create the HMMWV vehicle, set parameters, and initialize
    vehicle = veh.HMMWV_Full()
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(init_x, init_y, 0.6), chrono.ChQuaterniond(1, 0, 0, 0)))
    # vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
    # vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    # vehicle.SetDriveType(veh.DrivelineTypeWV_FWD)
    vehicle.SetEngineType(veh.EngineModelType_SIMPLE_MAP)
    vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    vehicle.SetDriveType(veh.DrivelineTypeWV_FWD)
    vehicle.SetTireType(veh.TireModelType_RIGID_MESH)
    vehicle.Initialize()

    vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

    vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # Create the (custom) driver
    driver = MyDriver(vehicle.GetVehicle())
    driver.Initialize()

    # Create the SCM deformable terrain patch
	## @TODO: Add randome terrain parameter function
    terrain = veh.SCMTerrain(vehicle.GetSystem())
    terrain.SetSoilParameters(2e6,   # Bekker Kphi
                              0,     # Bekker Kc
                              1.1,   # Bekker n exponent
                              0,     # Mohr cohesive limit (Pa)
                              30,    # Mohr friction limit (degrees)
                              0.01,  # Janosi shear coefficient (m)
                              2e8,   # Elastic stiffness (Pa/m), before plastic yield
                              3e4    # Damping (Pa s/m), proportional to negative vertical speed (optional)
    )

    # Optionally, enable moving patch feature (single patch around vehicle chassis)
    terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))

    # Set plot type for SCM (false color plotting)
    terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)
    # Initialize the SCM terrain, specifying the initial mesh grid
    terrain.Initialize(terrainLength, terrainWidth, delta)

    # Create the vehicle Irrlicht interface
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('HMMWV Deformable Soil Demo')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(vehicle.GetVehicle())

    # set render step size
    render_step_size = 1.0 / 20  # FPS = 20
    render_steps = math.ceil(render_step_size / step_size)
	# set logging step size
    log_step_size = 1.0 / 50  # FPS = 50
    log_steps = math.ceil(log_step_size / step_size)
	
    sim_steps = 0
    sim_end_time = 30
    # Simulation loop
    while vis.Run() :
        time = vehicle.GetSystem().GetChTime()

        # End simulation
        if (time >= sim_end_time):
            break
        if (sim_steps % render_steps == 0):  
            # Draw scene
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # Get driver inputs
        driver_inputs = driver.GetInputs()


        # Update modules (process inputs from other modules)
        driver.Synchronize(time)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        # Advance simulation for one timestep for all modules
        driver.Advance(step_size)
        terrain.Advance(step_size)
        vehicle.Advance(step_size)
        vis.Advance(step_size)

        sim_steps += 1
    return 0
  

veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')



main()
