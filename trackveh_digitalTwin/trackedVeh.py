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
# Authors: Alessandro Tasora
# =============================================================================
#
# Demo code about
# - modeling tracks with articulated shoes (as an example of complex model with
#   collisions and constraints)
# - using different meshes for collision and visualization
# - using clones of collision shapes
# - using DisallowCollisionsWith, SetFamily etc. to avoid
#   collisions between different families of bodies.
#
# =============================================================================

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math


# First of all, define a class for the 'tank' (that is, a set of
# bodies and links which are grouped within this class; so it is
# easier to manage data structures in this example).

class MySimpleTank:
    def __init__(self, sys, start_pos=chrono.ChVector3d(0, 0.5, 0)):
        # THE DATA
        
        self.throttleL = 0.0  # actual value 0...1 of gas throttle (left).
        self.throttleR = 0.0  # actual value 0...1 of gas throttle (right).
        self.max_motor_speed = 10.0  # the max rotation speed of the motor [rads/s]
        
        # The parts making the tank
        # .. truss:
        self.truss = None
        # .. right front suspension:
        self.wheelRF = None
        self.link_revoluteRF = None
        # .. left front suspension:
        self.wheelLF = None
        self.link_revoluteLF = None
        # .. right back suspension:
        self.wheelRB = None
        self.link_motorRB = None
        # .. left back suspension:
        self.wheelLB = None
        self.link_motorLB = None
        
        # Build and initialize the tank
        self._build_tank(sys, start_pos)
    
    def _build_tank(self, sys, start_pos):
        """Build and initialize the tank, creating all bodies and constraints."""
        
        my = 0.5  # left back hub pos
        mx = 0
        
        shoelength = 0.2
        shoemass = 2
        radiustrack = 0.31
        wheeldiameter = 0.280 * 2
        nwrap = 6
        ntiles = 7
        rlwidth = 1.20
        passo = (ntiles + 1) * shoelength
        
        cyl_displA = chrono.ChVector3d(0, 0.075 + 0.02, 0)
        cyl_displB = chrono.ChVector3d(0, -0.075 - 0.02, 0)
        cyl_thickness = 0.09
        
        # --- The tank body ---
        
        self.truss = chrono.ChBodyEasyMesh(
            chrono.GetChronoDataFile("models/bulldozer/bulldozerB10.obj"),
            1000,    # density
            False,   # compute mass automatically?
            True,    # visualization?
            False,   # collision?
            None,    # no contact material
            0        # mesh sweep sphere radius
        )
        sys.Add(self.truss)
        self.truss.SetPos(chrono.ChVector3d(mx + passo / 2, my + radiustrack, rlwidth / 2))
        self.truss.SetMass(350)
        self.truss.SetInertiaXX(chrono.ChVector3d(13.8, 13.5, 10))
        
        # --- Contact and visualization materials for wheels ---
        
        wheel_mat = chrono.ChContactMaterialNSC()
        wheel_mat.SetFriction(1.0)
        
        wheel_mat_vis = chrono.ChVisualMaterial()
        wheel_mat_vis.SetDiffuseColor(chrono.ChColor(0.2, 0.2, 0.2))
        
        # --- Wheel collision shape
        
        wheel_shape = chrono.ChCollisionShapeCylinder(wheel_mat, wheeldiameter / 2, cyl_thickness)
        
        # --- Right Front suspension ---
        
        # ..the tank right-front wheel
        self.wheelRF = chrono.ChBodyEasyMesh(
            chrono.GetChronoDataFile("models/bulldozer/wheel_view.obj"),
            1000,    # density
            False,   # compute mass automatically?
            True,    # visualization?
            False,   # collision?
            None,    # no contact material
            0        # mesh sweep sphere radius
        )
        sys.Add(self.wheelRF)
        self.wheelRF.SetPos(chrono.ChVector3d(mx + passo, my + radiustrack, 0))
        self.wheelRF.SetRot(chrono.QuatFromAngleX(chrono.CH_PI / 2))
        self.wheelRF.SetMass(9.0)
        self.wheelRF.SetInertiaXX(chrono.ChVector3d(1.2, 1.2, 1.2))
        
        self.wheelRF.AddCollisionShape(wheel_shape, chrono.ChFramed(cyl_displA, chrono.QuatFromAngleX(chrono.CH_PI_2)))
        self.wheelRF.AddCollisionShape(wheel_shape, chrono.ChFramed(cyl_displB, chrono.QuatFromAngleX(chrono.CH_PI_2)))
        self.wheelRF.EnableCollision(True)
        
        self.wheelRF.GetVisualShape(0).SetMaterial(0, wheel_mat_vis)
        
        # .. create the revolute joint between the wheel and the truss
        self.link_revoluteRF = chrono.ChLinkLockRevolute()
        self.link_revoluteRF.Initialize(self.wheelRF, self.truss,
                                        chrono.ChFramed(chrono.ChVector3d(mx + passo, my + radiustrack, 0), chrono.QUNIT))
        sys.AddLink(self.link_revoluteRF)
        
        # --- Left Front suspension ---
        
        # ..the tank left-front wheel
        self.wheelLF = chrono.ChBodyEasyMesh(
            chrono.GetChronoDataFile("models/bulldozer/wheel_view.obj"),
            1000,    # density
            False,   # compute mass automatically?
            True,    # visualization?
            False,   # collision?
            None,    # no contact material
            0        # mesh sweep sphere radius
        )
        sys.Add(self.wheelLF)
        self.wheelLF.SetPos(chrono.ChVector3d(mx + passo, my + radiustrack, rlwidth))
        self.wheelLF.SetRot(chrono.QuatFromAngleX(chrono.CH_PI / 2))
        self.wheelLF.SetMass(9.0)
        self.wheelLF.SetInertiaXX(chrono.ChVector3d(1.2, 1.2, 1.2))
        
        self.wheelLF.AddCollisionShape(wheel_shape, chrono.ChFramed(cyl_displA, chrono.QuatFromAngleX(chrono.CH_PI_2)))
        self.wheelLF.AddCollisionShape(wheel_shape, chrono.ChFramed(cyl_displB, chrono.QuatFromAngleX(chrono.CH_PI_2)))
        self.wheelLF.EnableCollision(True)
        
        self.wheelLF.GetVisualShape(0).SetMaterial(0, wheel_mat_vis)
        
        # .. create the revolute joint between the wheel and the truss
        self.link_revoluteLF = chrono.ChLinkLockRevolute()
        self.link_revoluteLF.Initialize(self.wheelLF, self.truss,
                                        chrono.ChFramed(chrono.ChVector3d(mx + passo, my + radiustrack, rlwidth), chrono.QUNIT))
        sys.AddLink(self.link_revoluteLF)
        
        # --- Right Back suspension ---
        
        # ..the tank right-back wheel
        self.wheelRB = chrono.ChBodyEasyMesh(
            chrono.GetChronoDataFile("models/bulldozer/wheel_view.obj"),
            1000,    # density
            False,   # compute mass automatically?
            True,    # visualization?
            False,   # collision?
            None,    # no contact material
            0        # mesh sweep sphere radius
        )
        sys.Add(self.wheelRB)
        self.wheelRB.SetPos(chrono.ChVector3d(mx, my + radiustrack, 0))
        self.wheelRB.SetRot(chrono.QuatFromAngleX(chrono.CH_PI / 2))
        self.wheelRB.SetMass(9.0)
        self.wheelRB.SetInertiaXX(chrono.ChVector3d(1.2, 1.2, 1.2))
        
        self.wheelRB.AddCollisionShape(wheel_shape, chrono.ChFramed(cyl_displA, chrono.QuatFromAngleX(chrono.CH_PI_2)))
        self.wheelRB.AddCollisionShape(wheel_shape, chrono.ChFramed(cyl_displB, chrono.QuatFromAngleX(chrono.CH_PI_2)))
        self.wheelRB.EnableCollision(True)
        
        self.wheelRB.GetVisualShape(0).SetMaterial(0, wheel_mat_vis)
        
        # .. create the motor joint between the wheel and the truss
        self.link_motorRB = chrono.ChLinkMotorRotationSpeed()
        self.speedfun_RB = chrono.ChFunctionConst(0)  # Store reference to function
        self.link_motorRB.SetSpeedFunction(self.speedfun_RB)
        self.link_motorRB.Initialize(self.wheelRB, self.truss,
                                     chrono.ChFramed(chrono.ChVector3d(mx, my + radiustrack, 0), chrono.QUNIT))
        sys.AddLink(self.link_motorRB)
        
        # --- Left Back suspension ---
        
        # ..the tank left-back wheel
        self.wheelLB = chrono.ChBodyEasyMesh(
            chrono.GetChronoDataFile("models/bulldozer/wheel_view.obj"),
            1000,    # density
            False,   # compute mass automatically?
            True,    # visualization?
            False,   # collision?
            None,    # no contact material
            0        # mesh sweep sphere radius
        )
        sys.Add(self.wheelLB)
        self.wheelLB.SetPos(chrono.ChVector3d(mx, my + radiustrack, rlwidth))
        self.wheelLB.SetRot(chrono.QuatFromAngleX(chrono.CH_PI / 2))
        self.wheelLB.SetMass(9.0)
        self.wheelLB.SetInertiaXX(chrono.ChVector3d(1.2, 1.2, 1.2))
        
        self.wheelLB.AddCollisionShape(wheel_shape, chrono.ChFramed(cyl_displA, chrono.QuatFromAngleX(chrono.CH_PI_2)))
        self.wheelLB.AddCollisionShape(wheel_shape, chrono.ChFramed(cyl_displB, chrono.QuatFromAngleX(chrono.CH_PI_2)))
        self.wheelLB.EnableCollision(True)
        
        self.wheelLB.GetVisualShape(0).SetMaterial(0, wheel_mat_vis)
        
        # .. create the motor joint between the wheel and the truss
        self.link_motorLB = chrono.ChLinkMotorRotationSpeed()
        self.speedfun_LB = chrono.ChFunctionConst(0)  # Store reference to function
        self.link_motorLB.SetSpeedFunction(self.speedfun_LB)
        self.link_motorLB.Initialize(self.wheelLB, self.truss,
                                     chrono.ChFramed(chrono.ChVector3d(mx, my + radiustrack, rlwidth), chrono.QUNIT))
        sys.AddLink(self.link_motorLB)
        
        # --- TRACKS ---
        
        # Shared visualization model
        shoe_trimesh = chrono.ChTriangleMeshConnected()
        shoe_trimesh.LoadWavefrontMesh(chrono.GetChronoDataFile("models/bulldozer/shoe_view.obj"))
        
        shoe_vis_mesh = chrono.ChVisualShapeTriangleMesh()
        shoe_vis_mesh.SetMesh(shoe_trimesh)
        shoe_vis_mesh.SetVisible(True)
        
        # Shared collision mesh
        shoe_coll_trimesh = chrono.ChTriangleMeshConnected()
        shoe_coll_trimesh.LoadWavefrontMesh(chrono.GetChronoDataFile("models/bulldozer/shoe_collision.obj"))
        
        shoe_coll_vis_mesh = chrono.ChVisualShapeTriangleMesh()
        shoe_coll_vis_mesh.SetMesh(shoe_coll_trimesh)
        shoe_coll_vis_mesh.SetVisible(False)
        
        # Contact material for shoes
        shoe_mat = chrono.ChContactMaterialNSC()
        
        # Mesh and joint displacements (as mesh origin is not in body center of mass)
        mesh_displacement = chrono.ChVector3d(shoelength * 0.5, 0, 0)
        joint_displacement = chrono.ChVector3d(-shoelength * 0.5, 0, 0)
        
        # Build tracks for both sides
        for side in range(2):
            mx = 0
            mx += shoelength
            
            mz = 0 if side == 0 else rlwidth
            
            # Create first shoe
            position = chrono.ChVector3d(mx, my, mz)
            rotation = chrono.QUNIT
            
            first_shoe = chrono.ChBody()
            sys.Add(first_shoe)
            first_shoe.SetMass(shoemass)
            first_shoe.SetPos(position)
            first_shoe.SetRot(rotation)
            first_shoe.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))
            
            # Add visualization
            first_shoe.AddVisualShape(shoe_vis_mesh, chrono.ChFramed(-mesh_displacement, chrono.ChMatrix33d(1)))
            first_shoe.AddVisualShape(shoe_coll_vis_mesh, chrono.ChFramed(-mesh_displacement, chrono.ChMatrix33d(1)))
            
            # Add collision
            shoe_coll_shape = chrono.ChCollisionShapeTriangleMesh(shoe_mat, shoe_coll_trimesh, False, False, 0.005)
            first_shoe.AddCollisionShape(shoe_coll_shape, chrono.ChFramed(mesh_displacement, chrono.QUNIT))
            first_shoe.EnableCollision(True)
            
            # Avoid collision with neighboring shoes
            first_shoe.GetCollisionModel().SetFamily(3)
            first_shoe.GetCollisionModel().DisallowCollisionsWith(3)
            
            previous_shoe = first_shoe
            
            # Bottom straight section
            for nshoe in range(1, ntiles):
                mx += shoelength
                position = chrono.ChVector3d(mx, my, mz)
                
                shoe = self._make_shoe(previous_shoe, position, rotation, sys, 
                                      shoe_vis_mesh, shoe_coll_vis_mesh, shoe_coll_shape,
                                      mesh_displacement, joint_displacement, shoemass)
                previous_shoe = shoe
            
            # Bottom curved section (wrap around rear wheel)
            for nshoe in range(nwrap):
                alpha = (chrono.CH_PI / (nwrap - 1.0)) * nshoe
                lx = mx + shoelength + radiustrack * math.sin(alpha)
                ly = my + radiustrack - radiustrack * math.cos(alpha)
                position = chrono.ChVector3d(lx, ly, mz)
                rotation = chrono.QuatFromAngleZ(alpha)
                
                shoe = self._make_shoe(previous_shoe, position, rotation, sys,
                                      shoe_vis_mesh, shoe_coll_vis_mesh, shoe_coll_shape,
                                      mesh_displacement, joint_displacement, shoemass)
                previous_shoe = shoe
            
            # Top straight section
            for nshoe in range(ntiles - 1, -1, -1):
                position = chrono.ChVector3d(mx, my + 2 * radiustrack, mz)
                
                shoe = self._make_shoe(previous_shoe, position, rotation, sys,
                                      shoe_vis_mesh, shoe_coll_vis_mesh, shoe_coll_shape,
                                      mesh_displacement, joint_displacement, shoemass)
                previous_shoe = shoe
                mx -= shoelength
            
            # Top curved section (wrap around front wheel)
            for nshoe in range(nwrap):
                alpha = chrono.CH_PI + (chrono.CH_PI / (nwrap - 1.0)) * nshoe
                lx = mx + 0 + radiustrack * math.sin(alpha)
                ly = my + radiustrack - radiustrack * math.cos(alpha)
                position = chrono.ChVector3d(lx, ly, mz)
                rotation = chrono.QuatFromAngleZ(alpha)
                
                shoe = self._make_shoe(previous_shoe, position, rotation, sys,
                                      shoe_vis_mesh, shoe_coll_vis_mesh, shoe_coll_shape,
                                      mesh_displacement, joint_displacement, shoemass)
                previous_shoe = shoe
            
            # Close the track loop
            linkpos = first_shoe.TransformPointLocalToParent(joint_displacement)
            link_revolute_shoeshoe = chrono.ChLinkLockRevolute()
            link_revolute_shoeshoe.Initialize(first_shoe, previous_shoe, chrono.ChFramed(linkpos, chrono.QUNIT))
            sys.AddLink(link_revolute_shoeshoe)
    
    def _make_shoe(self, previous_shoe, position, rotation, sys,
                   shoe_vis_mesh, shoe_coll_vis_mesh, shoe_coll_shape,
                   mesh_displacement, joint_displacement, shoemass):
        """Create a track shoe and connect it to the previous one."""
        shoe = chrono.ChBody()
        shoe.SetPos(position)
        shoe.SetRot(rotation)
        shoe.SetMass(shoemass)
        shoe.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))
        sys.Add(shoe)
        
        # Add visualization
        shoe.AddVisualShape(shoe_vis_mesh, chrono.ChFramed(-mesh_displacement, chrono.ChMatrix33d(1)))
        shoe.AddVisualShape(shoe_coll_vis_mesh, chrono.ChFramed(-mesh_displacement, chrono.ChMatrix33d(1)))
        
        # Add collision
        shoe.AddCollisionShape(shoe_coll_shape, chrono.ChFramed(mesh_displacement, chrono.QUNIT))
        shoe.EnableCollision(True)
        
        # Avoid collision with neighboring shoes
        shoe.GetCollisionModel().SetFamily(3)
        shoe.GetCollisionModel().DisallowCollisionsWith(3)
        
        # Create revolute joint with previous shoe
        if previous_shoe:
            linkpos = shoe.TransformPointLocalToParent(joint_displacement)
            link_revolute = chrono.ChLinkLockRevolute()
            link_revolute.Initialize(shoe, previous_shoe, chrono.ChFramed(linkpos, chrono.QUNIT))
            sys.AddLink(link_revolute)
        
        return shoe


# Main program
def main():
    print("Copyright (c) 2017 projectchrono.org")
    
    # Create a Chrono physical system
    sys = chrono.ChSystemNSC()
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    
    # Create the world
    ground_mat = chrono.ChContactMaterialNSC()
    ground_mat.SetFriction(1.0)
    
    my_ground = chrono.ChBodyEasyBox(60, 2, 60, 1000, True, True, ground_mat)
    my_ground.SetPos(chrono.ChVector3d(0, -1, 0))
    my_ground.SetFixed(True)
    my_ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
    sys.Add(my_ground)
    
    # Create some obstacles on the ground
    obst_mat = chrono.ChContactMaterialNSC()
    
    for i in range(50):
        obstacle = chrono.ChBodyEasyBox(
            0.6 * chrono.ChRandom.Get(), 
            0.6 * chrono.ChRandom.Get(), 
            0.6 * chrono.ChRandom.Get(),
            1000, True, True, obst_mat)
        obstacle.SetPos(chrono.ChVector3d(
            20 * (chrono.ChRandom.Get() - 0.5),
            2 * chrono.ChRandom.Get(),
            20 * (chrono.ChRandom.Get() - 0.5)))
        sys.Add(obstacle)
    
    # Create the tank
    mytank = MySimpleTank(sys)
    
    # Create the Irrlicht visualization
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(800, 600)
    vis.SetWindowTitle('Modeling a simplified tracked vehicle')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_chrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0, 0, -6), chrono.ChVector3d(-2, 2, 0))
    vis.AddTypicalLights()
    
    # Solver settings
    sys.SetSolverType(chrono.ChSolver.Type_PSOR)
    sys.GetSolver().AsIterative().SetMaxIterations(100)
    
    # Simulation loop
    timestep = 0.03
    realtime_timer = chrono.ChRealtimeStepTimer()
    
    # Simple time-based throttle control for demonstration
    # In a real application, you would connect this to user input
    time = 0.0
    
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        
        # Example: vary throttle over time to make the tank move in a pattern
        # Modify these values to control the tank's motion
        mytank.throttleL = 0.5 + 0.2 * math.sin(time * 0.5)
        mytank.throttleR = 0.5 + 0.2 * math.cos(time * 0.5)
        
        # Update motor speeds based on throttle
        speed_L = mytank.throttleL * mytank.max_motor_speed
        speed_R = mytank.throttleR * mytank.max_motor_speed
        
        # Set motor speeds by updating the stored function objects
        mytank.speedfun_LB.SetConstant(speed_L)
        mytank.speedfun_RB.SetConstant(speed_R)
        
        sys.DoStepDynamics(timestep)
        realtime_timer.Spin(timestep)
        time += timestep


if __name__ == "__main__":
    main()