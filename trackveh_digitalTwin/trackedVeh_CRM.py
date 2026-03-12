import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.fsi as fsi
import pychrono.vsg3d as vsg
import math

veh.ChWorldFrame.SetYUP()


class MySimpleTank:
    def __init__(self, sys, start_pos=chrono.ChVector3d(0, 0.5, 0)):
        self.throttleL = 0.0  # actual value 0...1 of gas throttle (left).
        self.throttleR = 0.0  # actual value 0...1 of gas throttle (right).
        self.max_motor_speed = 10.0  # the max rotation speed of the motor [rads/s]

        self.truss = None
        self.wheelRF = None
        self.link_revoluteRF = None
        self.wheelLF = None
        self.link_revoluteLF = None
        self.wheelRB = None
        self.link_motorRB = None
        self.wheelLB = None
        self.link_motorLB = None

        self.wheels = []
        self.shoes = []

        self._build_tank(sys, start_pos)

    def _build_tank(self, sys, start_pos):
        my = 0.5  # left back hub pos
        mx = 0

        self.shoelength = 0.2
        shoemass = 2
        radiustrack = 0.31
        wheeldiameter = 0.280 * 2
        nwrap = 6
        ntiles = 7
        self.track_width = 1.20
        passo = (ntiles + 1) * self.shoelength

        cyl_displA = chrono.ChVector3d(0, 0.075 + 0.02, 0)
        cyl_displB = chrono.ChVector3d(0, -0.075 - 0.02, 0)
        cyl_thickness = 0.09

        self.truss = chrono.ChBodyEasyMesh(
            chrono.GetChronoDataFile("models/bulldozer/bulldozerB10.obj"),
            1000,
            False,
            True,
            False,
            None,
            0
        )
        sys.Add(self.truss)
        self.truss.SetPos(chrono.ChVector3d(mx + passo / 2, my + radiustrack, self.track_width / 2))
        self.truss.SetMass(350)
        self.truss.SetInertiaXX(chrono.ChVector3d(13.8, 13.5, 10))

        wheel_mat = chrono.ChContactMaterialNSC()
        wheel_mat.SetFriction(1.0)

        wheel_mat_vis = chrono.ChVisualMaterial()
        wheel_mat_vis.SetDiffuseColor(chrono.ChColor(0.2, 0.2, 0.2))

        wheel_shape = chrono.ChCollisionShapeCylinder(wheel_mat, wheeldiameter / 2, cyl_thickness)

        self.wheelRF = chrono.ChBodyEasyMesh(
            chrono.GetChronoDataFile("models/bulldozer/wheel_view.obj"),
            1000,
            False,
            True,
            False,
            None,
            0
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
        self.wheels.append(self.wheelRF)

        self.link_revoluteRF = chrono.ChLinkLockRevolute()
        self.link_revoluteRF.Initialize(
            self.wheelRF, self.truss,
            chrono.ChFramed(chrono.ChVector3d(mx + passo, my + radiustrack, 0), chrono.QUNIT)
        )
        sys.AddLink(self.link_revoluteRF)

        self.wheelLF = chrono.ChBodyEasyMesh(
            chrono.GetChronoDataFile("models/bulldozer/wheel_view.obj"),
            1000,
            False,
            True,
            False,
            None,
            0
        )
        sys.Add(self.wheelLF)
        self.wheelLF.SetPos(chrono.ChVector3d(mx + passo, my + radiustrack, self.track_width))
        self.wheelLF.SetRot(chrono.QuatFromAngleX(chrono.CH_PI / 2))
        self.wheelLF.SetMass(9.0)
        self.wheelLF.SetInertiaXX(chrono.ChVector3d(1.2, 1.2, 1.2))
        self.wheelLF.AddCollisionShape(wheel_shape, chrono.ChFramed(cyl_displA, chrono.QuatFromAngleX(chrono.CH_PI_2)))
        self.wheelLF.AddCollisionShape(wheel_shape, chrono.ChFramed(cyl_displB, chrono.QuatFromAngleX(chrono.CH_PI_2)))
        self.wheelLF.EnableCollision(True)
        self.wheelLF.GetVisualShape(0).SetMaterial(0, wheel_mat_vis)
        self.wheels.append(self.wheelLF)

        self.link_revoluteLF = chrono.ChLinkLockRevolute()
        self.link_revoluteLF.Initialize(
            self.wheelLF, self.truss,
            chrono.ChFramed(chrono.ChVector3d(mx + passo, my + radiustrack, self.track_width), chrono.QUNIT)
        )
        sys.AddLink(self.link_revoluteLF)

        self.wheelRB = chrono.ChBodyEasyMesh(
            chrono.GetChronoDataFile("models/bulldozer/wheel_view.obj"),
            1000,
            False,
            True,
            False,
            None,
            0
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
        self.wheels.append(self.wheelRB)

        self.link_motorRB = chrono.ChLinkMotorRotationSpeed()
        self.speedfun_RB = chrono.ChFunctionConst(0)
        self.link_motorRB.SetSpeedFunction(self.speedfun_RB)
        self.link_motorRB.Initialize(
            self.wheelRB, self.truss,
            chrono.ChFramed(chrono.ChVector3d(mx, my + radiustrack, 0), chrono.QUNIT)
        )
        sys.AddLink(self.link_motorRB)

        self.wheelLB = chrono.ChBodyEasyMesh(
            chrono.GetChronoDataFile("models/bulldozer/wheel_view.obj"),
            1000,
            False,
            True,
            False,
            None,
            0
        )
        sys.Add(self.wheelLB)
        self.wheelLB.SetPos(chrono.ChVector3d(mx, my + radiustrack, self.track_width))
        self.wheelLB.SetRot(chrono.QuatFromAngleX(chrono.CH_PI / 2))
        self.wheelLB.SetMass(9.0)
        self.wheelLB.SetInertiaXX(chrono.ChVector3d(1.2, 1.2, 1.2))
        self.wheelLB.AddCollisionShape(wheel_shape, chrono.ChFramed(cyl_displA, chrono.QuatFromAngleX(chrono.CH_PI_2)))
        self.wheelLB.AddCollisionShape(wheel_shape, chrono.ChFramed(cyl_displB, chrono.QuatFromAngleX(chrono.CH_PI_2)))
        self.wheelLB.EnableCollision(True)
        self.wheelLB.GetVisualShape(0).SetMaterial(0, wheel_mat_vis)
        self.wheels.append(self.wheelLB)

        self.link_motorLB = chrono.ChLinkMotorRotationSpeed()
        self.speedfun_LB = chrono.ChFunctionConst(0)
        self.link_motorLB.SetSpeedFunction(self.speedfun_LB)
        self.link_motorLB.Initialize(
            self.wheelLB, self.truss,
            chrono.ChFramed(chrono.ChVector3d(mx, my + radiustrack, self.track_width), chrono.QUNIT)
        )
        sys.AddLink(self.link_motorLB)

        shoe_trimesh = chrono.ChTriangleMeshConnected()
        shoe_trimesh.LoadWavefrontMesh(chrono.GetChronoDataFile("models/bulldozer/shoe_view.obj"))

        shoe_vis_mesh = chrono.ChVisualShapeTriangleMesh()
        shoe_vis_mesh.SetMesh(shoe_trimesh)
        shoe_vis_mesh.SetVisible(True)

        shoe_coll_trimesh = chrono.ChTriangleMeshConnected()
        shoe_coll_trimesh.LoadWavefrontMesh(chrono.GetChronoDataFile("models/bulldozer/shoe_collision.obj"))

        shoe_coll_vis_mesh = chrono.ChVisualShapeTriangleMesh()
        shoe_coll_vis_mesh.SetMesh(shoe_coll_trimesh)
        shoe_coll_vis_mesh.SetVisible(False)

        shoe_mat = chrono.ChContactMaterialNSC()

        mesh_displacement = chrono.ChVector3d(self.shoelength * 0.5, 0, 0)
        joint_displacement = chrono.ChVector3d(-self.shoelength * 0.5, 0, 0)

        for side in range(2):
            mx = 0
            mx += self.shoelength

            mz = 0 if side == 0 else self.track_width

            position = chrono.ChVector3d(mx, my, mz)
            rotation = chrono.QUNIT

            first_shoe = chrono.ChBody()
            sys.Add(first_shoe)
            first_shoe.SetMass(shoemass)
            first_shoe.SetPos(position)
            first_shoe.SetRot(rotation)
            first_shoe.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))

            first_shoe.AddVisualShape(shoe_vis_mesh, chrono.ChFramed(-mesh_displacement, chrono.ChMatrix33d(1)))
            first_shoe.AddVisualShape(shoe_coll_vis_mesh, chrono.ChFramed(-mesh_displacement, chrono.ChMatrix33d(1)))

            shoe_coll_shape = chrono.ChCollisionShapeTriangleMesh(shoe_mat, shoe_coll_trimesh, False, False, 0.005)
            first_shoe.AddCollisionShape(shoe_coll_shape, chrono.ChFramed(mesh_displacement, chrono.QUNIT))
            first_shoe.EnableCollision(True)
            first_shoe.GetCollisionModel().SetFamily(3)
            first_shoe.GetCollisionModel().DisallowCollisionsWith(3)
            self.shoes.append(first_shoe)

            previous_shoe = first_shoe

            for nshoe in range(1, ntiles):
                mx += self.shoelength
                position = chrono.ChVector3d(mx, my, mz)

                shoe = self._make_shoe(
                    previous_shoe, position, rotation, sys,
                    shoe_vis_mesh, shoe_coll_vis_mesh, shoe_coll_shape,
                    mesh_displacement, joint_displacement, shoemass
                )
                previous_shoe = shoe

            for nshoe in range(nwrap):
                alpha = (chrono.CH_PI / (nwrap - 1.0)) * nshoe
                lx = mx + self.shoelength + radiustrack * math.sin(alpha)
                ly = my + radiustrack - radiustrack * math.cos(alpha)
                position = chrono.ChVector3d(lx, ly, mz)
                rotation = chrono.QuatFromAngleZ(alpha)

                shoe = self._make_shoe(
                    previous_shoe, position, rotation, sys,
                    shoe_vis_mesh, shoe_coll_vis_mesh, shoe_coll_shape,
                    mesh_displacement, joint_displacement, shoemass
                )
                previous_shoe = shoe

            for nshoe in range(ntiles - 1, -1, -1):
                position = chrono.ChVector3d(mx, my + 2 * radiustrack, mz)

                shoe = self._make_shoe(
                    previous_shoe, position, rotation, sys,
                    shoe_vis_mesh, shoe_coll_vis_mesh, shoe_coll_shape,
                    mesh_displacement, joint_displacement, shoemass
                )
                previous_shoe = shoe
                mx -= self.shoelength

            for nshoe in range(nwrap):
                alpha = chrono.CH_PI + (chrono.CH_PI / (nwrap - 1.0)) * nshoe
                lx = mx + 0 + radiustrack * math.sin(alpha)
                ly = my + radiustrack - radiustrack * math.cos(alpha)
                position = chrono.ChVector3d(lx, ly, mz)
                rotation = chrono.QuatFromAngleZ(alpha)

                shoe = self._make_shoe(
                    previous_shoe, position, rotation, sys,
                    shoe_vis_mesh, shoe_coll_vis_mesh, shoe_coll_shape,
                    mesh_displacement, joint_displacement, shoemass
                )
                previous_shoe = shoe

            linkpos = first_shoe.TransformPointLocalToParent(joint_displacement)
            link_revolute_shoeshoe = chrono.ChLinkLockRevolute()
            link_revolute_shoeshoe.Initialize(first_shoe, previous_shoe, chrono.ChFramed(linkpos, chrono.QUNIT))
            sys.AddLink(link_revolute_shoeshoe)

    def _make_shoe(
        self,
        previous_shoe,
        position,
        rotation,
        sys,
        shoe_vis_mesh,
        shoe_coll_vis_mesh,
        shoe_coll_shape,
        mesh_displacement,
        joint_displacement,
        shoemass,
    ):
        shoe = chrono.ChBody()
        shoe.SetPos(position)
        shoe.SetRot(rotation)
        shoe.SetMass(shoemass)
        shoe.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))
        sys.Add(shoe)

        shoe.AddVisualShape(shoe_vis_mesh, chrono.ChFramed(-mesh_displacement, chrono.ChMatrix33d(1)))
        shoe.AddVisualShape(shoe_coll_vis_mesh, chrono.ChFramed(-mesh_displacement, chrono.ChMatrix33d(1)))

        shoe.AddCollisionShape(shoe_coll_shape, chrono.ChFramed(mesh_displacement, chrono.QUNIT))
        shoe.EnableCollision(True)
        shoe.GetCollisionModel().SetFamily(3)
        shoe.GetCollisionModel().DisallowCollisionsWith(3)

        if previous_shoe:
            linkpos = shoe.TransformPointLocalToParent(joint_displacement)
            link_revolute = chrono.ChLinkLockRevolute()
            link_revolute.Initialize(shoe, previous_shoe, chrono.ChFramed(linkpos, chrono.QUNIT))
            sys.AddLink(link_revolute)

        self.shoes.append(shoe)
        return shoe


def AddFSITrackedVehicle(tank, terrain):
    wheel_mesh = chrono.GetChronoDataFile("models/bulldozer/wheel_view.obj")
    wheel_geometry = chrono.ChBodyGeometry()
    wheel_geometry.coll_meshes.append(chrono.TrimeshShape(chrono.VNULL, chrono.QUNIT, wheel_mesh, chrono.VNULL))

    for wheel in tank.wheels:
        terrain.AddRigidBody(wheel, wheel_geometry, False)

    shoe_mesh = chrono.GetChronoDataFile("models/bulldozer/shoe_collision.obj")
    shoe_geometry = chrono.ChBodyGeometry()
    shoe_geometry.coll_meshes.append(chrono.TrimeshShape(chrono.VNULL, chrono.QUNIT, shoe_mesh, chrono.VNULL))

    for shoe in tank.shoes:
        terrain.AddRigidBody(shoe, shoe_geometry, False)


def main():
    sys = chrono.ChSystemNSC()
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

    mytank = MySimpleTank(sys)

    # CRM terrain parameters
    spacing = 0.04
    step_size = 5e-4
    active_box_dim = chrono.ChVector3d(0.8, 0.8, 0.8)

    density = 1700
    cohesion = 5e3
    friction = 0.8
    youngs_modulus = 1e6
    poisson_ratio = 0.3

    terrain = veh.CRMTerrain(sys, spacing)
    sysFSI = terrain.GetFsiSystemSPH()
    terrain.SetVerbose(True)
    terrain.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
    terrain.SetStepSizeCFD(step_size)

    mat_props = fsi.ElasticMaterialProperties()
    mat_props.density = density
    mat_props.Young_modulus = youngs_modulus
    mat_props.Poisson_ratio = poisson_ratio
    mat_props.mu_I0 = 0.04
    mat_props.mu_fric_s = friction
    mat_props.mu_fric_2 = friction
    mat_props.average_diam = 0.005
    mat_props.cohesion_coeff = cohesion
    terrain.SetElasticSPH(mat_props)

    sph_params = fsi.SPHParameters()
    sph_params.integration_scheme = fsi.IntegrationScheme_RK2
    sph_params.initial_spacing = spacing
    sph_params.d0_multiplier = 1.2
    sph_params.kernel_threshold = 0.8
    sph_params.artificial_viscosity = 0.5
    sph_params.shifting_method = fsi.ShiftingMethod_PPST
    sph_params.shifting_ppst_push = 3.0
    sph_params.shifting_ppst_pull = 1.0
    sph_params.consistent_gradient_discretization = False
    sph_params.consistent_laplacian_discretization = False
    sph_params.viscosity_method = fsi.ViscosityMethod_ARTIFICIAL_BILATERAL
    sph_params.boundary_method = fsi.BoundaryMethod_ADAMI
    terrain.SetSPHParameters(sph_params)

    AddFSITrackedVehicle(mytank, terrain)

    terrain.SetActiveDomain(active_box_dim)

    terrain_length = 20.0
    terrain_width = 5.0
    terrain_depth = 0.25
    terrain.Construct(
        chrono.ChVector3d(terrain_length, terrain_depth, terrain_width),
        chrono.ChVector3d(-terrain_length/2 + 2.0, 0, -terrain_width/2),
        (fsi.BoxSide_ALL & ~fsi.BoxSide_Y_POS),
    )

    terrain.Initialize()

    aabb = terrain.GetSPHBoundingBox()
    print(f"  SPH particles:     {terrain.GetNumSPHParticles()}")
    print(f"  Bndry BCE markers: {terrain.GetNumBoundaryBCEMarkers()}")
    print(f"  SPH AABB:          {aabb.min}   {aabb.max}")

    render = True
    render_fps = 200
    vis = None
    if render:
        # col_callback = fsi.ParticleHeightColorCallback(aabb.min.y, aabb.max.y)
        visFSI = fsi.ChSphVisualizationVSG(sysFSI)
        visFSI.EnableFluidMarkers(True)
        visFSI.EnableBoundaryMarkers(False)
        visFSI.EnableRigidBodyMarkers(False)
        # visFSI.SetSPHColorCallback(col_callback, chrono.ChColormap.Type_BROWN)

        vis = vsg.ChVisualSystemVSG()
        vis.AttachSystem(sys)
        vis.SetWindowTitle("Tracked vehicle on CRM deformable terrain")
        vis.SetWindowSize(chrono.ChVector2i(1280, 800))
        vis.SetWindowPosition(chrono.ChVector2i(100, 100))
        vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
        vis.AddCamera(
            chrono.ChVector3d(-4, 2.5, -6),
            chrono.ChVector3d(2, 0.5, mytank.track_width / 2),
        )
        vis.SetLightIntensity(1.0)
        vis.SetLightDirection(chrono.CH_PI_2, chrono.CH_PI_4)
        vis.AttachPlugin(visFSI)
        vis.Initialize()

    timestep = step_size
    time = 0.0
    render_frame = 0
    realtime_timer = chrono.ChRealtimeStepTimer()

    while time < 5.0:
        if time < 3.0:
            mytank.throttleL = 0.4 
            mytank.throttleR = 0.4 
        else:
            mytank.throttleL = 0
            mytank.throttleR = 0

        speed_L = mytank.throttleL * mytank.max_motor_speed
        speed_R = mytank.throttleR * mytank.max_motor_speed
        mytank.speedfun_LB.SetConstant(speed_L)
        mytank.speedfun_RB.SetConstant(speed_R)

        if render and time >= render_frame / render_fps:
            if not vis.Run():
                break
            vis.Render()
            render_frame += 1

        terrain.DoStepDynamics(timestep)
        realtime_timer.Spin(timestep)
        time += timestep


if __name__ == "__main__":
    main()
