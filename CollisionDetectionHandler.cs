using System;
using System.Diagnostics;
using System.Numerics;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Memory;

namespace WpfCollision
{
    public delegate void CollisionRegisteredDelegate(string parameter);
    /// <summary>
    /// collision detection happens here. Similar to https://github.com/bepu/bepuphysics2/blob/master/Demos/Demos/SimpleSelfContainedDemo.cs
    /// seems like physics engine has Y axis going to top
    /// </summary>
    public class CollisionDetectionHandler
    {
       public event CollisionRegisteredDelegate OnCollisionRegistered;
        private Simulation simulation;
        private BufferPool pool;
        private BodyHandle cylinderHandle;
        private NarrowPhaseCallbacks narrowPhaseCallbacks;
       public void SimulationSimple()
        {
            initializeSimulation();
            // === 1. Create a static mesh collider ===
            /*
            var triangleVertices = new[]
            {
            new Vector3(-2, 0, -1),
            new Vector3( 2, 0, -1),
            new Vector3(-2, 0,  1),
            new Vector3( 2, 0,  1)
        };

            var trianglesArray = new[]
            {
            new Triangle(triangleVertices[0], triangleVertices[1], triangleVertices[2]),
            new Triangle(triangleVertices[2], triangleVertices[1], triangleVertices[3])
        };

            pool.Take<Triangle>(trianglesArray.Length, out var triangleBuffer);
            for (int i = 0; i < trianglesArray.Length; ++i)
            {
                triangleBuffer[i] = trianglesArray[i];
            }

            var mesh = new Mesh(triangleBuffer, new Vector3(1), pool);
            var meshShapeIndex = simulation.Shapes.Add(mesh);

            simulation.Statics.Add(new StaticDescription(new Vector3(0, 0, 0), meshShapeIndex));
            */

            InitStaticBoxShape(4f, 1f, 1f);
            // === 2. Create a kinematic cylinder ===
            InitCylindricShape(0.5f, 2f);
            // === 3. Step the simulation while moving the cylinder downward ===
            OnCollisionRegistered?.Invoke($"[{DateTime.Now}] Stepping over simulation");
            for (int step = 0; step < 60; step++)
            {
                float y = 2f - step * 0.05f;

                simulateMoveCylindrikSingleStep(new Vector3(0, y, 0));
            }
            // finalizeSimulation() should be called when we do not need physics simulation and collision detection
        }
        public void simulateMoveCylindrikSingleStep(Vector3 targetPosition)
        {
            var body = simulation.Bodies.GetBodyReference(cylinderHandle);
            body.Pose.Position = targetPosition;
            //body.Velocity.Linear = new Vector3(0, -3f, 0); //Must have non-zero velocity to trigger contacts!

            simulation.Timestep(1f / 60f);
        }
        
        public void initializeSimulation()
        {
            pool = new BufferPool();
            narrowPhaseCallbacks = new NarrowPhaseCallbacks();
            if (OnCollisionRegistered != null)
            {
                narrowPhaseCallbacks.OnCollisionRegistered += this.OnCollisionRegistered;
            }
            simulation = Simulation.Create(pool, narrowPhaseCallbacks, new PoseIntegratorCallbacks(), new SolveDescription(1, 1));
            OnCollisionRegistered?.Invoke($"[{DateTime.Now}] simulation initialized");
        }
        /// <summary>
        /// absolutely required to call this, it dealocates internal memory structures. Or else there will be memory leaks
        /// </summary>
        public void FinalizeSimulation()
        {
            OnCollisionRegistered?.Invoke($"[{DateTime.Now}] Simulation complete");
            simulation.Dispose();
            pool.Clear();
        }
        public void InitStaticBoxShape(float dimX, float dimY, float dimZ)
        {
            var boxShape = new Box(dimX, dimY, dimZ); // Axis-aligned box: 4 wide, 1 tall, 1 deep
            var boxShapeIndex = simulation.Shapes.Add(boxShape);

            var staticDescription = new StaticDescription(
                new Vector3(0, 0, 0), // Center of the box
                boxShapeIndex
            );
            simulation.Statics.Add(staticDescription);
            OnCollisionRegistered?.Invoke($"[{DateTime.Now}] Static box created");
        }
        /// <summary>
        /// create kynematic shape of cylindric for collision simulation
        /// </summary>
        public void InitCylindricShape(float radius, float length)
        {
            var cylinder = new Cylinder(radius, length);
            var cylinderShapeIndex = simulation.Shapes.Add(cylinder);

            var pose = new RigidPose(
                new Vector3(0, 0f, 0f)
                /*Quaternion.CreateFromAxisAngle(Vector3.UnitX, MathF.PI / 4)*/
            );
            // You could also force the kinematic to always be awake by settings its sleeping velocity threshold to a negative value in BodyActivityDescription.
            cylinderHandle = simulation.Bodies.Add(BodyDescription.CreateKinematic(
                pose,
                new CollidableDescription(cylinderShapeIndex, 0.1f),
                new BodyActivityDescription(-1)
            ));
            OnCollisionRegistered?.Invoke($"[{DateTime.Now}] Kinematics cylinder created");
        }
        /// <summary>
        /// should be called after InitCylindricShape. for consistency reason should not be called when movement is running (nothing bad happen but still)
        /// </summary>
        public void SetInitialCylindricCoordinates(float rotationXAngleRAD, float rotationYAngleRAD, float rotationZAngleRAD, float XPosition, float YPosition, float ZPosition)
        {
            var body = simulation.Bodies.GetBodyReference(cylinderHandle);
            body.Pose.Position = new Vector3(XPosition, YPosition, ZPosition);
            // https://simple.wikipedia.org/wiki/Pitch,_yaw,_and_roll#/media/File:6DOF_en.jpg
            body.Pose.Orientation = Quaternion.CreateFromYawPitchRoll(rotationZAngleRAD, rotationYAngleRAD, rotationXAngleRAD);
        }
        struct PoseIntegratorCallbacks : IPoseIntegratorCallbacks
        {
            public AngularIntegrationMode AngularIntegrationMode => AngularIntegrationMode.Nonconserving;
            public bool AllowSubstepsForUnconstrainedBodies => false;
            public bool IntegrateVelocityForKinematics => false;

            public void Initialize(Simulation simulation) { }

            public void PrepareForIntegration(float dt) { }

            public void IntegrateVelocity(
                Vector<int> bodyIndices,
                Vector3Wide position,
                QuaternionWide orientation,
                BodyInertiaWide localInertia,
                Vector<int> integrationMask,
                int workerIndex,
                Vector<float> dt,
                ref BodyVelocityWide velocity)
            {
                // No velocity integration; we're controlling the kinematic manually
            }
        }
        /// <summary>
        /// collision detection has 2 phases. Broad phase detects pairs of probable candidates for collision, narrow phase detects whether those pairs of elements may collide
        /// </summary>
        struct NarrowPhaseCallbacks : INarrowPhaseCallbacks
        {
            public event CollisionRegisteredDelegate OnCollisionRegistered;
            public void Initialize(Simulation simulation) { }

            public void Dispose() { }

            public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b, ref float speculativeMargin)
            {
                return true;
            }

            public bool AllowContactGeneration(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB)
            {
                return true;
            }
            /// <summary>
            /// this is where collision detected and processed
            /// </summary>
            /// <typeparam name="TManifold"></typeparam>
            /// <param name="workerIndex"></param>
            /// <param name="pair"></param>
            /// <param name="manifold"></param>
            /// <param name="material"></param>
            /// <returns></returns>
            public bool ConfigureContactManifold<TManifold>(
                int workerIndex,
                CollidablePair pair,
                ref TManifold manifold,
                out PairMaterialProperties material)
                where TManifold : unmanaged, IContactManifold<TManifold>
            {
                if (manifold.Count > 0)
                {
                    //Debug.WriteLine($"Collision between two bodies with {manifold.Count} contact(s).");
                    OnCollisionRegistered?.Invoke($"[{DateTime.Now}] Collision between two bodies with {manifold.Count} contact(s).");
                } else {
                    OnCollisionRegistered?.Invoke($"[{DateTime.Now}] No Collision registered");
                }

                material = new PairMaterialProperties
                {
                    FrictionCoefficient = 0f,
                    MaximumRecoveryVelocity = 0f,
                    SpringSettings = new SpringSettings(0f, 0f)
                };
                return true;
            }

            public void NotifySelectedContactManifold<TManifold>(
                int workerIndex,
                CollidablePair pair,
                ref TManifold manifold)
                where TManifold : unmanaged, IContactManifold<TManifold>
            {
                // No-op
            }

            public bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB, ref ConvexContactManifold manifold)
            {
                // I leave it empty by now - what does it do?
                return true;
            }
        }
    }
}
