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
    /// </summary>
    public class CollisionDetectionHandler
    {
       public event CollisionRegisteredDelegate OnCollisionRegistered;
        private Simulation simulation;
        private BufferPool pool;
        private NarrowPhaseCallbacks narrowPhaseCallbacks;
       public void SimulationSimple()
        {
            pool = new BufferPool();
            narrowPhaseCallbacks = new NarrowPhaseCallbacks();
            if (OnCollisionRegistered!=null)
            {
                narrowPhaseCallbacks.OnCollisionRegistered += this.OnCollisionRegistered;
            }
            simulation = Simulation.Create(pool, narrowPhaseCallbacks, new PoseIntegratorCallbacks(), new SolveDescription(1, 1));
            OnCollisionRegistered?.Invoke($"[{DateTime.Now}] simulation initialized");
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

            var boxShape = new Box(4f, 1f, 1f); // Axis-aligned box: 4 wide, 1 tall, 1 deep
            var boxShapeIndex = simulation.Shapes.Add(boxShape);

            var staticDescription = new StaticDescription(
                new Vector3(0, 0, 0), // Center of the box
                boxShapeIndex
            );
            simulation.Statics.Add(staticDescription);
            OnCollisionRegistered?.Invoke($"[{DateTime.Now}] Static box created");
            // === 2. Create a kinematic cylinder ===
            var cylinder = new Cylinder(0.5f, 2f);
            var cylinderShapeIndex = simulation.Shapes.Add(cylinder);

            var pose = new RigidPose(
                new Vector3(0, 2f, 0),
                Quaternion.CreateFromAxisAngle(Vector3.UnitX, MathF.PI / 4)
            );
            // You could also force the kinematic to always be awake by settings its sleeping velocity threshold to a negative value in BodyActivityDescription.
            var cylinderHandle = simulation.Bodies.Add(BodyDescription.CreateKinematic(
                pose,
                new CollidableDescription(cylinderShapeIndex, 0.1f),
                new BodyActivityDescription(-1)
            ));
            OnCollisionRegistered?.Invoke($"[{DateTime.Now}] Kinematics cylinder created");
            // === 3. Step the simulation while moving the cylinder downward ===
            OnCollisionRegistered?.Invoke($"[{DateTime.Now}] Stepping over simulation");
            for (int step = 0; step < 60; step++)
            {
                float y = 2f - step * 0.05f;
                // what may it be
                var body = simulation.Bodies.GetBodyReference(cylinderHandle);
                body.Pose.Position = new Vector3(0, y, 0);
                //body.Velocity.Linear = new Vector3(0, -3f, 0); //Must have non-zero velocity to trigger contacts!

                simulation.Timestep(1f / 60f);
            }
            OnCollisionRegistered?.Invoke($"[{DateTime.Now}] Simulation complete");
            simulation.Dispose();
            pool.Clear();
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
                    OnCollisionRegistered?.Invoke($"[{DateTime.Now}]Collision between two bodies with {manifold.Count} contact(s).");
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
