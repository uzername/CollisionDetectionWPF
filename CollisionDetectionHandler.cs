using System;
using System.Numerics;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Memory;

namespace WpfCollision
{
    public class CollisionDetectionHandler
    {
        static void Main()
        {
            var pool = new BufferPool();
            var simulation = Simulation.Create(pool, new NarrowPhaseCallbacks(), new PoseIntegratorCallbacks(), new SolveDescription(1, 1));

            // === 1. Create a static mesh collider ===
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

            // === 2. Create a kinematic cylinder ===
            var cylinder = new Cylinder(0.5f, 2f);
            var cylinderShapeIndex = simulation.Shapes.Add(cylinder);

            var pose = new RigidPose(
                new Vector3(0, 2f, 0),
                Quaternion.CreateFromAxisAngle(Vector3.UnitX, MathF.PI / 4)
            );

            var cylinderHandle = simulation.Bodies.Add(BodyDescription.CreateKinematic(
                pose,
                new CollidableDescription(cylinderShapeIndex, 0.1f),
                new BodyActivityDescription()
            ));

            // === 3. Step the simulation while moving the cylinder downward ===
            for (int step = 0; step < 60; step++)
            {
                float y = 2f - step * 0.05f;
                // tell AI
                var body = simulation.Bodies.GetBodyReference(cylinderHandle);
                body.Pose.Position = new Vector3(0, y, 0);

                simulation.Timestep(1f / 60f);
            }

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
                    // todo - tell AI about BodyHandle
                    Console.WriteLine($"Collision between {pair.A.BodyHandle} and {pair.B.BodyHandle    } with {manifold.Count} contact(s).");
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
                // I leave it empty by now - todo ask AI
                return true;
            }
        }
    }
}
