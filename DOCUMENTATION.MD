## Here is demo that shows collision detection with BEPU physics 2 and WPF for display

Collision detection is done in CollisionDetectionHandler, within `struct NarrowPhaseCallbacks : INarrowPhaseCallbacks`. Here is actual place:

```
public bool ConfigureContactManifold<TManifold>(
                int workerIndex,
                CollidablePair pair,
                ref TManifold manifold,
                out PairMaterialProperties material)
                where TManifold : unmanaged, IContactManifold<TManifold>
```

A contact manifold is a collection of one or more contact points that describe how two shapes are touching. TManifold is the type that holds all contact points. The engine populates it during collision detection.

Each contact point usually includes: Contact position (in world space), Normal vector (direction of the contact), Penetration depth, Feature IDs (optional – what part of each shape is involved).

BepuPhysics2 has different coordinate system, so I should supply Z coordinate from Helix Toolkit to Y coordinate of Bepu Physics

Framework     |"Up" Axis | Forward Axis | Notes                     |
--------------|----------|--------------|---------------------------|
BepuPhysics2  |Y         |Z             |Right-handed system        |
--------------|----------|--------------|---------------------------|
Helix Toolkit |Z         |Y (or -Y)     | Based on WPF 3D / DirectX |

### Cylinder in Bepu Is Always Aligned to Y-Axis
In BepuPhysics2, the Cylinder shape (just like Capsule) is defined as being implicitly aligned with the Y-axis. From [Bepu source comments and design]:

The Cylinder shape's height extends along the local Y-axis of the body, and it is not arbitrarily rotatable at the shape level.

So even if you apply rotation to the body, the collision shape itself does not rotate accordingly unless it is transformed properly in local space — but shape-level rotation isn’t directly supported.

### Workaround
To rotate a cylinder in simulation, you must apply the rotation at the body level, which you're already doing, but due to the nature of the Cylinder primitive, Bepu only supports axis-aligned cylinders.

To simulate a rotated cylinder, consider two options:

#### Option 1: Use a ConvexHull

Instead of using new Cylinder(...), generate a rotated cylinder mesh and convert it to a convex hull:

```
var vertices = ProceduralGenerator.BuildRotatedCylinder(radius, height, angleX, angleY, angleZ); // Your mesh builder
var convexHull = new ConvexHull(vertices, bufferPool, out _);
var shapeIndex = simulation.Shapes.Add(convexHull);
```

Then, the shape itself is already rotated, and you don’t need to rotate the body — it collides properly at all angles.

#### Option 2: Use a Compound Shape

If the cylinder is small or you want to simulate it precisely:

Build the shape from multiple spheres or capsules arranged in a tilted line (compound shape).

Tedious but effective if convex hull is not performant enough.