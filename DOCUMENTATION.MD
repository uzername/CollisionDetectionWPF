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


