# Maphy.Net

Maphy.Net is a fixed-point mathematics and physics prototype for Unity/.NET.

The current repository should be treated as an early implementation draft, not a
stable runtime package. The intended direction is to build a deterministic
fixed-point math core first, then layer geometry queries, collision detection,
and finally a small physics world on top of it.

## Scope

- `Maphy.Mathematics`: fixed-point scalar/vector/matrix/quaternion math.
- `Maphy.Mathematics.Geometry`: 2D/3D geometry primitives and query helpers.
- `Maphy.Physics`: shape definitions, collision queries, rigid bodies, and world
  update flow.
- `Maphy.Extension`: Unity interop helpers. This should stay thin and separate
  from deterministic core logic.

## Current State

- The API shape follows parts of Unity.Mathematics.
- Fixed-point math, primitive shape queries, broadphase, contact manifolds,
  solver behavior, CCD, lifecycle callbacks, and deterministic replay now have
  focused test coverage.
- `World` is the object-friendly physics layer.
- `PhysicsWorld` is the array-backed no-GC core for deterministic hot paths.
- Joint/constraint expansion and scene geometry types such as TriangleMesh,
  HeightField, and ConvexHull are intentionally separate work.

## Development Priority

1. Stabilize the fixed-point scalar type and its overflow/rounding behavior.
2. Validate vector, matrix, quaternion, trigonometric, and random APIs.
3. Build deterministic geometry and shape overlap tests.
4. Rework physics world ownership: bodies, colliders, broadphase, narrowphase,
   contact data, and callbacks.
5. Keep Unity-facing adapters outside the deterministic core.

See [ROADMAP.md](ROADMAP.md) for the planned module list and engineering cleanup
order.

## Core Build

The deterministic core can be checked outside Unity with:

```bash
dotnet build Maphy.Net.Core.csproj
```

Tests can be run with:

```bash
dotnet run --project Maphy.Net.Tests~/Maphy.Net.Tests.csproj
```

## No-GC Core Example

`PhysicsWorld` requires explicit capacity reservation. `Reserve` may allocate;
`Update`, `Step`, non-alloc queries, and handle lookups are designed to reuse
the reserved arrays.

```csharp
PhysicsWorldSettings settings = new PhysicsWorldSettings(false);
settings.enableCCD = true;

PhysicsWorld world = new PhysicsWorld(settings);
world.Reserve(new PhysicsWorldCapacity(
    bodyCapacity: 2,
    colliderCapacity: 2,
    pairCapacity: 4,
    contactManifoldCapacity: 4));

world.CreateBody(fix3.zero, quaternion.identity, RigidType.Dynamic, out BodyHandle body);
world.AddSphere(body, fix3.zero, fix._0_5, out ColliderHandle collider);
world.SetBodyVelocity(body, new fix3(10, 0, 0));

world.CreateBody(new fix3(5, 0, 0), quaternion.identity, RigidType.Static, out BodyHandle wall);
world.AddAABB(wall, fix3.zero, new fix3(1, 4, 4), out ColliderHandle _);

world.Step(fix.One / 60);
ulong replayHash = world.ComputeStateHash();
PhysicsWorldStepStats stats = world.LastStepStats;
PhysicsWorldMemoryBudget budget = world.EstimatedMemoryBudget;
```

The core works with `BodyHandle` and `ColliderHandle` instead of object
references. This keeps Unity-facing adapters thin: Unity code should convert
`Vector3`/`Quaternion` at the boundary, then pass fixed-point values and handles
into the deterministic core.

## Unity Interop

Unity-facing helpers live in `Maphy.Extension` under the `Maphy.Unity`
namespace. They are thin adapters over the deterministic core.

For code-first usage:

```csharp
using Maphy.Physics;
using Maphy.Unity;
using UnityEngine;

PhysicsWorld world = new PhysicsWorld(new PhysicsWorldSettings(false));
world.Reserve(PhysicsWorldCapacity.ForBodies(128, collidersPerBody: 2, candidatePairsPerCollider: 8));

world.CreateBody(transform, RigidType.Dynamic, out BodyHandle body);
world.AddLocalSphere(body, Vector3.zero, 0.5f, out ColliderHandle collider);
world.PushTransform(body, transform);
world.Step(Time.fixedDeltaTime);
world.PullTransform(body, transform);
```

For GameObject usage, add `MaphyPhysicsWorld` to a scene/root object, then add
`MaphyBody` and `MaphyCollider` to body objects. The component layer keeps only
`BodyHandle`/`ColliderHandle` bindings and forwards lifecycle changes to
`PhysicsWorld`.

Transform sync is explicit:

- `Authoring`: read the initial Transform when the body is created.
- `Dynamic`: the physics body drives the Transform after each world step.
- `Kinematic`: the Transform is pushed into the physics body before each step.

When a `MaphyCollider` is selected, Gizmos draw the authored local shape. While
the scene is playing, selecting `MaphyPhysicsWorld` can draw runtime collider
bounds and contact normals.

`MaphyUnityConvert` covers `Vector2/3/4`, `Quaternion`, `Matrix4x4`,
`Bounds`, `Ray`, and fixed-point hit conversion. If `com.unity.mathematics` is
installed, `MaphyUnityMathematicsConvert` is enabled automatically for
`float2/3/4`, `float3x3`, `float4x4`, and `Unity.Mathematics.quaternion`.
Unity-specific methods stay out of `Maphy.Physics/Core`, so the core remains
usable from plain .NET tests and deterministic servers.

Common component calls are available directly from the Unity layer:

```csharp
body.SetVelocity(new Vector3(5f, 0f, 0f));
body.Teleport(spawnPosition, spawnRotation);
body.Wake();

collider.SetTrigger(true);
collider.SetMaterial(1f, 0.6f, 0f);
collider.RebuildCollider();

int hitCount = world.RaycastNonAlloc(ray, 100f, rayHits);
int overlapCount = world.QueryBoundsNonAlloc(bounds, colliderHits);
```

## Capacity And Memory

`PhysicsWorldCapacity` is the allocation contract for the no-GC core. Use it to
reserve buffers up front and to estimate memory before entering gameplay:

```csharp
PhysicsWorldCapacity capacity = PhysicsWorldCapacity.ForBodies(
    bodyCapacity: 512,
    collidersPerBody: 2,
    candidatePairsPerCollider: 8);

PhysicsWorldMemoryBudget estimate = capacity.EstimateMemoryBudget();
world.Reserve(capacity);
```

`LastStepStats` reports active counts, reserved capacity, broadphase tree health,
pair/contact counts, narrowphase test counts, manifold new/reuse/drop counts,
dirty body/collider sync counts, moved broadphase proxy counts, solver contact
points, sleep counts, and overflow flags. Treat `HasOverflow` as a signal to
raise capacity for the current scene profile.

## Benchmarks

The lightweight benchmark harness is intentionally dependency-free:

```bash
dotnet run --project Maphy.Net.Benchmarks~/Maphy.Net.Benchmarks.csproj -- --quick --iterations 30
dotnet run --project Maphy.Net.Benchmarks~/Maphy.Net.Benchmarks.csproj -- --iterations 120
dotnet run --project Maphy.Net.Benchmarks~/Maphy.Net.Benchmarks.csproj -- --scenario StaticQuery,StackSolver --count 1000 --iterations 300 --out Benchmarks/latest.csv
```

It prints CSV rows with elapsed time, allocated bytes, replay hash, active
counts, dirty sync counts, broadphase/narrowphase counters, manifold reuse/drop
counters, solver contact points, sleep counts, and overflow state.
