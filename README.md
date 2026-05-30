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
- The math and physics layers are not yet covered by tests.
- Several low-level fixed-point and physics systems need correctness work before
  new features are added.
- The physics world is still a skeleton; broadphase, rigid integration,
  constraint solving, and callbacks are not production-ready.

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
