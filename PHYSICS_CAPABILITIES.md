# Physics entry-point capabilities

`World` and `PhysicsWorld` share primitive contact generation and low-level math,
but expose different ownership and integration contracts. Unity's
`MaphyPhysicsWorld` component uses **PhysicsWorld**.

| Capability | World / object Shape queries | PhysicsWorld / PhysicsShapeData / Unity component |
| --- | --- | --- |
| AABB, OBB, sphere, capsule | Supported | Supported |
| TriangleMesh, HeightField, ConvexHull | Not implemented | Not implemented |
| Auto and SAT contact selection | Supported | Supported |
| GJK/EPA selection | Supported, with SAT contact fallback | Not supported; throws NotSupportedException |
| Invalid algorithm enum | ArgumentOutOfRangeException at contact entry | ArgumentOutOfRangeException at configuration/contact entry |
| Primitive ray/shape casts and layer filtering | Supported | Supported through non-alloc buffers |
| Contact manifolds, friction, warm start, sleeping, CCD | Supported | Supported |
| Compound colliders and automatic mass | Supported | Supported |
| Collision/trigger enter, stay, exit callbacks | Supported with deferred lifecycle operations | Poll contact manifolds; no callbacks |
| Distance, point, spring-distance, hinge, fixed, slider joints | Supported at their current constraint API level | Not implemented |
| Motors, gear, cone-twist, six-DOF, breakable joints | Not implemented | Not implemented |
| Simulation allocation contract | Object/collection based; no blanket zero-allocation guarantee | Reserve first, then simulate/query within capacity on the same thread |
| Capacity exhaustion | Collections may grow | Creation returns false; pair/contact/query truncation exposes overflow flags |

The hinge implementation aligns axes and anchors; it does not expose an
arbitrary angular range or motor. Slider restricts motion to an axis; its
descriptor is not a claim that a complete configurable joint system exists.

Programmatic checks:

```csharp
bool coreGjk = PhysicsWorld.Capabilities.SupportsAlgorithm(NarrowPhaseAlgorithm.GJK); // false
bool objectDistance = World.Capabilities.SupportsConstraint(ConstraintType.Distance); // true
bool coreCallbacks = PhysicsWorld.Capabilities.SupportsCollisionCallbacks; // false
```

`MaphyPhysicsWorld.Capabilities` exposes the same core description. Requesting
GJK in core settings now fails during construction or settings assignment,
before simulation state is changed. Direct value-shape contact queries also
reject it. Previous releases silently ran SAT regardless of this setting;
callers that selected GJK must switch to SAT/Auto or use the object API.

Existing integration tests cover each implemented object joint, event lifecycle,
CCD and query API. Additional tests check rejected core settings and all 16
primitive contact combinations through both contact entry points. New feature
work should extend this table and test the public entry point before advertising
support; porting joints/callbacks into the reserved core is not part of this change.
