# Maphy.Net Roadmap

This document defines the intended content of Maphy.Net and the engineering
order for turning the current draft into a reliable deterministic math and
physics library.

## 1. Fixed-Point Mathematics

### 1.1 Scalar Core

Goal: make `fix` a trustworthy deterministic numeric type before building
anything else on top of it.

Required content:

- Raw-value construction and explicit raw constants.
- Conversion from `int`, `long`, `float`, `double`, and `decimal`.
- Conversion back to primitive numeric types.
- Addition, subtraction, multiplication, division, modulo, negation.
- Comparison, equality, hashing, formatting, parsing.
- Defined rounding policy: truncation, floor, ceil, round, frac.
- Overflow policy: unchecked wrap, saturating, or explicit invalid value.
- Division-by-zero policy.
- Min/max constants and special values.
- Deterministic behavior across Unity editor, IL2CPP, Mono, and .NET runtime.

Current priority fixes:

- Avoid constructing raw constants through normal scaled constructors.
- Ensure shift operators preserve raw representation.
- Define whether `NaN` is a supported fixed-point state or remove it.
- Add tests for boundary values, negative values, zero, one, and fractional
  conversion.

### 1.2 Vector Types

Goal: mirror the useful shape of Unity.Mathematics while keeping the API small.

Required content:

- `fix2`, `fix3`, `fix4`.
- Constructors from scalar and component values.
- Componentwise arithmetic.
- Dot, cross, length, length squared, distance, normalize.
- Min, max, clamp, abs, sign, floor, ceil, round, lerp.
- Indexers and swizzles only where they are actually used.
- Deterministic hash and equality behavior.

Quality requirements:

- No recursive properties or accidental stack overflows.
- No implicit float dependency in deterministic calculations except explicit
  conversion boundaries.
- Tests should compare raw values, not only decimal strings.

### 1.3 Matrix Types

Required content:

- `fix3x3`, `fix4x4`.
- Identity and zero constants.
- Transpose, determinant, inverse where needed.
- Matrix-vector multiplication.
- Matrix-matrix multiplication.
- TRS construction and transform/rotate helpers.

Priority:

- Recheck matrix multiplication semantics. Componentwise multiplication and
  actual matrix multiplication must be separate and tested.
- Match Unity.Mathematics naming where practical: operator `*` for
  componentwise only if the library clearly documents that choice; otherwise use
  `math.mul`.

### 1.4 Quaternion

Required content:

- Identity, construction, normalize, conjugate, inverse, dot.
- Quaternion-quaternion and quaternion-vector multiplication.
- Axis-angle, Euler orders, look rotation.
- Slerp and nlerp.
- Conversion to/from rotation matrix.

Priority:

- Fix obvious constructor and axis rotation errors before exposing as stable API.
- Add parity tests against Unity.Mathematics within fixed-point tolerance.

### 1.5 Trigonometry and Advanced Math

Required content:

- `sin`, `cos`, `sincos`, `tan`.
- `asin`, `acos`, `atan`, `atan2`.
- `sqrt`, `rsqrt`, `log`, `ln`, `pow` where needed.
- LUT generation policy and error bounds.

Quality requirements:

- Document input domain and units.
- Add accuracy tests over representative ranges.
- Add monotonicity and symmetry tests for LUT-backed functions.

### 1.6 Deterministic Random

Required content:

- Seeded deterministic random generator.
- `NextInt`, `NextFix`, `NextFix2`, `NextFix3`.
- Inclusive/exclusive range semantics.

Quality requirements:

- Same seed must produce the same raw sequence across supported runtimes.

## 2. Geometry

### 2.1 2D Geometry

Required content:

- Point, line, segment helpers.
- AABB2D, circle, triangle, rectangle, polygon, hexagon.
- Distance, containment, overlap, projection.
- Separating axis helpers for convex polygons.

Priority:

- Fix AABB intersection logic and add edge-touching tests.
- Keep geometry independent from UnityEngine.

### 2.2 3D Geometry

Required content:

- Ray, line, segment, plane, triangle.
- Closest point helpers.
- Distance queries.
- Projection and barycentric helpers where needed.

Priority:

- Build these as pure math primitives first. Physics should consume them, not
  duplicate them.

## 3. Collision and Physics

### 3.1 Shapes

Required content:

- AABB.
- OBB.
- Sphere.
- Capsule.
- Shape type identifiers.
- Shape bounds and support points.

Quality requirements:

- Shape constructors must clearly accept either center/size or min/max, not a
  mixed interpretation.
- Every shape must expose consistent center, extents, min, max, and bounds
  semantics.

### 3.2 Narrowphase Queries

Required content:

- AABB vs AABB.
- AABB vs OBB.
- AABB vs Sphere.
- AABB vs Capsule.
- OBB vs OBB.
- OBB vs Sphere.
- OBB vs Capsule.
- Sphere vs Sphere.
- Sphere vs Capsule.
- Capsule vs Capsule.
- Raycast against supported shapes.

Priority:

- Start with boolean overlap.
- Then add contact normal, penetration depth, and contact point.
- Then add manifold generation only if needed by solver.

### 3.3 Broadphase

Required content:

- Stable collider registration.
- AABB proxy bounds.
- Pair generation.
- Pair cache update/removal.
- Initial implementation can be brute force for correctness.
- Later implementation can use dynamic AABB tree, sweep-and-prune, or spatial
  grid.

Priority:

- Replace static global collision state with world-owned state.
- Do not depend on an uninitialized static tree.

### 3.4 Rigid Bodies and World

Required content:

- Body identity and ownership.
- Transform: position and orientation.
- Velocity and angular velocity.
- Force and torque accumulation.
- Mass, inverse mass, inertia.
- Body type: static, kinematic, dynamic.
- Gravity and integration.
- Collider attachment.
- Collision callbacks.

Update order:

1. Apply forces.
2. Integrate velocities.
3. Update broadphase proxies.
4. Generate potential pairs.
5. Run narrowphase.
6. Resolve constraints or report contacts.
7. Integrate transforms.
8. Dispatch callbacks.

### 3.5 Constraint Solver

Required content:

- Contact constraint data.
- Restitution and friction.
- Positional correction.
- Sleeping policy, if needed.

Priority:

- This should come after collision data is reliable.

## 4. Unity Integration

Required content:

- Conversion helpers between `Vector3`, `Quaternion`, Unity.Mathematics types,
  and Maphy fixed-point types.
- Optional debug drawing helpers.
- Demo scenes and sample MonoBehaviours.

Rules:

- UnityEngine types should not enter `Maphy.Mathematics` deterministic core.
- Unity-facing code should live in a separate adapter assembly when the project
  is split.

## 5. Engineering Cleanup

### 5.1 Assembly Layout

Target split:

- `Maphy.Mathematics`: no UnityEngine dependency.
- `Maphy.Geometry`: depends only on `Maphy.Mathematics`.
- `Maphy.Physics`: depends on mathematics and geometry, no UnityEngine
  dependency.
- `Maphy.Unity`: optional adapters and samples, UnityEngine allowed.

Current repository still uses a single assembly. Splitting should happen after
the math core is stabilized so compile errors are easier to isolate.

### 5.2 Tests

Required test groups:

- Fixed-point scalar boundary tests.
- Vector/matrix/quaternion parity tests.
- Trigonometric accuracy tests.
- Geometry edge cases.
- Shape overlap matrix tests.
- Broadphase pair generation tests.
- World update deterministic replay tests.

### 5.3 Documentation

Required docs:

- Numeric format and rounding policy.
- API compatibility notes versus Unity.Mathematics.
- Determinism guarantees and limitations.
- Physics update model.
- Supported shape/query matrix.

### 5.4 Performance

Benchmark only after correctness tests exist.

Benchmark areas:

- Scalar arithmetic.
- Vector operations.
- Trigonometry LUT.
- Narrowphase queries.
- Broadphase pair generation.
- World update with many bodies.

## 6. Recommended Implementation Order

1. Fix `fix` raw constants, operators, conversion, and special value policy.
2. Add minimal tests for scalar arithmetic.
3. Fix `fix2`, `fix3`, `fix4` correctness issues and add tests.
4. Fix matrix and quaternion operations with Unity.Mathematics parity tests.
5. Fix shape data structures and constructor semantics.
6. Implement brute-force broadphase inside `World`.
7. Implement narrowphase boolean overlap matrix with tests.
8. Add contact information.
9. Add rigid body integration.
10. Add solver and callbacks.
11. Split assemblies and move Unity adapters out of the core.
