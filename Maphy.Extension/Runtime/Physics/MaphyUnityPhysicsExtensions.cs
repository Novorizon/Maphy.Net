using System.Runtime.CompilerServices;
using Maphy.Mathematics;
using Maphy.Physics;
using UnityEngine;
using FixQuaternion = Maphy.Mathematics.quaternion;
using UnityBounds = UnityEngine.Bounds;
using UnityQuaternion = UnityEngine.Quaternion;
using UnityRay = UnityEngine.Ray;

namespace Maphy.Unity
{
    /// <summary>
    /// Unity-only helpers around PhysicsWorld. They convert at the boundary and keep
    /// UnityEngine types out of the deterministic core.
    /// </summary>
    public static class MaphyUnityPhysicsExtensions
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool CreateBody(
            this PhysicsWorld world,
            Transform transform,
            RigidType type,
            out BodyHandle handle)
        {
            handle = BodyHandle.Invalid;
            if (world == null || transform == null)
            {
                return false;
            }

            MaphyUnityConvert.ReadTransform(transform, out fix3 position, out FixQuaternion rotation);
            return world.CreateBody(position, rotation, type, out handle);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool PushTransform(this PhysicsWorld world, BodyHandle body, Transform transform)
        {
            if (world == null || transform == null)
            {
                return false;
            }

            MaphyUnityConvert.ReadTransform(transform, out fix3 position, out FixQuaternion rotation);
            return world.SetBodyTransform(body, position, rotation);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool PullTransform(this PhysicsWorld world, BodyHandle body, Transform transform)
        {
            if (world == null || transform == null || !world.TryGetBody(body, out PhysicsWorldBody bodyState))
            {
                return false;
            }

            MaphyUnityConvert.ApplyTransform(transform, bodyState.position, bodyState.rotation);
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool TryGetBodyPose(
            this PhysicsWorld world,
            BodyHandle body,
            out Vector3 position,
            out UnityQuaternion rotation)
        {
            position = default;
            rotation = UnityQuaternion.identity;
            if (world == null || !world.TryGetBody(body, out PhysicsWorldBody bodyState))
            {
                return false;
            }

            position = MaphyUnityConvert.ToVector3(bodyState.position);
            rotation = MaphyUnityConvert.ToUnityQuaternion(bodyState.rotation);
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool AddLocalAABB(
            this PhysicsWorld world,
            BodyHandle body,
            Vector3 center,
            Vector3 size,
            out ColliderHandle handle)
        {
            handle = ColliderHandle.Invalid;
            return world != null
                && world.AddAABB(body, MaphyUnityConvert.ToFix3(center), MaphyUnityConvert.ToFix3(size), out handle);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool AddLocalAABB(
            this PhysicsWorld world,
            BodyHandle body,
            UnityBounds localBounds,
            out ColliderHandle handle)
        {
            return AddLocalAABB(world, body, localBounds.center, localBounds.size, out handle);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool AddLocalOBB(
            this PhysicsWorld world,
            BodyHandle body,
            Vector3 center,
            Vector3 size,
            UnityQuaternion rotation,
            out ColliderHandle handle)
        {
            handle = ColliderHandle.Invalid;
            return world != null
                && world.AddOBB(
                    body,
                    MaphyUnityConvert.ToFix3(center),
                    MaphyUnityConvert.ToFix3(size),
                    MaphyUnityConvert.ToNormalizedFixQuaternion(rotation),
                    out handle);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool AddLocalSphere(
            this PhysicsWorld world,
            BodyHandle body,
            Vector3 center,
            float radius,
            out ColliderHandle handle)
        {
            handle = ColliderHandle.Invalid;
            return world != null
                && world.AddSphere(body, MaphyUnityConvert.ToFix3(center), MaphyUnityConvert.ToFix(radius), out handle);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool AddLocalCapsule(
            this PhysicsWorld world,
            BodyHandle body,
            Vector3 center,
            float radius,
            float height,
            UnityQuaternion rotation,
            out ColliderHandle handle)
        {
            handle = ColliderHandle.Invalid;
            return world != null
                && world.AddCapsule(
                    body,
                    MaphyUnityConvert.ToFix3(center),
                    MaphyUnityConvert.ToFix(radius),
                    MaphyUnityConvert.ToFix(height),
                    MaphyUnityConvert.ToNormalizedFixQuaternion(rotation),
                    out handle);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool TryGetColliderBounds(
            this PhysicsWorld world,
            ColliderHandle collider,
            out UnityBounds bounds)
        {
            bounds = default;
            if (world == null || !world.TryGetColliderBounds(collider, out AABB fixedBounds))
            {
                return false;
            }

            bounds = MaphyUnityConvert.ToBounds(fixedBounds);
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int QueryBoundsNonAlloc(
            this PhysicsWorld world,
            UnityBounds bounds,
            ColliderHandle[] results)
        {
            return world == null ? 0 : world.QueryAABBNonAlloc(MaphyUnityConvert.ToAABB(bounds), results);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int QueryBoundsNonAlloc(
            this PhysicsWorld world,
            UnityBounds bounds,
            int layerMask,
            ColliderHandle[] results)
        {
            return world == null ? 0 : world.QueryAABBNonAlloc(MaphyUnityConvert.ToAABB(bounds), layerMask, results);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int RaycastNonAlloc(
            this PhysicsWorld world,
            UnityRay ray,
            float maxDistance,
            PhysicsWorldRaycastHit[] results)
        {
            return world == null
                ? 0
                : world.RaycastNonAlloc(MaphyUnityConvert.ToRay(ray), MaphyUnityConvert.ToFix(maxDistance), results);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int RaycastNonAlloc(
            this PhysicsWorld world,
            UnityRay ray,
            float maxDistance,
            int layerMask,
            PhysicsWorldRaycastHit[] results)
        {
            return world == null
                ? 0
                : world.RaycastNonAlloc(
                    MaphyUnityConvert.ToRay(ray),
                    MaphyUnityConvert.ToFix(maxDistance),
                    layerMask,
                    results);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int ShapeCastNonAlloc(
            this PhysicsWorld world,
            PhysicsShapeData movingShape,
            Vector3 delta,
            PhysicsWorldShapeCastHit[] results)
        {
            return world == null ? 0 : world.ShapeCastNonAlloc(movingShape, MaphyUnityConvert.ToFix3(delta), results);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int ShapeCastNonAlloc(
            this PhysicsWorld world,
            PhysicsShapeData movingShape,
            Vector3 delta,
            int layerMask,
            PhysicsWorldShapeCastHit[] results)
        {
            return world == null
                ? 0
                : world.ShapeCastNonAlloc(movingShape, MaphyUnityConvert.ToFix3(delta), layerMask, results);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int ShapeCastNonAlloc(
            this PhysicsWorld world,
            ColliderHandle movingCollider,
            Vector3 delta,
            PhysicsWorldShapeCastHit[] results)
        {
            return world == null ? 0 : world.ShapeCastNonAlloc(movingCollider, MaphyUnityConvert.ToFix3(delta), results);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int ShapeCastNonAlloc(
            this PhysicsWorld world,
            ColliderHandle movingCollider,
            Vector3 delta,
            int layerMask,
            PhysicsWorldShapeCastHit[] results)
        {
            return world == null
                ? 0
                : world.ShapeCastNonAlloc(movingCollider, MaphyUnityConvert.ToFix3(delta), layerMask, results);
        }
    }
}
