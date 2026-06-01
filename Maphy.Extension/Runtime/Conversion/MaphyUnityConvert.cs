using System.Runtime.CompilerServices;
using Maphy.Mathematics;
using Maphy.Physics;
using UnityEngine;
using FixQuaternion = Maphy.Mathematics.quaternion;
using MaphyRay = Maphy.Physics.Ray;
using UnityBounds = UnityEngine.Bounds;
using UnityQuaternion = UnityEngine.Quaternion;
using UnityRay = UnityEngine.Ray;

namespace Maphy.Unity
{
    public static class MaphyUnityConvert
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix ToFix(float value)
        {
            return new fix(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float ToFloat(fix value)
        {
            return value;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix2 ToFix2(Vector2 value)
        {
            return new fix2(value.x, value.y);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2 ToVector2(fix2 value)
        {
            return new Vector2(value.x, value.y);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 ToFix3(Vector3 value)
        {
            return new fix3(value.x, value.y, value.z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 ToVector3(fix3 value)
        {
            return new Vector3(value.x, value.y, value.z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 ToFix4(Vector4 value)
        {
            return new fix4(value.x, value.y, value.z, value.w);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector4 ToVector4(fix4 value)
        {
            return new Vector4(value.x, value.y, value.z, value.w);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FixQuaternion ToFixQuaternion(UnityQuaternion value)
        {
            return new FixQuaternion(value.x, value.y, value.z, value.w);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FixQuaternion ToNormalizedFixQuaternion(UnityQuaternion value)
        {
            return Maphy.Mathematics.math.normalize(ToFixQuaternion(value));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static UnityQuaternion ToUnityQuaternion(FixQuaternion value)
        {
            return new UnityQuaternion(value.value.x, value.value.y, value.value.z, value.value.w);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3x3 ToFix3x3(Matrix4x4 value)
        {
            return new fix3x3(
                value.m00,
                value.m01,
                value.m02,
                value.m10,
                value.m11,
                value.m12,
                value.m20,
                value.m21,
                value.m22);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix4x4 ToMatrix4x4(fix3x3 value)
        {
            return new Matrix4x4(
                ToVector4(new fix4(value.c0, fix.Zero)),
                ToVector4(new fix4(value.c1, fix.Zero)),
                ToVector4(new fix4(value.c2, fix.Zero)),
                new Vector4(0f, 0f, 0f, 1f));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4x4 ToFix4x4(Matrix4x4 value)
        {
            return new fix4x4(
                value.m00,
                value.m01,
                value.m02,
                value.m03,
                value.m10,
                value.m11,
                value.m12,
                value.m13,
                value.m20,
                value.m21,
                value.m22,
                value.m23,
                value.m30,
                value.m31,
                value.m32,
                value.m33);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix4x4 ToMatrix4x4(fix4x4 value)
        {
            return new Matrix4x4(
                ToVector4(value.c0),
                ToVector4(value.c1),
                ToVector4(value.c2),
                ToVector4(value.c3));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4x4 ToFixLocalToWorld(Transform transform)
        {
            return ToFix4x4(transform.localToWorldMatrix);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ReadTransform(Transform transform, out fix3 translation, out FixQuaternion orientation)
        {
            translation = ToFix3(transform.position);
            orientation = ToNormalizedFixQuaternion(transform.rotation);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApplyTransform(Transform transform, fix3 translation, FixQuaternion orientation)
        {
            transform.SetPositionAndRotation(ToVector3(translation), ToUnityQuaternion(orientation));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static AABB ToAABB(UnityBounds value)
        {
            return new AABB(ToFix3(value.center), ToFix3(value.size));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static UnityBounds ToBounds(AABB value)
        {
            return new UnityBounds(ToVector3(value.center), ToVector3(value.size));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static MaphyRay ToRay(UnityRay value)
        {
            return new MaphyRay(ToFix3(value.origin), ToFix3(value.direction));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static UnityRay ToUnityRay(MaphyRay value)
        {
            return new UnityRay(ToVector3(value.origin), ToVector3(value.direction));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static MaphyUnityRaycastHit ToUnityRaycastHit(PhysicsWorldRaycastHit value)
        {
            return new MaphyUnityRaycastHit(
                value.collider,
                value.body,
                ToFloat(value.distance),
                ToVector3(value.point),
                ToVector3(value.normal),
                ToBounds(value.bounds));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static MaphyUnityShapeCastHit ToUnityShapeCastHit(PhysicsWorldShapeCastHit value)
        {
            return new MaphyUnityShapeCastHit(
                value.collider,
                value.body,
                ToFloat(value.fraction),
                ToVector3(value.point),
                ToVector3(value.normal),
                ToBounds(value.bounds));
        }
    }

    public static class MaphyUnityConversionExtensions
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix ToFix(this float value)
        {
            return MaphyUnityConvert.ToFix(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float ToFloat(this fix value)
        {
            return MaphyUnityConvert.ToFloat(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix2 ToFix2(this Vector2 value)
        {
            return MaphyUnityConvert.ToFix2(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2 ToVector2(this fix2 value)
        {
            return MaphyUnityConvert.ToVector2(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 ToFix3(this Vector3 value)
        {
            return MaphyUnityConvert.ToFix3(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 ToVector3(this fix3 value)
        {
            return MaphyUnityConvert.ToVector3(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 ToFix4(this Vector4 value)
        {
            return MaphyUnityConvert.ToFix4(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector4 ToVector4(this fix4 value)
        {
            return MaphyUnityConvert.ToVector4(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FixQuaternion ToFixQuaternion(this UnityQuaternion value)
        {
            return MaphyUnityConvert.ToFixQuaternion(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static UnityQuaternion ToUnityQuaternion(this FixQuaternion value)
        {
            return MaphyUnityConvert.ToUnityQuaternion(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4x4 ToFix4x4(this Matrix4x4 value)
        {
            return MaphyUnityConvert.ToFix4x4(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix4x4 ToMatrix4x4(this fix4x4 value)
        {
            return MaphyUnityConvert.ToMatrix4x4(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static AABB ToAABB(this UnityBounds value)
        {
            return MaphyUnityConvert.ToAABB(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static UnityBounds ToBounds(this AABB value)
        {
            return MaphyUnityConvert.ToBounds(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static MaphyRay ToMaphyRay(this UnityRay value)
        {
            return MaphyUnityConvert.ToRay(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static UnityRay ToUnityRay(this MaphyRay value)
        {
            return MaphyUnityConvert.ToUnityRay(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static MaphyUnityRaycastHit ToUnityRaycastHit(this PhysicsWorldRaycastHit value)
        {
            return MaphyUnityConvert.ToUnityRaycastHit(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static MaphyUnityShapeCastHit ToUnityShapeCastHit(this PhysicsWorldShapeCastHit value)
        {
            return MaphyUnityConvert.ToUnityShapeCastHit(value);
        }
    }

    public readonly struct MaphyUnityRaycastHit
    {
        public readonly ColliderHandle collider;
        public readonly BodyHandle body;
        public readonly float distance;
        public readonly Vector3 point;
        public readonly Vector3 normal;
        public readonly UnityBounds bounds;

        public MaphyUnityRaycastHit(
            ColliderHandle collider,
            BodyHandle body,
            float distance,
            Vector3 point,
            Vector3 normal,
            UnityBounds bounds)
        {
            this.collider = collider;
            this.body = body;
            this.distance = distance;
            this.point = point;
            this.normal = normal;
            this.bounds = bounds;
        }

        public bool IsValid => collider.IsValid;
    }

    public readonly struct MaphyUnityShapeCastHit
    {
        public readonly ColliderHandle collider;
        public readonly BodyHandle body;
        public readonly float fraction;
        public readonly Vector3 point;
        public readonly Vector3 normal;
        public readonly UnityBounds bounds;

        public MaphyUnityShapeCastHit(
            ColliderHandle collider,
            BodyHandle body,
            float fraction,
            Vector3 point,
            Vector3 normal,
            UnityBounds bounds)
        {
            this.collider = collider;
            this.body = body;
            this.fraction = fraction;
            this.point = point;
            this.normal = normal;
            this.bounds = bounds;
        }

        public bool IsValid => collider.IsValid;
    }
}
