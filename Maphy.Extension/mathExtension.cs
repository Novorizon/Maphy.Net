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
    public static partial class math
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix fix(float value)
        {
            return MaphyUnityConvert.ToFix(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Float(fix value)
        {
            return MaphyUnityConvert.ToFloat(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix2 fix2(Vector2 value)
        {
            return MaphyUnityConvert.ToFix2(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2 Vector2(fix2 value)
        {
            return MaphyUnityConvert.ToVector2(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 fix3(Vector3 value)
        {
            return MaphyUnityConvert.ToFix3(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 Vector3(fix3 value)
        {
            return MaphyUnityConvert.ToVector3(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 fix4(Vector4 value)
        {
            return MaphyUnityConvert.ToFix4(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector4 Vector4(fix4 value)
        {
            return MaphyUnityConvert.ToVector4(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FixQuaternion quaternion(UnityQuaternion value)
        {
            return MaphyUnityConvert.ToFixQuaternion(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static UnityQuaternion Quaternion(FixQuaternion value)
        {
            return MaphyUnityConvert.ToUnityQuaternion(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3x3 fix3x3(Matrix4x4 value)
        {
            return MaphyUnityConvert.ToFix3x3(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix4x4 Matrix4x4(fix3x3 value)
        {
            return MaphyUnityConvert.ToMatrix4x4(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4x4 fix4x4(Matrix4x4 value)
        {
            return MaphyUnityConvert.ToFix4x4(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix4x4 Matrix4x4(fix4x4 value)
        {
            return MaphyUnityConvert.ToMatrix4x4(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static AABB AABB(UnityBounds value)
        {
            return MaphyUnityConvert.ToAABB(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static UnityBounds Bounds(AABB value)
        {
            return MaphyUnityConvert.ToBounds(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static MaphyRay Ray(UnityRay value)
        {
            return MaphyUnityConvert.ToRay(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static UnityRay Ray(MaphyRay value)
        {
            return MaphyUnityConvert.ToUnityRay(value);
        }
    }
}
