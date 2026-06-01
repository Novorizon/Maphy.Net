#if MAPHY_UNITY_MATHEMATICS
using System.Runtime.CompilerServices;
using Maphy.Mathematics;
using Unity.Mathematics;
using FixQuaternion = Maphy.Mathematics.quaternion;
using UnityQuaternion = Unity.Mathematics.quaternion;

namespace Maphy.Unity
{
    public static class MaphyUnityMathematicsConvert
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix ToFix(float value)
        {
            return MaphyUnityConvert.ToFix(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float ToFloat(fix value)
        {
            return MaphyUnityConvert.ToFloat(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix2 ToFix2(float2 value)
        {
            return new fix2(value.x, value.y);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 ToFloat2(fix2 value)
        {
            return new float2(value.x, value.y);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 ToFix3(float3 value)
        {
            return new fix3(value.x, value.y, value.z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 ToFloat3(fix3 value)
        {
            return new float3(value.x, value.y, value.z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 ToFix4(float4 value)
        {
            return new fix4(value.x, value.y, value.z, value.w);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 ToFloat4(fix4 value)
        {
            return new float4(value.x, value.y, value.z, value.w);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FixQuaternion ToFixQuaternion(UnityQuaternion value)
        {
            return new FixQuaternion(value.value.x, value.value.y, value.value.z, value.value.w);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static UnityQuaternion ToUnityMathematicsQuaternion(FixQuaternion value)
        {
            return new UnityQuaternion(value.value.x, value.value.y, value.value.z, value.value.w);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3x3 ToFix3x3(float3x3 value)
        {
            return new fix3x3(
                ToFix3(value.c0),
                ToFix3(value.c1),
                ToFix3(value.c2));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x3 ToFloat3x3(fix3x3 value)
        {
            return new float3x3(
                ToFloat3(value.c0),
                ToFloat3(value.c1),
                ToFloat3(value.c2));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4x4 ToFix4x4(float4x4 value)
        {
            return new fix4x4(
                ToFix4(value.c0),
                ToFix4(value.c1),
                ToFix4(value.c2),
                ToFix4(value.c3));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 ToFloat4x4(fix4x4 value)
        {
            return new float4x4(
                ToFloat4(value.c0),
                ToFloat4(value.c1),
                ToFloat4(value.c2),
                ToFloat4(value.c3));
        }
    }

    public static class MaphyUnityMathematicsConversionExtensions
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix2 ToFix2(this float2 value)
        {
            return MaphyUnityMathematicsConvert.ToFix2(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 ToFloat2(this fix2 value)
        {
            return MaphyUnityMathematicsConvert.ToFloat2(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 ToFix3(this float3 value)
        {
            return MaphyUnityMathematicsConvert.ToFix3(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 ToFloat3(this fix3 value)
        {
            return MaphyUnityMathematicsConvert.ToFloat3(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 ToFix4(this float4 value)
        {
            return MaphyUnityMathematicsConvert.ToFix4(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 ToFloat4(this fix4 value)
        {
            return MaphyUnityMathematicsConvert.ToFloat4(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FixQuaternion ToFixQuaternion(this UnityQuaternion value)
        {
            return MaphyUnityMathematicsConvert.ToFixQuaternion(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static UnityQuaternion ToUnityMathematicsQuaternion(this FixQuaternion value)
        {
            return MaphyUnityMathematicsConvert.ToUnityMathematicsQuaternion(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3x3 ToFix3x3(this float3x3 value)
        {
            return MaphyUnityMathematicsConvert.ToFix3x3(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x3 ToFloat3x3(this fix3x3 value)
        {
            return MaphyUnityMathematicsConvert.ToFloat3x3(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4x4 ToFix4x4(this float4x4 value)
        {
            return MaphyUnityMathematicsConvert.ToFix4x4(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 ToFloat4x4(this fix4x4 value)
        {
            return MaphyUnityMathematicsConvert.ToFloat4x4(value);
        }
    }
}
#endif
