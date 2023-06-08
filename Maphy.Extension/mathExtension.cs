//using System;
//using System.Runtime.InteropServices;
//using System.Runtime.CompilerServices;
//using Mathematica;
//using UnityEngine;

//namespace Mathematica
//{
//    public static partial class math
//    {

//        [MethodImpl(MethodImplOptions.AggressiveInlining)]
//        public static fix3 fix3(Vector3 x) { return new fix3(x.x, x.y, x.z); }

//        [MethodImpl(MethodImplOptions.AggressiveInlining)]
//        public static Vector3 Vector3(fix3 x) { return new Vector3(x.x, x.y, x.z); }

//        [MethodImpl(MethodImplOptions.AggressiveInlining)]
//        public static quaternion quaternion(Quaternion x) { return new quaternion(x.x, x.y, x.z, x.w); }

//        [MethodImpl(MethodImplOptions.AggressiveInlining)]
//        public static Quaternion Quaternion(quaternion x) { return new Quaternion(x.value.x, x.value.y, x.value.z, x.value.w); }

//        [MethodImpl(MethodImplOptions.AggressiveInlining)]
//        public static quaternion quaternion(Unity.Mathematics.quaternion x) { return new quaternion(x.value.x, x.value.y, x.value.z, x.value.w); }

//        [MethodImpl(MethodImplOptions.AggressiveInlining)]
//        public static Unity.Mathematics.quaternion quaternion(quaternion x) { return new Unity.Mathematics.quaternion(x.value.x, x.value.y, x.value.z, x.value.w); }


//        [MethodImpl(MethodImplOptions.AggressiveInlining)]
//        public static fix4 fix4(Unity.Mathematics.float4 x) { return new fix4(x.x, x.y, x.z, x.w); }

//        [MethodImpl(MethodImplOptions.AggressiveInlining)]
//        public static Unity.Mathematics.float4 float4(fix4 x) { return new Unity.Mathematics.float4(x.x, x.y, x.z, x.w); }


//        [MethodImpl(MethodImplOptions.AggressiveInlining)]
//        public static fix3x3 fix3x3(Unity.Mathematics.float3x3 x)
//        {
//            return new fix3x3(
//            x.c0.x, x.c1.x, x.c2.x,
//            x.c0.y, x.c1.y, x.c2.y,
//            x.c0.z, x.c1.z, x.c2.z
//            );
//        }

//        [MethodImpl(MethodImplOptions.AggressiveInlining)]
//        public static Unity.Mathematics.float3x3 float3x3(fix3x3 x)
//        {
//            return new Unity.Mathematics.float3x3(
//            x.c0.x, x.c1.x, x.c2.x,
//            x.c0.y, x.c1.y, x.c2.y,
//            x.c0.z, x.c1.z, x.c2.z
//            );
//        }


//        [MethodImpl(MethodImplOptions.AggressiveInlining)]
//        public static Unity.Mathematics.float4x4 float4x4(fix4x4 x)
//        {
//            return new Unity.Mathematics.float4x4(
//            x.c0.x, x.c1.x, x.c2.x, x.c3.x,
//            x.c0.y, x.c1.y, x.c2.y, x.c3.y,
//            x.c0.z, x.c1.z, x.c2.z, x.c3.z,
//            x.c0.w, x.c1.w, x.c2.w, x.c3.w
//            );
//        }
//    }
//}