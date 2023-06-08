using System;
using System.Runtime.InteropServices;
using System.Runtime.CompilerServices;
using Maphy.Mathematics;
using UnityEngine;

namespace Mathematica
{
    public static partial class math
    {

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 fix3(Vector3 x) { return new fix3(x.x, x.y, x.z); }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 Vector3(fix3 x) { return new Vector3(x.x, x.y, x.z); }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 fix4(Vector4 x) { return new fix4(x.x, x.y, x.z, x.w); }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector4 Vector4(fix4 x) { return new Vector4(x.x, x.y, x.z, x.w); }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static quaternion quaternion(Quaternion x) { return new quaternion(x.x, x.y, x.z, x.w); }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Quaternion Quaternion(quaternion x) { return new Quaternion(x.value.x, x.value.y, x.value.z, x.value.w); }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4x4 fix3x3(Matrix4x4 x)
        {
            return new fix4x4(
            x.m00, x.m01, x.m02, x.m03,
            x.m10, x.m11, x.m12, x.m13,
            x.m20, x.m21, x.m22, x.m23,
            x.m30, x.m31, x.m32, x.m33
            );
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix4x4 Matrix4x4(fix4x4 x)
        {
            Vector4 c0 = Vector4(x.c0);
            Vector4 c1 = Vector4(x.c1);
            Vector4 c2 = Vector4(x.c2);
            Vector4 c3 = Vector4(x.c3);
            return new Matrix4x4(c0, c1, c2, c3);
        }
    }
}