using System;
using System.Runtime.InteropServices;
using System.Runtime.CompilerServices;

namespace Maphy.Mathematics
{
    public static partial class math
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4x4 fix4x4(fix4 c0, fix4 c1, fix4 c2, fix4 c3) { return new fix4x4(c0, c1, c2, c3); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4x4 fix4x4(fix m00, fix m01, fix m02, fix m03,
                                        fix m10, fix m11, fix m12, fix m13,
                                        fix m20, fix m21, fix m22, fix m23,
                                        fix m30, fix m31, fix m32, fix m33)
        {
            return new fix4x4(
                m00, m01, m02, m03,
                m10, m11, m12, m13,
                m20, m21, m22, m23,
                m30, m31, m32, m33);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4x4 fix4x4(fix v) { return new fix4x4(v); }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4x4 fix4x4(int v) { return new fix4x4(v); }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4x4 mul(fix4x4 x, fix4x4 y)
        {
            return x * transpose(y);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 rotate(fix4x4 a, fix3 b)
        {
            fix4 r = a.c0 * b.x + a.c1 * b.y + a.c2 * b.z;
            return new fix3(r.x, r.y, r.z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 transform(fix4x4 a, fix3 b)
        {
            fix4 r = a.c0 * b.x + a.c1 * b.y + a.c2 * b.z + a.c3;
            return new fix3(r.x, r.y, r.z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4x4 transpose(fix4x4 v)
        {
            return fix4x4(
                v.c0.x, v.c0.y, v.c0.z, v.c0.w,
                v.c1.x, v.c1.y, v.c1.z, v.c1.w,
                v.c2.x, v.c2.y, v.c2.z, v.c2.w,
                v.c3.x, v.c3.y, v.c3.z, v.c3.w);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix determinant(fix4x4 m)
        {
            fix4 c0 = m.c0;
            fix4 c1 = m.c1;
            fix4 c2 = m.c2;
            fix4 c3 = m.c3;

            fix m00 = c1.y * (c2.z * c3.w - c2.w * c3.z) - c2.y * (c1.z * c3.w - c1.w * c3.z) + c3.y * (c1.z * c2.w - c1.w * c2.z);
            fix m01 = c0.y * (c2.z * c3.w - c2.w * c3.z) - c2.y * (c0.z * c3.w - c0.w * c3.z) + c3.y * (c0.z * c2.w - c0.w * c2.z);
            fix m02 = c0.y * (c1.z * c3.w - c1.w * c3.z) - c1.y * (c0.z * c3.w - c0.w * c3.z) + c3.y * (c0.z * c1.w - c0.w * c1.z);
            fix m03 = c0.y * (c1.z * c2.w - c1.w * c2.z) - c1.y * (c0.z * c2.w - c0.w * c2.z) + c2.y * (c0.z * c1.w - c0.w * c1.z);

            return c0.x * m00 - c1.x * m01 + c2.x * m02 - c3.x * m03;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4x4 TRS(fix3 t, quaternion r, fix3 s)
        {
            fix4x4 res;
            res.c0.x = (1 - 2 * (r.value.y * r.value.y + r.value.z * r.value.z)) * s.x;
            res.c0.y = (r.value.x * r.value.y + r.value.z * r.value.w) * s.x * 2;
            res.c0.z = (r.value.x * r.value.z - r.value.y * r.value.w) * s.x * 2;
            res.c0.w = 0;
            res.c1.x = (r.value.x * r.value.y - r.value.z * r.value.w) * s.y * 2;
            res.c1.y = (1 - 2 * (r.value.x * r.value.x + r.value.z * r.value.z)) * s.y;
            res.c1.z = (r.value.y * r.value.z + r.value.x * r.value.w) * s.y * 2;
            res.c1.w = 0;
            res.c2.x = (r.value.x * r.value.z + r.value.y * r.value.w) * s.z * 2;
            res.c2.y = (r.value.y * r.value.z - r.value.x * r.value.w) * s.z * 2;
            res.c2.z = (1 - 2 * (r.value.x * r.value.x + r.value.y * r.value.y)) * s.z;
            res.c2.w = 0;
            res.c3.x = t.x;
            res.c3.y = t.y;
            res.c3.z = t.z;
            res.c3.w = 1;
            return res;
        }
    }
}
