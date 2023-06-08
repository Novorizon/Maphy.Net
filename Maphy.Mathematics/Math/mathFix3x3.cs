using System;
using System.Runtime.InteropServices;
using System.Runtime.CompilerServices;

namespace Maphy.Mathematics
{
    public static partial class math
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3x3 fix3x3(fix3 c0, fix3 c1, fix3 c2) { return new fix3x3(c0, c1, c2); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3x3 fix3x3(fix m00, fix m01, fix m02,
                                                 fix m10, fix m11, fix m12,
                                                 fix m20, fix m21, fix m22)
        {
            return new fix3x3(
                m00, m01, m02,
                m10, m11, m12,
                m20, m21, m22);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3x3 fix3x3(fix v) { return new fix3x3(v); }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3x3 fix3x3(int v) { return new fix3x3(v); }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3x3 mul(fix3x3 x, fix3x3 y)
        {
            return x * transpose(y);
        }

        ///转置矩阵
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3x3 transpose(fix3x3 v)
        {
            return fix3x3(
                v.c0.x, v.c0.y, v.c0.z,
                v.c1.x, v.c1.y, v.c1.z,
                v.c2.x, v.c2.y, v.c2.z);
        }

        /// 逆矩阵
        public static fix3x3 inverse(fix3x3 m)
        {
            fix3 c0 = m.c0;
            fix3 c1 = m.c1;
            fix3 c2 = m.c2;

            fix3 t0 = new fix3(c1.x, c2.x, c0.x);
            fix3 t1 = new fix3(c1.y, c2.y, c0.y);
            fix3 t2 = new fix3(c1.z, c2.z, c0.z);

            fix3 m0 = t1 * t2.yzx - t1.yzx * t2;
            fix3 m1 = t0.yzx * t2 - t0 * t2.yzx;
            fix3 m2 = t0 * t1.yzx - t0.yzx * t1;

            fix rcpDet = 1 / csum(t0.zxy * m0);
            return fix3x3(m0, m1, m2) * rcpDet;
        }

        /// 行列式
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix determinant(fix3x3 m)
        {
            fix3 c0 = m.c0;
            fix3 c1 = m.c1;
            fix3 c2 = m.c2;

            fix m00 = c1.y * c2.z - c1.z * c2.y;
            fix m01 = c0.y * c2.z - c0.z * c2.y;
            fix m02 = c0.y * c1.z - c0.z * c1.y;

            return c0.x * m00 - c1.x * m01 + c2.x * m02;
        }
    }
}
