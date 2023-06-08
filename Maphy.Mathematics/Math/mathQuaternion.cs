using System;
using System.Runtime.InteropServices;
using System.Runtime.CompilerServices;

namespace Maphy.Mathematics
{
    public static partial class math
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static quaternion quaternion(fix x, fix y, fix z, fix w) { return new quaternion(x, y, z, w); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static quaternion quaternion(fix4 value) { return new quaternion(value); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static quaternion quaternion(fix3x3 m) { return new quaternion(m); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static quaternion quaternion(fix4x4 m) { return new quaternion(m); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static quaternion conjugate(quaternion q)
        {
            return quaternion(q.value * new fix4(-1, -1, -1, 1));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static quaternion inverse(quaternion q)
        {
            fix4 x = q.value;
            return quaternion(rcp(dot(x, x)) * x * new fix4(-1, -1, -1, 1));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix dot(quaternion a, quaternion b)
        {
            return dot(a.value, b.value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix length(quaternion q)
        {
            return sqrt(dot(q.value, q.value));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix lengthsq(quaternion q)
        {
            return dot(q.value, q.value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static quaternion normalize(quaternion q)
        {
            fix4 x = q.value;
            return quaternion(rsqrt(dot(x, x)) * x);
        }

        /// <summary>Returns the result of transforming the quaternion b by the quaternion a.</summary>
        /// <param name="a">The quaternion on the left.</param>
        /// <param name="b">The quaternion on the right.</param>
        /// <returns>The result of transforming quaternion b by the quaternion a.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static quaternion mul(quaternion a, quaternion b)
        {
            return quaternion(a.value.wwww * b.value + (a.value.xyzx * b.value.wwwx + a.value.yzxy * b.value.zxyy) * new fix4(1, 1, 1, -1) - a.value.zxyz * b.value.yzxz);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 mul(quaternion q, fix3 v)
        {
            fix3 t = fix._2 * cross(q.value.xyz, v);
            return v + q.value.w * t + math.cross(q.value.xyz, t);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 rotate(quaternion q, fix3 v)
        {
            fix3 t = fix._2 * cross(q.value.xyz, v);
            return v + q.value.w * t + cross(q.value.xyz, t);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static quaternion nlerp(quaternion q1, quaternion q2, fix t)
        {
            fix dt = dot(q1, q2);
            if (dt < 0)
            {
                q2.value = -q2.value;
            }

            return normalize(quaternion(lerp(q1.value, q2.value, t)));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static quaternion slerp(quaternion q1, quaternion q2, fix t)
        {
            fix dt = dot(q1, q2);
            if (dt < 0)
            {
                dt = -dt;
                q2.value = -q2.value;
            }

            if (dt < fix._0_9995)
            {
                fix angle = acos(dt);
                fix s = rsqrt(1 - dt * dt);
                fix w1 = sin(angle * (1 - t)) * s;
                fix w2 = sin(angle * t) * s;
                return quaternion(q1.value * w1 + q2.value * w2);
            }
            else
            {
                // if the angle is small, use linear interpolation
                return nlerp(q1, q2, t);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix angle(quaternion q1, quaternion q2)
        {
            fix diff = asin(length(normalize(mul(conjugate(q1), q2)).value.xyz));
            return diff + diff;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 forward(quaternion q) { return mul(q, new fix3(0, 0, 1)); }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static quaternion Rotation(fix3x3 matrix)
        {
            fix m00 = matrix.c0.x;
            fix m01 = matrix.c1.x;
            fix m02 = matrix.c2.x;

            fix m10 = matrix.c0.y;
            fix m11 = matrix.c1.y;
            fix m12 = matrix.c2.y;

            fix m20 = matrix.c0.z;
            fix m21 = matrix.c1.z;
            fix m22 = matrix.c2.z;
            fix tr = m00 + m11 + m22;

            fix qx = fix._0, qy = fix._0, qz = fix._0, qw = fix._1;
            if (tr > 0)
            {
                fix S = sqrt(tr + fix._1) * 2; // S=4*qw 
                qw = fix._0_25 * S;
                qx = (m21 - m12) / S;
                qy = (m02 - m20) / S;
                qz = (m10 - m01) / S;
            }
            else if ((m00 > m11) & (m00 > m22))
            {
                fix S = sqrt(fix._1 + m00 - m11 - m22) * 2; // S=4*qx 
                qw = (m21 - m12) / S;
                qx = fix._0_25 * S;
                qy = (m01 + m10) / S;
                qz = (m02 + m20) / S;
            }
            else if (m11 > m22)
            {
                fix S = sqrt(fix._1 + m11 - m00 - m22) * 2; // S=4*qy
                qw = (m02 - m20) / S;
                qx = (m01 + m10) / S;
                qy = fix._0_25 * S;
                qz = (m12 + m21) / S;
            }
            else
            {
                fix S = sqrt(fix._1 + m22 - m00 - m11) * 2; // S=4*qz
                qw = (m10 - m01) / S;
                qx = (m02 + m20) / S;
                qy = (m12 + m21) / S;
                qz = fix._0_25 * S;
            }

            quaternion quaternion = new quaternion(qx, qy, qz, qw);
            return quaternion.normalize(quaternion);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static quaternion Rotation(fix4x4 matrix)
        {
            fix m00 = matrix.c0.x;
            fix m01 = matrix.c1.x;
            fix m02 = matrix.c2.x;

            fix m10 = matrix.c0.y;
            fix m11 = matrix.c1.y;
            fix m12 = matrix.c2.y;

            fix m20 = matrix.c0.z;
            fix m21 = matrix.c1.z;
            fix m22 = matrix.c2.z;
            fix tr = m00 + m11 + m22;

            fix qx = fix._0, qy = fix._0, qz = fix._0, qw = fix._1;
            if (tr > 0)
            {
                fix S = sqrt(tr + fix._1) * 2; // S=4*qw 
                qw = fix._0_25 * S;
                qx = (m21 - m12) / S;
                qy = (m02 - m20) / S;
                qz = (m10 - m01) / S;
            }
            else if ((m00 > m11) & (m00 > m22))
            {
                fix S = sqrt(fix._1 + m00 - m11 - m22) * 2; // S=4*qx 
                qw = (m21 - m12) / S;
                qx = fix._0_25 * S;
                qy = (m01 + m10) / S;
                qz = (m02 + m20) / S;
            }
            else if (m11 > m22)
            {
                fix S = sqrt(fix._1 + m11 - m00 - m22) * 2; // S=4*qy
                qw = (m02 - m20) / S;
                qx = (m01 + m10) / S;
                qy = fix._0_25 * S;
                qz = (m12 + m21) / S;
            }
            else
            {
                fix S = sqrt(fix._1 + m22 - m00 - m11) * 2; // S=4*qz
                qw = (m10 - m01) / S;
                qx = (m02 + m20) / S;
                qy = (m12 + m21) / S;
                qz = fix._0_25 * S;
            }

            quaternion quaternion = new quaternion(qx, qy, qz, qw);
            return quaternion.normalize(quaternion);
        }
    }
}
