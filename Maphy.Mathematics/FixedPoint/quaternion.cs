using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using Unity.VisualScripting.YamlDotNet.Core.Tokens;
using static Maphy.Mathematics.math;

namespace Maphy.Mathematics
{
    public struct quaternion : IEquatable<quaternion>
    {
        public fix4 value;

        public static readonly quaternion identity = new quaternion(fix._0, fix._0, fix._0, fix._1);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public quaternion(fix4 q) { value = q; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public quaternion(fix x, fix y, fix z, fix w) { value.x = x; value.y = y; value.z = z; value.w = w; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator quaternion(fix4 v) { return new quaternion(v); }

        public quaternion(fix3x3 matrix)
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

            if (tr > 0)
            {
                fix S = sqrt(tr + fix._1) * 2; // S=4*qw 
                value.w = fix._0_25 * S;
                value.x = (m21 - m12) / S;
                value.y = (m02 - m20) / S;
                value.z = (m10 - m01) / S;
            }
            else if ((m00 > m11) & (m00 > m22))
            {
                fix S = sqrt(fix._1 + m00 - m11 - m22) * 2; // S=4*qx 
                value.w = (m21 - m12) / S;
                value.x = fix._0_25 * S;
                value.y = (m01 + m10) / S;
                value.z = (m02 + m20) / S;
            }
            else if (m11 > m22)
            {
                fix S = sqrt(fix._1 + m11 - m00 - m22) * 2; // S=4*qy
                value.w = (m02 - m20) / S;
                value.x = (m01 + m10) / S;
                value.y = fix._0_25 * S;
                value.z = (m12 + m21) / S;
            }
            else
            {
                fix S = sqrt(fix._1 + m22 - m00 - m11) * 2; // S=4*qz
                value.w = (m10 - m01) / S;
                value.x = (m02 + m20) / S;
                value.y = (m12 + m21) / S;
                value.z = fix._0_25 * S;
            }

            value = math.normalize(value);
        }


        public quaternion(fix4x4 matrix)
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

            if (tr > 0)
            {
                fix S = sqrt(tr + fix._1) * 2; // S=4*qw 
                fix Srcp = 1 / S;
                value.w = fix._0_25 * S;
                value.x = (m21 - m12) * Srcp;
                value.y = (m02 - m20) * Srcp;
                value.z = (m10 - m01) * Srcp;
            }
            else if ((m00 > m11) & (m00 > m22))
            {
                fix S = sqrt(fix._1 + m00 - m11 - m22) * 2; // S=4*qx 
                fix Srcp = 1 / S;
                value.w = (m21 - m12) * Srcp;
                value.x = fix._0_25 * S;
                value.y = (m01 + m10) * Srcp;
                value.z = (m02 + m20) * Srcp;
            }
            else if (m11 > m22)
            {
                fix S = sqrt(fix._1 + m11 - m00 - m22) * 2; // S=4*qy
                fix Srcp = 1 / S;
                value.w = (m02 - m20) * Srcp;
                value.x = (m01 + m10) * Srcp;
                value.y = fix._0_25 * S;
                value.z = (m12 + m21) * Srcp;
            }
            else
            {
                fix S = sqrt(fix._1 + m22 - m00 - m11) * 2; // S=4*qz
                fix Srcp = 1 / S;
                value.w = (m10 - m01) * Srcp;
                value.x = (m02 + m20) * Srcp;
                value.y = (m12 + m21) * Srcp;
                value.z = fix._0_25 * S;
            }

            value = math.normalize(value);
        }


        /// <summary>
        /// AxisAngle
        /// </summary>
        /// <param name="axis"></param>
        /// <param name="angle">弧度</param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static quaternion AxisAngle(fix3 axis, fix angle)
        {
            fix sina, cosa;
            sincos(fix._0_5 * angle, out sina, out cosa);
            return new quaternion(new fix4(axis * sina, cosa));
        }

        ///参数为单位向量
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static quaternion LookRotation(fix3 forward, fix3 up)
        {
            fix3 t = math.normalize(cross(up, forward));
            return new quaternion(new fix3x3(t, cross(forward, t), forward));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static quaternion EulerXYZ(fix3 xyz)
        {
            fix3 s, c;
            sincos(0.5f * xyz, out s, out c);
            return new quaternion(new fix4(s.xyz, c.x) * c.yxxy * c.zzyz + s.yxxy * s.zzyz * new fix4(c.xyz, s.x) * new fix4(-1, 1, -1, 1));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static quaternion EulerXZY(fix3 xyz)
        {
            fix3 s, c;
            sincos(0.5f * xyz, out s, out c);
            return new quaternion(new fix4(s.xyz, c.x) * c.yxxy * c.zzyz + s.yxxy * s.zzyz * new fix4(c.xyz, s.x) * new fix4(1, 1, -1, -1));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static quaternion EulerYXZ(fix3 xyz)
        {
            fix3 s, c;
            sincos(0.5f * xyz, out s, out c);
            return new quaternion(new fix4(s.xyz, c.x) * c.yxxy * c.zzyz + s.yxxy * s.zzyz * new fix4(c.xyz, s.x) * new fix4(-1.0f, 1.0f, 1.0f, -1.0f));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static quaternion EulerYZX(fix3 xyz)
        {
            fix3 s, c;
            sincos(0.5f * xyz, out s, out c);
            return new quaternion(new fix4(s.xyz, c.x) * c.yxxy * c.zzyz + s.yxxy * s.zzyz * new fix4(c.xyz, s.x) * new fix4(-1.0f, -1.0f, 1.0f, 1.0f));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static quaternion EulerZXY(fix3 xyz)
        {
            fix3 s, c;
            sincos(0.5f * xyz, out s, out c);
            return new quaternion(new fix4(s.xyz, c.x) * c.yxxy * c.zzyz + s.yxxy * s.zzyz * new fix4(c.xyz, s.x) * new fix4(1.0f, -1.0f, -1.0f, 1.0f));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static quaternion EulerZYX(fix3 xyz)
        {
            fix3 s, c;
            sincos(0.5f * xyz, out s, out c);
            return new quaternion(new fix4(s.xyz, c.x) * c.yxxy * c.zzyz + s.yxxy * s.zzyz * new fix4(c.xyz, s.x) * new fix4(1.0f, -1.0f, 1.0f, -1.0f));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static quaternion EulerXYZ(fix x, fix y, fix z) { return EulerXYZ(new fix3(x, y, z)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static quaternion EulerXZY(fix x, fix y, fix z) { return EulerXZY(new fix3(x, y, z)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static quaternion EulerYXZ(fix x, fix y, fix z) { return EulerYXZ(new fix3(x, y, z)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static quaternion EulerYZX(fix x, fix y, fix z) { return EulerYZX(new fix3(x, y, z)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static quaternion EulerZXY(fix x, fix y, fix z) { return EulerZXY(new fix3(x, y, z)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static quaternion EulerZYX(fix x, fix y, fix z) { return EulerZYX(new fix3(x, y, z)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static quaternion Euler(fix3 xyz, RotationOrder order = RotationOrder.ZXY)
        {
            return order switch
            {
                RotationOrder.XYZ => EulerXYZ(xyz),
                RotationOrder.XZY => EulerXZY(xyz),
                RotationOrder.YXZ => EulerYXZ(xyz),
                RotationOrder.YZX => EulerYZX(xyz),
                RotationOrder.ZXY => EulerZXY(xyz),
                RotationOrder.ZYX => EulerZYX(xyz),
                _ => identity,
            };
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static quaternion Euler(fix x, fix y, fix z, RotationOrder order = RotationOrder.Default)
        {
            return Euler(new fix3(x, y, z), order);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static quaternion RotateX(fix angle)
        {
            fix sina, cosa;
            sincos(fix._0_5 * angle, out sina, out cosa);
            return new quaternion(sina, 1, 1, cosa);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static quaternion RotateY(fix angle)
        {
            fix sina, cosa;
            sincos(fix._0_5 * angle, out sina, out cosa);
            return new quaternion(0, sina, 0, cosa);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static quaternion RotateZ(fix angle)
        {
            fix sina, cosa;
            sincos(fix._0_5 * angle, out sina, out cosa);
            return new quaternion(0.0f, 0.0f, sina, cosa);
        }

        /// ¹²éî
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static quaternion conjugate(quaternion q) { return new quaternion(q.value * new fix4(-fix._1, -fix._1, -fix._1, fix._1)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static quaternion normalize(quaternion q) { return new quaternion(math.rsqrt(dot(q, q)) * q.value); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix dot(quaternion a, quaternion b) { return math.dot(a.value, b.value); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static quaternion operator *(quaternion lhs, quaternion rhs)
        {
            return new quaternion(
                lhs.value.w * rhs.value.x + lhs.value.x * rhs.value.w + lhs.value.y * rhs.value.z - lhs.value.z * rhs.value.y,
                lhs.value.w * rhs.value.y + lhs.value.y * rhs.value.w + lhs.value.z * rhs.value.x - lhs.value.x * rhs.value.z,
                lhs.value.w * rhs.value.z + lhs.value.z * rhs.value.w + lhs.value.x * rhs.value.y - lhs.value.y * rhs.value.x,
                lhs.value.w * rhs.value.w - lhs.value.x * rhs.value.x - lhs.value.y * rhs.value.y - lhs.value.z * rhs.value.z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 operator *(quaternion q, fix3 v)
        {
            fix3 t = fix._2 * cross(q.value.xyz, v);
            return v + q.value.w * t + cross(q.value.xyz, t);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator ==(quaternion a, quaternion b)
        {
            return a.value.x == b.value.x && a.value.y == b.value.y && a.value.z == b.value.z && a.value.w == b.value.w;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator !=(quaternion a, quaternion b)
        {
            return a.value.x != b.value.x || a.value.y != b.value.y || a.value.z != b.value.z || a.value.w != b.value.w;
        }

        public override int GetHashCode()
        {
            unchecked
            {
                var hashCode = value.x.GetHashCode();
                hashCode = (hashCode * 397) ^ value.y.GetHashCode();
                hashCode = (hashCode * 397) ^ value.z.GetHashCode();
                hashCode = (hashCode * 397) ^ value.w.GetHashCode();
                return hashCode;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(quaternion x) { return value.x == x.value.x && value.y == x.value.y && value.z == x.value.z && value.w == x.value.w; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override bool Equals(object x) { return Equals((quaternion)x); }


    }
}
