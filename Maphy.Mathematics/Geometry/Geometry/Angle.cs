using Maphy.Physics;
using System;

namespace Maphy.Mathematics
{
    public static partial class Geometry
    {

        /// Determine the signed angle between two vectors, with normal 'n' as the rotation axis.
        /// 两个向量之间的夹角，有符号. 顺时针为正 [0,180] 逆时针为正(0,180)
        public static fix AngleSigned(fix3 from, fix3 to, fix3 n)
        {
            return math.atan2(math.dot(n, math.cross(from, to)), math.dot(from, to)) * math.Rad2Deg;
        }

        /// Determine the signed angle between two vectors
        /// 两个向量之间的夹角，无符号 [0,180]
        public static fix kEpsilon = fix.Raw(1);
        public static fix kEpsilonNormalSqrt = fix.Raw(1);
        public static fix Angle(fix3 from, fix3 to)
        {
            fix denominator = math.sqrt(math.lengthsq(from) * math.lengthsq(to));
            if (fix.IsNaN(denominator))
                return fix.NaN;
            if (denominator <= math.max(fix.Zero, kEpsilonNormalSqrt))
                return fix.Zero;

            fix dot = math.dot(from, to) / denominator;
            if (fix.IsNaN(dot))
                return fix.NaN;
            return math.acos(math.clamp(dot, -fix.One, fix.One)) * math.Rad2Deg;
        }


        //点point在OA、OB组成的夹角内外
        public static bool IsPointInAngle(fix3 point, fix3 O, fix3 A, fix3 B)
        {
            fix3 p = point - O;
            fix3 a = A - O;
            fix3 b = B - O;
            fix dot1 = math.dot(a, p);
            fix dot2 = math.dot(b, p);

            return dot1 * dot2 <= 0;
        }

        //点在同一起点的射线组成的夹角内外
        public static bool IsPointInBetweenTheRays(fix3 point, Ray ray1, Ray ray2)
        {
            fix3 p = point - ray1.origin;
            fix dot1 = math.dot(ray1.direction, p);
            fix dot2 = math.dot(ray2.direction, p);

            return dot1 * dot2 <= 0;
        }
    }

}


