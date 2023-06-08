
using System;
using Maphy.Mathematics;

namespace Maphy.Physics
{
    public static partial class Physics
    {
        internal static bool IsOverlap1(AABB aabb, fix3 point)
        {
            fix3 test0 = aabb.Points[0];
            for (int i = 0; i < 4; i++)
            {
                fix3 test1 = (point - aabb.Points[i - 1]);
                fix3 n = AABB.Normals[i];

                if (math.dot(test0, n) * math.dot(test1, n) > 0)
                    return false;
            }
            return true;
        }
        internal static bool IsOverlap(AABB aabb, fix3 point)
        {
            if (aabb.Min.x > point.x || aabb.Min.y > point.y || aabb.Min.z > point.z || aabb.Max.x < point.x || aabb.Max.y < point.y || aabb.Max.z < point.z)
                return false;
            return true;
        }


        internal static bool IsOverlap(AABB a, AABB b)
        {
            return (a, b) switch
            {
                { a: _, b: _ } when (a.Center.x - a.BevelRadius.x >= b.Center.x + b.BevelRadius.x) => false,
                { a: _, b: _ } when (a.Center.x + a.BevelRadius.x <= b.Center.x - b.BevelRadius.x) => false,
                { a: _, b: _ } when (a.Center.y - a.BevelRadius.y >= b.Center.y + b.BevelRadius.y) => false,
                { a: _, b: _ } when (a.Center.y + a.BevelRadius.y <= b.Center.y - b.BevelRadius.y) => false,
                { a: _, b: _ } when (a.Center.z - a.BevelRadius.z >= b.Center.z + b.BevelRadius.z) => false,
                { a: _, b: _ } when (a.Center.z + a.BevelRadius.z <= b.Center.z - b.BevelRadius.z) => false,
                _ => true,
            };
        }


        public static fix3 CollidePoint(AABB aabb, Sphere sphere)
        {
            fix3 p = sphere.Center - aabb.Center;

            fix3 v = math.max(p, -p);
            fix3 u = math.max(v - aabb.BevelRadius, fix3.zero);
            if (math.length(u) <= sphere.Radius)
            {
                return sphere.Center - sphere.Radius * math.normalize(u);
            }
            return fix3.MinValue;
        }

        public static bool IsOverlap(AABB aabb, Ray ray)
        {
            fix min;
            fix max;
            for (var i = 0; i < 3; i++)
            {
                min = fix.Min;
                max = fix.Max;
                fix t0 = math.min((aabb.Min[i] - ray.origin[i]) / ray.direction[i],
                    (aabb.Max[i] - ray.origin[i]) / ray.direction[i]);
                fix t1 = math.max((aabb.Min[i] - ray.origin[i]) / ray.direction[i],
                    (aabb.Max[i] - ray.origin[i]) / ray.direction[i]);
                min = math.max(t0, min);
                max = math.min(t1, max);
                if (max <= min)
                    return false;
            }
            return true;
        }

        public static AABB MergeAABB(AABB a, AABB b)
        {
            fix3 min = new fix3(math.min(a.Min.x, b.Min.x), math.min(a.Min.y, b.Min.y), math.min(a.Min.z, b.Min.z));
            fix3 max = new fix3(math.max(a.Max.x, b.Max.x), math.max(a.Max.y, b.Max.y), math.max(a.Max.z, b.Max.z));
            return new AABB(min, max);
        }
    }

}
