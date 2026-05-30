using Maphy.Mathematics;

namespace Maphy.Physics
{
    public static partial class Physics
    {
        internal static bool IsOverlap(AABB aabb, fix3 point)
        {
            return aabb.min.x <= point.x
                && aabb.max.x >= point.x
                && aabb.min.y <= point.y
                && aabb.max.y >= point.y
                && aabb.min.z <= point.z
                && aabb.max.z >= point.z;
        }

        internal static bool IsOverlap(AABB a, AABB b)
        {
            return a.min.x <= b.max.x
                && a.max.x >= b.min.x
                && a.min.y <= b.max.y
                && a.max.y >= b.min.y
                && a.min.z <= b.max.z
                && a.max.z >= b.min.z;
        }

        public static bool IsOverlap(AABB aabb, Ray ray)
        {
            fix tMin = fix.Min;
            fix tMax = fix.Max;

            for (var i = 0; i < 3; i++)
            {
                if (ray.direction[i] == fix._0)
                {
                    if (ray.origin[i] < aabb.min[i] || ray.origin[i] > aabb.max[i])
                    {
                        return false;
                    }

                    continue;
                }

                fix t0 = (aabb.min[i] - ray.origin[i]) / ray.direction[i];
                fix t1 = (aabb.max[i] - ray.origin[i]) / ray.direction[i];
                fix axisMin = math.min(t0, t1);
                fix axisMax = math.max(t0, t1);

                tMin = math.max(axisMin, tMin);
                tMax = math.min(axisMax, tMax);
                if (tMax < tMin)
                {
                    return false;
                }
            }

            return true;
        }

        public static bool Overlaps(AABB aabb, Sphere sphere)
        {
            return Overlaps(sphere, aabb);
        }

        public static bool Overlaps(AABB a, AABB b)
        {
            return IsOverlap(a, b);
        }

        public static AABB FromMinMax(fix3 min, fix3 max)
        {
            return AABB.FromMinMax(min, max);
        }

        public static AABB Merge(AABB a, AABB b)
        {
            return AABB.FromMinMax(math.min(a.min, b.min), math.max(a.max, b.max));
        }
    }
}