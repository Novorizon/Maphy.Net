using Maphy.Mathematics;

namespace Maphy.Physics
{
    public static partial class Physics
    {
        internal static bool IsOverlap(Sphere sphere, fix3 point)
        {
            return math.distancesq(point, sphere.Center) <= sphere.Radius2;
        }

        internal static bool IsOverlap(Sphere a, Sphere b)
        {
            fix radius = a.Radius + b.Radius;
            return math.distancesq(a.Center, b.Center) <= radius * radius;
        }

        public static bool Overlaps(Sphere a, Sphere b)
        {
            fix3 vectorBetweenCenters = a.Center - b.Center;
            fix squaredDistanceBetweenCenters = math.lengthsq(vectorBetweenCenters);
            fix sumRadiuses = a.Radius + b.Radius;
            fix sumRadiusesProducts = sumRadiuses * sumRadiuses;

            if (squaredDistanceBetweenCenters > sumRadiusesProducts)
            {
                return false;
            }

            return true;
        }

        public static bool Overlaps(Sphere a, Capsule b)
        {
            fix3 closestPointOnSegment = GetClosestPointOnSegment(b.Center1, b.Center2, a.Center);
            fix3 sphereCenterToSegment = closestPointOnSegment - a.Center;
            fix sphereSegmentDistanceSquare = math.lengthsq(sphereCenterToSegment);
            fix sumRadius = a.Radius + b.Radius;

            if (sphereSegmentDistanceSquare > sumRadius * sumRadius)
            {
                return false;
            }

            return true;
        }

        public static bool Overlaps(Sphere sphere, AABB aabb)
        {
            return IsOverlap(sphere, aabb);
        }

        public static bool IsOverlap(Sphere sphere, AABB aabb)
        {
            fix3 p = sphere.Center - aabb.center;
            fix3 v = math.max(p, -p);
            fix3 u = math.max(v - aabb.extents, fix3.zero);
            return math.lengthsq(u) <= sphere.Radius2;
        }

        public static bool IsOverlap(Sphere sphere, OBB obb)
        {
            fix3 localCenter = quaternion.conjugate(obb.orientation) * (sphere.Center - obb.center);
            fix3 v = math.max(localCenter, -localCenter);
            fix3 u = math.max(v - obb.extents, fix3.zero);
            return math.lengthsq(u) <= sphere.Radius2;
        }
    }
}
