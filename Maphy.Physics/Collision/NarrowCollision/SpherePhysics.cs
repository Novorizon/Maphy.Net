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

            if (needCollisionInfo)
            {
                CollisionInfo collisionInfo = new CollisionInfo(0, 0);
                collisionInfo.penetrationDepth = sumRadiuses - math.sqrt(squaredDistanceBetweenCenters);

                if (collisionInfo.penetrationDepth > 0)
                {
                    collisionInfo.normal = squaredDistanceBetweenCenters > math.Epsilon
                        ? math.normalize(vectorBetweenCenters)
                        : fix3.up;
                    collisionInfo.contactPoint1 = a.Center + collisionInfo.normal * a.Radius;
                    collisionInfo.contactPoint2 = b.Center - collisionInfo.normal * b.Radius;
                }
            }

            return true;
        }

        public static bool Overlaps(Sphere a, Capsule b)
        {
            fix3 closestPointOnSegment = GetClosestPointOnSegment(b.Center1, b.Center2, a.Center);
            fix3 sphereCenterToSegment = closestPointOnSegment - a.Center;
            fix sphereSegmentDistanceSquare = math.lengthsq(sphereCenterToSegment);
            fix sumRadius = a.Radius + b.Radius;

            if (sphereSegmentDistanceSquare >= sumRadius * sumRadius)
            {
                return false;
            }

            if (needCollisionInfo)
            {
                CollisionInfo collisionInfo = new CollisionInfo(0, 0);
                if (sphereSegmentDistanceSquare > math.Epsilon)
                {
                    fix sphereSegmentDistance = math.sqrt(sphereSegmentDistanceSquare);
                    fix3 normal = sphereCenterToSegment / sphereSegmentDistance;
                    collisionInfo.penetrationDepth = sumRadius - sphereSegmentDistance;
                    collisionInfo.normal = normal;
                    collisionInfo.contactPoint1 = a.Center + normal * a.Radius;
                    collisionInfo.contactPoint2 = closestPointOnSegment - normal * b.Radius;
                }
                else
                {
                    collisionInfo.penetrationDepth = sumRadius;
                    fix3 capsuleSegment = math.normalize(b.Center2 - b.Center1);
                    fix cosA1 = math.abs(capsuleSegment.x);
                    fix cosA2 = math.abs(capsuleSegment.y);
                    collisionInfo.normal = cosA1 < cosA2
                        ? math.cross(capsuleSegment, fix3.right)
                        : math.cross(capsuleSegment, fix3.up);
                    collisionInfo.contactPoint1 = a.Center + collisionInfo.normal * a.Radius;
                    collisionInfo.contactPoint2 = a.Center - collisionInfo.normal * b.Radius;
                }
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
            return math.length(u) <= sphere.Radius;
        }

        public static bool IsOverlap(Sphere sphere, OBB obb)
        {
            fix3 localCenter = quaternion.conjugate(obb.orientation) * (sphere.Center - obb.center);
            fix3 v = math.max(localCenter, -localCenter);
            fix3 u = math.max(v - obb.extents, fix3.zero);
            return math.length(u) <= sphere.Radius;
        }
    }
}