
using Maphy.Mathematics;
using System;
using UnityEngine;

namespace Maphy.Physics
{
    public static partial class Physics
    {
        internal static bool IsOverlap(Sphere sphere, fix3 point) { return math.distancesq(point, sphere.Center) <= sphere.Radius2; }

        internal static bool IsOverlap(Sphere a, Sphere b) { return math.distancesq(a.Center, b.Center) < (a.Radius + b.Radius) * (a.Radius + b.Radius); }

        public static bool Overlaps(Sphere a, Sphere b)
        {
            CollisionInfo collisionInfo = new CollisionInfo(0, 0);

            // Compute the distance between the centers
            fix3 vectorBetweenCenters = a.Center - b.Center;
            fix squaredDistanceBetweenCenters = math.lengthsq(vectorBetweenCenters);


            fix sphere1Radius = a.Radius;
            fix sphere2Radius = b.Radius;

            // Compute the sum of the radius
            fix sumRadiuses = sphere1Radius + sphere2Radius;

            // Compute the product of the sum of the radius
            fix sumRadiusesProducts = sumRadiuses * sumRadiuses;


            // If the sphere collision shapes intersect
            if (squaredDistanceBetweenCenters < sumRadiusesProducts)
            {

                collisionInfo.penetrationDepth = sumRadiuses - math.sqrt(squaredDistanceBetweenCenters);

                // Make sure the penetration depth is not zero (even if the previous condition test was true the penetration depth can still be
                // zero because of precision issue of the computation at the previous line)
                if (collisionInfo.penetrationDepth > 0)
                {

                    // If the two sphere centers are not at the same position
                    if (squaredDistanceBetweenCenters > math.Epsilon)
                    {
                        collisionInfo.normal = math.normalize(vectorBetweenCenters);

                    }
                    else
                    {    // If the sphere centers are at the same position (degenerate case)

                        // Take any contact normal direction
                        collisionInfo.normal = fix3.up;
                    }
                    collisionInfo.contactPoint1 = a.Center + collisionInfo.normal * a.Radius;
                    collisionInfo.contactPoint2 = b.Center - collisionInfo.normal * b.Radius;
                }
            }
            return true;
        }

        public static bool Overlaps(Sphere a, Capsule b)
        {

            // Compute the point on the inner capsule segment that is the closes to center of sphere
            fix3 closestPointOnSegment = GetClosestPointOnSegment(b.Center1, b.Center2, a.Center);

            // Compute the distance between the sphere center and the closest point on the segment
            fix3 sphereCenterToSegment = closestPointOnSegment - a.Center;
            fix sphereSegmentDistanceSquare = math.lengthsq(sphereCenterToSegment);

            // Compute the sum of the radius of the sphere and the capsule (virtual sphere)
            fix sumRadius = a.Radius + b.Radius;


            // If the collision shapes overlap
            if (sphereSegmentDistanceSquare < sumRadius * sumRadius)
            {
                // If we need to report contacts
                if (needCollisionInfo)
                {
                    CollisionInfo collisionInfo = new CollisionInfo(0, 0);

                    // If the sphere center is not on the capsule inner segment
                    if (sphereSegmentDistanceSquare > math.Epsilon)
                    {
                        fix sphereSegmentDistance = math.sqrt(sphereSegmentDistanceSquare);
                        //单位向量
                        fix3 normal = sphereCenterToSegment / sphereSegmentDistance;
                        collisionInfo.penetrationDepth = sumRadius - sphereSegmentDistance;
                        collisionInfo.normal = normal;

                        collisionInfo.contactPoint1 = a.Center + normal * a.Radius;
                        collisionInfo.contactPoint2 = closestPointOnSegment - normal * b.Center;
                    }
                    else //球心在capsule的线段
                    {
                        collisionInfo.penetrationDepth = sumRadius;

                        fix3 capsuleSegment = math.normalize((b.Center2 - b.Center1));//线段单位法向量


                        // Get the vectors (among vec1 and vec2) that is the most orthogonal to the capsule inner segment (smallest absolute dot product)
                        // cosA=x/|segment|=x/|normal|=x
                        fix cosA1 = math.abs(capsuleSegment.x);     // abs(vec1.dot(seg2))
                        fix cosA2 = math.abs(capsuleSegment.y);     // abs(vec2.dot(seg2))


                        // 将与线段正交的向量作为接触法线
                        //谁的夹角更小，就按照某个轴鱼线段的叉乘作为法线
                        collisionInfo.normal = cosA1 < cosA2 ? math.cross(capsuleSegment, fix3.right) : math.cross(capsuleSegment, fix3.up);

                        // Compute the two local contact points
                        collisionInfo.contactPoint1 = a.Center + collisionInfo.normal * a.Radius;
                        collisionInfo.contactPoint2 = a.Center - collisionInfo.normal * b.Radius;
                    }

                }
                return true;

            }
            return false;
        }


        public static bool Overlaps(Sphere sphere, AABB aabb)
        {

            fix3 point = sphere.Center;
            Face face = GetFaceClosestToPointOnAABB(aabb, ref point);
            while (face > 0)
            {
                fix3 normal = GetFaceNormalOnAABB(aabb, face);
                fix3 supportPoint = GetSupportPoint(aabb, normal);
                if (Overlaps(sphere, supportPoint, normal))
                {
                    return true;
                }
                face = (Face)((int)face >> 1);
            }
            return false;
        }

        public static bool IsOverlap(Sphere sphere, AABB aabb)
        {
            fix3 p = sphere.Center - aabb.center;

            fix3 v = math.max(p, -p);
            fix3 u = math.max(v - aabb.extents, fix3.zero);
            if (math.length(u) <= sphere.Radius)
            {
                return true;
            }
            return false;
        }


        public static bool IsOverlap(Sphere sphere, OBB obb)
        {
            fix3 p = sphere.Center - obb.center;
            p = obb.orientation * p;
            p = obb.orientation * p;

            fix3 v = math.max(p, -p);
            fix3 u = math.max(v - obb.BevelRadius, fix3.zero);
            if (math.length(u) <= sphere.Radius)
            {
                return true;
            }
            return false;
        }

        /// <summary>
        /// 球与平面相交信息
        /// </summary>
        /// <param name="sphere"></param>
        /// <param name="planePoint">面上一点</param>
        /// <param name="planeNormal">面的法线</param>
        /// <returns></returns>
        private static bool Overlaps(Sphere sphere, fix3 planePoint, fix3 planeNormal)
        {
            fix3 vector = planePoint - sphere.Center;
            fix penetrationDepth = math.dot(vector, planeNormal);
            if (penetrationDepth + sphere.Radius > 0)
            {
                if (needCollisionInfo)
                {
                    CollisionInfo collisionInfo = new CollisionInfo(0, 0);
                    collisionInfo.normal = -planeNormal;
                    collisionInfo.penetrationDepth = penetrationDepth;
                    collisionInfo.contactPoint1 = -planeNormal * sphere.Center;
                    collisionInfo.contactPoint2 = sphere.Center + planeNormal * (penetrationDepth - sphere.Radius);
                }

                return true;
            }
            else
            {
                return false;
            }

        }
    }
}
