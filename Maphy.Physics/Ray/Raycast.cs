
using Maphy.Mathematics;

namespace Maphy.Physics
{
    public static partial class Physics
    {
        public static bool Raycast(Ray ray, out RaycastHit hitInfo, fix maxDistance, int layerMask)
        {
            hitInfo = new RaycastHit();
            return false;
        }

        public static bool Raycast(fix3 origin, fix3 direction, out RaycastHit hitInfo, fix maxDistance, int layerMask)
        {
            return Raycast(new Ray(origin, direction), out hitInfo, maxDistance, layerMask);
        }

        public static bool TryRaycast(Shape shape, Ray ray, fix maxDistance, out RaycastHit hitInfo)
        {
            hitInfo = default;
            if (shape == null || maxDistance < fix.Zero)
            {
                return false;
            }

            switch (shape.Type)
            {
                case ShapeType.AABB:
                    return RaycastAABB((AABB)shape, ray, maxDistance, out hitInfo);
                case ShapeType.OBB:
                    return RaycastOBB((OBB)shape, ray, maxDistance, out hitInfo);
                case ShapeType.Sphere:
                    return RaycastSphere((Sphere)shape, ray, maxDistance, out hitInfo);
                case ShapeType.Capsule:
                    return RaycastCapsule((Capsule)shape, ray, maxDistance, out hitInfo);
                default:
                    return false;
            }
        }

        internal static bool RaycastAABB(AABB aabb, Ray ray, fix maxDistance, out RaycastHit hitInfo)
        {
            hitInfo = default;
            if (!TryNormalizeRay(ray, out Ray normalizedRay) || maxDistance < fix.Zero)
            {
                return false;
            }

            if (!RaycastAABB(aabb, normalizedRay, maxDistance, out fix distance, out fix3 normal))
            {
                return false;
            }

            hitInfo.distance = distance;
            hitInfo.point = normalizedRay.GetPoint(distance);
            hitInfo.normal = normal;
            hitInfo.bounds = new Bounds(aabb.center, aabb.size);
            return true;
        }

        internal static bool RaycastAABB(AABB aabb, Ray ray, fix maxDistance, out fix distance)
        {
            return RaycastAABB(aabb, ray, maxDistance, out distance, out fix3 _);
        }

        internal static bool RaycastAABB(AABB aabb, Ray ray, fix maxDistance, out fix distance, out fix3 normal)
        {
            distance = fix.Zero;
            normal = fix3.zero;
            if (maxDistance < fix.Zero)
            {
                return false;
            }

            fix tMin = fix.Zero;
            fix tMax = maxDistance;
            fix3 hitNormal = fix3.zero;
            fix3 min = aabb.min;
            fix3 max = aabb.max;

            for (int i = 0; i < 3; i++)
            {
                fix origin = ray.origin[i];
                fix direction = ray.direction[i];
                fix slabMin = min[i];
                fix slabMax = max[i];

                if (math.abs(direction) <= math.Epsilon)
                {
                    if (origin < slabMin || origin > slabMax)
                    {
                        return false;
                    }

                    continue;
                }

                fix invDirection = fix.One / direction;
                fix t0 = (slabMin - origin) * invDirection;
                fix t1 = (slabMax - origin) * invDirection;
                fix3 normal0 = GetAxisNormal(i, -fix.One);
                fix3 normal1 = GetAxisNormal(i, fix.One);
                fix3 nearNormal = normal0;

                if (t0 > t1)
                {
                    fix temp = t0;
                    t0 = t1;
                    t1 = temp;
                    nearNormal = normal1;
                }

                if (t0 > tMin)
                {
                    tMin = t0;
                    hitNormal = nearNormal;
                }

                tMax = math.min(tMax, t1);
                if (tMin > tMax)
                {
                    return false;
                }
            }

            distance = tMin;
            normal = hitNormal;
            return true;
        }

        private static bool RaycastOBB(OBB obb, Ray ray, fix maxDistance, out RaycastHit hitInfo)
        {
            hitInfo = default;
            if (!TryNormalizeRay(ray, out Ray normalizedRay))
            {
                return false;
            }

            quaternion inverseRotation = quaternion.conjugate(obb.orientation);
            Ray localRay = new Ray(
                inverseRotation * (normalizedRay.origin - obb.center),
                inverseRotation * normalizedRay.direction);
            AABB localBounds = new AABB(fix3.zero, obb.size);
            if (!RaycastAABB(localBounds, localRay, maxDistance, out fix distance, out fix3 localNormal))
            {
                return false;
            }

            fix3 localPoint = localRay.GetPoint(distance);
            hitInfo.distance = distance;
            hitInfo.point = obb.center + obb.orientation * localPoint;
            hitInfo.normal = obb.orientation * localNormal;
            hitInfo.bounds = new Bounds(ComputeBounds(obb).center, ComputeBounds(obb).size);
            return true;
        }

        private static bool RaycastSphere(Sphere sphere, Ray ray, fix maxDistance, out RaycastHit hitInfo)
        {
            hitInfo = default;
            if (!TryNormalizeRay(ray, out Ray normalizedRay) || maxDistance < fix.Zero)
            {
                return false;
            }

            fix3 offset = normalizedRay.origin - sphere.Center;
            fix c = math.lengthsq(offset) - sphere.Radius2;
            if (c <= fix.Zero)
            {
                hitInfo.distance = fix.Zero;
                hitInfo.point = normalizedRay.origin;
                hitInfo.normal = fix3.zero;
                hitInfo.bounds = new Bounds(sphere.Bounds.center, sphere.Bounds.size);
                return true;
            }

            fix b = math.dot(offset, normalizedRay.direction);
            if (b > fix.Zero)
            {
                return false;
            }

            fix discriminant = b * b - c;
            if (discriminant < fix.Zero)
            {
                return false;
            }

            fix distance = -b - math.sqrt(discriminant);
            if (distance < fix.Zero || distance > maxDistance)
            {
                return false;
            }

            fix3 point = normalizedRay.GetPoint(distance);
            hitInfo.distance = distance;
            hitInfo.point = point;
            hitInfo.normal = math.normalize(point - sphere.Center);
            hitInfo.bounds = new Bounds(sphere.Bounds.center, sphere.Bounds.size);
            return true;
        }

        private static bool RaycastCapsule(Capsule capsule, Ray ray, fix maxDistance, out RaycastHit hitInfo)
        {
            hitInfo = default;
            if (!TryNormalizeRay(ray, out Ray normalizedRay) || maxDistance < fix.Zero)
            {
                return false;
            }

            if (IsOverlap(capsule, normalizedRay.origin))
            {
                hitInfo.distance = fix.Zero;
                hitInfo.point = normalizedRay.origin;
                hitInfo.normal = fix3.zero;
                AABB bounds = ComputeBounds(capsule);
                hitInfo.bounds = new Bounds(bounds.center, bounds.size);
                return true;
            }

            bool hasHit = false;
            fix bestDistance = maxDistance;
            fix3 bestNormal = fix3.zero;
            TryRaycastCapsuleSide(capsule, normalizedRay, maxDistance, ref hasHit, ref bestDistance, ref bestNormal);
            TryRaycastCapsuleCap(capsule.Center1, capsule.Radius, normalizedRay, maxDistance, ref hasHit, ref bestDistance, ref bestNormal);
            TryRaycastCapsuleCap(capsule.Center2, capsule.Radius, normalizedRay, maxDistance, ref hasHit, ref bestDistance, ref bestNormal);
            if (!hasHit)
            {
                return false;
            }

            AABB capsuleBounds = ComputeBounds(capsule);
            hitInfo.distance = bestDistance;
            hitInfo.point = normalizedRay.GetPoint(bestDistance);
            hitInfo.normal = bestNormal;
            hitInfo.bounds = new Bounds(capsuleBounds.center, capsuleBounds.size);
            return true;
        }

        private static void TryRaycastCapsuleSide(
            Capsule capsule,
            Ray ray,
            fix maxDistance,
            ref bool hasHit,
            ref fix bestDistance,
            ref fix3 bestNormal)
        {
            fix3 segment = capsule.Center2 - capsule.Center1;
            fix segmentLength = math.length(segment);
            if (segmentLength <= math.Epsilon)
            {
                return;
            }

            fix3 axis = segment / segmentLength;
            fix3 offset = ray.origin - capsule.Center1;
            fix axisOrigin = math.dot(offset, axis);
            fix axisDirection = math.dot(ray.direction, axis);
            fix3 radialOrigin = offset - axis * axisOrigin;
            fix3 radialDirection = ray.direction - axis * axisDirection;
            fix a = math.lengthsq(radialDirection);
            if (a <= math.Epsilon)
            {
                return;
            }

            fix b = fix._2 * math.dot(radialOrigin, radialDirection);
            fix c = math.lengthsq(radialOrigin) - capsule.Radius2;
            fix discriminant = b * b - fix._4 * a * c;
            if (discriminant < fix.Zero)
            {
                return;
            }

            fix sqrtDiscriminant = math.sqrt(discriminant);
            TryCapsuleSideDistance(
                (-b - sqrtDiscriminant) / (fix._2 * a),
                capsule,
                ray,
                axis,
                segmentLength,
                maxDistance,
                ref hasHit,
                ref bestDistance,
                ref bestNormal);
            TryCapsuleSideDistance(
                (-b + sqrtDiscriminant) / (fix._2 * a),
                capsule,
                ray,
                axis,
                segmentLength,
                maxDistance,
                ref hasHit,
                ref bestDistance,
                ref bestNormal);
        }

        private static void TryCapsuleSideDistance(
            fix distance,
            Capsule capsule,
            Ray ray,
            fix3 axis,
            fix segmentLength,
            fix maxDistance,
            ref bool hasHit,
            ref fix bestDistance,
            ref fix3 bestNormal)
        {
            if (distance < fix.Zero || distance > maxDistance || distance >= bestDistance)
            {
                return;
            }

            fix3 point = ray.GetPoint(distance);
            fix axisDistance = math.dot(point - capsule.Center1, axis);
            if (axisDistance < fix.Zero || axisDistance > segmentLength)
            {
                return;
            }

            fix3 closestAxisPoint = capsule.Center1 + axis * axisDistance;
            fix3 normal = math.normalize(point - closestAxisPoint);
            hasHit = true;
            bestDistance = distance;
            bestNormal = normal;
        }

        private static void TryRaycastCapsuleCap(
            fix3 center,
            fix radius,
            Ray ray,
            fix maxDistance,
            ref bool hasHit,
            ref fix bestDistance,
            ref fix3 bestNormal)
        {
            Sphere sphere = new Sphere(center, radius);
            if (!RaycastSphere(sphere, ray, maxDistance, out RaycastHit sphereHit))
            {
                return;
            }

            if (sphereHit.distance >= bestDistance)
            {
                return;
            }

            hasHit = true;
            bestDistance = sphereHit.distance;
            bestNormal = sphereHit.normal;
        }

        internal static bool TryNormalizeRay(Ray ray, out Ray normalizedRay)
        {
            fix lengthSq = math.lengthsq(ray.direction);
            if (lengthSq <= math.Epsilon)
            {
                normalizedRay = default;
                return false;
            }

            normalizedRay = new Ray(ray.origin, ray.direction / math.sqrt(lengthSq));
            return true;
        }

        private static fix3 GetAxisNormal(int axis, fix sign)
        {
            switch (axis)
            {
                case 0:
                    return new fix3(sign, fix.Zero, fix.Zero);
                case 1:
                    return new fix3(fix.Zero, sign, fix.Zero);
                default:
                    return new fix3(fix.Zero, fix.Zero, sign);
            }
        }
    }
}
