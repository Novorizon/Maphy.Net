using Maphy.Mathematics;

namespace Maphy.Physics
{
    public static partial class Physics
    {
        public static bool TryComputeContact(Shape shape0, Shape shape1, out CollisionInfo collision)
        {
            collision = default;
            if (shape0 == null || shape1 == null)
            {
                return false;
            }

            switch (shape0.Type, shape1.Type)
            {
                case (ShapeType.Sphere, ShapeType.AABB):
                    return TryComputeSphereAABBContact((Sphere)shape0, (AABB)shape1, out collision);
                case (ShapeType.AABB, ShapeType.Sphere):
                    if (!TryComputeSphereAABBContact((Sphere)shape1, (AABB)shape0, out collision))
                    {
                        return false;
                    }

                    collision = collision.Flipped();
                    return true;
                case (ShapeType.Sphere, ShapeType.OBB):
                    return TryComputeSphereOBBContact((Sphere)shape0, (OBB)shape1, out collision);
                case (ShapeType.OBB, ShapeType.Sphere):
                    if (!TryComputeSphereOBBContact((Sphere)shape1, (OBB)shape0, out collision))
                    {
                        return false;
                    }

                    collision = collision.Flipped();
                    return true;
                case (ShapeType.Sphere, ShapeType.Sphere):
                    return TryComputeSphereSphereContact((Sphere)shape0, (Sphere)shape1, out collision);
                case (ShapeType.Sphere, ShapeType.Capsule):
                    return TryComputeSphereCapsuleContact((Sphere)shape0, (Capsule)shape1, out collision);
                case (ShapeType.Capsule, ShapeType.Sphere):
                    if (!TryComputeSphereCapsuleContact((Sphere)shape1, (Capsule)shape0, out collision))
                    {
                        return false;
                    }

                    collision = collision.Flipped();
                    return true;
                case (ShapeType.Capsule, ShapeType.Capsule):
                    return TryComputeCapsuleCapsuleContact((Capsule)shape0, (Capsule)shape1, out collision);
                default:
                    if (!Overlaps(shape0, shape1))
                    {
                        return false;
                    }

                    return TryComputeBoundsContact(shape0, shape1, out collision);
            }
        }

        internal static bool TryComputeContact(BroadCollisionPair pair, out CollisionInfo collision)
        {
            if (!TryComputeContact(pair.collider0.shape, pair.collider1.shape, out collision))
            {
                return false;
            }

            collision.SetPair(pair);
            return true;
        }

        private static bool TryComputeSphereSphereContact(Sphere a, Sphere b, out CollisionInfo collision)
        {
            fix3 delta = b.Center - a.Center;
            fix distanceSq = math.lengthsq(delta);
            fix radius = a.Radius + b.Radius;

            if (distanceSq > radius * radius)
            {
                collision = default;
                return false;
            }

            fix distance = distanceSq > math.Epsilon ? math.sqrt(distanceSq) : fix.Zero;
            fix3 normal = distanceSq > math.Epsilon ? delta / distance : fix3.up;
            fix penetrationDepth = radius - distance;
            fix3 contactPoint1 = a.Center + normal * a.Radius;
            fix3 contactPoint2 = b.Center - normal * b.Radius;

            collision = new CollisionInfo(penetrationDepth, normal, contactPoint1, contactPoint2);
            return true;
        }

        private static bool TryComputeSphereCapsuleContact(Sphere sphere, Capsule capsule, out CollisionInfo collision)
        {
            fix3 closestPoint = GetClosestPointOnSegment(capsule.Center1, capsule.Center2, sphere.Center);
            fix3 delta = closestPoint - sphere.Center;
            fix distanceSq = math.lengthsq(delta);
            fix radius = sphere.Radius + capsule.Radius;

            if (distanceSq > radius * radius)
            {
                collision = default;
                return false;
            }

            fix distance = distanceSq > math.Epsilon ? math.sqrt(distanceSq) : fix.Zero;
            fix3 normal = distanceSq > math.Epsilon ? delta / distance : GetPerpendicularNormal(capsule.Axis);
            fix penetrationDepth = radius - distance;
            fix3 contactPoint1 = sphere.Center + normal * sphere.Radius;
            fix3 contactPoint2 = closestPoint - normal * capsule.Radius;

            collision = new CollisionInfo(penetrationDepth, normal, contactPoint1, contactPoint2);
            return true;
        }

        private static bool TryComputeSphereAABBContact(Sphere sphere, AABB aabb, out CollisionInfo collision)
        {
            fix3 localCenter = sphere.Center - aabb.center;
            fix3 closestLocal = math.clamp(localCenter, -aabb.extents, aabb.extents);
            bool centerInside = closestLocal == localCenter;

            if (centerInside)
            {
                closestLocal = GetClosestPointOnBoxSurface(localCenter, aabb.extents, out fix3 localNormal);
                fix surfaceDistance = math.length(closestLocal - localCenter);
                fix3 normalInside = localNormal;
                collision = new CollisionInfo(
                    sphere.Radius + surfaceDistance,
                    normalInside,
                    sphere.Center + normalInside * sphere.Radius,
                    aabb.center + closestLocal);
                return true;
            }

            fix3 closest = aabb.center + closestLocal;
            fix3 delta = closest - sphere.Center;
            fix distanceSq = math.lengthsq(delta);
            if (distanceSq > sphere.Radius2)
            {
                collision = default;
                return false;
            }

            fix distance = distanceSq > math.Epsilon ? math.sqrt(distanceSq) : fix.Zero;
            fix3 normal = distanceSq > math.Epsilon ? delta / distance : fix3.up;
            collision = new CollisionInfo(
                sphere.Radius - distance,
                normal,
                sphere.Center + normal * sphere.Radius,
                closest);
            return true;
        }

        private static bool TryComputeSphereOBBContact(Sphere sphere, OBB obb, out CollisionInfo collision)
        {
            quaternion inverseRotation = quaternion.conjugate(obb.orientation);
            fix3 localCenter = inverseRotation * (sphere.Center - obb.center);
            fix3 closestLocal = math.clamp(localCenter, -obb.extents, obb.extents);
            bool centerInside = closestLocal == localCenter;

            if (centerInside)
            {
                closestLocal = GetClosestPointOnBoxSurface(localCenter, obb.extents, out fix3 localNormal);
                fix3 normalInside = obb.orientation * localNormal;
                fix surfaceDistance = math.length(closestLocal - localCenter);
                collision = new CollisionInfo(
                    sphere.Radius + surfaceDistance,
                    normalInside,
                    sphere.Center + normalInside * sphere.Radius,
                    obb.center + obb.orientation * closestLocal);
                return true;
            }

            fix3 closest = obb.center + obb.orientation * closestLocal;
            fix3 delta = closest - sphere.Center;
            fix distanceSq = math.lengthsq(delta);
            if (distanceSq > sphere.Radius2)
            {
                collision = default;
                return false;
            }

            fix distance = distanceSq > math.Epsilon ? math.sqrt(distanceSq) : fix.Zero;
            fix3 normal = distanceSq > math.Epsilon ? delta / distance : obb.orientation * fix3.up;
            collision = new CollisionInfo(
                sphere.Radius - distance,
                normal,
                sphere.Center + normal * sphere.Radius,
                closest);
            return true;
        }

        private static bool TryComputeCapsuleCapsuleContact(Capsule a, Capsule b, out CollisionInfo collision)
        {
            GetClosestPointsBetweenSegments(
                a.Center1,
                a.Center2,
                b.Center1,
                b.Center2,
                out fix3 closestA,
                out fix3 closestB);

            fix3 delta = closestB - closestA;
            fix distanceSq = math.lengthsq(delta);
            fix radius = a.Radius + b.Radius;

            if (distanceSq > radius * radius)
            {
                collision = default;
                return false;
            }

            fix distance = distanceSq > math.Epsilon ? math.sqrt(distanceSq) : fix.Zero;
            fix3 normal = distanceSq > math.Epsilon ? delta / distance : GetPerpendicularNormal(a.Axis);
            fix penetrationDepth = radius - distance;
            fix3 contactPoint1 = closestA + normal * a.Radius;
            fix3 contactPoint2 = closestB - normal * b.Radius;

            collision = new CollisionInfo(penetrationDepth, normal, contactPoint1, contactPoint2);
            return true;
        }

        private static bool TryComputeBoundsContact(Shape shape0, Shape shape1, out CollisionInfo collision)
        {
            AABB bounds0 = ComputeBounds(shape0);
            AABB bounds1 = ComputeBounds(shape1);
            fix overlapX = math.min(bounds0.max.x, bounds1.max.x) - math.max(bounds0.min.x, bounds1.min.x);
            fix overlapY = math.min(bounds0.max.y, bounds1.max.y) - math.max(bounds0.min.y, bounds1.min.y);
            fix overlapZ = math.min(bounds0.max.z, bounds1.max.z) - math.max(bounds0.min.z, bounds1.min.z);

            if (overlapX < fix.Zero || overlapY < fix.Zero || overlapZ < fix.Zero)
            {
                collision = default;
                return false;
            }

            fix3 center0 = GetShapeCenter(shape0);
            fix3 center1 = GetShapeCenter(shape1);
            fix3 delta = center1 - center0;
            fix penetrationDepth = overlapX;
            fix3 normal = delta.x >= fix.Zero ? fix3.right : fix3.left;

            if (overlapY < penetrationDepth)
            {
                penetrationDepth = overlapY;
                normal = delta.y >= fix.Zero ? fix3.up : fix3.down;
            }

            if (overlapZ < penetrationDepth)
            {
                penetrationDepth = overlapZ;
                normal = delta.z >= fix.Zero ? fix3.forward : fix3.backward;
            }

            fix3 contactPoint1 = GetSupportPointOnShape(shape0, normal);
            fix3 contactPoint2 = GetSupportPointOnShape(shape1, -normal);
            collision = new CollisionInfo(penetrationDepth, normal, contactPoint1, contactPoint2);
            return true;
        }

        private static fix3 GetShapeCenter(Shape shape)
        {
            switch (shape.Type)
            {
                case ShapeType.AABB:
                    return ((AABB)shape).center;
                case ShapeType.OBB:
                    return ((OBB)shape).center;
                case ShapeType.Sphere:
                    return ((Sphere)shape).Center;
                case ShapeType.Capsule:
                    return ((Capsule)shape).Center;
                default:
                    return ComputeBounds(shape).center;
            }
        }

        private static fix3 GetSupportPointOnShape(Shape shape, fix3 direction)
        {
            fix3 normal = NormalizeOrDefault(direction);
            switch (shape.Type)
            {
                case ShapeType.AABB:
                    return GetSupportPoint((AABB)shape, normal);
                case ShapeType.OBB:
                    return GetSupportPointOnOBB((OBB)shape, normal);
                case ShapeType.Sphere:
                    Sphere sphere = (Sphere)shape;
                    return sphere.Center + normal * sphere.Radius;
                case ShapeType.Capsule:
                    Capsule capsule = (Capsule)shape;
                    fix center1Dot = math.dot(capsule.Center1, normal);
                    fix center2Dot = math.dot(capsule.Center2, normal);
                    fix3 capCenter = center1Dot >= center2Dot ? capsule.Center1 : capsule.Center2;
                    return capCenter + normal * capsule.Radius;
                default:
                    return GetSupportPoint(ComputeBounds(shape), normal);
            }
        }

        private static fix3 GetSupportPointOnOBB(OBB obb, fix3 direction)
        {
            fix3 localDirection = quaternion.conjugate(obb.orientation) * direction;
            fix3 localSupport = new fix3(
                localDirection.x < fix.Zero ? -obb.extents.x : obb.extents.x,
                localDirection.y < fix.Zero ? -obb.extents.y : obb.extents.y,
                localDirection.z < fix.Zero ? -obb.extents.z : obb.extents.z);
            return obb.center + obb.orientation * localSupport;
        }

        private static fix3 NormalizeOrDefault(fix3 value)
        {
            fix lengthSq = math.lengthsq(value);
            return lengthSq > math.Epsilon ? value / math.sqrt(lengthSq) : fix3.up;
        }

        private static fix3 GetPerpendicularNormal(fix3 axis)
        {
            fix3 normalizedAxis = NormalizeOrDefault(axis);
            fix3 candidate = math.abs(normalizedAxis.x) < math.abs(normalizedAxis.y)
                ? math.cross(normalizedAxis, fix3.right)
                : math.cross(normalizedAxis, fix3.up);
            return NormalizeOrDefault(candidate);
        }

        private static fix3 GetClosestPointOnBoxSurface(fix3 localPoint, fix3 extents, out fix3 normal)
        {
            fix distanceX = extents.x - math.abs(localPoint.x);
            fix distanceY = extents.y - math.abs(localPoint.y);
            fix distanceZ = extents.z - math.abs(localPoint.z);

            if (distanceX <= distanceY && distanceX <= distanceZ)
            {
                normal = localPoint.x >= fix.Zero ? fix3.right : fix3.left;
                return new fix3(normal.x * extents.x, localPoint.y, localPoint.z);
            }

            if (distanceY <= distanceZ)
            {
                normal = localPoint.y >= fix.Zero ? fix3.up : fix3.down;
                return new fix3(localPoint.x, normal.y * extents.y, localPoint.z);
            }

            normal = localPoint.z >= fix.Zero ? fix3.forward : fix3.backward;
            return new fix3(localPoint.x, localPoint.y, normal.z * extents.z);
        }
    }
}
