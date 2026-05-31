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
                case (ShapeType.AABB, ShapeType.AABB):
                    return TryComputeBoxBoxContact(BoxData.FromAABB((AABB)shape0), BoxData.FromAABB((AABB)shape1), out collision);
                case (ShapeType.AABB, ShapeType.OBB):
                    return TryComputeBoxBoxContact(BoxData.FromAABB((AABB)shape0), BoxData.FromOBB((OBB)shape1), out collision);
                case (ShapeType.OBB, ShapeType.AABB):
                    return TryComputeBoxBoxContact(BoxData.FromOBB((OBB)shape0), BoxData.FromAABB((AABB)shape1), out collision);
                case (ShapeType.OBB, ShapeType.OBB):
                    return TryComputeBoxBoxContact(BoxData.FromOBB((OBB)shape0), BoxData.FromOBB((OBB)shape1), out collision);
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
                case (ShapeType.AABB, ShapeType.Capsule):
                    return TryComputeCapsuleBoxContact((Capsule)shape1, BoxData.FromAABB((AABB)shape0), false, out collision);
                case (ShapeType.Capsule, ShapeType.AABB):
                    return TryComputeCapsuleBoxContact((Capsule)shape0, BoxData.FromAABB((AABB)shape1), true, out collision);
                case (ShapeType.OBB, ShapeType.Capsule):
                    return TryComputeCapsuleBoxContact((Capsule)shape1, BoxData.FromOBB((OBB)shape0), false, out collision);
                case (ShapeType.Capsule, ShapeType.OBB):
                    return TryComputeCapsuleBoxContact((Capsule)shape0, BoxData.FromOBB((OBB)shape1), true, out collision);
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

        private static bool TryComputeBoxBoxContact(BoxData box0, BoxData box1, out CollisionInfo collision)
        {
            collision = default;
            fix penetrationDepth = fix.Max;
            fix3 normal = fix3.zero;
            int referenceAxisIndex = -1;
            bool referenceIsBox0 = true;
            bool faceContact = true;
            fix3 delta = box1.center - box0.center;

            for (int i = 0; i < 3; i++)
            {
                if (!TestBoxAxis(box0, box1, box0.GetAxis(i), delta, i, true, true, ref penetrationDepth, ref normal, ref referenceAxisIndex, ref referenceIsBox0, ref faceContact))
                {
                    return false;
                }
            }

            for (int i = 0; i < 3; i++)
            {
                if (!TestBoxAxis(box0, box1, box1.GetAxis(i), delta, i, false, true, ref penetrationDepth, ref normal, ref referenceAxisIndex, ref referenceIsBox0, ref faceContact))
                {
                    return false;
                }
            }

            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    fix3 axis = math.cross(box0.GetAxis(i), box1.GetAxis(j));
                    if (math.lengthsq(axis) <= fix._0_0001)
                    {
                        continue;
                    }

                    if (!TestBoxAxis(box0, box1, axis, delta, -1, true, false, ref penetrationDepth, ref normal, ref referenceAxisIndex, ref referenceIsBox0, ref faceContact))
                    {
                        return false;
                    }
                }
            }

            collision = new CollisionInfo(0, 0)
            {
                penetrationDepth = penetrationDepth,
                normal = normal,
            };

            if (faceContact)
            {
                if (referenceIsBox0)
                {
                    AddBoxFaceContacts(box0, box1, normal, referenceAxisIndex, true, penetrationDepth, ref collision);
                }
                else
                {
                    AddBoxFaceContacts(box1, box0, -normal, referenceAxisIndex, false, penetrationDepth, ref collision);
                }
            }

            if (!collision.hasContact)
            {
                collision.AddContact(GetSupportPoint(box0, normal), GetSupportPoint(box1, -normal), penetrationDepth);
            }

            return true;
        }

        private static bool TestBoxAxis(
            BoxData box0,
            BoxData box1,
            fix3 axis,
            fix3 delta,
            int axisIndex,
            bool axisFromBox0,
            bool isFaceAxis,
            ref fix bestPenetrationDepth,
            ref fix3 bestNormal,
            ref int bestReferenceAxisIndex,
            ref bool bestReferenceIsBox0,
            ref bool bestFaceContact)
        {
            fix lengthSq = math.lengthsq(axis);
            if (lengthSq <= math.Epsilon)
            {
                return true;
            }

            axis = axis / math.sqrt(lengthSq);
            fix distance = math.dot(delta, axis);
            fix penetrationDepth = box0.ProjectRadius(axis) + box1.ProjectRadius(axis) - math.abs(distance);
            if (penetrationDepth < fix.Zero)
            {
                return false;
            }

            if (penetrationDepth + fix._0_0001 < bestPenetrationDepth)
            {
                bestPenetrationDepth = penetrationDepth;
                bestNormal = distance >= fix.Zero ? axis : -axis;
                bestReferenceAxisIndex = axisIndex;
                bestReferenceIsBox0 = axisFromBox0;
                bestFaceContact = isFaceAxis;
            }

            return true;
        }

        private static void AddBoxFaceContacts(
            BoxData reference,
            BoxData incident,
            fix3 normalFromReferenceToIncident,
            int referenceAxisIndex,
            bool referenceIsShape0,
            fix penetrationDepth,
            ref CollisionInfo collision)
        {
            if (referenceAxisIndex < 0)
            {
                return;
            }

            fix3 referenceAxis = reference.GetAxis(referenceAxisIndex);
            fix normalSign = math.dot(referenceAxis, normalFromReferenceToIncident) >= fix.Zero ? fix.One : -fix.One;
            fix3 normal = referenceAxis * normalSign;
            int tangentIndex0 = (referenceAxisIndex + 1) % 3;
            int tangentIndex1 = (referenceAxisIndex + 2) % 3;
            fix3 tangent0 = reference.GetAxis(tangentIndex0);
            fix3 tangent1 = reference.GetAxis(tangentIndex1);
            fix facePlane = math.dot(reference.center, normal) + reference.GetExtent(referenceAxisIndex);

            GetOverlapInterval(reference, incident, tangent0, tangentIndex0, out fix min0, out fix max0);
            GetOverlapInterval(reference, incident, tangent1, tangentIndex1, out fix min1, out fix max1);
            if (min0 > max0 || min1 > max1)
            {
                return;
            }

            fix incidentPlane = math.dot(incident.center, normal) - incident.ProjectRadius(normal);
            fix depth = math.max(fix.Zero, facePlane - incidentPlane);
            fix contactDepth = math.max(penetrationDepth, depth);
            AddBoxFaceContact(referenceIsShape0, normal, facePlane, tangent0, tangent1, min0, min1, incidentPlane, contactDepth, ref collision);
            AddBoxFaceContact(referenceIsShape0, normal, facePlane, tangent0, tangent1, min0, max1, incidentPlane, contactDepth, ref collision);
            AddBoxFaceContact(referenceIsShape0, normal, facePlane, tangent0, tangent1, max0, min1, incidentPlane, contactDepth, ref collision);
            AddBoxFaceContact(referenceIsShape0, normal, facePlane, tangent0, tangent1, max0, max1, incidentPlane, contactDepth, ref collision);
        }

        private static void AddBoxFaceContact(
            bool referenceIsShape0,
            fix3 normal,
            fix facePlane,
            fix3 tangent0,
            fix3 tangent1,
            fix tangentDistance0,
            fix tangentDistance1,
            fix incidentPlane,
            fix penetrationDepth,
            ref CollisionInfo collision)
        {
            fix3 pointOnReference = normal * facePlane + tangent0 * tangentDistance0 + tangent1 * tangentDistance1;
            fix3 pointOnIncident = pointOnReference - normal * (facePlane - incidentPlane);
            if (referenceIsShape0)
            {
                collision.AddContact(pointOnReference, pointOnIncident, penetrationDepth);
            }
            else
            {
                collision.AddContact(pointOnIncident, pointOnReference, penetrationDepth);
            }
        }

        private static void GetOverlapInterval(BoxData reference, BoxData incident, fix3 axis, int referenceExtentIndex, out fix min, out fix max)
        {
            fix referenceCenter = math.dot(reference.center, axis);
            fix referenceMin = referenceCenter - reference.GetExtent(referenceExtentIndex);
            fix referenceMax = referenceCenter + reference.GetExtent(referenceExtentIndex);
            fix incidentCenter = math.dot(incident.center, axis);
            fix incidentRadius = incident.ProjectRadius(axis);
            fix incidentMin = incidentCenter - incidentRadius;
            fix incidentMax = incidentCenter + incidentRadius;
            min = math.max(referenceMin, incidentMin);
            max = math.min(referenceMax, incidentMax);
        }

        private static fix3 GetSupportPoint(BoxData box, fix3 direction)
        {
            return box.center
                + box.axis0 * (math.dot(box.axis0, direction) >= fix.Zero ? box.extents.x : -box.extents.x)
                + box.axis1 * (math.dot(box.axis1, direction) >= fix.Zero ? box.extents.y : -box.extents.y)
                + box.axis2 * (math.dot(box.axis2, direction) >= fix.Zero ? box.extents.z : -box.extents.z);
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

        private static bool TryComputeCapsuleBoxContact(Capsule capsule, BoxData box, bool capsuleIsShape0, out CollisionInfo collision)
        {
            fix3 localSegment0 = box.WorldToLocalPoint(capsule.Center1);
            fix3 localSegment1 = box.WorldToLocalPoint(capsule.Center2);
            GetClosestPointsBetweenSegmentAndAABB(localSegment0, localSegment1, box.extents, out fix3 localCapsulePoint, out fix3 localBoxPoint);

            fix3 delta = localBoxPoint - localCapsulePoint;
            fix distanceSq = math.lengthsq(delta);
            if (distanceSq > capsule.Radius2)
            {
                collision = default;
                return false;
            }

            fix3 normalLocal;
            fix penetrationDepth;
            if (distanceSq > math.Epsilon)
            {
                fix distance = math.sqrt(distanceSq);
                normalLocal = delta / distance;
                penetrationDepth = capsule.Radius - distance;
            }
            else
            {
                localBoxPoint = GetClosestPointOnBoxSurface(localCapsulePoint, box.extents, out normalLocal);
                penetrationDepth = capsule.Radius + math.length(localBoxPoint - localCapsulePoint);
            }

            fix3 normalWorld = NormalizeOrDefault(box.LocalToWorldDirection(normalLocal));
            fix3 capsuleAxisPoint = box.LocalToWorldPoint(localCapsulePoint);
            fix3 pointOnCapsule = capsuleAxisPoint + normalWorld * capsule.Radius;
            fix3 pointOnBox = box.LocalToWorldPoint(localBoxPoint);

            collision = new CollisionInfo(0, 0)
            {
                penetrationDepth = penetrationDepth,
                normal = capsuleIsShape0 ? normalWorld : -normalWorld,
            };

            if (capsuleIsShape0)
            {
                collision.AddContact(pointOnCapsule, pointOnBox, penetrationDepth);
            }
            else
            {
                collision.AddContact(pointOnBox, pointOnCapsule, penetrationDepth);
            }

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

            collision = new CollisionInfo(0, 0)
            {
                penetrationDepth = penetrationDepth,
                normal = normal,
            };

            AddBoundsFaceContacts(bounds0, bounds1, normal, penetrationDepth, ref collision);
            if (!collision.hasContact)
            {
                collision.AddContact(GetSupportPointOnShape(shape0, normal), GetSupportPointOnShape(shape1, -normal), penetrationDepth);
            }

            return true;
        }

        private static void AddBoundsFaceContacts(AABB bounds0, AABB bounds1, fix3 normal, fix penetrationDepth, ref CollisionInfo collision)
        {
            fix3 min = math.max(bounds0.min, bounds1.min);
            fix3 max = math.min(bounds0.max, bounds1.max);

            if (normal.x != fix.Zero)
            {
                fix x0 = normal.x > fix.Zero ? bounds0.max.x : bounds0.min.x;
                fix x1 = normal.x > fix.Zero ? bounds1.min.x : bounds1.max.x;
                AddContact(ref collision, new fix3(x0, min.y, min.z), new fix3(x1, min.y, min.z), penetrationDepth);
                AddContact(ref collision, new fix3(x0, min.y, max.z), new fix3(x1, min.y, max.z), penetrationDepth);
                AddContact(ref collision, new fix3(x0, max.y, min.z), new fix3(x1, max.y, min.z), penetrationDepth);
                AddContact(ref collision, new fix3(x0, max.y, max.z), new fix3(x1, max.y, max.z), penetrationDepth);
                return;
            }

            if (normal.y != fix.Zero)
            {
                fix y0 = normal.y > fix.Zero ? bounds0.max.y : bounds0.min.y;
                fix y1 = normal.y > fix.Zero ? bounds1.min.y : bounds1.max.y;
                AddContact(ref collision, new fix3(min.x, y0, min.z), new fix3(min.x, y1, min.z), penetrationDepth);
                AddContact(ref collision, new fix3(min.x, y0, max.z), new fix3(min.x, y1, max.z), penetrationDepth);
                AddContact(ref collision, new fix3(max.x, y0, min.z), new fix3(max.x, y1, min.z), penetrationDepth);
                AddContact(ref collision, new fix3(max.x, y0, max.z), new fix3(max.x, y1, max.z), penetrationDepth);
                return;
            }

            fix z0 = normal.z > fix.Zero ? bounds0.max.z : bounds0.min.z;
            fix z1 = normal.z > fix.Zero ? bounds1.min.z : bounds1.max.z;
            AddContact(ref collision, new fix3(min.x, min.y, z0), new fix3(min.x, min.y, z1), penetrationDepth);
            AddContact(ref collision, new fix3(min.x, max.y, z0), new fix3(min.x, max.y, z1), penetrationDepth);
            AddContact(ref collision, new fix3(max.x, min.y, z0), new fix3(max.x, min.y, z1), penetrationDepth);
            AddContact(ref collision, new fix3(max.x, max.y, z0), new fix3(max.x, max.y, z1), penetrationDepth);
        }

        private static void AddContact(ref CollisionInfo collision, fix3 pointOnCollider0, fix3 pointOnCollider1, fix penetrationDepth)
        {
            collision.AddContact(pointOnCollider0, pointOnCollider1, penetrationDepth);
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

        private static void GetClosestPointsBetweenSegmentAndAABB(
            fix3 segment0,
            fix3 segment1,
            fix3 extents,
            out fix3 closestSegment,
            out fix3 closestBox)
        {
            if (TryGetSegmentAABBOverlapInterval(segment0, segment1, extents, out fix tMin, out fix tMax))
            {
                fix t = (tMin + tMax) * fix._0_5;
                closestSegment = segment0 + (segment1 - segment0) * t;
                closestBox = math.clamp(closestSegment, -extents, extents);
                return;
            }

            fix minDistanceSq = fix.Max;
            closestSegment = segment0;
            closestBox = math.clamp(segment0, -extents, extents);

            CheckSegmentAABBPointClosest(segment0, extents, ref minDistanceSq, ref closestSegment, ref closestBox);
            CheckSegmentAABBPointClosest(segment1, extents, ref minDistanceSq, ref closestSegment, ref closestBox);

            CheckSegmentAABBFaceClosest(segment0, segment1, extents, 0, -extents.x, ref minDistanceSq, ref closestSegment, ref closestBox);
            CheckSegmentAABBFaceClosest(segment0, segment1, extents, 0, extents.x, ref minDistanceSq, ref closestSegment, ref closestBox);
            CheckSegmentAABBFaceClosest(segment0, segment1, extents, 1, -extents.y, ref minDistanceSq, ref closestSegment, ref closestBox);
            CheckSegmentAABBFaceClosest(segment0, segment1, extents, 1, extents.y, ref minDistanceSq, ref closestSegment, ref closestBox);
            CheckSegmentAABBFaceClosest(segment0, segment1, extents, 2, -extents.z, ref minDistanceSq, ref closestSegment, ref closestBox);
            CheckSegmentAABBFaceClosest(segment0, segment1, extents, 2, extents.z, ref minDistanceSq, ref closestSegment, ref closestBox);

            CheckSegmentAABBEdgeClosest(segment0, segment1, new fix3(-extents.x, -extents.y, -extents.z), new fix3(extents.x, -extents.y, -extents.z), ref minDistanceSq, ref closestSegment, ref closestBox);
            CheckSegmentAABBEdgeClosest(segment0, segment1, new fix3(-extents.x, extents.y, -extents.z), new fix3(extents.x, extents.y, -extents.z), ref minDistanceSq, ref closestSegment, ref closestBox);
            CheckSegmentAABBEdgeClosest(segment0, segment1, new fix3(-extents.x, -extents.y, extents.z), new fix3(extents.x, -extents.y, extents.z), ref minDistanceSq, ref closestSegment, ref closestBox);
            CheckSegmentAABBEdgeClosest(segment0, segment1, new fix3(-extents.x, extents.y, extents.z), new fix3(extents.x, extents.y, extents.z), ref minDistanceSq, ref closestSegment, ref closestBox);

            CheckSegmentAABBEdgeClosest(segment0, segment1, new fix3(-extents.x, -extents.y, -extents.z), new fix3(-extents.x, extents.y, -extents.z), ref minDistanceSq, ref closestSegment, ref closestBox);
            CheckSegmentAABBEdgeClosest(segment0, segment1, new fix3(extents.x, -extents.y, -extents.z), new fix3(extents.x, extents.y, -extents.z), ref minDistanceSq, ref closestSegment, ref closestBox);
            CheckSegmentAABBEdgeClosest(segment0, segment1, new fix3(-extents.x, -extents.y, extents.z), new fix3(-extents.x, extents.y, extents.z), ref minDistanceSq, ref closestSegment, ref closestBox);
            CheckSegmentAABBEdgeClosest(segment0, segment1, new fix3(extents.x, -extents.y, extents.z), new fix3(extents.x, extents.y, extents.z), ref minDistanceSq, ref closestSegment, ref closestBox);

            CheckSegmentAABBEdgeClosest(segment0, segment1, new fix3(-extents.x, -extents.y, -extents.z), new fix3(-extents.x, -extents.y, extents.z), ref minDistanceSq, ref closestSegment, ref closestBox);
            CheckSegmentAABBEdgeClosest(segment0, segment1, new fix3(extents.x, -extents.y, -extents.z), new fix3(extents.x, -extents.y, extents.z), ref minDistanceSq, ref closestSegment, ref closestBox);
            CheckSegmentAABBEdgeClosest(segment0, segment1, new fix3(-extents.x, extents.y, -extents.z), new fix3(-extents.x, extents.y, extents.z), ref minDistanceSq, ref closestSegment, ref closestBox);
            CheckSegmentAABBEdgeClosest(segment0, segment1, new fix3(extents.x, extents.y, -extents.z), new fix3(extents.x, extents.y, extents.z), ref minDistanceSq, ref closestSegment, ref closestBox);
        }

        private static bool TryGetSegmentAABBOverlapInterval(fix3 segment0, fix3 segment1, fix3 extents, out fix tMin, out fix tMax)
        {
            fix3 direction = segment1 - segment0;
            tMin = fix.Zero;
            tMax = fix.One;

            return IsSegmentOverlapAABBSlab(segment0.x, direction.x, -extents.x, extents.x, ref tMin, ref tMax)
                && IsSegmentOverlapAABBSlab(segment0.y, direction.y, -extents.y, extents.y, ref tMin, ref tMax)
                && IsSegmentOverlapAABBSlab(segment0.z, direction.z, -extents.z, extents.z, ref tMin, ref tMax);
        }

        private static void CheckSegmentAABBPointClosest(
            fix3 point,
            fix3 extents,
            ref fix minDistanceSq,
            ref fix3 closestSegment,
            ref fix3 closestBox)
        {
            fix3 boxPoint = math.clamp(point, -extents, extents);
            UpdateClosestSegmentBoxPoints(point, boxPoint, ref minDistanceSq, ref closestSegment, ref closestBox);
        }

        private static void CheckSegmentAABBFaceClosest(
            fix3 segment0,
            fix3 segment1,
            fix3 extents,
            int axis,
            fix faceCoordinate,
            ref fix minDistanceSq,
            ref fix3 closestSegment,
            ref fix3 closestBox)
        {
            fix3 direction = segment1 - segment0;
            int tangent0 = (axis + 1) % 3;
            int tangent1 = (axis + 2) % 3;
            fix tMin = fix.Zero;
            fix tMax = fix.One;

            if (!ClipSegmentAxisToRange(segment0, direction, tangent0, -GetComponent(extents, tangent0), GetComponent(extents, tangent0), ref tMin, ref tMax)
                || !ClipSegmentAxisToRange(segment0, direction, tangent1, -GetComponent(extents, tangent1), GetComponent(extents, tangent1), ref tMin, ref tMax))
            {
                return;
            }

            fix axisDirection = GetComponent(direction, axis);
            fix t = (tMin + tMax) * fix._0_5;
            if (math.abs(axisDirection) > math.Epsilon)
            {
                t = math.clamp((faceCoordinate - GetComponent(segment0, axis)) / axisDirection, tMin, tMax);
            }

            fix3 segmentPoint = segment0 + direction * t;
            fix3 boxPoint = segmentPoint;
            boxPoint = SetComponent(boxPoint, axis, faceCoordinate);
            boxPoint = SetComponent(boxPoint, tangent0, math.clamp(GetComponent(boxPoint, tangent0), -GetComponent(extents, tangent0), GetComponent(extents, tangent0)));
            boxPoint = SetComponent(boxPoint, tangent1, math.clamp(GetComponent(boxPoint, tangent1), -GetComponent(extents, tangent1), GetComponent(extents, tangent1)));

            UpdateClosestSegmentBoxPoints(segmentPoint, boxPoint, ref minDistanceSq, ref closestSegment, ref closestBox);
        }

        private static bool ClipSegmentAxisToRange(fix3 segment0, fix3 direction, int axis, fix min, fix max, ref fix tMin, ref fix tMax)
        {
            return IsSegmentOverlapAABBSlab(GetComponent(segment0, axis), GetComponent(direction, axis), min, max, ref tMin, ref tMax);
        }

        private static void CheckSegmentAABBEdgeClosest(
            fix3 segment0,
            fix3 segment1,
            fix3 edge0,
            fix3 edge1,
            ref fix minDistanceSq,
            ref fix3 closestSegment,
            ref fix3 closestBox)
        {
            GetClosestPointsBetweenSegments(segment0, segment1, edge0, edge1, out fix3 segmentPoint, out fix3 boxPoint);
            UpdateClosestSegmentBoxPoints(segmentPoint, boxPoint, ref minDistanceSq, ref closestSegment, ref closestBox);
        }

        private static void UpdateClosestSegmentBoxPoints(
            fix3 segmentPoint,
            fix3 boxPoint,
            ref fix minDistanceSq,
            ref fix3 closestSegment,
            ref fix3 closestBox)
        {
            fix distanceSq = math.distancesq(segmentPoint, boxPoint);
            if (distanceSq < minDistanceSq)
            {
                minDistanceSq = distanceSq;
                closestSegment = segmentPoint;
                closestBox = boxPoint;
            }
        }

        private static fix GetComponent(fix3 value, int axis)
        {
            switch (axis)
            {
                case 0:
                    return value.x;
                case 1:
                    return value.y;
                default:
                    return value.z;
            }
        }

        private static fix3 SetComponent(fix3 value, int axis, fix component)
        {
            switch (axis)
            {
                case 0:
                    value.x = component;
                    break;
                case 1:
                    value.y = component;
                    break;
                default:
                    value.z = component;
                    break;
            }

            return value;
        }

        private readonly struct BoxData
        {
            public readonly fix3 center;
            public readonly fix3 extents;
            public readonly fix3 axis0;
            public readonly fix3 axis1;
            public readonly fix3 axis2;

            private BoxData(fix3 center, fix3 extents, fix3 axis0, fix3 axis1, fix3 axis2)
            {
                this.center = center;
                this.extents = extents;
                this.axis0 = axis0;
                this.axis1 = axis1;
                this.axis2 = axis2;
            }

            public static BoxData FromAABB(AABB aabb)
            {
                return new BoxData(aabb.center, aabb.extents, fix3.right, fix3.up, fix3.forward);
            }

            public static BoxData FromOBB(OBB obb)
            {
                return new BoxData(
                    obb.center,
                    obb.extents,
                    obb.orientation * fix3.right,
                    obb.orientation * fix3.up,
                    obb.orientation * fix3.forward);
            }

            public fix3 GetAxis(int index)
            {
                switch (index)
                {
                    case 0:
                        return axis0;
                    case 1:
                        return axis1;
                    default:
                        return axis2;
                }
            }

            public fix GetExtent(int index)
            {
                switch (index)
                {
                    case 0:
                        return extents.x;
                    case 1:
                        return extents.y;
                    default:
                        return extents.z;
                }
            }

            public fix ProjectRadius(fix3 axis)
            {
                return extents.x * math.abs(math.dot(axis0, axis))
                    + extents.y * math.abs(math.dot(axis1, axis))
                    + extents.z * math.abs(math.dot(axis2, axis));
            }

            public fix3 WorldToLocalPoint(fix3 point)
            {
                fix3 delta = point - center;
                return new fix3(math.dot(delta, axis0), math.dot(delta, axis1), math.dot(delta, axis2));
            }

            public fix3 LocalToWorldPoint(fix3 point)
            {
                return center + LocalToWorldDirection(point);
            }

            public fix3 LocalToWorldDirection(fix3 direction)
            {
                return axis0 * direction.x + axis1 * direction.y + axis2 * direction.z;
            }
        }
    }
}
