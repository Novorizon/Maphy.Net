using Maphy.Mathematics;

namespace Maphy.Physics
{
    public static partial class Physics
    {
        public static bool TryShapeCast(Shape movingShape, Shape targetShape, fix3 delta, out ShapeCastHit hit)
        {
            hit = default;
            if (movingShape == null || targetShape == null)
            {
                return false;
            }

            if (movingShape.Type == ShapeType.Sphere)
            {
                Sphere movingSphere = (Sphere)movingShape;
                switch (targetShape.Type)
                {
                    case ShapeType.Sphere:
                        return TrySweepSphereSphere(movingSphere, (Sphere)targetShape, delta, out hit);
                    case ShapeType.AABB:
                        return TrySweepSphereAABB(movingSphere, (AABB)targetShape, delta, out hit);
                    case ShapeType.OBB:
                        return TrySweepSphereOBB(movingSphere, (OBB)targetShape, delta, out hit);
                    case ShapeType.Capsule:
                        return TrySweepSphereCapsule(movingSphere, (Capsule)targetShape, delta, out hit);
                }
            }

            if (movingShape.Type == ShapeType.Capsule)
            {
                Capsule movingCapsule = (Capsule)movingShape;
                switch (targetShape.Type)
                {
                    case ShapeType.Sphere:
                        return TrySweepCapsule(movingCapsule, targetShape, delta, out hit);
                    case ShapeType.AABB:
                        return TrySweepCapsule(movingCapsule, targetShape, delta, out hit);
                    case ShapeType.OBB:
                        return TrySweepCapsule(movingCapsule, targetShape, delta, out hit);
                    case ShapeType.Capsule:
                        return TrySweepCapsuleCapsule(movingCapsule, (Capsule)targetShape, delta, out hit);
                }
            }

            if (TryGetBoxCastData(movingShape, out BoxCastData movingBox)
                && TryGetBoxCastData(targetShape, out BoxCastData targetBox))
            {
                return TrySweepBoxBox(movingShape, targetShape, movingBox, targetBox, delta, out hit);
            }

            return TryBoundsShapeCast(movingShape, targetShape, delta, out hit);
        }

        public static bool TryBoundsShapeCast(Shape movingShape, Shape targetShape, fix3 delta, out ShapeCastHit hit)
        {
            hit = default;
            if (movingShape == null
                || targetShape == null
                || !IsShapeTypeImplemented(movingShape.Type)
                || !IsShapeTypeImplemented(targetShape.Type))
            {
                return false;
            }

            return TryAABBCast(ComputeBounds(movingShape), ComputeBounds(targetShape), delta, out hit);
        }

        public static bool TryAABBCast(AABB movingBounds, AABB targetBounds, fix3 delta, out ShapeCastHit hit)
        {
            if (IsOverlap(movingBounds, targetBounds))
            {
                fix3 overlapNormal = GetAABBOverlapNormal(movingBounds, targetBounds);
                hit = new ShapeCastHit(fix.Zero, overlapNormal, movingBounds.center);
                return true;
            }

            AABB expandedTarget = AABB.FromMinMax(targetBounds.min - movingBounds.extents, targetBounds.max + movingBounds.extents);
            fix tEnter = fix.Zero;
            fix tExit = fix.One;
            fix3 normal = fix3.zero;

            if (!SweepAxis(movingBounds.center.x, delta.x, expandedTarget.min.x, expandedTarget.max.x, fix3.right, ref tEnter, ref tExit, ref normal)
                || !SweepAxis(movingBounds.center.y, delta.y, expandedTarget.min.y, expandedTarget.max.y, fix3.up, ref tEnter, ref tExit, ref normal)
                || !SweepAxis(movingBounds.center.z, delta.z, expandedTarget.min.z, expandedTarget.max.z, fix3.forward, ref tEnter, ref tExit, ref normal))
            {
                hit = default;
                return false;
            }

            if (tEnter < fix.Zero || tEnter > fix.One)
            {
                hit = default;
                return false;
            }

            if (normal == fix3.zero)
            {
                normal = GetAABBOverlapNormal(movingBounds, targetBounds);
            }

            fix3 impactCenter = movingBounds.center + delta * tEnter;
            hit = new ShapeCastHit(tEnter, normal, impactCenter);
            return true;
        }

        private static bool TrySweepSphereSphere(Sphere movingSphere, Sphere targetSphere, fix3 delta, out ShapeCastHit hit)
        {
            fix3 offset = movingSphere.Center - targetSphere.Center;
            fix radius = movingSphere.Radius + targetSphere.Radius;
            fix radiusSq = radius * radius;
            fix distanceSq = math.lengthsq(offset);
            if (distanceSq <= radiusSq)
            {
                fix3 overlapNormal = NormalizeCastOrDefault(targetSphere.Center - movingSphere.Center, fix3.right);
                hit = new ShapeCastHit(fix.Zero, overlapNormal, movingSphere.Center + overlapNormal * movingSphere.Radius);
                return true;
            }

            fix a = math.dot(delta, delta);
            if (a <= math.Epsilon)
            {
                hit = default;
                return false;
            }

            fix b = fix._2 * math.dot(offset, delta);
            if (b >= fix.Zero)
            {
                hit = default;
                return false;
            }

            fix c = distanceSq - radiusSq;
            fix discriminant = b * b - new fix(4) * a * c;
            if (discriminant < fix.Zero)
            {
                hit = default;
                return false;
            }

            fix time = (-b - math.sqrt(discriminant)) / (fix._2 * a);
            if (time < fix.Zero || time > fix.One)
            {
                hit = default;
                return false;
            }

            fix3 impactCenter = movingSphere.Center + delta * time;
            fix3 normal = NormalizeCastOrDefault(targetSphere.Center - impactCenter, fix3.right);
            hit = new ShapeCastHit(time, normal, impactCenter + normal * movingSphere.Radius);
            return true;
        }

        private static bool TrySweepSphereAABB(Sphere movingSphere, AABB targetBounds, fix3 delta, out ShapeCastHit hit)
        {
            fix3 radius = new fix3(movingSphere.Radius, movingSphere.Radius, movingSphere.Radius);
            AABB expandedTarget = AABB.FromMinMax(targetBounds.min - radius, targetBounds.max + radius);
            if (!TrySweepPointAABB(movingSphere.Center, expandedTarget, delta, out fix fraction, out fix3 normal))
            {
                hit = default;
                return false;
            }

            fix3 impactCenter = movingSphere.Center + delta * fraction;
            hit = new ShapeCastHit(fraction, normal, impactCenter + normal * movingSphere.Radius);
            return true;
        }

        private static bool TrySweepSphereOBB(Sphere movingSphere, OBB targetBounds, fix3 delta, out ShapeCastHit hit)
        {
            quaternion inverseOrientation = quaternion.conjugate(targetBounds.orientation);
            fix3 localOrigin = inverseOrientation * (movingSphere.Center - targetBounds.center);
            fix3 localDelta = inverseOrientation * delta;
            fix3 radius = new fix3(movingSphere.Radius, movingSphere.Radius, movingSphere.Radius);
            fix3 expandedExtents = targetBounds.extents + radius;
            AABB expandedTarget = new AABB(fix3.zero, expandedExtents * fix._2);
            if (!TrySweepPointAABB(localOrigin, expandedTarget, localDelta, out fix fraction, out fix3 localNormal))
            {
                hit = default;
                return false;
            }

            fix3 normal = NormalizeCastOrDefault(targetBounds.orientation * localNormal, fix3.right);
            fix3 impactCenter = movingSphere.Center + delta * fraction;
            hit = new ShapeCastHit(fraction, normal, impactCenter + normal * movingSphere.Radius);
            return true;
        }

        private static bool TrySweepSphereCapsule(Sphere movingSphere, Capsule targetCapsule, fix3 delta, out ShapeCastHit hit)
        {
            return TrySweepPointCapsule(movingSphere.Center, movingSphere.Radius, targetCapsule, delta, out hit);
        }

        private static bool TrySweepCapsule(Capsule movingCapsule, Shape targetShape, fix3 delta, out ShapeCastHit hit)
        {
            hit = default;
            bool found = false;
            TrySelectEarlierCast(CreateSphereAt(movingCapsule.Center1, movingCapsule.Radius), targetShape, delta, ref found, ref hit);
            TrySelectEarlierCast(CreateSphereAt(movingCapsule.Center2, movingCapsule.Radius), targetShape, delta, ref found, ref hit);
            TrySelectEarlierCast(CreateSphereAt(movingCapsule.Center, movingCapsule.Radius), targetShape, delta, ref found, ref hit);
            return found;
        }

        private static bool TrySweepCapsuleCapsule(Capsule movingCapsule, Capsule targetCapsule, fix3 delta, out ShapeCastHit hit)
        {
            hit = default;
            bool found = false;
            TrySelectEarlierCapsulePoint(movingCapsule.Center1, movingCapsule.Radius, targetCapsule, delta, ref found, ref hit);
            TrySelectEarlierCapsulePoint(movingCapsule.Center2, movingCapsule.Radius, targetCapsule, delta, ref found, ref hit);
            TrySelectEarlierCapsulePoint(movingCapsule.Center, movingCapsule.Radius, targetCapsule, delta, ref found, ref hit);
            return found;
        }

        private static void TrySelectEarlierCast(Sphere movingSphere, Shape targetShape, fix3 delta, ref bool found, ref ShapeCastHit bestHit)
        {
            if (TryShapeCast(movingSphere, targetShape, delta, out ShapeCastHit candidate))
            {
                SelectEarlier(candidate, ref found, ref bestHit);
            }
        }

        private static void TrySelectEarlierCapsulePoint(
            fix3 movingPoint,
            fix movingRadius,
            Capsule targetCapsule,
            fix3 delta,
            ref bool found,
            ref ShapeCastHit bestHit)
        {
            if (TrySweepPointCapsule(movingPoint, movingRadius, targetCapsule, delta, out ShapeCastHit candidate))
            {
                SelectEarlier(candidate, ref found, ref bestHit);
            }
        }

        private static void SelectEarlier(ShapeCastHit candidate, ref bool found, ref ShapeCastHit bestHit)
        {
            if (!found || candidate.fraction < bestHit.fraction)
            {
                found = true;
                bestHit = candidate;
            }
        }

        private static bool TrySweepPointCapsule(fix3 point, fix radius, Capsule targetCapsule, fix3 delta, out ShapeCastHit hit)
        {
            hit = default;
            fix combinedRadius = radius + targetCapsule.Radius;
            if (TrySweepSphereSphere(CreateSphereAt(point, radius), CreateSphereAt(targetCapsule.Center1, targetCapsule.Radius), delta, out ShapeCastHit endpoint0))
            {
                hit = endpoint0;
            }

            bool found = hit.normal != fix3.zero || hit.fraction != fix.Zero;
            if (TrySweepSphereSphere(CreateSphereAt(point, radius), CreateSphereAt(targetCapsule.Center2, targetCapsule.Radius), delta, out ShapeCastHit endpoint1))
            {
                SelectEarlier(endpoint1, ref found, ref hit);
            }

            if (TrySweepPointCapsuleBody(point, radius, combinedRadius, targetCapsule, delta, out ShapeCastHit bodyHit))
            {
                SelectEarlier(bodyHit, ref found, ref hit);
            }

            return found;
        }

        private static bool TrySweepPointCapsuleBody(
            fix3 point,
            fix movingRadius,
            fix combinedRadius,
            Capsule targetCapsule,
            fix3 delta,
            out ShapeCastHit hit)
        {
            fix3 segment = targetCapsule.Center2 - targetCapsule.Center1;
            fix segmentLengthSq = math.lengthsq(segment);
            if (segmentLengthSq <= math.Epsilon)
            {
                hit = default;
                return false;
            }

            fix3 offset = point - targetCapsule.Center1;
            fix projection = math.dot(offset, segment) / segmentLengthSq;
            fix3 closest = targetCapsule.Center1 + segment * math.clamp(projection, fix.Zero, fix.One);
            fix3 initialDelta = point - closest;
            if (projection >= fix.Zero && projection <= fix.One && math.lengthsq(initialDelta) <= combinedRadius * combinedRadius)
            {
                fix3 normal = NormalizeCastOrDefault(closest - point, fix3.right);
                hit = new ShapeCastHit(fix.Zero, normal, point + normal * movingRadius);
                return true;
            }

            fix3 lineVelocity = delta - segment * (math.dot(delta, segment) / segmentLengthSq);
            fix3 lineOffset = offset - segment * (math.dot(offset, segment) / segmentLengthSq);
            fix a = math.dot(lineVelocity, lineVelocity);
            if (a <= math.Epsilon)
            {
                hit = default;
                return false;
            }

            fix b = fix._2 * math.dot(lineOffset, lineVelocity);
            if (b >= fix.Zero)
            {
                hit = default;
                return false;
            }

            fix c = math.dot(lineOffset, lineOffset) - combinedRadius * combinedRadius;
            fix discriminant = b * b - new fix(4) * a * c;
            if (discriminant < fix.Zero)
            {
                hit = default;
                return false;
            }

            fix time = (-b - math.sqrt(discriminant)) / (fix._2 * a);
            if (time < fix.Zero || time > fix.One)
            {
                hit = default;
                return false;
            }

            fix3 impactCenter = point + delta * time;
            fix impactProjection = math.dot(impactCenter - targetCapsule.Center1, segment) / segmentLengthSq;
            if (impactProjection < fix.Zero || impactProjection > fix.One)
            {
                hit = default;
                return false;
            }

            fix3 impactClosest = targetCapsule.Center1 + segment * impactProjection;
            fix3 impactNormal = NormalizeCastOrDefault(impactClosest - impactCenter, fix3.right);
            hit = new ShapeCastHit(time, impactNormal, impactCenter + impactNormal * movingRadius);
            return true;
        }

        private static Sphere CreateSphereAt(fix3 center, fix radius)
        {
            return new Sphere(center, radius);
        }

        private static bool TrySweepBoxBox(
            Shape movingShape,
            Shape targetShape,
            BoxCastData movingBox,
            BoxCastData targetBox,
            fix3 delta,
            out ShapeCastHit hit)
        {
            if (TryComputeContact(movingShape, targetShape, NarrowPhaseAlgorithm.SAT, out CollisionInfo collision))
            {
                fix3 normal = math.lengthsq(collision.normal) > math.Epsilon ? collision.normal : NormalizeCastOrDefault(targetBox.center - movingBox.center, fix3.right);
                fix3 point = collision.hasContact ? collision[0].position : GetBoxSupport(movingBox, normal);
                hit = new ShapeCastHit(fix.Zero, normal, point);
                return true;
            }

            fix tEnter = fix.Zero;
            fix tExit = fix.One;
            fix3 normalEnter = fix3.zero;

            for (int i = 0; i < 3; i++)
            {
                if (!SweepBoxAxis(movingBox, targetBox, movingBox.GetAxis(i), delta, ref tEnter, ref tExit, ref normalEnter))
                {
                    hit = default;
                    return false;
                }
            }

            for (int i = 0; i < 3; i++)
            {
                if (!SweepBoxAxis(movingBox, targetBox, targetBox.GetAxis(i), delta, ref tEnter, ref tExit, ref normalEnter))
                {
                    hit = default;
                    return false;
                }
            }

            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    fix3 axis = math.cross(movingBox.GetAxis(i), targetBox.GetAxis(j));
                    if (!SweepBoxAxis(movingBox, targetBox, axis, delta, ref tEnter, ref tExit, ref normalEnter))
                    {
                        hit = default;
                        return false;
                    }
                }
            }

            if (tEnter < fix.Zero || tEnter > fix.One || tEnter > tExit)
            {
                hit = default;
                return false;
            }

            if (math.lengthsq(normalEnter) <= math.Epsilon)
            {
                normalEnter = NormalizeCastOrDefault(targetBox.center - movingBox.center, fix3.right);
            }

            BoxCastData impactBox = movingBox.Move(delta * tEnter);
            fix3 point0 = GetBoxSupport(impactBox, normalEnter);
            fix3 point1 = GetBoxSupport(targetBox, -normalEnter);
            hit = new ShapeCastHit(tEnter, normalEnter, (point0 + point1) * fix._0_5);
            return true;
        }

        private static bool SweepBoxAxis(
            BoxCastData movingBox,
            BoxCastData targetBox,
            fix3 axis,
            fix3 delta,
            ref fix tEnter,
            ref fix tExit,
            ref fix3 normalEnter)
        {
            fix lengthSq = math.lengthsq(axis);
            if (lengthSq <= fix._0_0001)
            {
                return true;
            }

            axis /= math.sqrt(lengthSq);
            fix movingRadius = movingBox.ProjectRadius(axis);
            fix targetRadius = targetBox.ProjectRadius(axis);
            fix movingCenter = math.dot(movingBox.center, axis);
            fix targetCenter = math.dot(targetBox.center, axis);
            fix velocity = math.dot(delta, axis);
            fix movingMin = movingCenter - movingRadius;
            fix movingMax = movingCenter + movingRadius;
            fix targetMin = targetCenter - targetRadius;
            fix targetMax = targetCenter + targetRadius;

            return SweepInterval(movingMin, movingMax, targetMin, targetMax, velocity, axis, ref tEnter, ref tExit, ref normalEnter);
        }

        private static bool SweepInterval(
            fix movingMin,
            fix movingMax,
            fix targetMin,
            fix targetMax,
            fix velocity,
            fix3 axis,
            ref fix tEnter,
            ref fix tExit,
            ref fix3 normalEnter)
        {
            fix axisEnter = fix.Zero;
            fix axisExit = fix.One;
            fix3 axisNormal = fix3.zero;

            if (movingMax < targetMin)
            {
                if (velocity <= math.Epsilon)
                {
                    return false;
                }

                axisEnter = (targetMin - movingMax) / velocity;
                axisExit = (targetMax - movingMin) / velocity;
                axisNormal = axis;
            }
            else if (targetMax < movingMin)
            {
                if (velocity >= -math.Epsilon)
                {
                    return false;
                }

                axisEnter = (targetMax - movingMin) / velocity;
                axisExit = (targetMin - movingMax) / velocity;
                axisNormal = -axis;
            }
            else
            {
                if (velocity > math.Epsilon)
                {
                    axisExit = (targetMax - movingMin) / velocity;
                }
                else if (velocity < -math.Epsilon)
                {
                    axisExit = (targetMin - movingMax) / velocity;
                }
            }

            if (axisEnter > tEnter)
            {
                tEnter = axisEnter;
                normalEnter = axisNormal;
            }

            if (axisExit < tExit)
            {
                tExit = axisExit;
            }

            return tEnter <= tExit && tExit >= fix.Zero && tEnter <= fix.One;
        }

        private static bool TryGetBoxCastData(Shape shape, out BoxCastData box)
        {
            switch (shape.Type)
            {
                case ShapeType.AABB:
                    AABB aabb = (AABB)shape;
                    box = new BoxCastData(aabb.center, aabb.extents, fix3.right, fix3.up, fix3.forward);
                    return true;
                case ShapeType.OBB:
                    OBB obb = (OBB)shape;
                    box = new BoxCastData(
                        obb.center,
                        obb.extents,
                        obb.orientation * fix3.right,
                        obb.orientation * fix3.up,
                        obb.orientation * fix3.forward);
                    return true;
                default:
                    box = default;
                    return false;
            }
        }

        private static fix3 GetBoxSupport(BoxCastData box, fix3 direction)
        {
            return box.center
                + box.axis0 * (math.dot(box.axis0, direction) >= fix.Zero ? box.extents.x : -box.extents.x)
                + box.axis1 * (math.dot(box.axis1, direction) >= fix.Zero ? box.extents.y : -box.extents.y)
                + box.axis2 * (math.dot(box.axis2, direction) >= fix.Zero ? box.extents.z : -box.extents.z);
        }

        private static bool TrySweepPointAABB(fix3 point, AABB targetBounds, fix3 delta, out fix fraction, out fix3 normal)
        {
            if (IsOverlap(targetBounds, point))
            {
                fraction = fix.Zero;
                normal = GetAABBOverlapNormal(new AABB(point, fix3.zero), targetBounds);
                return true;
            }

            fix tEnter = fix.Zero;
            fix tExit = fix.One;
            normal = fix3.zero;
            if (!SweepAxis(point.x, delta.x, targetBounds.min.x, targetBounds.max.x, fix3.right, ref tEnter, ref tExit, ref normal)
                || !SweepAxis(point.y, delta.y, targetBounds.min.y, targetBounds.max.y, fix3.up, ref tEnter, ref tExit, ref normal)
                || !SweepAxis(point.z, delta.z, targetBounds.min.z, targetBounds.max.z, fix3.forward, ref tEnter, ref tExit, ref normal))
            {
                fraction = fix.Zero;
                normal = fix3.zero;
                return false;
            }

            if (tEnter < fix.Zero || tEnter > fix.One)
            {
                fraction = fix.Zero;
                normal = fix3.zero;
                return false;
            }

            fraction = tEnter;
            if (normal == fix3.zero)
            {
                normal = GetAABBOverlapNormal(new AABB(point, fix3.zero), targetBounds);
            }

            return true;
        }

        private static bool SweepAxis(
            fix origin,
            fix delta,
            fix min,
            fix max,
            fix3 axis,
            ref fix tEnter,
            ref fix tExit,
            ref fix3 normal)
        {
            if (math.abs(delta) <= math.Epsilon)
            {
                return origin >= min && origin <= max;
            }

            fix t0 = (min - origin) / delta;
            fix t1 = (max - origin) / delta;
            if (t0 > t1)
            {
                fix swap = t0;
                t0 = t1;
                t1 = swap;
            }

            if (t0 > tEnter)
            {
                tEnter = t0;
                normal = delta > fix.Zero ? axis : -axis;
            }

            if (t1 < tExit)
            {
                tExit = t1;
            }

            return tEnter <= tExit;
        }

        private static fix3 GetAABBOverlapNormal(AABB movingBounds, AABB targetBounds)
        {
            fix overlapX = math.min(movingBounds.max.x, targetBounds.max.x) - math.max(movingBounds.min.x, targetBounds.min.x);
            fix overlapY = math.min(movingBounds.max.y, targetBounds.max.y) - math.max(movingBounds.min.y, targetBounds.min.y);
            fix overlapZ = math.min(movingBounds.max.z, targetBounds.max.z) - math.max(movingBounds.min.z, targetBounds.min.z);
            fix3 delta = targetBounds.center - movingBounds.center;

            if (overlapX <= overlapY && overlapX <= overlapZ)
            {
                return delta.x >= fix.Zero ? fix3.right : fix3.left;
            }

            if (overlapY <= overlapZ)
            {
                return delta.y >= fix.Zero ? fix3.up : fix3.down;
            }

            return delta.z >= fix.Zero ? fix3.forward : fix3.backward;
        }

        private static fix3 NormalizeCastOrDefault(fix3 value, fix3 fallback)
        {
            fix lengthSq = math.lengthsq(value);
            return lengthSq > math.Epsilon ? value / math.sqrt(lengthSq) : fallback;
        }

        private readonly struct BoxCastData
        {
            public readonly fix3 center;
            public readonly fix3 extents;
            public readonly fix3 axis0;
            public readonly fix3 axis1;
            public readonly fix3 axis2;

            public BoxCastData(fix3 center, fix3 extents, fix3 axis0, fix3 axis1, fix3 axis2)
            {
                this.center = center;
                this.extents = extents;
                this.axis0 = axis0;
                this.axis1 = axis1;
                this.axis2 = axis2;
            }

            public BoxCastData Move(fix3 translation)
            {
                return new BoxCastData(center + translation, extents, axis0, axis1, axis2);
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

            public fix ProjectRadius(fix3 axis)
            {
                return extents.x * math.abs(math.dot(axis0, axis))
                    + extents.y * math.abs(math.dot(axis1, axis))
                    + extents.z * math.abs(math.dot(axis2, axis));
            }
        }
    }
}
