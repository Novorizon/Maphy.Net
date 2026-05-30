using Maphy.Mathematics;

namespace Maphy.Physics
{
    public static partial class Physics
    {
        public enum Axis
        {
            XAxis,
            YAxis,
            ZAxis,
            AnyAxis,
            Default = AnyAxis
        }

        public static bool Overlaps(Shape shape0, Shape shape1)
        {
            if (shape0 == null || shape1 == null)
            {
                return false;
            }

            switch (shape0.Type, shape1.Type)
            {
                case (ShapeType.AABB, ShapeType.AABB):
                    return Overlaps((AABB)shape0, (AABB)shape1);
                case (ShapeType.AABB, ShapeType.OBB):
                    return Overlaps((AABB)shape0, (OBB)shape1);
                case (ShapeType.AABB, ShapeType.Sphere):
                    return Overlaps((AABB)shape0, (Sphere)shape1);
                case (ShapeType.AABB, ShapeType.Capsule):
                    return Overlaps((AABB)shape0, (Capsule)shape1);
                case (ShapeType.OBB, ShapeType.AABB):
                    return Overlaps((OBB)shape0, (AABB)shape1);
                case (ShapeType.OBB, ShapeType.OBB):
                    return Overlaps((OBB)shape0, (OBB)shape1);
                case (ShapeType.OBB, ShapeType.Sphere):
                    return Overlaps((OBB)shape0, (Sphere)shape1);
                case (ShapeType.OBB, ShapeType.Capsule):
                    return Overlaps((OBB)shape0, (Capsule)shape1);
                case (ShapeType.Sphere, ShapeType.AABB):
                    return Overlaps((Sphere)shape0, (AABB)shape1);
                case (ShapeType.Sphere, ShapeType.OBB):
                    return Overlaps((OBB)shape1, (Sphere)shape0);
                case (ShapeType.Sphere, ShapeType.Sphere):
                    return Overlaps((Sphere)shape0, (Sphere)shape1);
                case (ShapeType.Sphere, ShapeType.Capsule):
                    return Overlaps((Sphere)shape0, (Capsule)shape1);
                case (ShapeType.Capsule, ShapeType.AABB):
                    return Overlaps((AABB)shape1, (Capsule)shape0);
                case (ShapeType.Capsule, ShapeType.OBB):
                    return Overlaps((OBB)shape1, (Capsule)shape0);
                case (ShapeType.Capsule, ShapeType.Sphere):
                    return Overlaps((Sphere)shape1, (Capsule)shape0);
                case (ShapeType.Capsule, ShapeType.Capsule):
                    return Overlaps((Capsule)shape0, (Capsule)shape1);
                default:
                    return false;
            }
        }

        internal static AABB ComputeBounds(Shape shape)
        {
            switch (shape.Type)
            {
                case ShapeType.AABB:
                    return (AABB)shape;
                case ShapeType.OBB:
                    return ComputeBounds((OBB)shape);
                case ShapeType.Sphere:
                    return ((Sphere)shape).Bounds;
                case ShapeType.Capsule:
                    return ComputeBounds((Capsule)shape);
                default:
                    return new AABB(fix3.zero, fix3.zero);
            }
        }

        internal static AABB ComputeBounds(OBB obb)
        {
            fix3 axisX = obb.orientation * fix3.right;
            fix3 axisY = obb.orientation * fix3.up;
            fix3 axisZ = obb.orientation * fix3.forward;
            fix3 extents = math.abs(axisX) * obb.extents.x
                + math.abs(axisY) * obb.extents.y
                + math.abs(axisZ) * obb.extents.z;
            return new AABB(obb.center, extents * 2);
        }

        internal static AABB ComputeBounds(Capsule capsule)
        {
            fix3 radius = new fix3(capsule.Radius);
            fix3 min = math.min(capsule.Center1, capsule.Center2) - radius;
            fix3 max = math.max(capsule.Center1, capsule.Center2) + radius;
            return AABB.FromMinMax(min, max);
        }

        public static bool Overlaps(AABB a, OBB b) { return IsBoxOverlap(a, b); }
        public static bool Overlaps(AABB a, Capsule b) { return IsCapsuleOverlapAABB(b, a); }
        public static bool Overlaps(OBB a, AABB b) { return IsBoxOverlap(b, a); }
        public static bool Overlaps(OBB a, OBB b) { return IsBoxOverlap(a, b); }
        public static bool Overlaps(OBB a, Sphere b) { return IsOverlap(a, b); }
        public static bool Overlaps(OBB a, Capsule b) { return IsCapsuleOverlapOBB(b, a); }
        public static bool Overlaps(Capsule a, Capsule b) { return IsOverlap(a, b); }

        public static bool IsOverlap(AABB a, fix3 point, Axis axis = Axis.Default) { return IsOverlap(a, point); }
        public static bool IsOverlap(fix3 point, AABB a, Axis axis = Axis.Default) { return IsOverlap(a, point); }
        public static bool IsOverlap(OBB a, fix3 point, Axis axis = Axis.Default) { return IsOverlap(a, point); }
        public static bool IsOverlap(fix3 point, OBB a, Axis axis = Axis.Default) { return IsOverlap(a, point); }
        public static bool IsOverlap(Sphere a, fix3 point, Axis axis = Axis.Default) { return IsOverlap(a, point); }
        public static bool IsOverlap(fix3 point, Sphere a, Axis axis = Axis.Default) { return IsOverlap(a, point); }
        public static bool IsOverlap(Capsule a, fix3 point, Axis axis = Axis.Default) { return IsOverlap(a, point); }
        public static bool IsOverlap(fix3 point, Capsule a, Axis axis = Axis.Default) { return IsOverlap(a, point); }

        public static bool IsOverlap(AABB a, AABB b, Axis axis = Axis.Default) { return IsOverlap(a, b); }
        public static bool IsOverlap(AABB a, OBB b, Axis axis = Axis.Default) { return IsBoxOverlap(a, b); }
        public static bool IsOverlap(AABB a, Sphere b, Axis axis = Axis.Default) { return IsOverlap(b, a); }
        public static bool IsOverlap(AABB a, Capsule b, Axis axis = Axis.Default) { return IsCapsuleOverlapAABB(b, a); }

        public static bool IsOverlap(OBB a, OBB b, Axis axis = Axis.Default) { return IsBoxOverlap(a, b); }
        public static bool IsOverlap(OBB a, AABB b, Axis axis = Axis.Default) { return IsOverlap(b, a); }
        public static bool IsOverlap(OBB a, Sphere b, Axis axis = Axis.Default) { return IsOverlap(b, a); }
        public static bool IsOverlap(OBB a, Capsule b, Axis axis = Axis.Default) { return IsCapsuleOverlapOBB(b, a); }

        public static bool IsOverlap(Sphere a, Sphere b, Axis axis = Axis.Default) { return IsOverlap(a, b); }
        public static bool IsOverlap(Sphere a, AABB b, Axis axis = Axis.Default) { return IsOverlap(a, b); }
        public static bool IsOverlap(Sphere a, OBB b, Axis axis = Axis.Default) { return IsOverlap(a, b); }
        public static bool IsOverlap(Sphere a, Capsule b, Axis axis = Axis.Default) { return Overlaps(a, b); }

        public static bool IsOverlap(Capsule a, Capsule b, Axis axis = Axis.Default) { return IsOverlap(a, b); }
        public static bool IsOverlap(Capsule a, AABB b, Axis axis = Axis.Default) { return IsOverlap(b, a); }
        public static bool IsOverlap(Capsule a, OBB b, Axis axis = Axis.Default) { return IsOverlap(b, a); }
        public static bool IsOverlap(Capsule a, Sphere b, Axis axis = Axis.Default) { return Overlaps(b, a); }

        private static bool IsBoxOverlap(AABB a, OBB b)
        {
            return IsBoxOverlap(
                a.center,
                a.extents,
                fix3.right,
                fix3.up,
                fix3.forward,
                b.center,
                b.extents,
                b.orientation * fix3.right,
                b.orientation * fix3.up,
                b.orientation * fix3.forward);
        }

        private static bool IsBoxOverlap(OBB a, OBB b)
        {
            return IsBoxOverlap(
                a.center,
                a.extents,
                a.orientation * fix3.right,
                a.orientation * fix3.up,
                a.orientation * fix3.forward,
                b.center,
                b.extents,
                b.orientation * fix3.right,
                b.orientation * fix3.up,
                b.orientation * fix3.forward);
        }

        private static bool IsBoxOverlap(
            fix3 centerA,
            fix3 extentsA,
            fix3 axisA0,
            fix3 axisA1,
            fix3 axisA2,
            fix3 centerB,
            fix3 extentsB,
            fix3 axisB0,
            fix3 axisB1,
            fix3 axisB2)
        {
            fix3 delta = centerB - centerA;

            if (IsSeparated(delta, extentsA, axisA0, axisA1, axisA2, extentsB, axisB0, axisB1, axisB2, axisA0)) return false;
            if (IsSeparated(delta, extentsA, axisA0, axisA1, axisA2, extentsB, axisB0, axisB1, axisB2, axisA1)) return false;
            if (IsSeparated(delta, extentsA, axisA0, axisA1, axisA2, extentsB, axisB0, axisB1, axisB2, axisA2)) return false;
            if (IsSeparated(delta, extentsA, axisA0, axisA1, axisA2, extentsB, axisB0, axisB1, axisB2, axisB0)) return false;
            if (IsSeparated(delta, extentsA, axisA0, axisA1, axisA2, extentsB, axisB0, axisB1, axisB2, axisB1)) return false;
            if (IsSeparated(delta, extentsA, axisA0, axisA1, axisA2, extentsB, axisB0, axisB1, axisB2, axisB2)) return false;

            if (IsSeparatedOnCrossAxis(delta, extentsA, axisA0, axisA1, axisA2, extentsB, axisB0, axisB1, axisB2, axisA0, axisB0)) return false;
            if (IsSeparatedOnCrossAxis(delta, extentsA, axisA0, axisA1, axisA2, extentsB, axisB0, axisB1, axisB2, axisA0, axisB1)) return false;
            if (IsSeparatedOnCrossAxis(delta, extentsA, axisA0, axisA1, axisA2, extentsB, axisB0, axisB1, axisB2, axisA0, axisB2)) return false;
            if (IsSeparatedOnCrossAxis(delta, extentsA, axisA0, axisA1, axisA2, extentsB, axisB0, axisB1, axisB2, axisA1, axisB0)) return false;
            if (IsSeparatedOnCrossAxis(delta, extentsA, axisA0, axisA1, axisA2, extentsB, axisB0, axisB1, axisB2, axisA1, axisB1)) return false;
            if (IsSeparatedOnCrossAxis(delta, extentsA, axisA0, axisA1, axisA2, extentsB, axisB0, axisB1, axisB2, axisA1, axisB2)) return false;
            if (IsSeparatedOnCrossAxis(delta, extentsA, axisA0, axisA1, axisA2, extentsB, axisB0, axisB1, axisB2, axisA2, axisB0)) return false;
            if (IsSeparatedOnCrossAxis(delta, extentsA, axisA0, axisA1, axisA2, extentsB, axisB0, axisB1, axisB2, axisA2, axisB1)) return false;
            if (IsSeparatedOnCrossAxis(delta, extentsA, axisA0, axisA1, axisA2, extentsB, axisB0, axisB1, axisB2, axisA2, axisB2)) return false;

            return true;
        }

        private static bool IsSeparatedOnCrossAxis(
            fix3 delta,
            fix3 extentsA,
            fix3 axisA0,
            fix3 axisA1,
            fix3 axisA2,
            fix3 extentsB,
            fix3 axisB0,
            fix3 axisB1,
            fix3 axisB2,
            fix3 axis0,
            fix3 axis1)
        {
            fix3 axis = math.cross(axis0, axis1);
            return math.lengthsq(axis) > math.Epsilon
                && IsSeparated(delta, extentsA, axisA0, axisA1, axisA2, extentsB, axisB0, axisB1, axisB2, axis);
        }

        private static bool IsSeparated(
            fix3 delta,
            fix3 extentsA,
            fix3 axisA0,
            fix3 axisA1,
            fix3 axisA2,
            fix3 extentsB,
            fix3 axisB0,
            fix3 axisB1,
            fix3 axisB2,
            fix3 axis)
        {
            fix distance = math.abs(math.dot(delta, axis));
            fix radiusA = ProjectBox(extentsA, axisA0, axisA1, axisA2, axis);
            fix radiusB = ProjectBox(extentsB, axisB0, axisB1, axisB2, axis);
            return distance > radiusA + radiusB;
        }

        private static fix ProjectBox(fix3 extents, fix3 axis0, fix3 axis1, fix3 axis2, fix3 projectionAxis)
        {
            return extents.x * math.abs(math.dot(axis0, projectionAxis))
                + extents.y * math.abs(math.dot(axis1, projectionAxis))
                + extents.z * math.abs(math.dot(axis2, projectionAxis));
        }

        private static bool IsCapsuleOverlapAABB(Capsule capsule, AABB aabb)
        {
            fix3 local0 = capsule.Center1 - aabb.center;
            fix3 local1 = capsule.Center2 - aabb.center;
            return SegmentAABBDistanceSq(local0, local1, aabb.extents) <= capsule.Radius2;
        }

        private static bool IsCapsuleOverlapOBB(Capsule capsule, OBB obb)
        {
            quaternion inverseRotation = quaternion.conjugate(obb.orientation);
            fix3 local0 = inverseRotation * (capsule.Center1 - obb.center);
            fix3 local1 = inverseRotation * (capsule.Center2 - obb.center);
            return SegmentAABBDistanceSq(local0, local1, obb.extents) <= capsule.Radius2;
        }

        private static fix SegmentAABBDistanceSq(fix3 segment0, fix3 segment1, fix3 extents)
        {
            if (IsSegmentOverlapAABB(segment0, segment1, extents))
            {
                return fix.Zero;
            }

            fix minDistanceSq = math.min(PointAABBDistanceSq(segment0, extents), PointAABBDistanceSq(segment1, extents));
            CheckSegmentAABBEdgeDistance(segment0, segment1, extents, new fix3(-extents.x, -extents.y, -extents.z), new fix3(extents.x, -extents.y, -extents.z), ref minDistanceSq);
            CheckSegmentAABBEdgeDistance(segment0, segment1, extents, new fix3(-extents.x, extents.y, -extents.z), new fix3(extents.x, extents.y, -extents.z), ref minDistanceSq);
            CheckSegmentAABBEdgeDistance(segment0, segment1, extents, new fix3(-extents.x, -extents.y, extents.z), new fix3(extents.x, -extents.y, extents.z), ref minDistanceSq);
            CheckSegmentAABBEdgeDistance(segment0, segment1, extents, new fix3(-extents.x, extents.y, extents.z), new fix3(extents.x, extents.y, extents.z), ref minDistanceSq);

            CheckSegmentAABBEdgeDistance(segment0, segment1, extents, new fix3(-extents.x, -extents.y, -extents.z), new fix3(-extents.x, extents.y, -extents.z), ref minDistanceSq);
            CheckSegmentAABBEdgeDistance(segment0, segment1, extents, new fix3(extents.x, -extents.y, -extents.z), new fix3(extents.x, extents.y, -extents.z), ref minDistanceSq);
            CheckSegmentAABBEdgeDistance(segment0, segment1, extents, new fix3(-extents.x, -extents.y, extents.z), new fix3(-extents.x, extents.y, extents.z), ref minDistanceSq);
            CheckSegmentAABBEdgeDistance(segment0, segment1, extents, new fix3(extents.x, -extents.y, extents.z), new fix3(extents.x, extents.y, extents.z), ref minDistanceSq);

            CheckSegmentAABBEdgeDistance(segment0, segment1, extents, new fix3(-extents.x, -extents.y, -extents.z), new fix3(-extents.x, -extents.y, extents.z), ref minDistanceSq);
            CheckSegmentAABBEdgeDistance(segment0, segment1, extents, new fix3(extents.x, -extents.y, -extents.z), new fix3(extents.x, -extents.y, extents.z), ref minDistanceSq);
            CheckSegmentAABBEdgeDistance(segment0, segment1, extents, new fix3(-extents.x, extents.y, -extents.z), new fix3(-extents.x, extents.y, extents.z), ref minDistanceSq);
            CheckSegmentAABBEdgeDistance(segment0, segment1, extents, new fix3(extents.x, extents.y, -extents.z), new fix3(extents.x, extents.y, extents.z), ref minDistanceSq);

            return minDistanceSq;
        }

        private static void CheckSegmentAABBEdgeDistance(fix3 segment0, fix3 segment1, fix3 extents, fix3 edge0, fix3 edge1, ref fix minDistanceSq)
        {
            GetClosestPointsBetweenSegments(segment0, segment1, edge0, edge1, out fix3 closestSegment, out fix3 closestEdge);
            fix distanceSq = math.distancesq(closestSegment, closestEdge);
            minDistanceSq = math.min(minDistanceSq, distanceSq);
        }

        private static fix PointAABBDistanceSq(fix3 point, fix3 extents)
        {
            fix3 closest = math.clamp(point, -extents, extents);
            return math.distancesq(point, closest);
        }

        private static bool IsSegmentOverlapAABB(fix3 segment0, fix3 segment1, fix3 extents)
        {
            fix3 direction = segment1 - segment0;
            fix tMin = fix.Zero;
            fix tMax = fix.One;

            return IsSegmentOverlapAABBSlab(segment0.x, direction.x, -extents.x, extents.x, ref tMin, ref tMax)
                && IsSegmentOverlapAABBSlab(segment0.y, direction.y, -extents.y, extents.y, ref tMin, ref tMax)
                && IsSegmentOverlapAABBSlab(segment0.z, direction.z, -extents.z, extents.z, ref tMin, ref tMax);
        }

        private static bool IsSegmentOverlapAABBSlab(fix origin, fix direction, fix min, fix max, ref fix tMin, ref fix tMax)
        {
            if (math.abs(direction) <= math.Epsilon)
            {
                return origin >= min && origin <= max;
            }

            fix invDirection = fix.One / direction;
            fix t0 = (min - origin) * invDirection;
            fix t1 = (max - origin) * invDirection;
            if (t0 > t1)
            {
                fix temp = t0;
                t0 = t1;
                t1 = temp;
            }

            tMin = math.max(tMin, t0);
            tMax = math.min(tMax, t1);
            return tMin <= tMax;
        }
    }
}
