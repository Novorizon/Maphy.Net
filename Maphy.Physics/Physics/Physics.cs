using Maphy.Mathematics;

namespace Maphy.Physics
{
    public static partial class Physics
    {
        static readonly bool needCollisionInfo = true;

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

        public static bool Overlaps(AABB a, OBB b) { return IsOverlap(a, b); }
        public static bool Overlaps(AABB a, Capsule b) { return IsOverlap(a, b); }
        public static bool Overlaps(OBB a, AABB b) { return IsOverlap(a, b); }
        public static bool Overlaps(OBB a, OBB b) { return IsOverlap(a, b); }
        public static bool Overlaps(OBB a, Sphere b) { return IsOverlap(a, b); }
        public static bool Overlaps(OBB a, Capsule b) { return IsOverlap(a, b); }
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
        public static bool IsOverlap(AABB a, OBB b, Axis axis = Axis.Default) { return IsOverlap(a, ComputeBounds(b)); }
        public static bool IsOverlap(AABB a, Sphere b, Axis axis = Axis.Default) { return IsOverlap(b, a); }
        public static bool IsOverlap(AABB a, Capsule b, Axis axis = Axis.Default) { return IsOverlap(a, ComputeBounds(b)); }

        public static bool IsOverlap(OBB a, OBB b, Axis axis = Axis.Default) { return IsOverlap(ComputeBounds(a), ComputeBounds(b)); }
        public static bool IsOverlap(OBB a, AABB b, Axis axis = Axis.Default) { return IsOverlap(b, a); }
        public static bool IsOverlap(OBB a, Sphere b, Axis axis = Axis.Default) { return IsOverlap(b, a); }
        public static bool IsOverlap(OBB a, Capsule b, Axis axis = Axis.Default) { return IsOverlap(ComputeBounds(a), ComputeBounds(b)); }

        public static bool IsOverlap(Sphere a, Sphere b, Axis axis = Axis.Default) { return IsOverlap(a, b); }
        public static bool IsOverlap(Sphere a, AABB b, Axis axis = Axis.Default) { return IsOverlap(a, b); }
        public static bool IsOverlap(Sphere a, OBB b, Axis axis = Axis.Default) { return IsOverlap(a, b); }
        public static bool IsOverlap(Sphere a, Capsule b, Axis axis = Axis.Default) { return Overlaps(a, b); }

        public static bool IsOverlap(Capsule a, Capsule b, Axis axis = Axis.Default) { return IsOverlap(a, b); }
        public static bool IsOverlap(Capsule a, AABB b, Axis axis = Axis.Default) { return IsOverlap(b, a); }
        public static bool IsOverlap(Capsule a, OBB b, Axis axis = Axis.Default) { return IsOverlap(b, a); }
        public static bool IsOverlap(Capsule a, Sphere b, Axis axis = Axis.Default) { return Overlaps(b, a); }
    }
}