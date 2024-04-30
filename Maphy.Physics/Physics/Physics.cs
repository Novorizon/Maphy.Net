
using Maphy.Mathematics;
namespace Maphy.Physics
{
    public static partial class Physics
    {
        static readonly bool needCollisionInfo=true;
        public enum Axis
        {
            XAxis,
            YAxis,
            ZAxis,
            AnyAxis,//ÈÎÒâÖáÏò
            Default = AnyAxis
        }; 
        
        
        public static bool Overlaps(Shape shape0, Shape shape1)
        {
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
                    return Overlaps((AABB)shape1, (OBB)shape0);

                case (ShapeType.OBB, ShapeType.OBB):
                    return Overlaps((OBB)shape0, (OBB)shape1);

                case (ShapeType.OBB, ShapeType.Sphere):
                    return Overlaps((OBB)shape0, (Sphere)shape1);

                case (ShapeType.OBB, ShapeType.Capsule):
                    return Overlaps((OBB)shape0, (Capsule)shape1);

                case (ShapeType.Sphere, ShapeType.AABB):
                    return Overlaps((AABB)shape1, (Sphere)shape0);

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

        public static bool IsOverlap(AABB a, fix3 point, Axis axis = Axis.Default) => IsOverlap(a, point);
        public static bool IsOverlap(fix3 point, AABB a, Axis axis = Axis.Default) => IsOverlap(a, point);
        public static bool IsOverlap(OBB a, fix3 point, Axis axis = Axis.Default) => IsOverlap(a, point);
        public static bool IsOverlap(fix3 point, OBB a, Axis axis = Axis.Default) => IsOverlap(a, point);
        public static bool IsOverlap(Sphere a, fix3 point, Axis axis = Axis.Default) => IsOverlap(a, point);
        public static bool IsOverlap(fix3 point, Sphere a, Axis axis = Axis.Default) => IsOverlap(a, point);
        public static bool IsOverlap(Capsule a, fix3 point, Axis axis = Axis.Default) => IsOverlap(a, point);
        public static bool IsOverlap(fix3 point, Capsule a, Axis axis = Axis.Default) => IsOverlap(a, point);

        public static bool IsOverlap(AABB a, AABB b, Axis axis = Axis.Default) => IsOverlap(a, b);

        public static bool IsOverlap(AABB a, OBB b, Axis axis = Axis.Default) => IsOverlap(a, b);

        public static bool IsOverlap(AABB a, Sphere b, Axis axis = Axis.Default) => IsOverlap(a, b);

        public static bool IsOverlap(AABB a, Capsule b, Axis axis = Axis.Default) => IsOverlap(a, b);


        public static bool IsOverlap(OBB a, OBB b, Axis axis = Axis.Default) => IsOverlap(a, b);

        public static bool IsOverlap(OBB a, AABB b, Axis axis = Axis.Default) => IsOverlap(a, b);

        public static bool IsOverlap(OBB a, Sphere b, Axis axis = Axis.Default) => IsOverlap(a, b);

        public static bool IsOverlap(OBB a, Capsule b, Axis axis = Axis.Default) => IsOverlap(a, b);

        public static bool IsOverlap(Sphere a, Sphere b, Axis axis = Axis.Default) => IsOverlap(a, b);


        public static bool IsOverlap(Sphere a, AABB b, Axis axis = Axis.Default) => IsOverlap(a, b);

        public static bool IsOverlap(Sphere a, OBB b, Axis axis = Axis.Default) => IsOverlap(a, b);

        public static bool IsOverlap(Sphere a, Capsule b, Axis axis = Axis.Default) => IsOverlap(a, b);

        public static bool IsOverlap(Capsule a, Capsule b, Axis axis = Axis.Default) => IsOverlap(a, b);

        public static bool IsOverlap(Capsule a, AABB b, Axis axis = Axis.Default) => IsOverlap(a, b);

        public static bool IsOverlap(Capsule a, OBB b, Axis axis = Axis.Default) => IsOverlap(a, b);

        public static bool IsOverlap(Capsule a, Sphere b, Axis axis = Axis.Default) => IsOverlap(a, b);
    }
}
