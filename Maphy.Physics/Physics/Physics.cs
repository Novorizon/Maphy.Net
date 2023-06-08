
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
            AnyAxis,//ÈÎÒâÖáÏò
            Default = YAxis
        };

        public static bool IsOverlap(AABB a, fix3 point, Axis axis = Axis.Default) => IsOverlap(a, point);
        public static bool IsOverlap(fix3 point, AABB a, Axis axis = Axis.Default) => IsOverlap(a, point);
        public static bool IsOverlap(OBB a, fix3 point, Axis axis = Axis.Default) => IsOverlap(a, point);
        public static bool IsOverlap(fix3 point, OBB a, Axis axis = Axis.Default) => IsOverlap(a, point);
        public static bool IsOverlap(Sphere a, fix3 point, Axis axis = Axis.Default) => IsOverlap(a, point);
        public static bool IsOverlap(fix3 point, Sphere a, Axis axis = Axis.Default) => IsOverlap(a, point);
        public static bool IsOverlap(Capsule a, fix3 point, Axis axis = Axis.Default) => IsOverlap(a, point);
        public static bool IsOverlap(fix3 point, Capsule a, Axis axis = Axis.Default) => IsOverlap(a, point);

        public static bool IsOverlap(AABB a, AABB b, Axis axis = Axis.Default)
        {
            return axis switch
            {
                Axis.XAxis => IsOverlap(a, b),
                Axis.YAxis => IsOverlap(a, b),
                Axis.ZAxis => IsOverlap(a, b),
                Axis.AnyAxis => IsOverlap(a, b),
                _ => IsOverlap(a, b)
            };
        }

        public static bool IsOverlap(AABB a, OBB b, Axis axis = Axis.Default)
        {
            return axis switch
            {
                Axis.XAxis => SeparatingAxisTest(a, b),
                Axis.YAxis => SeparatingAxisTestY(a, b),
                Axis.ZAxis => SeparatingAxisTest(a, b),
                Axis.AnyAxis => SeparatingAxisTest(a, b),
                _ => SeparatingAxisTest(a, b)
            };
        }

        public static bool IsOverlap(AABB a, Sphere b, Axis axis = Axis.Default)
        {
            return axis switch
            {
                Axis.XAxis => SeparatingAxisTest(a, b),
                Axis.YAxis => SeparatingAxisTest(a, b),
                Axis.ZAxis => SeparatingAxisTest(a, b),
                Axis.AnyAxis => SeparatingAxisTest(a, b),
                _ => SeparatingAxisTest(a, b)
            };
        }

        public static bool IsOverlap(AABB a, Capsule b, Axis axis = Axis.Default)
        {
            return axis switch
            {
                Axis.XAxis => SeparatingAxisTest(a, b),
                Axis.YAxis => SeparatingAxisTestY(a, b),
                Axis.ZAxis => SeparatingAxisTest(a, b),
                Axis.AnyAxis => SeparatingAxisTest(a, b),
                _ => SeparatingAxisTest(a, b)
            };
        }


        public static bool IsOverlap(OBB a, OBB b, Axis axis = Axis.Default)
        {
            return axis switch
            {
                Axis.XAxis => SeparatingAxisTest(a, b),
                Axis.YAxis => SeparatingAxisTestY(a, b),
                Axis.ZAxis => SeparatingAxisTest(a, b),
                Axis.AnyAxis => SeparatingAxisTest(a, b),
                _ => SeparatingAxisTest(a, b)
            };
        }

        public static bool IsOverlap(OBB a, AABB b, Axis axis = Axis.Default)
        {
            return axis switch
            {
                Axis.XAxis => SeparatingAxisTest(b, a),
                Axis.YAxis => SeparatingAxisTestY(b, a),
                Axis.ZAxis => SeparatingAxisTest(b, a),
                Axis.AnyAxis => SeparatingAxisTest(b, a),
                _ => SeparatingAxisTest(b, a)
            };
        }

        public static bool IsOverlap(OBB a, Sphere b, Axis axis = Axis.Default)
        {
            return axis switch
            {
                Axis.XAxis => SeparatingAxisTest(a, b),
                Axis.YAxis => SeparatingAxisTest(a, b),
                Axis.ZAxis => SeparatingAxisTest(a, b),
                Axis.AnyAxis => SeparatingAxisTest(a, b),
                _ => SeparatingAxisTest(a, b)
            };
        }

        public static bool IsOverlap(OBB a, Capsule b, Axis axis = Axis.Default)
        {
            return axis switch
            {
                Axis.XAxis => SeparatingAxisTest(a, b),
                Axis.YAxis => SeparatingAxisTestY(a, b),
                Axis.ZAxis => SeparatingAxisTest(a, b),
                Axis.AnyAxis => SeparatingAxisTest(a, b),
                _ => SeparatingAxisTest(a, b)
            };
        }

        public static bool IsOverlap(Sphere a, Sphere b, Axis axis = Axis.Default)
        {
            return axis switch
            {
                Axis.XAxis => IsOverlap(a, b),
                Axis.YAxis => IsOverlap(a, b),
                Axis.ZAxis => IsOverlap(a, b),
                Axis.AnyAxis => IsOverlap(a, b),
                _ => IsOverlap(a, b)
            };
        }


        public static bool IsOverlap(Sphere a, AABB b, Axis axis = Axis.Default)
        {
            return axis switch
            {
                Axis.XAxis => SeparatingAxisTest(b, a),
                Axis.YAxis => SeparatingAxisTest(b, a),
                Axis.ZAxis => SeparatingAxisTest(b, a),
                Axis.AnyAxis => SeparatingAxisTest(b, a),
                _ => SeparatingAxisTest(b, a)
            };
        }

        public static bool IsOverlap(Sphere a, OBB b, Axis axis = Axis.Default)
        {
            return axis switch
            {
                Axis.XAxis => SeparatingAxisTest(b, a),
                Axis.YAxis => SeparatingAxisTest(b, a),
                Axis.ZAxis => SeparatingAxisTest(b, a),
                Axis.AnyAxis => SeparatingAxisTest(b, a),
                _ => SeparatingAxisTest(b, b)
            };
        }

        public static bool IsOverlap(Sphere a, Capsule b, Axis axis = Axis.Default)
        {
            return axis switch
            {
                Axis.XAxis => SeparatingAxisTest(b, a),
                Axis.YAxis => SeparatingAxisTestY(b, a),
                Axis.ZAxis => SeparatingAxisTest(b, a),
                Axis.AnyAxis => SeparatingAxisTest(b, a),
                _ => SeparatingAxisTest(b, a)
            };
        }

        public static bool IsOverlap(Capsule a, Capsule b, Axis axis = Axis.Default)
        {
            return axis switch
            {
                Axis.XAxis => IsOverlapParallel(a, b),
                Axis.YAxis => IsOverlapParallel(a, b),
                Axis.ZAxis => IsOverlapParallel(a, b),
                Axis.AnyAxis => IsOverlap(a, b),
                _ => IsOverlap(a, b)
            };
        }

        public static bool IsOverlap(Capsule a, AABB b, Axis axis = Axis.Default)
        {
            return axis switch
            {
                Axis.XAxis => SeparatingAxisTest(b, a),
                Axis.YAxis => SeparatingAxisTestY(b, a),
                Axis.ZAxis => SeparatingAxisTest(b, a),
                Axis.AnyAxis => SeparatingAxisTest(b, a),
                _ => SeparatingAxisTest(b, a)
            };
        }

        public static bool IsOverlap(Capsule a, OBB b, Axis axis = Axis.Default)
        {
            return axis switch
            {
                Axis.XAxis => SeparatingAxisTest(b, a),
                Axis.YAxis => SeparatingAxisTest(b, a),
                Axis.ZAxis => SeparatingAxisTest(b, a),
                Axis.AnyAxis => SeparatingAxisTest(b, a),
                _ => SeparatingAxisTest(b, a)
            };
        }

        public static bool IsOverlap(Capsule a, Sphere b, Axis axis = Axis.Default)
        {
            return axis switch
            {
                Axis.XAxis => SeparatingAxisTest(a, b),
                Axis.YAxis => SeparatingAxisTest(a, b),
                Axis.ZAxis => SeparatingAxisTest(a, b),
                Axis.AnyAxis => SeparatingAxisTest(a, b),
                _ => SeparatingAxisTest(a, b)
            };
        }
    }
}
