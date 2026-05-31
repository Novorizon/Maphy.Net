using Maphy.Mathematics;

namespace Maphy.Physics
{
    public enum ShapeType
    {
        None = 0,
        AABB = 1,
        OBB = 2,
        Sphere = 3,
        Capsule = 4,
        ConvexHull = 5,
        TriangleMesh = 6,
        HeightField = 7,
    }

    public enum ShapeCategory
    {
        None = 0,
        Primitive = 1,
        Convex = 2,
        SceneGeometry = 3,
    }

    [System.Flags]
    public enum ShapeCapabilities
    {
        None = 0,
        Bounds = 1 << 0,
        SupportMapping = 1 << 1,
        Overlap = 1 << 2,
        ContactManifold = 1 << 3,
        Raycast = 1 << 4,
        ShapeCast = 1 << 5,
        StaticGeometry = 1 << 6,
    }

    public interface Shape
    {
        ShapeType Type { get; }
    }

    public interface BoundedShape : Shape
    {
        AABB Bounds { get; }
    }

    public interface SupportMappedShape : Shape
    {
        fix3 GetSupportPoint(fix3 direction);
    }

    public interface SceneGeometryShape : Shape
    {
        int Version { get; }
        AABB Bounds { get; }
    }

    public readonly struct ShapeDescriptor
    {
        public readonly ShapeType type;
        public readonly ShapeCategory category;
        public readonly ShapeCapabilities capabilities;
        public readonly bool implemented;

        public ShapeDescriptor(ShapeType type, ShapeCategory category, ShapeCapabilities capabilities, bool implemented)
        {
            this.type = type;
            this.category = category;
            this.capabilities = capabilities;
            this.implemented = implemented;
        }
    }
}
