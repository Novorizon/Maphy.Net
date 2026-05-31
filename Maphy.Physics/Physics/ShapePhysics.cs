namespace Maphy.Physics
{
    public static partial class Physics
    {
        public static bool IsShapeTypeImplemented(ShapeType type)
        {
            return GetShapeDescriptor(type).implemented;
        }

        public static ShapeDescriptor GetShapeDescriptor(ShapeType type)
        {
            switch (type)
            {
                case ShapeType.AABB:
                    return new ShapeDescriptor(
                        type,
                        ShapeCategory.Primitive,
                        ShapeCapabilities.Bounds | ShapeCapabilities.SupportMapping | ShapeCapabilities.Overlap | ShapeCapabilities.ContactManifold | ShapeCapabilities.Raycast | ShapeCapabilities.ShapeCast,
                        true);
                case ShapeType.OBB:
                    return new ShapeDescriptor(
                        type,
                        ShapeCategory.Primitive,
                        ShapeCapabilities.Bounds | ShapeCapabilities.SupportMapping | ShapeCapabilities.Overlap | ShapeCapabilities.ContactManifold | ShapeCapabilities.Raycast,
                        true);
                case ShapeType.Sphere:
                    return new ShapeDescriptor(
                        type,
                        ShapeCategory.Primitive,
                        ShapeCapabilities.Bounds | ShapeCapabilities.SupportMapping | ShapeCapabilities.Overlap | ShapeCapabilities.ContactManifold | ShapeCapabilities.Raycast | ShapeCapabilities.ShapeCast,
                        true);
                case ShapeType.Capsule:
                    return new ShapeDescriptor(
                        type,
                        ShapeCategory.Primitive,
                        ShapeCapabilities.Bounds | ShapeCapabilities.SupportMapping | ShapeCapabilities.Overlap | ShapeCapabilities.ContactManifold | ShapeCapabilities.Raycast | ShapeCapabilities.ShapeCast,
                        true);
                case ShapeType.ConvexHull:
                    return new ShapeDescriptor(
                        type,
                        ShapeCategory.Convex,
                        ShapeCapabilities.Bounds | ShapeCapabilities.SupportMapping,
                        false);
                case ShapeType.TriangleMesh:
                case ShapeType.HeightField:
                    return new ShapeDescriptor(
                        type,
                        ShapeCategory.SceneGeometry,
                        ShapeCapabilities.Bounds | ShapeCapabilities.Raycast | ShapeCapabilities.StaticGeometry,
                        false);
                default:
                    return new ShapeDescriptor(ShapeType.None, ShapeCategory.None, ShapeCapabilities.None, false);
            }
        }
    }
}
