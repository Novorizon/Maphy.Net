using Maphy.Mathematics;

namespace Maphy.Physics
{
    /// <summary>
    /// Tagged value shape for the no-GC core. It keeps the concrete shape data in one
    /// struct, so hot paths do not need Shape interface dispatch or boxed structs.
    /// </summary>
    public struct PhysicsShapeData
    {
        public ShapeType type;
        public AABB aabb;
        public OBB obb;
        public Sphere sphere;
        public Capsule capsule;

        public static PhysicsShapeData AABB(fix3 center, fix3 size)
        {
            return new PhysicsShapeData
            {
                type = ShapeType.AABB,
                aabb = new AABB(center, size),
            };
        }

        public static PhysicsShapeData OBB(fix3 center, fix3 size, quaternion rotation)
        {
            return new PhysicsShapeData
            {
                type = ShapeType.OBB,
                obb = new OBB(center, size, rotation),
            };
        }

        public static PhysicsShapeData Sphere(fix3 center, fix radius)
        {
            return new PhysicsShapeData
            {
                type = ShapeType.Sphere,
                sphere = new Sphere(center, radius),
            };
        }

        public static PhysicsShapeData Capsule(fix3 center, fix radius, fix height, quaternion rotation)
        {
            return new PhysicsShapeData
            {
                type = ShapeType.Capsule,
                capsule = new Capsule(center, radius, height, rotation, fix3.up),
            };
        }

        public PhysicsShapeData Transform(fix3 translation, quaternion orientation)
        {
            switch (type)
            {
                case ShapeType.AABB:
                    AABB transformedAabb = aabb;
                    transformedAabb.Update(translation + orientation * aabb.center);
                    return new PhysicsShapeData
                    {
                        type = ShapeType.AABB,
                        aabb = transformedAabb,
                    };
                case ShapeType.OBB:
                    OBB transformedObb = obb;
                    transformedObb.Update(translation + orientation * obb.center, orientation * obb.orientation);
                    return new PhysicsShapeData
                    {
                        type = ShapeType.OBB,
                        obb = transformedObb,
                    };
                case ShapeType.Sphere:
                    Sphere transformedSphere = sphere;
                    transformedSphere.Update(translation + orientation * sphere.Center);
                    return new PhysicsShapeData
                    {
                        type = ShapeType.Sphere,
                        sphere = transformedSphere,
                    };
                case ShapeType.Capsule:
                    Capsule transformedCapsule = capsule;
                    transformedCapsule.Update(translation + orientation * capsule.Center, orientation * capsule.Orientation);
                    return new PhysicsShapeData
                    {
                        type = ShapeType.Capsule,
                        capsule = transformedCapsule,
                    };
                default:
                    return default;
            }
        }

        public AABB ComputeBounds()
        {
            switch (type)
            {
                case ShapeType.AABB:
                    return aabb;
                case ShapeType.OBB:
                    return Physics.ComputeBounds(obb);
                case ShapeType.Sphere:
                    return sphere.Bounds;
                case ShapeType.Capsule:
                    return Physics.ComputeBounds(capsule);
                default:
                    return new AABB(fix3.zero, fix3.zero);
            }
        }

        public fix3 GetLocalCenter()
        {
            switch (type)
            {
                case ShapeType.AABB:
                    return aabb.center;
                case ShapeType.OBB:
                    return obb.center;
                case ShapeType.Sphere:
                    return sphere.Center;
                case ShapeType.Capsule:
                    return capsule.Center;
                default:
                    return fix3.zero;
            }
        }
    }
}
