using Maphy.Mathematics;

namespace Maphy.Physics
{
    public readonly struct MassProperties
    {
        public readonly fix mass;
        public readonly fix3 inertia;

        public MassProperties(fix mass, fix3 inertia)
        {
            this.mass = mass;
            this.inertia = inertia;
        }
    }

    public static partial class Physics
    {
        public static MassProperties ComputeMassProperties(Shape shape, fix density)
        {
            if (shape == null || density <= fix.Zero)
            {
                return new MassProperties(fix.Zero, fix3.zero);
            }

            switch (shape.Type)
            {
                case ShapeType.AABB:
                    return ComputeBoxMassProperties(((AABB)shape).size, density);
                case ShapeType.OBB:
                    return ComputeBoxMassProperties(((OBB)shape).size, density);
                case ShapeType.Sphere:
                    return ComputeSphereMassProperties((Sphere)shape, density);
                case ShapeType.Capsule:
                    return ComputeCapsuleMassProperties((Capsule)shape, density);
                default:
                    return new MassProperties(fix.Zero, fix3.zero);
            }
        }

        public static MassProperties ComputeMassProperties(PhysicsShapeData shape, fix density)
        {
            if (density <= fix.Zero)
            {
                return new MassProperties(fix.Zero, fix3.zero);
            }

            switch (shape.type)
            {
                case ShapeType.AABB:
                    return ComputeBoxMassProperties(shape.aabb.size, density);
                case ShapeType.OBB:
                    return ComputeBoxMassProperties(shape.obb.size, density);
                case ShapeType.Sphere:
                    return ComputeSphereMassProperties(shape.sphere, density);
                case ShapeType.Capsule:
                    return ComputeCapsuleMassProperties(shape.capsule, density);
                default:
                    return new MassProperties(fix.Zero, fix3.zero);
            }
        }

        private static MassProperties ComputeBoxMassProperties(fix3 size, fix density)
        {
            fix width = math.abs(size.x);
            fix height = math.abs(size.y);
            fix depth = math.abs(size.z);
            fix mass = density * width * height * depth;
            return new MassProperties(mass, ComputeBoxInertia(size, mass));
        }

        private static MassProperties ComputeSphereMassProperties(Sphere sphere, fix density)
        {
            fix radius = math.max(fix.Zero, sphere.Radius);
            fix radiusSq = radius * radius;
            fix mass = density * fix._4 * math.PI * radiusSq * radius / 3;
            fix inertia = fix._2 * mass * radiusSq / 5;
            return new MassProperties(mass, new fix3(inertia, inertia, inertia));
        }

        private static MassProperties ComputeCapsuleMassProperties(Capsule capsule, fix density)
        {
            AABB bounds = ComputeBounds(capsule);
            fix radius = math.max(fix.Zero, capsule.Radius);
            fix cylinderLength = math.max(fix.Zero, capsule.Height - radius * 2);
            fix volume = math.PI * radius * radius * cylinderLength
                + fix._4 * math.PI * radius * radius * radius / 3;

            fix mass = density * volume;
            return new MassProperties(mass, ComputeBoxInertia(bounds.size, mass));
        }

        private static fix3 ComputeBoxInertia(fix3 size, fix mass)
        {
            fix width = math.abs(size.x);
            fix height = math.abs(size.y);
            fix depth = math.abs(size.z);
            fix widthSq = width * width;
            fix heightSq = height * height;
            fix depthSq = depth * depth;

            return new fix3(
                mass * (heightSq + depthSq) / 12,
                mass * (widthSq + depthSq) / 12,
                mass * (widthSq + heightSq) / 12);
        }
    }
}
