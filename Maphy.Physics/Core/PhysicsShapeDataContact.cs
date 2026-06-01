namespace Maphy.Physics
{
    public static partial class Physics
    {
        public static bool TryComputeContact(
            PhysicsShapeData shape0,
            PhysicsShapeData shape1,
            out CollisionInfo collision)
        {
            return TryComputeContact(shape0, shape1, NarrowPhaseAlgorithm.Auto, out collision);
        }

        public static bool TryComputeContact(
            PhysicsShapeData shape0,
            PhysicsShapeData shape1,
            NarrowPhaseAlgorithm algorithm,
            out CollisionInfo collision)
        {
            // The no-GC core currently uses the SAT/contact implementation for every
            // algorithm mode. A no-GC GJK/EPA path can slot in here without changing
            // PhysicsWorld storage or solver code.
            return TryComputePhysicsShapeSATContact(shape0, shape1, out collision);
        }

        internal static void ReserveContactScratch()
        {
            if (boxClipBuffer0 == null)
            {
                boxClipBuffer0 = new Maphy.Mathematics.fix3[8];
            }

            if (boxClipBuffer1 == null)
            {
                boxClipBuffer1 = new Maphy.Mathematics.fix3[8];
            }
        }

        private static bool TryComputePhysicsShapeSATContact(
            PhysicsShapeData shape0,
            PhysicsShapeData shape1,
            out CollisionInfo collision)
        {
            collision = default;
            switch (shape0.type, shape1.type)
            {
                case (ShapeType.AABB, ShapeType.AABB):
                    return TryComputeBoxBoxContact(BoxData.FromAABB(shape0.aabb), BoxData.FromAABB(shape1.aabb), out collision);
                case (ShapeType.AABB, ShapeType.OBB):
                    return TryComputeBoxBoxContact(BoxData.FromAABB(shape0.aabb), BoxData.FromOBB(shape1.obb), out collision);
                case (ShapeType.OBB, ShapeType.AABB):
                    return TryComputeBoxBoxContact(BoxData.FromOBB(shape0.obb), BoxData.FromAABB(shape1.aabb), out collision);
                case (ShapeType.OBB, ShapeType.OBB):
                    return TryComputeBoxBoxContact(BoxData.FromOBB(shape0.obb), BoxData.FromOBB(shape1.obb), out collision);
                case (ShapeType.Sphere, ShapeType.AABB):
                    return TryComputeSphereAABBContact(shape0.sphere, shape1.aabb, out collision);
                case (ShapeType.AABB, ShapeType.Sphere):
                    if (!TryComputeSphereAABBContact(shape1.sphere, shape0.aabb, out collision))
                    {
                        return false;
                    }

                    collision = collision.Flipped();
                    return true;
                case (ShapeType.Sphere, ShapeType.OBB):
                    return TryComputeSphereOBBContact(shape0.sphere, shape1.obb, out collision);
                case (ShapeType.OBB, ShapeType.Sphere):
                    if (!TryComputeSphereOBBContact(shape1.sphere, shape0.obb, out collision))
                    {
                        return false;
                    }

                    collision = collision.Flipped();
                    return true;
                case (ShapeType.Sphere, ShapeType.Sphere):
                    return TryComputeSphereSphereContact(shape0.sphere, shape1.sphere, out collision);
                case (ShapeType.Sphere, ShapeType.Capsule):
                    return TryComputeSphereCapsuleContact(shape0.sphere, shape1.capsule, out collision);
                case (ShapeType.Capsule, ShapeType.Sphere):
                    if (!TryComputeSphereCapsuleContact(shape1.sphere, shape0.capsule, out collision))
                    {
                        return false;
                    }

                    collision = collision.Flipped();
                    return true;
                case (ShapeType.AABB, ShapeType.Capsule):
                    return TryComputeCapsuleBoxContact(shape1.capsule, BoxData.FromAABB(shape0.aabb), false, out collision);
                case (ShapeType.Capsule, ShapeType.AABB):
                    return TryComputeCapsuleBoxContact(shape0.capsule, BoxData.FromAABB(shape1.aabb), true, out collision);
                case (ShapeType.OBB, ShapeType.Capsule):
                    return TryComputeCapsuleBoxContact(shape1.capsule, BoxData.FromOBB(shape0.obb), false, out collision);
                case (ShapeType.Capsule, ShapeType.OBB):
                    return TryComputeCapsuleBoxContact(shape0.capsule, BoxData.FromOBB(shape1.obb), true, out collision);
                case (ShapeType.Capsule, ShapeType.Capsule):
                    return TryComputeCapsuleCapsuleContact(shape0.capsule, shape1.capsule, out collision);
                default:
                    return false;
            }
        }
    }
}
