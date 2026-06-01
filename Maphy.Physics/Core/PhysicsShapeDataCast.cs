using Maphy.Mathematics;

namespace Maphy.Physics
{
    public static partial class Physics
    {
        public static bool TryRaycast(
            PhysicsShapeData shape,
            Ray ray,
            fix maxDistance,
            out RaycastHit hitInfo)
        {
            switch (shape.type)
            {
                case ShapeType.AABB:
                    return RaycastAABB(shape.aabb, ray, maxDistance, out hitInfo);
                case ShapeType.OBB:
                    return RaycastOBB(shape.obb, ray, maxDistance, out hitInfo);
                case ShapeType.Sphere:
                    return RaycastSphere(shape.sphere, ray, maxDistance, out hitInfo);
                case ShapeType.Capsule:
                    return RaycastCapsule(shape.capsule, ray, maxDistance, out hitInfo);
                default:
                    hitInfo = default;
                    return false;
            }
        }

        public static bool TryShapeCast(
            PhysicsShapeData movingShape,
            PhysicsShapeData targetShape,
            fix3 delta,
            out ShapeCastHit hit)
        {
            if (movingShape.type == ShapeType.Sphere)
            {
                switch (targetShape.type)
                {
                    case ShapeType.Sphere:
                        return TrySweepSphereSphere(movingShape.sphere, targetShape.sphere, delta, out hit);
                    case ShapeType.AABB:
                        return TrySweepSphereAABB(movingShape.sphere, targetShape.aabb, delta, out hit);
                    case ShapeType.OBB:
                        return TrySweepSphereOBB(movingShape.sphere, targetShape.obb, delta, out hit);
                    case ShapeType.Capsule:
                        return TrySweepSphereCapsule(movingShape.sphere, targetShape.capsule, delta, out hit);
                }
            }

            if (movingShape.type == ShapeType.Capsule)
            {
                if (targetShape.type == ShapeType.Capsule)
                {
                    return TrySweepCapsuleCapsule(movingShape.capsule, targetShape.capsule, delta, out hit);
                }

                return TrySweepCapsule(movingShape.capsule, targetShape, delta, out hit);
            }

            if (TryGetBoxCastData(movingShape, out BoxCastData movingBox)
                && TryGetBoxCastData(targetShape, out BoxCastData targetBox))
            {
                return TrySweepBoxBox(movingShape, targetShape, movingBox, targetBox, delta, out hit);
            }

            return TryAABBCast(movingShape.ComputeBounds(), targetShape.ComputeBounds(), delta, out hit);
        }

        private static bool TrySweepCapsule(
            Capsule movingCapsule,
            PhysicsShapeData targetShape,
            fix3 delta,
            out ShapeCastHit hit)
        {
            hit = default;
            bool found = false;
            TrySelectEarlierCast(PhysicsShapeData.Sphere(movingCapsule.Center1, movingCapsule.Radius), targetShape, delta, ref found, ref hit);
            TrySelectEarlierCast(PhysicsShapeData.Sphere(movingCapsule.Center2, movingCapsule.Radius), targetShape, delta, ref found, ref hit);
            TrySelectEarlierCast(PhysicsShapeData.Sphere(movingCapsule.Center, movingCapsule.Radius), targetShape, delta, ref found, ref hit);
            return found;
        }

        private static void TrySelectEarlierCast(
            PhysicsShapeData movingShape,
            PhysicsShapeData targetShape,
            fix3 delta,
            ref bool found,
            ref ShapeCastHit bestHit)
        {
            if (TryShapeCast(movingShape, targetShape, delta, out ShapeCastHit candidate))
            {
                SelectEarlier(candidate, ref found, ref bestHit);
            }
        }

        private static bool TrySweepBoxBox(
            PhysicsShapeData movingShape,
            PhysicsShapeData targetShape,
            BoxCastData movingBox,
            BoxCastData targetBox,
            fix3 delta,
            out ShapeCastHit hit)
        {
            if (TryComputeContact(movingShape, targetShape, NarrowPhaseAlgorithm.SAT, out CollisionInfo collision))
            {
                fix3 normal = math.lengthsq(collision.normal) > math.Epsilon
                    ? collision.normal
                    : NormalizeCastOrDefault(targetBox.center - movingBox.center, fix3.right);
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

        private static bool TryGetBoxCastData(PhysicsShapeData shape, out BoxCastData box)
        {
            switch (shape.type)
            {
                case ShapeType.AABB:
                    box = new BoxCastData(shape.aabb.center, shape.aabb.extents, fix3.right, fix3.up, fix3.forward);
                    return true;
                case ShapeType.OBB:
                    box = new BoxCastData(
                        shape.obb.center,
                        shape.obb.extents,
                        shape.obb.orientation * fix3.right,
                        shape.obb.orientation * fix3.up,
                        shape.obb.orientation * fix3.forward);
                    return true;
                default:
                    box = default;
                    return false;
            }
        }
    }
}
