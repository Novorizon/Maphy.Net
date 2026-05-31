using Maphy.Mathematics;

namespace Maphy.Physics
{
    internal static class ConstraintUtility
    {
        public static bool TryGetAnchorData(
            SolverContext context,
            ulong rigidId0,
            ulong rigidId1,
            fix3 localAnchor0,
            fix3 localAnchor1,
            out ConstraintAnchorData data)
        {
            data = default;
            if (!context.TryGetBody(rigidId0, out Rigid rigid0, out Entity entity0, out fix inverseMass0)
                || !context.TryGetBody(rigidId1, out Rigid rigid1, out Entity entity1, out fix inverseMass1)
                || inverseMass0 + inverseMass1 <= fix.Zero)
            {
                return false;
            }

            fix3 relativeAnchor0 = entity0.orientation * localAnchor0;
            fix3 relativeAnchor1 = entity1.orientation * localAnchor1;
            data = new ConstraintAnchorData(
                rigid0,
                rigid1,
                entity0,
                entity1,
                inverseMass0,
                inverseMass1,
                relativeAnchor0,
                relativeAnchor1,
                entity0.translation + relativeAnchor0,
                entity1.translation + relativeAnchor1);
            return true;
        }

        public static void WarmStartPoint(SolverContext context, ConstraintAnchorData data, fix3 accumulatedImpulse)
        {
            context.ApplyImpulse(
                true,
                data.rigid0,
                data.inverseMass0,
                data.relativeAnchor0,
                true,
                data.rigid1,
                data.inverseMass1,
                data.relativeAnchor1,
                accumulatedImpulse * context.warmStartScale);
        }

        public static void SolvePointVelocity(SolverContext context, ConstraintAnchorData data, ref fix3 accumulatedImpulse)
        {
            SolvePointVelocityAxis(context, data, fix3.right, ref accumulatedImpulse);
            SolvePointVelocityAxis(context, data, fix3.up, ref accumulatedImpulse);
            SolvePointVelocityAxis(context, data, fix3.forward, ref accumulatedImpulse);
        }

        public static void SolvePointPosition(SolverContext context, ConstraintAnchorData data)
        {
            fix3 delta = data.worldAnchor1 - data.worldAnchor0;
            if (math.lengthsq(delta) <= math.Epsilon)
            {
                return;
            }

            fix inverseMassSum = data.inverseMass0 + data.inverseMass1;
            if (inverseMassSum <= fix.Zero)
            {
                return;
            }

            fix3 correction = delta / inverseMassSum;
            if (data.inverseMass0 > fix.Zero)
            {
                context.TranslateEntity(data.rigid0.id, correction * data.inverseMass0);
            }

            if (data.inverseMass1 > fix.Zero)
            {
                context.TranslateEntity(data.rigid1.id, -correction * data.inverseMass1);
            }
        }

        public static fix3 NormalizeOrDefault(fix3 value, fix3 fallback)
        {
            fix lengthSq = math.lengthsq(value);
            return lengthSq > math.Epsilon ? value / math.sqrt(lengthSq) : fallback;
        }

        private static void SolvePointVelocityAxis(SolverContext context, ConstraintAnchorData data, fix3 axis, ref fix3 accumulatedImpulse)
        {
            fix3 velocity0 = SolverContext.GetVelocityAtPoint(data.rigid0, data.relativeAnchor0);
            fix3 velocity1 = SolverContext.GetVelocityAtPoint(data.rigid1, data.relativeAnchor1);
            fix constraintVelocity = math.dot(velocity1 - velocity0, axis);
            fix effectiveMass = data.inverseMass0 + data.inverseMass1
                + SolverContext.GetAngularEffectiveMass(data.rigid0, data.relativeAnchor0, axis)
                + SolverContext.GetAngularEffectiveMass(data.rigid1, data.relativeAnchor1, axis);
            if (effectiveMass <= fix.Zero)
            {
                return;
            }

            fix impulseDelta = -constraintVelocity / effectiveMass;
            fix3 impulse = axis * impulseDelta;
            accumulatedImpulse += impulse;
            context.ApplyImpulse(
                true,
                data.rigid0,
                data.inverseMass0,
                data.relativeAnchor0,
                true,
                data.rigid1,
                data.inverseMass1,
                data.relativeAnchor1,
                impulse);
        }
    }

    internal readonly struct ConstraintAnchorData
    {
        public readonly Rigid rigid0;
        public readonly Rigid rigid1;
        public readonly Entity entity0;
        public readonly Entity entity1;
        public readonly fix inverseMass0;
        public readonly fix inverseMass1;
        public readonly fix3 relativeAnchor0;
        public readonly fix3 relativeAnchor1;
        public readonly fix3 worldAnchor0;
        public readonly fix3 worldAnchor1;

        public ConstraintAnchorData(
            Rigid rigid0,
            Rigid rigid1,
            Entity entity0,
            Entity entity1,
            fix inverseMass0,
            fix inverseMass1,
            fix3 relativeAnchor0,
            fix3 relativeAnchor1,
            fix3 worldAnchor0,
            fix3 worldAnchor1)
        {
            this.rigid0 = rigid0;
            this.rigid1 = rigid1;
            this.entity0 = entity0;
            this.entity1 = entity1;
            this.inverseMass0 = inverseMass0;
            this.inverseMass1 = inverseMass1;
            this.relativeAnchor0 = relativeAnchor0;
            this.relativeAnchor1 = relativeAnchor1;
            this.worldAnchor0 = worldAnchor0;
            this.worldAnchor1 = worldAnchor1;
        }
    }
}
