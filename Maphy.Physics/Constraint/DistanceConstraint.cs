using Maphy.Mathematics;

namespace Maphy.Physics
{
    public sealed class DistanceConstraint : Constraint
    {
        public fix3 localAnchor0 { get; private set; }
        public fix3 localAnchor1 { get; private set; }
        public fix distance { get; private set; }

        public DistanceConstraint(ulong rigidId0, ulong rigidId1, fix3 localAnchor0, fix3 localAnchor1, fix distance)
            : base(rigidId0, rigidId1)
        {
            this.localAnchor0 = localAnchor0;
            this.localAnchor1 = localAnchor1;
            this.distance = math.max(fix.Zero, distance);
        }

        public void SetLocalAnchors(fix3 localAnchor0, fix3 localAnchor1)
        {
            this.localAnchor0 = localAnchor0;
            this.localAnchor1 = localAnchor1;
            ClearWarmStart();
        }

        public void SetDistance(fix distance)
        {
            this.distance = math.max(fix.Zero, distance);
            ClearWarmStart();
        }

        internal override void WarmStart(SolverContext context)
        {
            if (accumulatedImpulse == fix.Zero)
            {
                return;
            }

            if (!TryGetSolveData(context, out SolveData data))
            {
                ClearWarmStart();
                return;
            }

            context.ApplyImpulse(
                true,
                data.rigid0,
                data.inverseMass0,
                data.relativeAnchor0,
                true,
                data.rigid1,
                data.inverseMass1,
                data.relativeAnchor1,
                data.axis * accumulatedImpulse);
        }

        internal override void SolveVelocity(SolverContext context)
        {
            if (!TryGetSolveData(context, out SolveData data))
            {
                ClearWarmStart();
                return;
            }

            fix effectiveMass = data.inverseMass0 + data.inverseMass1
                + SolverContext.GetAngularEffectiveMass(data.rigid0, data.relativeAnchor0, data.axis)
                + SolverContext.GetAngularEffectiveMass(data.rigid1, data.relativeAnchor1, data.axis);
            if (effectiveMass <= fix.Zero)
            {
                ClearWarmStart();
                return;
            }

            fix3 velocity0 = SolverContext.GetVelocityAtPoint(data.rigid0, data.relativeAnchor0);
            fix3 velocity1 = SolverContext.GetVelocityAtPoint(data.rigid1, data.relativeAnchor1);
            fix constraintVelocity = math.dot(velocity1 - velocity0, data.axis);
            fix impulseDelta = -constraintVelocity / effectiveMass;
            accumulatedImpulse += impulseDelta;

            context.ApplyImpulse(
                true,
                data.rigid0,
                data.inverseMass0,
                data.relativeAnchor0,
                true,
                data.rigid1,
                data.inverseMass1,
                data.relativeAnchor1,
                data.axis * impulseDelta);
        }

        internal override void SolvePosition(SolverContext context)
        {
            if (!TryGetSolveData(context, out SolveData data))
            {
                ClearWarmStart();
                return;
            }

            fix error = data.currentDistance - distance;
            if (math.abs(error) <= math.Epsilon)
            {
                return;
            }

            fix inverseMassSum = data.inverseMass0 + data.inverseMass1;
            if (inverseMassSum <= fix.Zero)
            {
                ClearWarmStart();
                return;
            }

            fix3 correction = data.axis * (error / inverseMassSum);
            if (data.inverseMass0 > fix.Zero)
            {
                context.TranslateEntity(data.rigid0.id, correction * data.inverseMass0);
            }

            if (data.inverseMass1 > fix.Zero)
            {
                context.TranslateEntity(data.rigid1.id, -correction * data.inverseMass1);
            }
        }

        private bool TryGetSolveData(SolverContext context, out SolveData data)
        {
            data = default;
            if (!enabled
                || !context.TryGetBody(rigidId0, out Rigid rigid0, out Entity entity0, out fix inverseMass0)
                || !context.TryGetBody(rigidId1, out Rigid rigid1, out Entity entity1, out fix inverseMass1)
                || inverseMass0 + inverseMass1 <= fix.Zero)
            {
                return false;
            }

            fix3 relativeAnchor0 = entity0.orientation * localAnchor0;
            fix3 relativeAnchor1 = entity1.orientation * localAnchor1;
            fix3 worldAnchor0 = entity0.translation + relativeAnchor0;
            fix3 worldAnchor1 = entity1.translation + relativeAnchor1;
            fix3 delta = worldAnchor1 - worldAnchor0;
            fix distanceSq = math.lengthsq(delta);
            if (distanceSq <= math.Epsilon)
            {
                return false;
            }

            fix currentDistance = math.sqrt(distanceSq);
            data = new SolveData(
                rigid0,
                rigid1,
                inverseMass0,
                inverseMass1,
                relativeAnchor0,
                relativeAnchor1,
                delta / currentDistance,
                currentDistance);
            return true;
        }

        private readonly struct SolveData
        {
            public readonly Rigid rigid0;
            public readonly Rigid rigid1;
            public readonly fix inverseMass0;
            public readonly fix inverseMass1;
            public readonly fix3 relativeAnchor0;
            public readonly fix3 relativeAnchor1;
            public readonly fix3 axis;
            public readonly fix currentDistance;

            public SolveData(
                Rigid rigid0,
                Rigid rigid1,
                fix inverseMass0,
                fix inverseMass1,
                fix3 relativeAnchor0,
                fix3 relativeAnchor1,
                fix3 axis,
                fix currentDistance)
            {
                this.rigid0 = rigid0;
                this.rigid1 = rigid1;
                this.inverseMass0 = inverseMass0;
                this.inverseMass1 = inverseMass1;
                this.relativeAnchor0 = relativeAnchor0;
                this.relativeAnchor1 = relativeAnchor1;
                this.axis = axis;
                this.currentDistance = currentDistance;
            }
        }
    }
}
