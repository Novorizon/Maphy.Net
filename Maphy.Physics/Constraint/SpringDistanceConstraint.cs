using Maphy.Mathematics;

namespace Maphy.Physics
{
    public sealed class SpringDistanceConstraint : Constraint
    {
        public override ConstraintType Type => ConstraintType.SpringDistance;
        public override ConstraintCapabilities Capabilities => ConstraintCapabilities.LinearLimit | ConstraintCapabilities.Spring;

        public fix3 localAnchor0 { get; private set; }
        public fix3 localAnchor1 { get; private set; }
        public fix restDistance { get; private set; }
        public fix stiffness { get; private set; }
        public fix damping { get; private set; }

        private fix accumulatedSpringImpulse;

        public SpringDistanceConstraint(
            ulong rigidId0,
            ulong rigidId1,
            fix3 localAnchor0,
            fix3 localAnchor1,
            fix restDistance,
            fix stiffness,
            fix damping)
            : base(rigidId0, rigidId1)
        {
            this.localAnchor0 = localAnchor0;
            this.localAnchor1 = localAnchor1;
            this.restDistance = math.max(fix.Zero, restDistance);
            this.stiffness = math.max(fix.Zero, stiffness);
            this.damping = math.max(fix.Zero, damping);
            accumulatedSpringImpulse = fix.Zero;
        }

        public void SetLocalAnchors(fix3 localAnchor0, fix3 localAnchor1)
        {
            this.localAnchor0 = localAnchor0;
            this.localAnchor1 = localAnchor1;
            ClearWarmStart();
        }

        public void SetSpring(fix restDistance, fix stiffness, fix damping)
        {
            this.restDistance = math.max(fix.Zero, restDistance);
            this.stiffness = math.max(fix.Zero, stiffness);
            this.damping = math.max(fix.Zero, damping);
            ClearWarmStart();
        }

        internal override void ClearWarmStart()
        {
            base.ClearWarmStart();
            accumulatedSpringImpulse = fix.Zero;
        }

        internal override void WarmStart(SolverContext context)
        {
            if (accumulatedSpringImpulse == fix.Zero || !TryGetData(context, out SpringData data))
            {
                return;
            }

            ApplyImpulse(context, data, accumulatedSpringImpulse * context.warmStartScale);
        }

        internal override void SolveVelocity(SolverContext context)
        {
            if (!TryGetData(context, out SpringData data))
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
            fix relativeVelocity = math.dot(velocity1 - velocity0, data.axis);
            fix displacement = data.currentDistance - restDistance;
            fix targetVelocity = -displacement * stiffness;
            fix impulseDelta = (targetVelocity - relativeVelocity * damping) / effectiveMass;

            accumulatedSpringImpulse += impulseDelta;
            ApplyImpulse(context, data, impulseDelta);
        }

        internal override void SolvePosition(SolverContext context)
        {
            if (stiffness <= fix.Zero || !TryGetData(context, out SpringData data))
            {
                return;
            }

            fix error = data.currentDistance - restDistance;
            if (math.abs(error) <= math.Epsilon)
            {
                return;
            }

            fix inverseMassSum = data.inverseMass0 + data.inverseMass1;
            if (inverseMassSum <= fix.Zero)
            {
                return;
            }

            fix positionPercent = math.min(fix.One, stiffness);
            fix3 correction = data.axis * (error * positionPercent / inverseMassSum);
            if (data.inverseMass0 > fix.Zero)
            {
                context.TranslateEntity(data.rigid0.id, correction * data.inverseMass0);
            }

            if (data.inverseMass1 > fix.Zero)
            {
                context.TranslateEntity(data.rigid1.id, -correction * data.inverseMass1);
            }
        }

        private bool TryGetData(SolverContext context, out SpringData data)
        {
            data = default;
            if (!enabled || !ConstraintUtility.TryGetAnchorData(context, rigidId0, rigidId1, localAnchor0, localAnchor1, out ConstraintAnchorData anchorData))
            {
                return false;
            }

            fix3 delta = anchorData.worldAnchor1 - anchorData.worldAnchor0;
            fix distanceSq = math.lengthsq(delta);
            if (distanceSq <= math.Epsilon)
            {
                return false;
            }

            fix currentDistance = math.sqrt(distanceSq);
            data = new SpringData(anchorData, delta / currentDistance, currentDistance);
            return true;
        }

        private static void ApplyImpulse(SolverContext context, SpringData data, fix impulseMagnitude)
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
                data.axis * impulseMagnitude);
        }

        private readonly struct SpringData
        {
            public readonly Rigid rigid0;
            public readonly Rigid rigid1;
            public readonly fix inverseMass0;
            public readonly fix inverseMass1;
            public readonly fix3 relativeAnchor0;
            public readonly fix3 relativeAnchor1;
            public readonly fix3 axis;
            public readonly fix currentDistance;

            public SpringData(ConstraintAnchorData anchorData, fix3 axis, fix currentDistance)
            {
                rigid0 = anchorData.rigid0;
                rigid1 = anchorData.rigid1;
                inverseMass0 = anchorData.inverseMass0;
                inverseMass1 = anchorData.inverseMass1;
                relativeAnchor0 = anchorData.relativeAnchor0;
                relativeAnchor1 = anchorData.relativeAnchor1;
                this.axis = axis;
                this.currentDistance = currentDistance;
            }
        }
    }
}
