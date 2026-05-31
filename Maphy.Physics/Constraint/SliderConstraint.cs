using Maphy.Mathematics;

namespace Maphy.Physics
{
    public sealed class SliderConstraint : Constraint
    {
        public override ConstraintType Type => ConstraintType.Slider;
        public override ConstraintCapabilities Capabilities => ConstraintCapabilities.LinearLimit | ConstraintCapabilities.AngularLimit;

        public fix3 localAnchor0 { get; private set; }
        public fix3 localAnchor1 { get; private set; }
        public fix3 localAxis0 { get; private set; }
        public fix3 localAxis1 { get; private set; }
        public fix linearStiffness { get; private set; }
        public fix linearDamping { get; private set; }
        public fix angularStiffness { get; private set; }
        public fix angularDamping { get; private set; }

        private fix3 accumulatedLinearImpulse;
        private fix3 accumulatedAngularImpulse;

        public SliderConstraint(
            ulong rigidId0,
            ulong rigidId1,
            fix3 localAnchor0,
            fix3 localAnchor1,
            fix3 localAxis0,
            fix3 localAxis1)
            : base(rigidId0, rigidId1)
        {
            this.localAnchor0 = localAnchor0;
            this.localAnchor1 = localAnchor1;
            this.localAxis0 = ConstraintUtility.NormalizeOrDefault(localAxis0, fix3.right);
            this.localAxis1 = ConstraintUtility.NormalizeOrDefault(localAxis1, fix3.right);
            linearStiffness = fix.One;
            linearDamping = fix.One;
            angularStiffness = fix.One;
            angularDamping = fix.One;
            accumulatedLinearImpulse = fix3.zero;
            accumulatedAngularImpulse = fix3.zero;
        }

        public void SetLocalAnchors(fix3 localAnchor0, fix3 localAnchor1)
        {
            this.localAnchor0 = localAnchor0;
            this.localAnchor1 = localAnchor1;
            ClearWarmStart();
        }

        public void SetLocalAxes(fix3 localAxis0, fix3 localAxis1)
        {
            this.localAxis0 = ConstraintUtility.NormalizeOrDefault(localAxis0, fix3.right);
            this.localAxis1 = ConstraintUtility.NormalizeOrDefault(localAxis1, fix3.right);
            ClearWarmStart();
        }

        public void SetLinearResponse(fix stiffness, fix damping)
        {
            linearStiffness = math.max(fix.Zero, stiffness);
            linearDamping = math.max(fix.Zero, damping);
            ClearWarmStart();
        }

        public void SetAngularResponse(fix stiffness, fix damping)
        {
            angularStiffness = math.max(fix.Zero, stiffness);
            angularDamping = math.max(fix.Zero, damping);
            ClearWarmStart();
        }

        internal override void ClearWarmStart()
        {
            base.ClearWarmStart();
            accumulatedLinearImpulse = fix3.zero;
            accumulatedAngularImpulse = fix3.zero;
        }

        internal override void WarmStart(SolverContext context)
        {
            if (!TryGetData(context, out ConstraintAnchorData data))
            {
                ClearWarmStart();
                return;
            }

            if (accumulatedLinearImpulse != fix3.zero)
            {
                ConstraintUtility.WarmStartPoint(context, data, accumulatedLinearImpulse);
            }

            if (accumulatedAngularImpulse != fix3.zero)
            {
                context.ApplyAngularImpulse(true, data.rigid0, true, data.rigid1, accumulatedAngularImpulse * context.warmStartScale);
            }
        }

        internal override void SolveVelocity(SolverContext context)
        {
            if (!TryGetData(context, out ConstraintAnchorData data))
            {
                ClearWarmStart();
                return;
            }

            fix3 sliderAxis = GetSliderAxis(data);
            GetPerpendicularAxes(sliderAxis, out fix3 tangent0, out fix3 tangent1);
            SolveLinearAxis(context, data, tangent0, ref accumulatedLinearImpulse);
            SolveLinearAxis(context, data, tangent1, ref accumulatedLinearImpulse);
            SolveAngularVelocity(context, data, sliderAxis);
        }

        internal override void SolvePosition(SolverContext context)
        {
            if (!TryGetData(context, out ConstraintAnchorData data))
            {
                ClearWarmStart();
                return;
            }

            fix3 sliderAxis = GetSliderAxis(data);
            SolveLinearPosition(context, data, sliderAxis);
            SolveAngularPosition(context, data);
        }

        private bool TryGetData(SolverContext context, out ConstraintAnchorData data)
        {
            data = default;
            return enabled && ConstraintUtility.TryGetAnchorData(context, rigidId0, rigidId1, localAnchor0, localAnchor1, out data);
        }

        private fix3 GetSliderAxis(ConstraintAnchorData data)
        {
            fix3 axis0 = ConstraintUtility.NormalizeOrDefault(data.entity0.orientation * localAxis0, fix3.right);
            return axis0;
        }

        private void SolveLinearAxis(SolverContext context, ConstraintAnchorData data, fix3 axis, ref fix3 accumulatedImpulse)
        {
            fix3 velocity0 = SolverContext.GetVelocityAtPoint(data.rigid0, data.relativeAnchor0);
            fix3 velocity1 = SolverContext.GetVelocityAtPoint(data.rigid1, data.relativeAnchor1);
            fix constraintVelocity = math.dot(velocity1 - velocity0, axis) * linearDamping;
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

        private void SolveLinearPosition(SolverContext context, ConstraintAnchorData data, fix3 sliderAxis)
        {
            fix3 delta = data.worldAnchor1 - data.worldAnchor0;
            fix3 error = (delta - sliderAxis * math.dot(delta, sliderAxis)) * math.min(fix.One, linearStiffness);
            if (math.lengthsq(error) <= math.Epsilon)
            {
                return;
            }

            fix inverseMassSum = data.inverseMass0 + data.inverseMass1;
            if (inverseMassSum <= fix.Zero)
            {
                return;
            }

            fix3 correction = error / inverseMassSum;
            if (data.inverseMass0 > fix.Zero)
            {
                context.TranslateEntity(data.rigid0.id, correction * data.inverseMass0);
            }

            if (data.inverseMass1 > fix.Zero)
            {
                context.TranslateEntity(data.rigid1.id, -correction * data.inverseMass1);
            }
        }

        private void SolveAngularVelocity(SolverContext context, ConstraintAnchorData data, fix3 sliderAxis)
        {
            fix3 axis0 = ConstraintUtility.NormalizeOrDefault(data.entity0.orientation * localAxis0, fix3.right);
            fix3 axis1 = ConstraintUtility.NormalizeOrDefault(data.entity1.orientation * localAxis1, axis0);
            fix3 axisError = math.cross(axis0, axis1) * angularStiffness;
            fix3 relativeAngularVelocity = (data.rigid1.angularVelocity - data.rigid0.angularVelocity) * angularDamping;
            relativeAngularVelocity -= sliderAxis * math.dot(relativeAngularVelocity, sliderAxis);
            fix3 correctionVelocity = relativeAngularVelocity + axisError;

            SolveAngularAxis(context, data, fix3.right, correctionVelocity);
            SolveAngularAxis(context, data, fix3.up, correctionVelocity);
            SolveAngularAxis(context, data, fix3.forward, correctionVelocity);
        }

        private void SolveAngularAxis(SolverContext context, ConstraintAnchorData data, fix3 axis, fix3 correctionVelocity)
        {
            fix component = math.dot(correctionVelocity, axis);
            if (math.abs(component) <= math.Epsilon)
            {
                return;
            }

            fix effectiveMass = SolverContext.GetAngularOnlyEffectiveMass(data.rigid0, data.rigid1, axis);
            if (effectiveMass <= fix.Zero)
            {
                return;
            }

            fix impulseMagnitude = -component / effectiveMass;
            fix3 impulse = axis * impulseMagnitude;
            accumulatedAngularImpulse += impulse;
            context.ApplyAngularImpulse(true, data.rigid0, true, data.rigid1, impulse);
        }

        private void SolveAngularPosition(SolverContext context, ConstraintAnchorData data)
        {
            fix3 axis0 = ConstraintUtility.NormalizeOrDefault(data.entity0.orientation * localAxis0, fix3.right);
            fix3 axis1 = ConstraintUtility.NormalizeOrDefault(data.entity1.orientation * localAxis1, axis0);
            fix3 error = math.cross(axis0, axis1) * math.min(fix.One, angularStiffness);
            if (math.lengthsq(error) <= math.Epsilon)
            {
                return;
            }

            fix inverseMassSum = data.inverseMass0 + data.inverseMass1;
            if (inverseMassSum <= fix.Zero)
            {
                return;
            }

            if (data.inverseMass0 > fix.Zero)
            {
                context.RotateEntity(data.rigid0.id, error * (data.inverseMass0 / inverseMassSum));
            }

            if (data.inverseMass1 > fix.Zero)
            {
                context.RotateEntity(data.rigid1.id, -error * (data.inverseMass1 / inverseMassSum));
            }
        }

        private static void GetPerpendicularAxes(fix3 axis, out fix3 tangent0, out fix3 tangent1)
        {
            fix3 reference = math.abs(axis.x) < math.abs(axis.y) ? fix3.right : fix3.up;
            tangent0 = ConstraintUtility.NormalizeOrDefault(math.cross(axis, reference), fix3.forward);
            tangent1 = ConstraintUtility.NormalizeOrDefault(math.cross(axis, tangent0), fix3.up);
        }
    }
}
