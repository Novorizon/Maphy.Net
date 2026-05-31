using Maphy.Mathematics;

namespace Maphy.Physics
{
    public sealed class FixedConstraint : Constraint
    {
        public override ConstraintType Type => ConstraintType.Fixed;
        public override ConstraintCapabilities Capabilities => ConstraintCapabilities.LinearLimit | ConstraintCapabilities.AngularLimit;

        public fix3 localAnchor0 { get; private set; }
        public fix3 localAnchor1 { get; private set; }
        public quaternion targetLocalRotation { get; private set; }
        public fix angularStiffness { get; private set; }
        public fix angularDamping { get; private set; }

        private fix3 accumulatedLinearImpulse;
        private fix3 accumulatedAngularImpulse;

        public FixedConstraint(
            ulong rigidId0,
            ulong rigidId1,
            fix3 localAnchor0,
            fix3 localAnchor1,
            quaternion targetLocalRotation)
            : base(rigidId0, rigidId1)
        {
            this.localAnchor0 = localAnchor0;
            this.localAnchor1 = localAnchor1;
            this.targetLocalRotation = quaternion.normalize(targetLocalRotation);
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

        public void SetTargetLocalRotation(quaternion targetLocalRotation)
        {
            this.targetLocalRotation = quaternion.normalize(targetLocalRotation);
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

            ConstraintUtility.SolvePointVelocity(context, data, ref accumulatedLinearImpulse);
            SolveAngularVelocity(context, data);
        }

        internal override void SolvePosition(SolverContext context)
        {
            if (!TryGetData(context, out ConstraintAnchorData data))
            {
                ClearWarmStart();
                return;
            }

            ConstraintUtility.SolvePointPosition(context, data);
            SolveAngularPosition(context, data);
        }

        private bool TryGetData(SolverContext context, out ConstraintAnchorData data)
        {
            data = default;
            return enabled && ConstraintUtility.TryGetAnchorData(context, rigidId0, rigidId1, localAnchor0, localAnchor1, out data);
        }

        private void SolveAngularVelocity(SolverContext context, ConstraintAnchorData data)
        {
            quaternion desiredOrientation1 = quaternion.normalize(data.entity0.orientation * targetLocalRotation);
            fix3 angularError = ComputeAngularError(desiredOrientation1, data.entity1.orientation) * angularStiffness;
            fix3 relativeAngularVelocity = (data.rigid1.angularVelocity - data.rigid0.angularVelocity) * angularDamping;
            fix3 correctionVelocity = relativeAngularVelocity + angularError;

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
            quaternion desiredOrientation1 = quaternion.normalize(data.entity0.orientation * targetLocalRotation);
            fix3 error = ComputeAngularError(desiredOrientation1, data.entity1.orientation) * math.min(fix.One, angularStiffness);
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

        private static fix3 ComputeAngularError(quaternion desired, quaternion current)
        {
            fix3 desiredX = desired * fix3.right;
            fix3 desiredY = desired * fix3.up;
            fix3 desiredZ = desired * fix3.forward;
            fix3 currentX = current * fix3.right;
            fix3 currentY = current * fix3.up;
            fix3 currentZ = current * fix3.forward;
            return (math.cross(desiredX, currentX)
                + math.cross(desiredY, currentY)
                + math.cross(desiredZ, currentZ)) * fix._0_5;
        }
    }
}
