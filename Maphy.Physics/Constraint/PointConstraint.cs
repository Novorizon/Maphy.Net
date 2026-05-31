using Maphy.Mathematics;

namespace Maphy.Physics
{
    public sealed class PointConstraint : Constraint
    {
        public override ConstraintType Type => ConstraintType.Point;
        public override ConstraintCapabilities Capabilities => ConstraintCapabilities.LinearLimit;

        public fix3 localAnchor0 { get; private set; }
        public fix3 localAnchor1 { get; private set; }

        private fix3 accumulatedLinearImpulse;

        public PointConstraint(ulong rigidId0, ulong rigidId1, fix3 localAnchor0, fix3 localAnchor1)
            : base(rigidId0, rigidId1)
        {
            this.localAnchor0 = localAnchor0;
            this.localAnchor1 = localAnchor1;
            accumulatedLinearImpulse = fix3.zero;
        }

        public void SetLocalAnchors(fix3 localAnchor0, fix3 localAnchor1)
        {
            this.localAnchor0 = localAnchor0;
            this.localAnchor1 = localAnchor1;
            ClearWarmStart();
        }

        internal override void ClearWarmStart()
        {
            base.ClearWarmStart();
            accumulatedLinearImpulse = fix3.zero;
        }

        internal override void WarmStart(SolverContext context)
        {
            if (accumulatedLinearImpulse == fix3.zero)
            {
                return;
            }

            if (!TryGetData(context, out ConstraintAnchorData data))
            {
                ClearWarmStart();
                return;
            }

            ConstraintUtility.WarmStartPoint(context, data, accumulatedLinearImpulse);
        }

        internal override void SolveVelocity(SolverContext context)
        {
            if (!TryGetData(context, out ConstraintAnchorData data))
            {
                ClearWarmStart();
                return;
            }

            ConstraintUtility.SolvePointVelocity(context, data, ref accumulatedLinearImpulse);
        }

        internal override void SolvePosition(SolverContext context)
        {
            if (!TryGetData(context, out ConstraintAnchorData data))
            {
                ClearWarmStart();
                return;
            }

            ConstraintUtility.SolvePointPosition(context, data);
        }

        private bool TryGetData(SolverContext context, out ConstraintAnchorData data)
        {
            data = default;
            return enabled && ConstraintUtility.TryGetAnchorData(context, rigidId0, rigidId1, localAnchor0, localAnchor1, out data);
        }
    }
}
