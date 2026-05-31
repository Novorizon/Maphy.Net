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
    }
}
