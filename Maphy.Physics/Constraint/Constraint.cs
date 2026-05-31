using Maphy.Mathematics;

namespace Maphy.Physics
{
    public abstract class Constraint
    {
        public ulong id { get; internal set; }
        public ulong rigidId0 { get; private set; }
        public ulong rigidId1 { get; private set; }
        public bool enabled { get; private set; }

        internal fix accumulatedImpulse;

        protected Constraint(ulong rigidId0, ulong rigidId1)
        {
            this.rigidId0 = rigidId0;
            this.rigidId1 = rigidId1;
            enabled = true;
            accumulatedImpulse = fix.Zero;
        }

        public void SetEnabled(bool enabled)
        {
            this.enabled = enabled;
            if (!enabled)
            {
                ClearWarmStart();
            }
        }

        internal bool ContainsRigid(ulong rigidId)
        {
            return rigidId0 == rigidId || rigidId1 == rigidId;
        }

        internal virtual void ClearWarmStart()
        {
            accumulatedImpulse = fix.Zero;
        }

        internal virtual void WarmStart(SolverContext context)
        {
        }

        internal virtual void SolveVelocity(SolverContext context)
        {
        }

        internal virtual void SolvePosition(SolverContext context)
        {
        }
    }
}
