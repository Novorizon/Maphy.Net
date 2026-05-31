using Maphy.Mathematics;

namespace Maphy.Physics
{
    public enum ConstraintType
    {
        None = 0,
        Custom = 1,
        Distance = 2,
        Point = 3,
        SpringDistance = 4,
        Hinge = 5,
        Fixed = 6,
        Slider = 7,
        BallSocket = 8,
        ConeTwist = 9,
        SixDof = 10,
        Motor = 11,
        Gear = 12,
    }

    [System.Flags]
    public enum ConstraintCapabilities
    {
        None = 0,
        LinearLimit = 1 << 0,
        AngularLimit = 1 << 1,
        LinearMotor = 1 << 2,
        AngularMotor = 1 << 3,
        Spring = 1 << 4,
        Breakable = 1 << 5,
    }

    public readonly struct ConstraintDescriptor
    {
        public readonly ConstraintType type;
        public readonly ConstraintCapabilities capabilities;
        public readonly bool implemented;

        public ConstraintDescriptor(ConstraintType type, ConstraintCapabilities capabilities, bool implemented)
        {
            this.type = type;
            this.capabilities = capabilities;
            this.implemented = implemented;
        }
    }

    public struct JointLimit
    {
        public bool enabled;
        public fix min;
        public fix max;
        public fix stiffness;
        public fix damping;

        public JointLimit(bool enabled, fix min, fix max, fix stiffness, fix damping)
        {
            this.enabled = enabled;
            this.min = min;
            this.max = max;
            this.stiffness = stiffness;
            this.damping = damping;
        }
    }

    public struct JointMotor
    {
        public bool enabled;
        public fix targetVelocity;
        public fix maxImpulse;

        public JointMotor(bool enabled, fix targetVelocity, fix maxImpulse)
        {
            this.enabled = enabled;
            this.targetVelocity = targetVelocity;
            this.maxImpulse = maxImpulse;
        }
    }

    public readonly struct ConstraintFrame
    {
        public readonly fix3 localAnchor;
        public readonly quaternion localRotation;

        public ConstraintFrame(fix3 localAnchor, quaternion localRotation)
        {
            this.localAnchor = localAnchor;
            this.localRotation = localRotation;
        }
    }

    public abstract class Constraint
    {
        public ulong id { get; internal set; }
        public ulong rigidId0 { get; private set; }
        public ulong rigidId1 { get; private set; }
        public bool enabled { get; private set; }
        public virtual ConstraintType Type => ConstraintType.Custom;
        public virtual ConstraintCapabilities Capabilities => ConstraintCapabilities.None;

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
