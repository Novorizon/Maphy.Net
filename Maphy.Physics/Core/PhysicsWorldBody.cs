using Maphy.Mathematics;

namespace Maphy.Physics
{
    /// <summary>
    /// Value snapshot for a body in PhysicsWorld. The no-GC core stores this directly in
    /// arrays; callers work with BodyHandle instead of object references.
    /// </summary>
    public struct PhysicsWorldBody
    {
        public BodyHandle handle;
        public RigidType type;
        public fix3 position;
        public quaternion rotation;
        public fix3 velocity;
        public fix3 angularVelocity;
        public fix mass;
        public fix3 inertia;
        public bool autoMass;
        public bool autoInertia;
        public bool enabled;
        public bool useGravity;
        public bool allowSleep;
        public bool isSleeping;
        public bool useCCD;
        public fix sleepTime;

        public bool IsDynamic => enabled && type == RigidType.Dynamic;
        public bool IsAwakeDynamic => IsDynamic && !isSleeping;
        public bool IsKinematic => enabled && type == RigidType.Kinematic;
        public bool IsStatic => !enabled || type == RigidType.Static;
        public fix inverseMass => IsAwakeDynamic && mass > fix.Zero ? fix.One / mass : fix.Zero;
        public fix3 inverseInertia => IsAwakeDynamic
            ? new fix3(
                inertia.x > fix.Zero ? fix.One / inertia.x : fix.Zero,
                inertia.y > fix.Zero ? fix.One / inertia.y : fix.Zero,
                inertia.z > fix.Zero ? fix.One / inertia.z : fix.Zero)
            : fix3.zero;
    }
}
