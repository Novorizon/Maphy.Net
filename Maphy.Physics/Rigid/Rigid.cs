using Maphy.Mathematics;

namespace Maphy.Physics
{
    public enum RigidType
    {
        Static = 0,
        Kinematic = 1,
        Dynamic = 2
    }

    public struct Rigid
    {
        public ulong id;
        public ulong colliderId;
        public RigidType type;
        public fix3 force;
        public fix3 velocity;
        public fix3 acceleration;
        public fix mass;
        public bool useGravity;

        public bool IsDynamic => type == RigidType.Dynamic;
        public bool IsKinematic => type == RigidType.Kinematic;
        public bool IsStatic => type == RigidType.Static;
        public fix inverseMass => IsDynamic && mass > fix.Zero ? fix.One / mass : fix.Zero;

        public delegate void CollisionCallback(CollisionInfo collision);
        public event CollisionCallback OnCollision;

        public void Listener(CollisionInfo collision)
        {
            OnCollision?.Invoke(collision);
        }

        public void SetCollider(ulong colliderId)
        {
            this.colliderId = colliderId;
        }

        public void AddForce(fix3 force)
        {
            this.force += force;
        }

        public void ClearForces()
        {
            force = fix3.zero;
        }
    }
}
