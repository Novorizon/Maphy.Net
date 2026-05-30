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
    }
}
