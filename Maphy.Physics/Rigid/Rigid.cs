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

        public Collider AddBoxCollider(fix3 center, fix3 size, quaternion rotation, bool isAabb = false)
        {
            Collider collider = new Collider();
            if (isAabb)
            {
                collider.AddAABBCollider(id, center, size);
            }
            else
            {
                collider.AddOBBCollider(id, center, size, rotation);
            }

            colliderId = collider.id;
            BroadCollisionSystem.Register(collider);
            return collider;
        }
    }
}