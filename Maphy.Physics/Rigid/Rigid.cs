using Maphy.Mathematics;

namespace Maphy.Physics
{
    public enum RigidType
    {
        Static = 0,
        Kinematic = 1,
        Dynamic = 2
    }

    public class Rigid
    {
        public ulong id;
        public ulong colliderId;
        public RigidType type;
        public fix3 force;
        public fix3 velocity;
        public fix3 acceleration;
        public fix3 torque;
        public fix3 angularVelocity;
        public fix3 angularAcceleration;
        public fix3 inertia;
        public fix mass;
        public bool useGravity;
        public bool autoMass;
        public bool autoInertia;
        public bool enabled = true;

        public bool IsDynamic => enabled && type == RigidType.Dynamic;
        public bool IsKinematic => enabled && type == RigidType.Kinematic;
        public bool IsStatic => enabled && type == RigidType.Static;
        public fix inverseMass => IsDynamic && mass > fix.Zero ? fix.One / mass : fix.Zero;
        public fix3 inverseInertia => IsDynamic
            ? new fix3(
                inertia.x > fix.Zero ? fix.One / inertia.x : fix.Zero,
                inertia.y > fix.Zero ? fix.One / inertia.y : fix.Zero,
                inertia.z > fix.Zero ? fix.One / inertia.z : fix.Zero)
            : fix3.zero;

        public delegate void CollisionCallback(CollisionInfo collision);
        public event CollisionCallback OnCollision;
        public event CollisionCallback OnCollisionEnter;
        public event CollisionCallback OnCollisionStay;
        public event CollisionCallback OnCollisionExit;
        public event CollisionCallback OnTriggerEnter;
        public event CollisionCallback OnTriggerStay;
        public event CollisionCallback OnTriggerExit;

        public void Listener(CollisionInfo collision)
        {
            OnCollision?.Invoke(collision);
        }

        internal void DispatchCollisionEnter(CollisionInfo collision)
        {
            OnCollision?.Invoke(collision);
            if (collision.isTrigger)
            {
                OnTriggerEnter?.Invoke(collision);
            }
            else
            {
                OnCollisionEnter?.Invoke(collision);
            }
        }

        internal void DispatchCollisionStay(CollisionInfo collision)
        {
            OnCollision?.Invoke(collision);
            if (collision.isTrigger)
            {
                OnTriggerStay?.Invoke(collision);
            }
            else
            {
                OnCollisionStay?.Invoke(collision);
            }
        }

        internal void DispatchCollisionExit(CollisionInfo collision)
        {
            if (collision.isTrigger)
            {
                OnTriggerExit?.Invoke(collision);
            }
            else
            {
                OnCollisionExit?.Invoke(collision);
            }
        }

        public void SetCollider(ulong colliderId)
        {
            this.colliderId = colliderId;
        }

        public void SetEnabled(bool enabled)
        {
            this.enabled = enabled;
        }

        public void AddForce(fix3 force)
        {
            this.force += force;
        }

        public void AddTorque(fix3 torque)
        {
            this.torque += torque;
        }

        public void ClearForces()
        {
            force = fix3.zero;
        }

        public void ClearTorques()
        {
            torque = fix3.zero;
        }
    }
}
