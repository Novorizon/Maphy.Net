using Maphy.Mathematics;
using System.Collections.Generic;

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
        private readonly List<ulong> colliderIds = new List<ulong>();

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
        public bool allowSleep = true;
        public bool isSleeping;
        public bool useCCD = true;
        public fix sleepTime;

        public bool IsDynamic => enabled && type == RigidType.Dynamic;
        public bool IsAwakeDynamic => IsDynamic && !isSleeping;
        public bool IsKinematic => enabled && type == RigidType.Kinematic;
        public bool IsStatic => enabled && type == RigidType.Static;
        public fix inverseMass => IsAwakeDynamic && mass > fix.Zero ? fix.One / mass : fix.Zero;
        public fix3 inverseInertia => IsAwakeDynamic
            ? new fix3(
                inertia.x > fix.Zero ? fix.One / inertia.x : fix.Zero,
                inertia.y > fix.Zero ? fix.One / inertia.y : fix.Zero,
                inertia.z > fix.Zero ? fix.One / inertia.z : fix.Zero)
            : fix3.zero;
        public IReadOnlyList<ulong> ColliderIds => colliderIds;
        public int colliderCount => colliderIds.Count;

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
            if (colliderId != 0)
            {
                AddCollider(colliderId);
            }
            else
            {
                colliderIds.Clear();
            }
        }

        public void AddCollider(ulong colliderId)
        {
            if (colliderId == 0)
            {
                return;
            }

            if (!colliderIds.Contains(colliderId))
            {
                colliderIds.Add(colliderId);
            }

            if (this.colliderId == 0)
            {
                this.colliderId = colliderId;
            }
        }

        public void RemoveCollider(ulong colliderId)
        {
            colliderIds.Remove(colliderId);
            if (this.colliderId == colliderId)
            {
                this.colliderId = colliderIds.Count > 0 ? colliderIds[0] : 0;
            }
        }

        public bool ContainsCollider(ulong colliderId)
        {
            return colliderIds.Contains(colliderId);
        }

        public void SetEnabled(bool enabled)
        {
            this.enabled = enabled;
            if (!enabled)
            {
                WakeUp();
            }
        }

        public void SetAllowSleep(bool allowSleep)
        {
            this.allowSleep = allowSleep;
            if (!allowSleep)
            {
                WakeUp();
            }
        }

        public void WakeUp()
        {
            isSleeping = false;
            sleepTime = fix.Zero;
        }

        public void Sleep()
        {
            if (!IsDynamic || !allowSleep)
            {
                return;
            }

            isSleeping = true;
            sleepTime = fix.Zero;
            velocity = fix3.zero;
            angularVelocity = fix3.zero;
            ClearForces();
            ClearTorques();
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
