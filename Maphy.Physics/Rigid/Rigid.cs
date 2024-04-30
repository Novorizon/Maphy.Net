
using Maphy.Mathematics;
using UnityEngine;

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
        public fix force;
        public fix velosity;
        public fix acc;
        public fix mass;
        public bool useGravity;


        //碰撞回调
        public delegate void CollisionCallback(Collision collision);
        public event CollisionCallback OnCollision;

        public void Listener()
        {
            OnCollision?.Invoke(default);
        }

        public void AddBoxCollider(fix3 center,fix3 size, quaternion rotation,bool isAabb=false)
        {
            Collider collider = new Collider();
            if(isAabb)
            {
                collider.AddAABBCollider(id, center, size);
            }
            else
            {
                collider.AddOBBCollider(id, center, size, rotation);
            }
        }
    }
}