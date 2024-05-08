
using Maphy.Mathematics;
using System;
using System.Collections.Generic;
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
        public static readonly Rigid Default = new Rigid();

        public ulong id;
        //public ulong colliderId;
        public RigidType type;
        public fix3 force;
        public fix3 linearVelocity;
        public fix3 angularVelocity;
        //public fix acc;
        public bool useGravity;
        public fix mass;
        //public List<ulong> colliderIds;
        public List<Collider> colliders;//使用id记录就需要某个Manager管理各个collider。此处先直接存储collider
        //public Memory<Collider>colliders;
        public Rigid(ulong id)
        {
            this.id = id;
            colliders = new List<Collider>();
            type = RigidType.Static;
            force = new fix3();
            linearVelocity = new fix3();
            angularVelocity = new fix3();
            useGravity = false;
            mass = 0;
            OnCollision = null;
        }

        //碰撞回调
        public delegate void CollisionCallback(Collision collision);
        public event CollisionCallback OnCollision;

        public void Listener()
        {
            OnCollision?.Invoke(default);
        }

        public void AddBoxCollider(fix3 center, fix3 size, quaternion rotation, bool isAabb = false)
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
            colliders.Add(collider);
        }

        //public fix3 LinearVelocity { get { return linearVelocity; } set { linearVelocity = value; } }

        //public fix3 AngularVelocity { get { return angularVelocity; } set { angularVelocity = value; } }


        //public bool UseGravity { get { return useGravity; } set { useGravity = value; } }

        //public fix3 Force { get { return force; } private set { force = value; } }

        //更符合使用习惯，统一get set命名


        public void SetLinearVelocity(fix3 velocity)
        {
            linearVelocity = velocity;
        }
        public fix3 GetLinearVelocity()
        {
            return linearVelocity;
        }

        public void SetAngularVelocity(fix3 velocity)
        {

        }
        public fix3 GetAngularVelocity()
        {
            return angularVelocity;
        }

        public void SetForce(fix3 force)
        {
            this.force = force;

        }

        public void AddForce(fix3 force)
        {
            this.force += force;

        }

        public fix3 GetForce()
        {
            return force;
        }


        public bool IsUseGravity()
        {
            return useGravity;
        }

        public void SetUseGravity(bool isUseGravity)
        {
            useGravity = isUseGravity;
        }



        public fix GetMass()
        {
            fix mass = 0;
            for (int i = 0; i < colliders.Count; i++)
            {
                mass += colliders[i].GetMass();
            }
            return mass;
        }



        public Collider GetCollider(int i)
        {
            if (i > -1 && i < colliders.Count)
            {
                return colliders[i];
            }
            return Collider.Default;
        }
    }
}