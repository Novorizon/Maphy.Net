
using Maphy.Mathematics;
using UnityEngine;

namespace Maphy.Physics
{
    public struct Collider
    {
        public static ulong Id = 0;
        public ulong id { get; set; }
        public ulong rigidId { get; set; }

        public Shape shape { get; set; }
        //材质
        public Material material { get; set; }
        public  bool isTrigger        {            get;            set; }
        CollisionInfo collisionInfo { get; set; }

        public void AddAABBCollider(ulong rigidId,fix3 center,fix3 size)
        {
            id = Id++;
            this.rigidId= rigidId;

            AABB aabb = new AABB(center, size);
            shape = aabb;
            material = new Material();
            isTrigger = false;
            collisionInfo = new CollisionInfo();


        }

        public void AddOBBCollider(ulong rigidId, fix3 center, fix3 size, quaternion rotation)
        {
            id = Id++;
            this.rigidId = rigidId;

            OBB obb = new OBB(center, size,rotation);
            shape = obb;
            material = new Material();
            isTrigger = false;
            collisionInfo = new CollisionInfo();
        }

        public void AddSphereCollider(ulong rigidId, fix3 center, fix radius)
        {
            id = Id++;
            this.rigidId = rigidId;

            Sphere sphere = new Sphere(center, radius);
            shape = sphere;
            material = new Material();
            isTrigger = false;
            collisionInfo = new CollisionInfo();
        }

        public void AddCapsuleCollider(ulong rigidId, fix3 center, fix radius, fix height, quaternion rotation)
        {
            id = Id++;
            this.rigidId = rigidId;

            Capsule capsule = new Capsule(center, radius, height, rotation,fix3.zero);
            shape = capsule;
            material = new Material();
            isTrigger = false;
            collisionInfo = new CollisionInfo();
        }

        public CollisionInfo collision
        {
            get { return collisionInfo; }
            set
            {
                collisionInfo = value;
                OnCollision?.Invoke(collisionInfo);
            }
        }
        //碰撞回调
        public delegate void CollisionCallback(CollisionInfo collision);
        public event CollisionCallback OnCollision;

        public void Collides(Collider collider)
        {
            //if(Physics.IsOverlap(this,collider,ref  collisionInfo))
            //{

            //    OnCollision?.Invoke(collisionInfo);
            //}
            // 设置一个射线或者其他检测方式，用来检测碰撞
            Ray ray = new Ray(fix3.zero, fix3.forward);
            RaycastHit hit;

            //if (Physics.Raycast(ray, out hit))
            //{
            //    // 当射线与碰撞器相交时调用回调函数
            //    OnCollision?.Invoke(hit.collider);
            //}
        }
    }
}