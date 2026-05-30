using Maphy.Mathematics;

namespace Maphy.Physics
{
    public struct Collider
    {
        public static ulong Id = 0;

        public ulong id { get; set; }
        public ulong rigidId { get; set; }
        public fix3 localCenter { get; private set; }
        public quaternion localOrientation { get; private set; }

        public Shape shape { get; set; }
        public Material material { get; set; }
        public bool isTrigger { get; set; }
        private CollisionInfo collisionInfo { get; set; }

        public void AddAABBCollider(ulong rigidId, fix3 center, fix3 size)
        {
            id = Id++;
            this.rigidId = rigidId;
            localCenter = center;
            localOrientation = quaternion.identity;
            shape = new AABB(center, size);
            material = new Material();
            isTrigger = false;
            collisionInfo = new CollisionInfo();
        }

        public void AddOBBCollider(ulong rigidId, fix3 center, fix3 size, quaternion rotation)
        {
            id = Id++;
            this.rigidId = rigidId;
            localCenter = center;
            localOrientation = rotation;
            shape = new OBB(center, size, rotation);
            material = new Material();
            isTrigger = false;
            collisionInfo = new CollisionInfo();
        }

        public void AddSphereCollider(ulong rigidId, fix3 center, fix radius)
        {
            id = Id++;
            this.rigidId = rigidId;
            localCenter = center;
            localOrientation = quaternion.identity;
            shape = new Sphere(center, radius);
            material = new Material();
            isTrigger = false;
            collisionInfo = new CollisionInfo();
        }

        public void AddCapsuleCollider(ulong rigidId, fix3 center, fix radius, fix height, quaternion rotation)
        {
            id = Id++;
            this.rigidId = rigidId;
            localCenter = center;
            localOrientation = rotation;
            shape = new Capsule(center, radius, height, rotation, fix3.up);
            material = new Material();
            isTrigger = false;
            collisionInfo = new CollisionInfo();
        }

        public void SyncTransform(fix3 translation, quaternion orientation)
        {
            fix3 worldCenter = translation + orientation * localCenter;
            quaternion worldOrientation = orientation * localOrientation;

            switch (shape.Type)
            {
                case ShapeType.AABB:
                    AABB aabb = (AABB)shape;
                    aabb.Update(worldCenter);
                    shape = aabb;
                    break;
                case ShapeType.OBB:
                    OBB obb = (OBB)shape;
                    obb.Update(worldCenter, worldOrientation);
                    shape = obb;
                    break;
                case ShapeType.Sphere:
                    Sphere sphere = (Sphere)shape;
                    sphere.Update(worldCenter);
                    shape = sphere;
                    break;
                case ShapeType.Capsule:
                    Capsule capsule = (Capsule)shape;
                    capsule.Update(worldCenter, worldOrientation);
                    shape = capsule;
                    break;
            }
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

        public delegate void CollisionCallback(CollisionInfo collision);
        public event CollisionCallback OnCollision;
    }
}
