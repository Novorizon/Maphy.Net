using Maphy.Mathematics;

namespace Maphy.Physics
{
    public class Collider
    {
        public static ulong Id = 0;
        public const int MinLayer = 0;
        public const int MaxLayer = 31;
        public const int DefaultLayer = 0;
        public const int AllLayers = -1;

        public ulong id { get; set; }
        public ulong rigidId { get; set; }
        public int layer { get; private set; }
        public int collisionMask { get; private set; }
        public fix3 localCenter { get; private set; }
        public quaternion localOrientation { get; private set; }

        public Shape shape { get; set; }
        public Material material { get; set; }
        public bool isTrigger { get; set; }
        public bool enabled { get; private set; }
        private CollisionInfo collisionInfo { get; set; }

        public void AddAABBCollider(ulong rigidId, fix3 center, fix3 size)
        {
            id = Id++;
            this.rigidId = rigidId;
            localCenter = center;
            localOrientation = quaternion.identity;
            shape = new AABB(center, size);
            material = Material.Default;
            isTrigger = false;
            enabled = true;
            layer = DefaultLayer;
            collisionMask = AllLayers;
            collisionInfo = new CollisionInfo();
        }

        public void AddOBBCollider(ulong rigidId, fix3 center, fix3 size, quaternion rotation)
        {
            id = Id++;
            this.rigidId = rigidId;
            localCenter = center;
            localOrientation = rotation;
            shape = new OBB(center, size, rotation);
            material = Material.Default;
            isTrigger = false;
            enabled = true;
            layer = DefaultLayer;
            collisionMask = AllLayers;
            collisionInfo = new CollisionInfo();
        }

        public void AddSphereCollider(ulong rigidId, fix3 center, fix radius)
        {
            id = Id++;
            this.rigidId = rigidId;
            localCenter = center;
            localOrientation = quaternion.identity;
            shape = new Sphere(center, radius);
            material = Material.Default;
            isTrigger = false;
            enabled = true;
            layer = DefaultLayer;
            collisionMask = AllLayers;
            collisionInfo = new CollisionInfo();
        }

        public void AddCapsuleCollider(ulong rigidId, fix3 center, fix radius, fix height, quaternion rotation)
        {
            id = Id++;
            this.rigidId = rigidId;
            localCenter = center;
            localOrientation = rotation;
            shape = new Capsule(center, radius, height, rotation, fix3.up);
            material = Material.Default;
            isTrigger = false;
            enabled = true;
            layer = DefaultLayer;
            collisionMask = AllLayers;
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
                DispatchActiveCollision(value);
            }
        }

        internal void DispatchCollisionEnter(CollisionInfo collision)
        {
            DispatchActiveCollision(collision);
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
            DispatchActiveCollision(collision);
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
            collisionInfo = collision;
            if (collision.isTrigger)
            {
                OnTriggerExit?.Invoke(collision);
            }
            else
            {
                OnCollisionExit?.Invoke(collision);
            }
        }

        private void DispatchActiveCollision(CollisionInfo collision)
        {
            collisionInfo = collision;
            OnCollision?.Invoke(collisionInfo);
        }

        public void SetTrigger(bool isTrigger)
        {
            this.isTrigger = isTrigger;
        }

        public void SetEnabled(bool enabled)
        {
            this.enabled = enabled;
        }

        public void SetMaterial(Material material)
        {
            this.material = material;
        }

        public bool SetLayer(int layer)
        {
            if (layer < MinLayer || layer > MaxLayer)
            {
                return false;
            }

            this.layer = layer;
            return true;
        }

        public void SetCollisionMask(int collisionMask)
        {
            this.collisionMask = collisionMask;
        }

        public bool IsInLayerMask(int layerMask)
        {
            return (layerMask & GetLayerBit(layer)) != 0;
        }

        public bool CanCollideWith(Collider other)
        {
            return other != null
                && enabled
                && other.enabled
                && IsLayerEnabled(collisionMask, other.layer)
                && IsLayerEnabled(other.collisionMask, layer);
        }

        public static int GetLayerBit(int layer)
        {
            return layer < MinLayer || layer > MaxLayer ? 0 : 1 << layer;
        }

        private static bool IsLayerEnabled(int mask, int layer)
        {
            return (mask & GetLayerBit(layer)) != 0;
        }

        public delegate void CollisionCallback(CollisionInfo collision);
        public event CollisionCallback OnCollision;
        public event CollisionCallback OnCollisionEnter;
        public event CollisionCallback OnCollisionStay;
        public event CollisionCallback OnCollisionExit;
        public event CollisionCallback OnTriggerEnter;
        public event CollisionCallback OnTriggerStay;
        public event CollisionCallback OnTriggerExit;
    }
}
