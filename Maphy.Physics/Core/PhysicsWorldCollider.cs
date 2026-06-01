namespace Maphy.Physics
{
    /// <summary>
    /// Value snapshot for a collider in PhysicsWorld. Local shape is authored relative
    /// to the body, while worldShape and bounds are synchronized during Update/Query.
    /// </summary>
    public struct PhysicsWorldCollider
    {
        public ColliderHandle handle;
        public BodyHandle body;
        public int layer;
        public int collisionMask;
        public bool enabled;
        public bool isTrigger;
        public Material material;
        public PhysicsShapeData localShape;
        public PhysicsShapeData worldShape;
        public AABB bounds;

        public bool IsActive => enabled && body.IsValid;
    }
}
