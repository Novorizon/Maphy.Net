using Maphy.Mathematics;

namespace Maphy.Physics
{
    /// <summary>
    /// Raycast hit returned by PhysicsWorld. It stores handles only, so queries do not
    /// allocate wrappers or expose object-layer Collider instances.
    /// </summary>
    public struct PhysicsWorldRaycastHit
    {
        public ColliderHandle collider;
        public BodyHandle body;
        public fix distance;
        public fix3 point;
        public fix3 normal;
        public AABB bounds;

        public bool IsValid => collider.IsValid;
    }
}
