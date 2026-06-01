using Maphy.Mathematics;

namespace Maphy.Physics
{
    /// <summary>
    /// Shape cast hit returned by PhysicsWorld. Fraction is normalized to the supplied
    /// delta, matching ShapeCastHit while keeping collider/body handles attached.
    /// </summary>
    public struct PhysicsWorldShapeCastHit
    {
        public ColliderHandle collider;
        public BodyHandle body;
        public fix fraction;
        public fix3 point;
        public fix3 normal;
        public AABB bounds;

        public bool IsValid => collider.IsValid;
    }
}
