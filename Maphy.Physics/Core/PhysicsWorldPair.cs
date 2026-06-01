namespace Maphy.Physics
{
    /// <summary>
    /// Broadphase pair produced by PhysicsWorld. The pair is handle-only so callers can
    /// inspect it without receiving mutable collider objects or allocating wrappers.
    /// </summary>
    public readonly struct PhysicsWorldPair
    {
        public readonly ColliderHandle collider0;
        public readonly ColliderHandle collider1;
        public readonly BodyHandle body0;
        public readonly BodyHandle body1;
        public readonly AABB bounds0;
        public readonly AABB bounds1;

        public PhysicsWorldPair(
            ColliderHandle collider0,
            ColliderHandle collider1,
            BodyHandle body0,
            BodyHandle body1,
            AABB bounds0,
            AABB bounds1)
        {
            this.collider0 = collider0;
            this.collider1 = collider1;
            this.body0 = body0;
            this.body1 = body1;
            this.bounds0 = bounds0;
            this.bounds1 = bounds1;
        }
    }
}
