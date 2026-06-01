using Maphy.Mathematics;

namespace Maphy.Physics
{
    /// <summary>
    /// Contact point stored by the no-GC world. Solver impulses live here so warm start
    /// can be added without introducing a dictionary-backed manifold cache later.
    /// </summary>
    public struct PhysicsWorldContactPoint
    {
        public fix3 position;
        public fix3 pointOnCollider0;
        public fix3 pointOnCollider1;
        public fix penetrationDepth;
        public fix normalImpulse;
        public fix tangentImpulse0;
        public fix tangentImpulse1;
        public fix3 tangent0;
        public fix3 tangent1;
        public int featureId;
        public int lifetime;

        public PhysicsWorldContactPoint(
            CollisionContact contact,
            PhysicsWorldContactPoint previous,
            bool preserveImpulse)
        {
            position = contact.position;
            pointOnCollider0 = contact.pointOnCollider0;
            pointOnCollider1 = contact.pointOnCollider1;
            penetrationDepth = contact.penetrationDepth;
            normalImpulse = preserveImpulse ? previous.normalImpulse : fix.Zero;
            tangentImpulse0 = preserveImpulse ? previous.tangentImpulse0 : fix.Zero;
            tangentImpulse1 = preserveImpulse ? previous.tangentImpulse1 : fix.Zero;
            tangent0 = preserveImpulse ? previous.tangent0 : fix3.zero;
            tangent1 = preserveImpulse ? previous.tangent1 : fix3.zero;
            featureId = contact.featureId;
            lifetime = preserveImpulse ? previous.lifetime + 1 : 1;
        }
    }
}
