using Maphy.Mathematics;

namespace Maphy.Physics
{
    public struct CollisionInfo
    {
        public BroadCollisionPairKey key;
        public ulong id;
        public ulong otherId;
        public ulong rigidId;
        public ulong otherRigidId;
        public fix penetrationDepth;
        public fix3 normal;
        public fix3 contactPoint1;
        public fix3 contactPoint2;
        public bool hasContact;

        public fix3 contactPoint => (contactPoint1 + contactPoint2) * fix._0_5;

        public CollisionInfo(ulong id, ulong otherId)
        {
            key = new BroadCollisionPairKey(id, otherId);
            this.id = id;
            this.otherId = otherId;
            rigidId = 0;
            otherRigidId = 0;
            penetrationDepth = fix.Zero;
            normal = fix3.zero;
            contactPoint1 = fix3.zero;
            contactPoint2 = fix3.zero;
            hasContact = false;
        }

        public CollisionInfo(int id, int otherId)
            : this((ulong)id, (ulong)otherId)
        {
        }

        public CollisionInfo(fix penetrationDepth, fix3 normal, fix3 contactPoint1, fix3 contactPoint2)
        {
            key = new BroadCollisionPairKey(0, 0);
            id = 0;
            otherId = 0;
            rigidId = 0;
            otherRigidId = 0;
            this.penetrationDepth = penetrationDepth;
            this.normal = normal;
            this.contactPoint1 = contactPoint1;
            this.contactPoint2 = contactPoint2;
            hasContact = true;
        }

        public void SetPair(BroadCollisionPair pair)
        {
            key = pair.key;
            id = pair.colliderId0;
            otherId = pair.colliderId1;
            rigidId = pair.rigidId0;
            otherRigidId = pair.rigidId1;
        }

        public CollisionInfo Flipped()
        {
            return new CollisionInfo
            {
                key = key,
                id = otherId,
                otherId = id,
                rigidId = otherRigidId,
                otherRigidId = rigidId,
                penetrationDepth = penetrationDepth,
                normal = -normal,
                contactPoint1 = contactPoint2,
                contactPoint2 = contactPoint1,
                hasContact = hasContact,
            };
        }
    }
}
