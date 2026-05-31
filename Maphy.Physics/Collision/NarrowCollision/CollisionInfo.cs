using Maphy.Mathematics;

namespace Maphy.Physics
{
    public struct CollisionContact
    {
        public fix3 pointOnCollider0;
        public fix3 pointOnCollider1;
        public fix penetrationDepth;

        public CollisionContact(fix3 pointOnCollider0, fix3 pointOnCollider1, fix penetrationDepth)
        {
            this.pointOnCollider0 = pointOnCollider0;
            this.pointOnCollider1 = pointOnCollider1;
            this.penetrationDepth = penetrationDepth;
        }

        public fix3 position => (pointOnCollider0 + pointOnCollider1) * fix._0_5;

        public CollisionContact Flipped()
        {
            return new CollisionContact(pointOnCollider1, pointOnCollider0, penetrationDepth);
        }
    }

    public struct CollisionInfo
    {
        public const int MaxContactCount = 4;

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
        public int contactCount;
        private CollisionContact contact0;
        private CollisionContact contact1;
        private CollisionContact contact2;
        private CollisionContact contact3;

        public fix3 contactPoint => (contactPoint1 + contactPoint2) * fix._0_5;

        public CollisionContact this[int index]
        {
            get
            {
                if ((uint)index >= (uint)contactCount)
                {
                    throw new System.IndexOutOfRangeException();
                }

                switch (index)
                {
                    case 0:
                        return contact0;
                    case 1:
                        return contact1;
                    case 2:
                        return contact2;
                    default:
                        return contact3;
                }
            }
        }

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
            contactCount = 0;
            contact0 = default;
            contact1 = default;
            contact2 = default;
            contact3 = default;
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
            contactCount = 1;
            contact0 = new CollisionContact(contactPoint1, contactPoint2, penetrationDepth);
            contact1 = default;
            contact2 = default;
            contact3 = default;
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
            CollisionInfo flipped = new CollisionInfo
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
                contactCount = contactCount,
                contact0 = contact0.Flipped(),
                contact1 = contact1.Flipped(),
                contact2 = contact2.Flipped(),
                contact3 = contact3.Flipped(),
            };

            return flipped;
        }

        public bool AddContact(fix3 pointOnCollider0, fix3 pointOnCollider1, fix penetrationDepth)
        {
            return AddContact(new CollisionContact(pointOnCollider0, pointOnCollider1, penetrationDepth));
        }

        public bool AddContact(CollisionContact contact)
        {
            if (contactCount >= MaxContactCount)
            {
                return false;
            }

            if (ContainsContact(contact.position))
            {
                return false;
            }

            SetContact(contactCount, contact);
            contactCount++;
            hasContact = true;

            if (contactCount == 1)
            {
                penetrationDepth = contact.penetrationDepth;
                contactPoint1 = contact.pointOnCollider0;
                contactPoint2 = contact.pointOnCollider1;
            }

            return true;
        }

        private void SetContact(int index, CollisionContact contact)
        {
            switch (index)
            {
                case 0:
                    contact0 = contact;
                    break;
                case 1:
                    contact1 = contact;
                    break;
                case 2:
                    contact2 = contact;
                    break;
                default:
                    contact3 = contact;
                    break;
            }
        }

        private bool ContainsContact(fix3 position)
        {
            fix maxDistanceSq = fix._0_0001 * fix._0_0001;
            for (int i = 0; i < contactCount; i++)
            {
                if (math.distancesq(this[i].position, position) <= maxDistanceSq)
                {
                    return true;
                }
            }

            return false;
        }
    }
}
