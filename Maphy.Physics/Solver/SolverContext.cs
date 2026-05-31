using System.Collections.Generic;
using Maphy.Mathematics;

namespace Maphy.Physics
{
    internal readonly struct SolverContext
    {
        private readonly Dictionary<ulong, Rigid> rigids;
        private readonly Dictionary<ulong, Entity> entities;

        public SolverContext(Dictionary<ulong, Rigid> rigids, Dictionary<ulong, Entity> entities)
        {
            this.rigids = rigids;
            this.entities = entities;
        }

        public bool TryGetRigidMassData(ulong rigidId, out Rigid rigid, out fix inverseMass)
        {
            if (rigids.TryGetValue(rigidId, out rigid))
            {
                inverseMass = rigid.inverseMass;
                return true;
            }

            rigid = default;
            inverseMass = fix.Zero;
            return false;
        }

        public bool TryGetBody(ulong rigidId, out Rigid rigid, out Entity entity, out fix inverseMass)
        {
            if (rigids.TryGetValue(rigidId, out rigid) && entities.TryGetValue(rigidId, out entity))
            {
                inverseMass = rigid.inverseMass;
                return rigid.enabled;
            }

            rigid = default;
            entity = default;
            inverseMass = fix.Zero;
            return false;
        }

        public fix3 GetEntityPosition(ulong entityId)
        {
            return entities.TryGetValue(entityId, out Entity entity) ? entity.translation : fix3.zero;
        }

        public void TranslateEntity(ulong entityId, fix3 delta)
        {
            if (!entities.TryGetValue(entityId, out Entity entity))
            {
                return;
            }

            entity.translation += delta;
            entities[entityId] = entity;
        }

        public void ApplyImpulse(
            bool hasRigid0,
            Rigid rigid0,
            fix inverseMass0,
            fix3 relativePoint0,
            bool hasRigid1,
            Rigid rigid1,
            fix inverseMass1,
            fix3 relativePoint1,
            fix3 impulse)
        {
            if (impulse == fix3.zero)
            {
                return;
            }

            if (hasRigid0 && inverseMass0 > fix.Zero)
            {
                rigid0.velocity -= impulse * inverseMass0;
                rigid0.angularVelocity -= rigid0.inverseInertia * math.cross(relativePoint0, impulse);
                rigids[rigid0.id] = rigid0;
            }

            if (hasRigid1 && inverseMass1 > fix.Zero)
            {
                rigid1.velocity += impulse * inverseMass1;
                rigid1.angularVelocity += rigid1.inverseInertia * math.cross(relativePoint1, impulse);
                rigids[rigid1.id] = rigid1;
            }
        }

        public static fix3 GetVelocityAtPoint(Rigid rigid, fix3 relativePoint)
        {
            return rigid.velocity + math.cross(rigid.angularVelocity, relativePoint);
        }

        public static fix GetAngularEffectiveMass(Rigid rigid, fix3 relativePoint, fix3 direction)
        {
            if (rigid == null || !rigid.IsDynamic)
            {
                return fix.Zero;
            }

            fix3 angularVelocityPerImpulse = rigid.inverseInertia * math.cross(relativePoint, direction);
            return math.dot(math.cross(angularVelocityPerImpulse, relativePoint), direction);
        }
    }
}
