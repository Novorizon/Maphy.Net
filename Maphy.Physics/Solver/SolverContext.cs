using System.Collections.Generic;
using Maphy.Mathematics;

namespace Maphy.Physics
{
    internal readonly struct SolverContext
    {
        private readonly Dictionary<ulong, Rigid> rigids;
        private readonly Dictionary<ulong, Entity> entities;
        public readonly fix warmStartScale;

        public SolverContext(Dictionary<ulong, Rigid> rigids, Dictionary<ulong, Entity> entities)
            : this(rigids, entities, fix.One)
        {
        }

        public SolverContext(Dictionary<ulong, Rigid> rigids, Dictionary<ulong, Entity> entities, fix warmStartScale)
        {
            this.rigids = rigids;
            this.entities = entities;
            this.warmStartScale = math.clamp(warmStartScale, fix.Zero, fix.One);
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

        public quaternion GetEntityOrientation(ulong entityId)
        {
            return entities.TryGetValue(entityId, out Entity entity) ? entity.orientation : quaternion.identity;
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

        public void RotateEntity(ulong entityId, fix3 angularDelta)
        {
            if (!entities.TryGetValue(entityId, out Entity entity))
            {
                return;
            }

            fix angleSq = math.lengthsq(angularDelta);
            if (angleSq <= math.Epsilon)
            {
                return;
            }

            fix angle = math.sqrt(angleSq);
            quaternion deltaRotation = quaternion.AxisAngle(angularDelta / angle, angle);
            entity.orientation = quaternion.normalize(deltaRotation * entity.orientation);
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

        public void ApplyAngularImpulse(
            bool hasRigid0,
            Rigid rigid0,
            bool hasRigid1,
            Rigid rigid1,
            fix3 impulse)
        {
            if (impulse == fix3.zero)
            {
                return;
            }

            if (hasRigid0 && rigid0 != null && rigid0.IsAwakeDynamic)
            {
                rigid0.angularVelocity -= rigid0.inverseInertia * impulse;
                rigids[rigid0.id] = rigid0;
            }

            if (hasRigid1 && rigid1 != null && rigid1.IsAwakeDynamic)
            {
                rigid1.angularVelocity += rigid1.inverseInertia * impulse;
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

        public static fix GetAngularOnlyEffectiveMass(Rigid rigid0, Rigid rigid1, fix3 axis)
        {
            fix effectiveMass = fix.Zero;
            if (rigid0 != null && rigid0.IsDynamic)
            {
                effectiveMass += math.dot(rigid0.inverseInertia * axis, axis);
            }

            if (rigid1 != null && rigid1.IsDynamic)
            {
                effectiveMass += math.dot(rigid1.inverseInertia * axis, axis);
            }

            return effectiveMass;
        }
    }
}
