using System;
using Maphy.Mathematics;

namespace Maphy.Physics
{
    public sealed partial class PhysicsWorld
    {
        private readonly PhysicsWorldAABBTree ccdTree = new PhysicsWorldAABBTree(fix._0_01);
        private int[] bodyColliderHeads = Array.Empty<int>();
        private int[] colliderNext = Array.Empty<int>();
        private fix3[] ccdTargetVelocities = Array.Empty<fix3>();
        private bool ccdPrepared, ccdIndexUsable, ccdIndexOverflowed;
        private bool ccdHasEnvelope;
        private fix3 ccdEnvelopeMin, ccdEnvelopeMax;
        private fix ccdHorizon;
        private long ccdQueries, ccdCandidates, ccdShapeTests, ccdFallbackScans, ccdFallbackCandidates, ccdMovingColliders;
        private long ccdProxiesCreated, ccdProxiesUpdated, ccdProxiesReused, ccdProxiesRemoved, ccdIndexRebuilds;

        /// <summary>Diagnostic/reference switch. Both paths must produce identical simulation state.</summary>
        public bool EnableCCDBroadphase { get; set; } = true;
        public PhysicsWorldCCDStats LastCCDStats { get; private set; }

        private void ReserveCCDStorage()
        {
            int previous = bodyColliderHeads.Length;
            EnsureCapacity(ref bodyColliderHeads, bodies.Length);
            for (int i = previous; i < bodyColliderHeads.Length; i++) bodyColliderHeads[i] = -1;
            previous = colliderNext.Length;
            EnsureCapacity(ref colliderNext, colliders.Length);
            for (int i = previous; i < colliderNext.Length; i++) colliderNext[i] = -1;
            EnsureCapacity(ref ccdTargetVelocities, bodies.Length);
            ccdTree.Reserve(colliders.Length);
            ccdPrepared = false;
        }

        private void LinkBodyCollider(int body, int collider)
        {
            int previous = -1, current = bodyColliderHeads[body];
            while (current != -1 && current < collider) { previous = current; current = colliderNext[current]; }
            colliderNext[collider] = current;
            if (previous == -1) bodyColliderHeads[body] = collider;
            else colliderNext[previous] = collider;
        }

        private void UnlinkBodyCollider(int body, int collider)
        {
            int previous = -1, current = bodyColliderHeads[body];
            while (current != -1 && current != collider) { previous = current; current = colliderNext[current]; }
            if (current == -1) return;
            if (previous == -1) bodyColliderHeads[body] = colliderNext[current];
            else colliderNext[previous] = colliderNext[current];
            colliderNext[current] = -1;
        }

        private fix3 GetCCDTargetVelocity(BodyHandle handle)
        {
            if (!IsBodyAlive(handle)) return fix3.zero;
            PhysicsWorldBody body = bodies[handle.index];
            return body.IsKinematic || (settings.enableDynamicCCD && body.IsAwakeDynamic)
                ? body.velocity : fix3.zero;
        }

        private void PrepareCCD(fix deltaTime)
        {
            ccdQueries = ccdCandidates = ccdShapeTests = ccdFallbackScans = ccdFallbackCandidates = ccdMovingColliders = 0;
            ccdProxiesCreated = ccdProxiesUpdated = ccdProxiesReused = ccdProxiesRemoved = ccdIndexRebuilds = 0;
            ccdPrepared = settings.enableCCD && EnableCCDBroadphase && deltaTime > fix.Zero;
            ccdIndexOverflowed = false;
            ccdIndexUsable = ccdPrepared;
            ccdHasEnvelope = false;
            if (!ccdPrepared) return;
            ccdHorizon = deltaTime;
            ccdTree.ResetOverflow();
            for (int i = 0; i < bodyHighWaterMark; i++)
                ccdTargetVelocities[i] = bodyAlive[i] ? GetCCDTargetVelocity(bodies[i].handle) : fix3.zero;
            bool hadProxies = ccdTree.ProxyCount > 0;
            // Remove stale versions and inactive leaves before refitting. Include
            // retained fat bounds so every intermediate tree union is representable.
            for (int i = 0; i < colliderHighWaterMark; i++)
            {
                if (!ccdTree.TryGetProxy(i, out ColliderHandle handle, out AABB _, out AABB fat)) continue;
                if (!IsColliderActive(i) || colliders[i].isTrigger || handle != colliders[i].handle)
                {
                    ccdTree.RemoveProxy(handle);
                    ccdProxiesRemoved++;
                }
                else if (!AccumulateCCDEnvelope(fat.min, fat.max))
                {
                    RebuildCCDIndex();
                    hadProxies = false;
                    break;
                }
            }
            for (int i = 0; i < colliderHighWaterMark; i++) UpdateCCDProxy(i);
            if (!ccdIndexUsable && !ccdIndexOverflowed && hadProxies)
            {
                // A large teleport can make old+new bounds unrepresentable even
                // though the current scene alone fits. Retry a clean index once.
                RebuildCCDIndex();
                for (int i = 0; i < colliderHighWaterMark; i++) UpdateCCDProxy(i);
            }
        }

        private void RebuildCCDIndex()
        {
            ccdProxiesRemoved += ccdTree.ProxyCount;
            ccdTree.Clear();
            ccdIndexRebuilds++;
            ccdHasEnvelope = false;
            ccdIndexUsable = true;
        }

        private bool AccumulateCCDEnvelope(fix3 min, fix3 max)
        {
            if (ccdHasEnvelope) { min = math.min(min, ccdEnvelopeMin); max = math.max(max, ccdEnvelopeMax); }
            if (!PhysicsSafety.IsFinite(min) || !PhysicsSafety.IsFinite(max) || !PhysicsSafety.IsFinite(max - min)) return false;
            ccdEnvelopeMin = min; ccdEnvelopeMax = max; ccdHasEnvelope = true;
            return true;
        }

        private void RefreshCCDBody(int index)
        {
            if (!ccdPrepared) return;
            ccdTargetVelocities[index] = GetCCDTargetVelocity(bodies[index].handle);
            for (int i = bodyColliderHeads[index]; i != -1; i = colliderNext[i]) UpdateCCDProxy(i);
        }

        private void UpdateCCDProxy(int index)
        {
            if (!ccdPrepared || !ccdIndexUsable || !IsColliderActive(index) || colliders[index].isTrigger) return;
            PhysicsWorldCollider collider = colliders[index];
            fix3 delta = ccdTargetVelocities[collider.body.index] * ccdHorizon;
            if (!TryCCDSweptBounds(collider.bounds, delta, out AABB envelope))
            {
                ccdIndexUsable = false;
                return;
            }
            bool existing = ccdTree.TryGetProxy(index, out ColliderHandle previous, out AABB tight, out AABB _)
                && previous == collider.handle;
            if (existing && tight.center == envelope.center && tight.extents == envelope.extents)
            {
                ccdProxiesReused++;
                return;
            }
            // Individually finite leaves can still produce an unrepresentable
            // internal tree union. Include fat margins and fall back before insert.
            fix3 padding = new fix3(fix._0_01);
            fix3 min = envelope.min - padding;
            fix3 max = envelope.max + padding;
            if (!AccumulateCCDEnvelope(min, max))
            {
                ccdIndexUsable = false;
                return;
            }
            if (!ccdTree.CreateProxy(collider.handle, envelope))
            {
                ccdIndexUsable = false;
                ccdIndexOverflowed = true;
            }
            else if (existing) ccdProxiesUpdated++;
            else ccdProxiesCreated++;
        }

        private static bool TryCCDSweptBounds(AABB bounds, fix3 delta, out AABB envelope)
        {
            // Padding also covers fixed-point rounding at touching bounds.
            fix3 padding = new fix3(fix._0_01);
            fix3 min = bounds.min + math.min(fix3.zero, delta) - padding;
            fix3 max = bounds.max + math.max(fix3.zero, delta) + padding;
            envelope = default;
            if (!PhysicsSafety.IsFinite(min) || !PhysicsSafety.IsFinite(max)
                || min.x > max.x || min.y > max.y || min.z > max.z) return false;
            envelope = AABB.FromMinMax(min, max);
            return PhysicsSafety.IsFinite(envelope.center) && PhysicsSafety.IsFinite(envelope.extents)
                && envelope.extents.x >= fix.Zero && envelope.extents.y >= fix.Zero && envelope.extents.z >= fix.Zero;
        }

        private bool FindCCDTimeOfImpact(PhysicsWorldBody movingBody, fix3 translationDelta, fix deltaTime, out ContinuousHit hit)
        {
            bool found = false;
            fix bestFraction = fix.One;
            fix3 bestNormal = fix3.zero;
            int bestColliderIndex = int.MaxValue;
            for (int i = bodyColliderHeads[movingBody.handle.index]; i != -1; i = colliderNext[i])
            {
                if (!IsColliderActive(i) || colliders[i].isTrigger) continue;
                ccdMovingColliders++;
                PhysicsWorldCollider moving = colliders[i];
                PhysicsShapeData shape = moving.localShape.Transform(movingBody.position, movingBody.rotation);
                AABB bounds = shape.ComputeBounds();
                AABB envelope = default;
                bool scan = !ccdIndexUsable || !TryCCDSweptBounds(bounds, translationDelta, out envelope);
                if (!scan)
                {
                    ccdQueries++;
                    ccdTree.BeginQuery(envelope);
                    while (ccdTree.TryGetNext(out ColliderHandle candidate, out AABB _))
                    {
                        ccdCandidates++;
                        if (IsColliderAlive(candidate)) ConsiderCCDTarget(movingBody, moving, shape, bounds,
                            candidate.index, translationDelta, deltaTime, ref found, ref bestFraction, ref bestNormal, ref bestColliderIndex);
                    }
                    if (ccdTree.Overflowed)
                    {
                        ccdIndexOverflowed = true;
                        ccdIndexUsable = false;
                        scan = true;
                    }
                }
                if (scan)
                {
                    ccdFallbackScans++;
                    for (int j = 0; j < colliderHighWaterMark; j++)
                    {
                        ccdFallbackCandidates++;
                        ConsiderCCDTarget(movingBody, moving, shape, bounds, j, translationDelta, deltaTime,
                            ref found, ref bestFraction, ref bestNormal, ref bestColliderIndex);
                    }
                }
            }
            hit = new ContinuousHit(bestFraction, bestNormal);
            return found;
        }

        private void ConsiderCCDTarget(in PhysicsWorldBody movingBody, in PhysicsWorldCollider moving,
            in PhysicsShapeData movingShape, in AABB movingBounds, int index, fix3 translationDelta, fix deltaTime,
            ref bool found, ref fix bestFraction, ref fix3 bestNormal, ref int bestColliderIndex)
        {
            if (!IsColliderActive(index) || colliders[index].body == movingBody.handle || colliders[index].isTrigger
                || !CanCollide(moving, colliders[index])
                || (!settings.enableDynamicCCD && IsAwakeDynamicBody(colliders[index].body))) return;
            fix3 relativeDelta = translationDelta - GetContinuousTargetDelta(colliders[index].body, deltaTime);
            if (math.lengthsq(relativeDelta) <= math.Epsilon) return;
            ccdShapeTests++;
            if (!TryComputeColliderTimeOfImpact(movingShape, colliders[index].worldShape, movingBounds,
                colliders[index].bounds, relativeDelta, out fix fraction, out fix3 normal)) return;
            int target = colliders[index].handle.Index;
            if (fraction > bestFraction || (fraction == bestFraction && target >= bestColliderIndex)) return;
            found = true; bestFraction = fraction; bestNormal = normal; bestColliderIndex = target;
        }

        private void CaptureCCDStats()
        {
            LastCCDStats = new PhysicsWorldCCDStats(ccdQueries, ccdCandidates, ccdShapeTests, ccdFallbackScans,
                ccdFallbackCandidates, ccdMovingColliders, ccdPrepared ? ccdTree.ProxyCount : 0, ccdIndexOverflowed,
                ccdProxiesCreated, ccdProxiesUpdated, ccdProxiesReused, ccdProxiesRemoved, ccdIndexRebuilds);
        }
    }
}
