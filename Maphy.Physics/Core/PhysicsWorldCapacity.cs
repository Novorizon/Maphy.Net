using System;

namespace Maphy.Physics
{
    /// <summary>
    /// Capacity contract for PhysicsWorld. Reserve may allocate to satisfy this shape;
    /// simulation and non-alloc query methods then stay inside these buffers.
    /// </summary>
    public readonly struct PhysicsWorldCapacity
    {
        public readonly int bodyCapacity;
        public readonly int colliderCapacity;
        public readonly int pairCapacity;
        public readonly int contactManifoldCapacity;

        public PhysicsWorldCapacity(
            int bodyCapacity,
            int colliderCapacity,
            int pairCapacity,
            int contactManifoldCapacity)
        {
            this.bodyCapacity = bodyCapacity;
            this.colliderCapacity = colliderCapacity;
            this.pairCapacity = pairCapacity;
            this.contactManifoldCapacity = contactManifoldCapacity;
        }

        public static PhysicsWorldCapacity Empty => default;

        public bool IsValid =>
            bodyCapacity >= 0
            && colliderCapacity >= 0
            && pairCapacity >= 0
            && contactManifoldCapacity >= 0;

        public static PhysicsWorldCapacity ForBodies(
            int bodyCapacity,
            int collidersPerBody,
            int candidatePairsPerCollider)
        {
            bodyCapacity = Math.Max(0, bodyCapacity);
            collidersPerBody = Math.Max(0, collidersPerBody);
            candidatePairsPerCollider = Math.Max(0, candidatePairsPerCollider);

            int colliderCapacity = SaturatingMultiply(bodyCapacity, collidersPerBody);
            int pairCapacity = SaturatingMultiply(colliderCapacity, candidatePairsPerCollider);
            return new PhysicsWorldCapacity(bodyCapacity, colliderCapacity, pairCapacity, pairCapacity);
        }

        public PhysicsWorldMemoryBudget EstimateMemoryBudget()
        {
            PhysicsWorldCapacity normalized = Normalize();
            long bodyBytes = EstimateArrayBytes(normalized.bodyCapacity, 256);
            long colliderBytes = EstimateArrayBytes(normalized.colliderCapacity, 512);
            long broadphaseBytes = EstimateArrayBytes(SaturatingMultiply(normalized.colliderCapacity, 2), 160)
                + EstimateArrayBytes(normalized.colliderCapacity, 16);
            long pairBytes = EstimateArrayBytes(normalized.pairCapacity, 160);
            long contactBytes = EstimateArrayBytes(normalized.contactManifoldCapacity, 512);
            long scratchBytes = EstimateArrayBytes(normalized.contactManifoldCapacity, 512);
            return new PhysicsWorldMemoryBudget(
                bodyBytes,
                colliderBytes,
                broadphaseBytes,
                pairBytes,
                contactBytes,
                scratchBytes);
        }

        internal PhysicsWorldCapacity Normalize()
        {
            return new PhysicsWorldCapacity(
                Math.Max(0, bodyCapacity),
                Math.Max(0, colliderCapacity),
                Math.Max(0, pairCapacity),
                Math.Max(0, contactManifoldCapacity));
        }

        private static long EstimateArrayBytes(int count, int bytesPerElement)
        {
            return count <= 0 ? 0L : (long)count * bytesPerElement;
        }

        private static int SaturatingMultiply(int a, int b)
        {
            long result = (long)a * b;
            return result > int.MaxValue ? int.MaxValue : (int)result;
        }
    }

    /// <summary>
    /// Conservative memory estimate for reserved PhysicsWorld buffers. It is a sizing
    /// guide, not an exact CLR/IL2CPP object-layout report.
    /// </summary>
    public readonly struct PhysicsWorldMemoryBudget
    {
        public readonly long bodyBytes;
        public readonly long colliderBytes;
        public readonly long broadphaseBytes;
        public readonly long pairBytes;
        public readonly long contactManifoldBytes;
        public readonly long contactScratchBytes;
        public readonly long totalBytes;

        public PhysicsWorldMemoryBudget(
            long bodyBytes,
            long colliderBytes,
            long broadphaseBytes,
            long pairBytes,
            long contactManifoldBytes,
            long contactScratchBytes)
        {
            this.bodyBytes = Math.Max(0L, bodyBytes);
            this.colliderBytes = Math.Max(0L, colliderBytes);
            this.broadphaseBytes = Math.Max(0L, broadphaseBytes);
            this.pairBytes = Math.Max(0L, pairBytes);
            this.contactManifoldBytes = Math.Max(0L, contactManifoldBytes);
            this.contactScratchBytes = Math.Max(0L, contactScratchBytes);
            totalBytes = this.bodyBytes
                + this.colliderBytes
                + this.broadphaseBytes
                + this.pairBytes
                + this.contactManifoldBytes
                + this.contactScratchBytes;
        }
    }
}
