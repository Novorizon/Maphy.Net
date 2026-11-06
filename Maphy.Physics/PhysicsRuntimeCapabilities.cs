using System;

namespace Maphy.Physics
{
    /// <summary>Capabilities of the actual simulation entry point.</summary>
    public readonly struct PhysicsRuntimeCapabilities
    {
        private readonly bool objectWorld;
        private PhysicsRuntimeCapabilities(bool objectWorld) { this.objectWorld = objectWorld; }
        public static PhysicsRuntimeCapabilities Objects => new PhysicsRuntimeCapabilities(true);
        public static PhysicsRuntimeCapabilities Core => new PhysicsRuntimeCapabilities(false);
        public bool SupportsCollisionCallbacks => objectWorld;
        public bool RequiresReservedCapacity => !objectWorld;

        public bool SupportsAlgorithm(NarrowPhaseAlgorithm algorithm)
        {
            return algorithm == NarrowPhaseAlgorithm.Auto || algorithm == NarrowPhaseAlgorithm.SAT
                || (objectWorld && algorithm == NarrowPhaseAlgorithm.GJK);
        }

        public bool SupportsConstraint(ConstraintType type)
        {
            return objectWorld && ConstraintRegistry.IsImplemented(type);
        }

        internal void EnsureAlgorithm(NarrowPhaseAlgorithm algorithm)
        {
            if (algorithm < NarrowPhaseAlgorithm.Auto || algorithm > NarrowPhaseAlgorithm.GJK)
                throw new ArgumentOutOfRangeException(nameof(algorithm));
            if (!SupportsAlgorithm(algorithm))
                throw new NotSupportedException("PhysicsWorld supports Auto and SAT. GJK/EPA is available through World and the object Shape query API.");
        }
    }
}
