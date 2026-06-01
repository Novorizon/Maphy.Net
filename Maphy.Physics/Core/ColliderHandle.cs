using System;

namespace Maphy.Physics
{
    /// <summary>
    /// Stable handle for collider storage in PhysicsWorld. A handle is just data, so
    /// passing it around does not allocate and does not depend on object identity.
    /// </summary>
    public readonly struct ColliderHandle : IEquatable<ColliderHandle>
    {
        internal readonly int index;
        internal readonly int version;

        internal ColliderHandle(int index, int version)
        {
            this.index = index;
            this.version = version;
        }

        public bool IsValid => index >= 0 && version > 0;
        public int Index => index;
        public int Version => version;

        public static ColliderHandle Invalid => new ColliderHandle(-1, 0);

        public bool Equals(ColliderHandle other)
        {
            return index == other.index && version == other.version;
        }

        public override bool Equals(object obj)
        {
            return obj is ColliderHandle other && Equals(other);
        }

        public override int GetHashCode()
        {
            unchecked
            {
                return (index * 397) ^ version;
            }
        }

        public static bool operator ==(ColliderHandle left, ColliderHandle right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(ColliderHandle left, ColliderHandle right)
        {
            return !left.Equals(right);
        }
    }
}
