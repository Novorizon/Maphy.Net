using System;

namespace Maphy.Physics
{
    /// <summary>
    /// Stable handle used by the no-GC core world. It avoids exposing object references
    /// from hot paths and lets the storage validate stale handles with a version.
    /// </summary>
    public readonly struct BodyHandle : IEquatable<BodyHandle>
    {
        internal readonly int index;
        internal readonly int version;

        internal BodyHandle(int index, int version)
        {
            this.index = index;
            this.version = version;
        }

        public bool IsValid => index >= 0 && version > 0;
        public int Index => index;
        public int Version => version;

        public static BodyHandle Invalid => new BodyHandle(-1, 0);

        public bool Equals(BodyHandle other)
        {
            return index == other.index && version == other.version;
        }

        public override bool Equals(object obj)
        {
            return obj is BodyHandle other && Equals(other);
        }

        public override int GetHashCode()
        {
            unchecked
            {
                return (index * 397) ^ version;
            }
        }

        public static bool operator ==(BodyHandle left, BodyHandle right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(BodyHandle left, BodyHandle right)
        {
            return !left.Equals(right);
        }
    }
}
