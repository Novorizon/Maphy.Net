
using System.Runtime.CompilerServices;
using System;
using Maphy.Mathematics;

namespace Maphy.Physics
{
    public struct FatAABB : IEquatable<FatAABB>
    {
        private fix3 center;
        private fix3 extents;
        private int expansion;

        public fix3 Center
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return center; }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            set { center = value; }
        }

        public fix3 Size
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return extents * 2f; }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            set { extents = value * fix._0_5; }
        }

        public fix3 Extents
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return extents; }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            set { extents = value; }
        }

        public fix3 min
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return center - extents; }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            set { min = value; }
        }

        public fix3 max
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return center + extents; }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            set { max = value; }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public FatAABB(fix3 center, fix3 size, int expansion=1)
        {
            this.center = center;
            extents = size * fix._0_5;
            this.expansion = expansion;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override int GetHashCode()
        {
            return center.GetHashCode() ^ (extents.GetHashCode() << 2);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override bool Equals(object other)
        {
            if (!(other is FatAABB))
            {
                return false;
            }

            return Equals((FatAABB)other);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(FatAABB other)
        {
            return center.Equals(other.center) && extents.Equals(other.extents);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator ==(FatAABB lhs, FatAABB rhs)
        {
            return lhs.center == rhs.center && lhs.extents == rhs.extents;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator !=(FatAABB lhs, FatAABB rhs)
        {
            return !(lhs == rhs);
        }

        public void Expand(float amount)
        {
            amount *= fix._0_5;
            extents += new fix3(amount, amount, amount);
        }

    }

}