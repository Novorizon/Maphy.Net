
using System.Runtime.CompilerServices;
using System;
using Maphy.Mathematics;

namespace Maphy.Physics
{
    public struct Bounds : IEquatable<Bounds>
    {
        private fix3 center;
        private fix3 extents;

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
            set { extents = value * 0.5f; }
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
            set { SetMinMax(value, max); }
        }

        public fix3 max
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return center + extents; }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            set { SetMinMax(min, value); }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Bounds(fix3 center, fix3 size)
        {
            this.center = center;
            extents = size * 0.5f;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SetMinMax(fix3 min, fix3 max)
        {
            extents = (max - min) * fix._0_5;
            center = min + extents;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override int GetHashCode()
        {
            return center.GetHashCode() ^ (extents.GetHashCode() << 2);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override bool Equals(object other)
        {
            if (!(other is Bounds))
            {
                return false;
            }

            return Equals((Bounds)other);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(Bounds other)
        {
            return center.Equals(other.center) && extents.Equals(other.extents);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator ==(Bounds lhs, Bounds rhs)
        {
            return lhs.center == rhs.center && lhs.extents == rhs.extents;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator !=(Bounds lhs, Bounds rhs)
        {
            return !(lhs == rhs);
        }

        public void Expand(float amount)
        {
            amount *= 0.5f;
            extents += new fix3(amount, amount, amount);
        }

    }

}