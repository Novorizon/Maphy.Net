using System.Runtime.CompilerServices;

namespace Maphy.Mathematics
{
    public struct Random
    {
        public static readonly ulong ONE = 1 << fix.PRECISION;

        public uint state;

        /// Seed must be non-zero
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Random(uint seed)
        {
            state = seed;
            NextState();
        }

        /// Seed must be non-zero
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SetState(uint seed)
        {
            state = seed;
            NextState();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private uint NextState()
        {
            uint t = state;
            state ^= state << 13;
            state ^= state >> 17;
            state ^= state << 5;
            return t;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool NextBool() { return (NextState() & 1) == 1; }

        /// <summary>Returns value in range [-2147483647, 2147483647]</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int NextInt() { return (int)NextState() ^ -2147483648; }

        /// <summary>Returns value in range [0, max]</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int NextInt(int max) { return (int)((NextState() * (ulong)max) >> 32); }

        /// <summary>Returns value in range [min, max]</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int NextInt(int min, int max)
        {
            uint range = (uint)(max - min);
            return (int)(NextState() * (ulong)range >> 32) + min;
        }

        ///<summary>Returns value in range [0, 1]</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fix NextFix()
        {
            long r = (long)(NextState() * ONE >> 32);
            return fix.Raw(r);
        }

        /// <summary>Returns vector with all components in range [0, 1]</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fix2 NextFix2() { return new fix2(NextFix(), NextFix()); }

        /// <summary>Returns vector with all components in range [0, 1]</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fix3 NextFix3() { return new fix3(NextFix(), NextFix(), NextFix()); }

        /// <summary>Returns vector with all components in range [0, 1]</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fix4 NextFix4() { return new fix4(NextFix(), NextFix(), NextFix(), NextFix()); }


        /// Returns value in range [0, max]
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fix NextFix(fix max) { return NextFix() * max; }

        /// <summary>Returns vector with all components in range [0, max]</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fix2 NextFix2(fix2 max) { return NextFix2() * max; }

        /// <summary>Returns vector with all components in range [0, max]</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fix3 NextFix3(fix3 max) { return NextFix3() * max; }

        /// <summary>Returns vector with all components in range [0, max]</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fix4 NextFix4(fix4 max) { return NextFix4() * max; }

        /// <summary>Returns value in range [min, max]</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fix NextFix(fix min, fix max) { return NextFix() * (max - min) + min; }

        /// <summary>Returns vector with all components in range [min, max]</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fix2 NextFix2(fix2 min, fix2 max) { return NextFix2() * (max - min) + min; }

        /// <summary>Returns vector with all components in range [min, max]</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fix3 NextFix3(fix3 min, fix3 max) { return NextFix3() * (max - min) + min; }

        /// <summary>Returns vector with all components in range [min, max]</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fix4 NextFix4(fix4 min, fix4 max) { return NextFix4() * (max - min) + min; }

        /// <summary>Returns a fix2 vector </summary>
        public fix2 NextFix2Direction()
        {
            fix angle = NextFix() * math.PI * fix._2;
            math.sincos(angle, out fix sin, out fix cos);
            return new fix2(sin, cos);
        }

        /// <summary>Returns a fix3 vector </summary>
        public fix3 NextFix3Direction()
        {
            fix z = NextFix(fix._2) - fix._1;
            fix r = math.sqrt(math.max(fix._1 - z * z, fix._0));
            fix angle = NextFix(math.PI2);
            math.sincos(angle, out fix sin, out fix cos);
            return new fix3(cos * r, sin * r, z);
        }
    }
}