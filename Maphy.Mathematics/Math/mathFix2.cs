using System.Runtime.CompilerServices;

namespace Maphy.Mathematics
{
    public static partial class math
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix2 inverse(fix2 x) { return new fix2(inverse(x.x), inverse(x.y)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix2 max(fix2 x, fix2 y) { return new fix2(max(x.x, y.x), max(x.y, y.y)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix2 min(fix2 x, fix2 y) { return new fix2(min(x.x, y.x), min(x.y, y.y)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix2 abs(fix2 v) { return new fix2(abs(v.x), abs(v.y)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix2 sign(fix2 x) { return new fix2(sign(x.x), sign(x.y)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix2 floor(fix2 x) { return new fix2(floor(x.x), floor(x.y)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix2 ceil(fix2 x) { return new fix2(ceil(x.x), ceil(x.y)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix2 sqrt(fix2 x) { return new fix2(sqrt(x.x), sqrt(x.y)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix2 rsqrt(fix2 x) { return 1 / sqrt(x); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix dot(fix2 x, fix2 y) { return x.x * y.x + x.y * y.y; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix cross(fix2 x, fix2 y) { return x.x * y.y - x.y * y.x; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix length(fix2 x) { return sqrt(dot(x, x)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix lengthsq(fix2 x) { return dot(x, x); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix distance(fix2 x, fix2 y) { return length(y - x); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix distancesq(fix2 x, fix2 y) { return lengthsq(y - x); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix2 normalize(fix2 x) { return rsqrt(dot(x, x)) * x; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix csum(fix2 x) { return x.x + x.y; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix2 round(fix2 x) { return new fix2(round(x.x), round(x.y)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix2 log(fix2 x) { return new fix2(log(x.x), log(x.y)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix2 rcp(fix2 x) { return 1 / x; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix2 radians(fix2 x) { return x * Deg2Rad; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix2 degrees(fix2 x) { return x * Rad2Deg; }
    }
}
