using System;
using System.Runtime.InteropServices;
using System.Runtime.CompilerServices;

namespace Maphy.Mathematics
{
    public static partial class math
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 fix4(fix3 xyz, fix w) { return new fix4(xyz, w); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 inverse(fix4 x) { return new fix4(inverse(x.x), inverse(x.y), inverse(x.z), inverse(x.w)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 max(fix4 x, fix4 y) { return new fix4(max(x.x, y.x), max(x.y, y.y), max(x.z, y.z), max(x.w, y.w)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 min(fix4 x, fix4 y) { return new fix4(min(x.x, y.x), min(x.y, y.y), min(x.z, y.z), min(x.w, y.w)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 abs(fix4 v) { return new fix4(abs(v.x), abs(v.y), abs(v.z), abs(v.w)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 sign(fix4 x) { return new fix4(sign(x.x), sign(x.y), sign(x.z), sign(x.w)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 floor(fix4 x) { return new fix4(floor(x.x), floor(x.y), floor(x.z), floor(x.w)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 ceil(fix4 x) { return new fix4(ceil(x.x), ceil(x.y), ceil(x.z), ceil(x.w)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 sqrt(fix4 x) { return new fix4(sqrt(x.x), sqrt(x.y), sqrt(x.z), sqrt(x.w)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 rsqrt(fix4 x) { return 1 / sqrt(x); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix dot(fix4 x, fix4 y) { return x.x * y.x + x.y * y.y + x.z * y.z + x.w * y.w; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix length(fix4 x) { return sqrt(dot(x, x)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix lengthsq(fix4 x) { return dot(x, x); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix distance(fix4 x, fix4 y) { return length(y - x); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix distancesq(fix4 x, fix4 y) { return lengthsq(y - x); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 normalize(fix4 x) { return rsqrt(dot(x, x)) * x; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int csum(fix4 x) { return x.x + x.y + x.z; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 round(fix4 x) { return new fix4(round(x.x), round(x.y), round(x.z), round(x.w)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 log(fix4 x) { return new fix4(log(x.x), log(x.y), log(x.z), log(x.w)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 rcp(fix4 x) { return 1 / x; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 radians(fix4 x) { return x * Deg2Rad; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 degrees(fix4 x) { return x * Rad2Deg; }
    }
}
