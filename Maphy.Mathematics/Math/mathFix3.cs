using System;
using System.Runtime.InteropServices;
using System.Runtime.CompilerServices;

namespace Maphy.Mathematics
{
    public static partial class math
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 inverse(fix3 x) { return new fix3(inverse(x.x), inverse(x.y), inverse(x.z)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 max(fix3 x, fix3 y) { return new fix3(max(x.x, y.x), max(x.y, y.y), max(x.z, y.z)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 min(fix3 x, fix3 y) { return new fix3(min(x.x, y.x), min(x.y, y.y), min(x.z, y.z)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 abs(fix3 v) { return new fix3(abs(v.x), abs(v.y), abs(v.z)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 sign(fix3 x) { return new fix3(sign(x.x), sign(x.y), sign(x.z)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 floor(fix3 x) { return new fix3(floor(x.x), floor(x.y), floor(x.z)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 ceil(fix3 x) { return new fix3(ceil(x.x), ceil(x.y), ceil(x.z)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 sqrt(fix3 x) { return new fix3(sqrt(x.x), sqrt(x.y), sqrt(x.z)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 rsqrt(fix3 x) { return 1 / sqrt(x); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix dot(fix3 x, fix3 y) { return x.x * y.x + x.y * y.y + x.z * y.z; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 cross(fix3 x, fix3 y) { return (x * y.yzx - x.yzx * y).yzx; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix length(fix3 x) { return sqrt(dot(x, x)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix lengthsq(fix3 x) { return dot(x, x); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix distance(fix3 x, fix3 y) { return length(y - x); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix distancesq(fix3 x, fix3 y) { return lengthsq(y - x); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 normalize(fix3 x) { return rsqrt(dot(x, x)) * x; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix csum(fix3 x) { return x.x + x.y + x.z; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 round(fix3 x) { return new fix3(round(x.x), round(x.y), round(x.z)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 log(fix3 x) { return new fix3(log(x.x), log(x.y), log(x.z)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 rcp(fix3 x) { return 1 / x; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix lerp(fix x, fix y, fix s) { return x + s * (y - x); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix2 lerp(fix2 x, fix2 y, fix s) { return x + s * (y - x); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 lerp(fix3 x, fix3 y, fix s) { return x + s * (y - x); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 lerp(fix4 x, fix4 y, fix s) { return x + s * (y - x); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 radians(fix3 x) { return x * Deg2Rad; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 degrees(fix3 x) { return x * Rad2Deg; }
    }
}
