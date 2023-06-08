using System;
using System.Runtime.InteropServices;
using System.Runtime.CompilerServices;

namespace Maphy.Mathematics
{
    /// angle in radians
    public static partial class math
    {
        private static readonly fix atan2Number1 = fix.Raw(-883);
        private static readonly fix atan2Number2 = fix.Raw(3767);
        private static readonly fix atan2Number3 = fix.Raw(7945);
        private static readonly fix atan2Number4 = fix.Raw(12821);
        private static readonly fix atan2Number5 = fix.Raw(21822);
        private static readonly fix atan2Number6 = fix.Raw(65536);
        private static readonly fix atan2Number7 = fix.Raw(102943);
        private static readonly fix atan2Number8 = fix.Raw(205887);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix sin(fix x) { x.value %= PI2.value; x *= PI2rcp; x.value = trigonometric.sin(x.value); return x; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix2 sin(fix2 x) { return new fix2(sin(x.x), sin(x.y)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 sin(fix3 x) { return new fix3(sin(x.x), sin(x.y), sin(x.z)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 sin(fix4 x) { return new fix4(sin(x.x), sin(x.y), sin(x.z), sin(x.w)); }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix cos(fix x) { x.value %= PI2.value; x *= PI2rcp; x.value = trigonometric.cos(x.value); return x; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix2 cos(fix2 x) { return new fix2(cos(x.x), cos(x.y)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 cos(fix3 x) { return new fix3(cos(x.x), cos(x.y), cos(x.z)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 cos(fix4 x) { return new fix4(cos(x.x), cos(x.y), cos(x.z), cos(x.w)); }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void sincos(fix x, out fix s, out fix c) { s = sin(x); c = cos(x); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void sincos(fix2 x, out fix2 s, out fix2 c) { s = sin(x); c = cos(x); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void sincos(fix3 x, out fix3 s, out fix3 c) { s = sin(x); c = cos(x); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void sincos(fix4 x, out fix4 s, out fix4 c) { s = sin(x); c = cos(x); }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix tan(fix x) { x.value %= PI2.value; x *= PI2rcp; x.value = trigonometric.tan(x.value); return x; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix2 tan(fix2 x) { return new fix2(tan(x.x), tan(x.y)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 tan(fix3 x) { return new fix3(tan(x.x), tan(x.y), tan(x.z)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 tan(fix4 x) { return new fix4(tan(x.x), tan(x.y), tan(x.z), tan(x.w)); }

        /// Sin [-1, 1]
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix asin(fix x) { x.value += trigonometric.ONE; x *= fix._0_5; x.value = trigonometric.asin(x.value); return x; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix2 asin(fix2 x) { return new fix2(asin(x.x), asin(x.y)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 asin(fix3 x) { return new fix3(asin(x.x), asin(x.y), asin(x.z)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 asin(fix4 x) { return new fix4(asin(x.x), asin(x.y), asin(x.z), asin(x.w)); }

        /// Cos [-1, 1]
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix acos(fix x) { x.value += fix.ONE; x *= fix._0_5; x.value = trigonometric.acos(x.value); return x; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix2 acos(fix2 x) { return new fix2(acos(x.x), acos(x.y)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 acos(fix3 x) { return new fix3(acos(x.x), acos(x.y), acos(x.z)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 acos(fix4 x) { return new fix4(acos(x.x), acos(x.y), acos(x.z), acos(x.w)); }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix atan(fix x) { return atan2(x, fix._1); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix2 atan(fix2 x) { return new fix2(atan(x.x), atan(x.y)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 atan(fix3 x) { return new fix3(atan(x.x), atan(x.y), atan(x.z)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 atan(fix4 x) { return new fix4(atan(x.x), atan(x.y), atan(x.z), atan(x.w)); }

        /// "x"Denominator  "y"Numerator 
        public static fix atan2(fix y, fix x)
        {
            var absX = abs(x);
            var absY = abs(y);
            var t3 = absX;
            var t1 = absY;
            var t0 = max(t3, t1);
            t1 = min(t3, t1);
            t3 = fix._1 / t0;
            t3 = t1 * t3;
            var t4 = t3 * t3;
            t0 = atan2Number1;
            t0 = t0 * t4 + atan2Number2;
            t0 = t0 * t4 - atan2Number3;
            t0 = t0 * t4 + atan2Number4;
            t0 = t0 * t4 - atan2Number5;
            t0 = t0 * t4 + atan2Number6;
            t3 = t0 * t3;
            t3 = absY > absX ? atan2Number7 - t3 : t3;
            t3 = x < fix._0 ? atan2Number8 - t3 : t3;
            t3 = y < fix._0 ? -t3 : t3;
            return t3;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix2 atan2(fix2 y, fix2 x) { return new fix2(atan2(y.x, x.x), atan2(y.y, x.y)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix3 atan2(fix3 y, fix3 x) { return new fix3(atan2(y.x, x.x), atan2(y.y, x.y), atan2(y.z, x.z)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix4 atan2(fix4 y, fix4 x) { return new fix4(atan2(y.x, x.x), atan2(y.y, x.y), atan2(y.z, x.z), atan2(y.w, x.w)); }


        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //public static fix sine(fix x)
        //{
        //    fix a = x;
        //    fix sin = x;
        //    fix t = 0.0001;

        //    fix i = 1, negation = 1;//取反
        //    fix sum;
        //    fix index = x;//指数
        //    long Factorial = 1;//阶乘
        //    double TaylorExpansion = x;//泰勒展开式求和

        //    while (abs(a) > t)
        //    {
        //        Factorial = Factorial * (i + 1) * (i + 2);//求阶乘
        //        index *= x * x;//求num2的次方
        //        negation = -negation;//每次循环取反
        //        sum = index / Factorial * negation;
        //        TaylorExpansion += sum;
        //        i += 2;
        //    }
        //    return sin;
        //}

        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //public static fix cosine(fix x)
        //{
        //    fix i = 0;
        //    fix a = x;
        //    fix cos = 1;
        //    fix tl = 1;
        //    fix fz = 1;
        //    fix t = 0.00001;

        //    while (abs(t) > t)
        //    {
        //        i = i + 2;
        //        tl = (i - 1) * i * tl;
        //        fz = x * x * fz * (-1);
        //        a = fz / (tl);
        //        cos = cos + a;
        //    }

        //    return cos;
        //}

        //public static fix acosine(fix x)
        //{
        //    if (x < -fix.one || x > fix.one)
        //        return fix.NaN;

        //    return x;
        //}


        //public static fix fastcos(fix x)
        //{
        //    //var xl = x.value;
        //    //var rawAngle = xl + (xl > 0 ? -PI.value - PI_OVER_2 : PI_OVER_2);
        //    //return sin(new fix(rawAngle));
        //    return x;
        //}

        //public static fix arctan(fix z)
        //{
        //    return z;
        //}

        //public static fix arctan2(fix y, fix x)
        //{
        //    return x;
        //}
    }
}
