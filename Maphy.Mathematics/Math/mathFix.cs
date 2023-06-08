using System;
using System.Runtime.InteropServices;
using System.Runtime.CompilerServices;

namespace Maphy.Mathematics
{
    public static partial class math
    {
        /// 相反数
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix inverse(fix x) { return x.value == fix.Min ? fix.Max : -x; }

        /// 最值
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix max(fix x, fix y) { return x > y ? x : y; }

        /// 最值
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix min(fix x, fix y) { return x > y ? y : x; }


        /// 绝对值  http://www.strchr.com/optimized_abs_function
        /// value.value + mask) ^ mask 或者 (value.value + mask) ^ mask 
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix abs(fix x) { return fix.Raw((x.value + (x.value >> 63)) ^ (x.value >> 63)); }

        /// Returns 0 if the value is positive or 0  ,  -1 if it is negative.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix sign(fix x) { return (int)(x >> 63); }

        /// <summary>Returns the result of truncating a float value to an integral float value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix trunc(fix x)
        {
            x.value = x.value >> fix.PRECISION << fix.PRECISION;
            if (sign(x) != 0)
                x.value = x.value + (1 << fix.PRECISION);
            return x;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix frac(fix x) { return abs(x - trunc(x)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix floor(fix x) { x.value = x.value >> fix.PRECISION << fix.PRECISION; return x; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix ceil(fix value)
        {
            if ((value.value << (fix.BITS - fix.PRECISION)) == 0)
                return value;

            value.value = value.value >> fix.PRECISION << fix.PRECISION;
            value += fix.One;
            return value;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix round(fix x)
        {
            fix fractional = frac(x);
            fix truncation = trunc(x);
            if (fractional.value < (1 << (fix.PRECISION - 2)))
            {
                return truncation;
            }
            else
            {
                int sign = (int)(x >> 63);
                if (sign == 0)
                {
                    x.value = truncation.value + fix.ONE;
                    return x;
                }
                else
                {
                    x.value = truncation.value - fix.ONE;
                    return x;
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix add(fix a, fix b)
        {
            long x = a.value;
            long y = b.value;
            long sum = x + y;
            if (((~(x ^ y) & (x ^ sum)) & fix.Min) != 0)
            {
                return fix.NaN;
            }
            return fix.Raw(sum);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix sub(fix x, fix y)
        {
            var xl = x.value;
            var yl = y.value;
            var diff = xl - yl;
            if ((((xl ^ yl) & (xl ^ diff)) & fix.Min) != 0)
            {
                return fix.NaN;
            }
            return fix.Raw(diff);
        }

        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //static long AddOverflowHelper(long x, long y, ref bool overflow)
        //{
        //    var sum = x + y;
        //    // x + y overflows if sign(x) ^ sign(y) != sign(sum)
        //    overflow |= ((x ^ y ^ sum) & fix.MinValue) != 0;
        //    return sum;
        //}

        //TODO
        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //public static fix mul(fix x, fix y)
        //{
        //    return new fix(y);           
        //}

        //TODO
        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //public static fix div(fix x, fix y)
        //{

        //    return new fix(y);
        //}

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix mod(fix x, fix y) { x.value = x.value == fix.Min & y.value == -1 ? 0 : x.value % y.value; return x; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static fix pow(fix x, int n)
        {
            if (x == 0)
                return 0;

            if (n == int.MinValue)
            {
                return x == 1 || x == -1 ? 1 : 0;
            }
            fix res = 1;

            if (n < 0)
            {
                n = -n;
                x = 1 / x;
            }

            while (n > 0)
            {
                if ((n & 1) > 0)
                    res *= x;
                x *= x;
                n >>= 1;
            }
            return res;
        }


        /// Returns the logarithm of a specified number-y in a specified base-x.
        /// 以x为底的y的对数
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix log(fix x, fix y) { return ln(y) / ln(x); }

        /// <summary>Returns the natural logarithm of a float value.</summary>
        /// 以2为底，x的对数
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix log(fix x) { return ln(x) / Ln2; }

        /// Returns the base-2 logarithm of a specified number.
        /// 以2为底，x的对数
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static fix log2(fix x) { return ln(x) / Ln2; }

        /// 以10为底，x的对数
        public static fix log10(fix x) { return ln(x) / Ln10; }

        /// 以自然对数e为底，x的对数
        /// lnx=ln(1+y)/(1-y)=2y(1/1+1/3*y*y+1/5*y*y*y*y+......)
        public static fix ln(fix x)
        {
            if (x <= 0)
                return fix.NaN;

            int k = 0, l = 0;
            for (; x > 1; k++)
                x /= 10;

            for (; x <= 0.1m; k--)
                x *= 10;        // ( 0.1, 1 ]

            for (; x < 0.9047m; l--)
                x *= 1.2217m; // [ 0.9047, 1.10527199 )

            return k * Ln10 + l * Lnr + Logarithm((x - 1) / (x + 1));
        }

        internal static fix Logarithm(fix x)
        {
            // y in ( -0.05-, 0.05+ ), return ln((1+y)/(1-y))
            fix v = 1, x2 = x * x, t = x2, z = t / 3;
            for (int i = 3; z != 0; z = (t *= x2) / (i += 2))
                v += z;
            return v * x * 2;
        }

        /// 牛顿迭代法 泰勒展开
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix sqrt(fix a)
        {
            if (a.value < 0)
                return fix.NaN;

            if (a.value == 0)
                return fix.Zero;

            fix x = a;
            long b = (x.value >> 2) + 1L;
            x.value = (b + (a.value << fix.PRECISION) / x.value) >> 1;
            while (x.value < b)
            {
                b = x.value;
                x.value = (x.value + (a.value << fix.PRECISION) / x.value) >> 1;
            }
            return x;
        }

        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //public static fix sqrt(fix a)
        //{
        //    if (a.value < 0) return fix.NaN;
        //    if (a.value == 0) return fix.Zero;

        //    fix x = a;
        //    for (int i = 0; i < 20; i++) { x.value = (x.value + (a.value << fix.PRECISION) / x.value) >> 1; }
        //    return x;

        //    //关系模式在 C# 8.0 中不可用
        //    //switch (a.value)
        //    //{
        //    //    case < 0:
        //    //        return fix.NaN;
        //    //    case > 0:
        //    //        fix x = a;
        //    //        for (int i = 0; i < 20; i++) { x.value = (x.value + (a.value << fix.PRECISION) / x.value) >> 1; }
        //    //        return x;
        //    //    default:
        //    //        return fix.Zero;

        //    //}
        //}


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix rsqrt(fix x) { return 1 / sqrt(x); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix distance(fix x, fix y) { return abs(y - x); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix distancesq(fix x, fix y) { return (y - x) * (y - x); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix length(fix x) { return abs(x); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix lengthsq(fix x) { return x * x; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix dot(fix x, fix y) { return x * y; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix rcp(fix x) { return 1 / x; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix radians(fix x) { return x * Deg2Rad; }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix degrees(fix x) { return x * Rad2Deg; }

    }
}
