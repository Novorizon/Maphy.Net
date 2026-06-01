using System;
using System.Runtime.CompilerServices;

namespace Maphy.Mathematics
{
    internal static class fixWide
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix FromInteger(long value)
        {
#if NET7_0_OR_GREATER
            return FromCheckedRaw((Int128)value << fix.PRECISION);
#else
            if (value > (long.MaxValue >> fix.PRECISION) || value < ((long.MinValue + 1) >> fix.PRECISION))
            {
                return fix.NaN;
            }

            return fix.Raw(value << fix.PRECISION);
#endif
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix Add(fix a, fix b)
        {
            if (fix.IsNaN(a) || fix.IsNaN(b))
            {
                return fix.NaN;
            }

#if NET7_0_OR_GREATER
            return FromCheckedRaw((Int128)a.value + b.value);
#else
            long x = a.value;
            long y = b.value;
            long sum = x + y;
            if (((~(x ^ y) & (x ^ sum)) & long.MinValue) != 0 || sum == long.MinValue)
            {
                return fix.NaN;
            }

            return fix.Raw(sum);
#endif
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix Sub(fix a, fix b)
        {
            if (fix.IsNaN(a) || fix.IsNaN(b))
            {
                return fix.NaN;
            }

#if NET7_0_OR_GREATER
            return FromCheckedRaw((Int128)a.value - b.value);
#else
            long x = a.value;
            long y = b.value;
            long diff = x - y;
            if ((((x ^ y) & (x ^ diff)) & long.MinValue) != 0 || diff == long.MinValue)
            {
                return fix.NaN;
            }

            return fix.Raw(diff);
#endif
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix Mul(fix a, fix b)
        {
            if (fix.IsNaN(a) || fix.IsNaN(b))
            {
                return fix.NaN;
            }

#if NET7_0_OR_GREATER
            return FromCheckedRaw(((Int128)a.value * b.value) >> fix.PRECISION);
#else
            return fix.Raw((a.value * b.value) >> fix.PRECISION);
#endif
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix Div(fix a, fix b)
        {
            if (fix.IsNaN(a) || fix.IsNaN(b) || b.value == 0)
            {
                return fix.NaN;
            }

#if NET7_0_OR_GREATER
            return FromCheckedRaw(((Int128)a.value << fix.PRECISION) / b.value);
#else
            return fix.Raw((a.value << fix.PRECISION) / b.value);
#endif
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix ShiftLeft(fix value, int amount)
        {
            if (fix.IsNaN(value))
            {
                return fix.NaN;
            }

#if NET7_0_OR_GREATER
            return FromCheckedRaw((Int128)value.value << amount);
#else
            return fix.Raw(value.value << amount);
#endif
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix Sqrt(fix value)
        {
            if (value.value < 0)
            {
                return fix.NaN;
            }

            if (value.value == 0)
            {
                return fix.Zero;
            }

#if NET7_0_OR_GREATER
            Int128 scaled = (Int128)value.value << fix.PRECISION;
            Int128 current = value > fix.One ? value.value : fix.ONE;
            while (true)
            {
                Int128 next = (current + scaled / current) >> 1;
                if (next >= current)
                {
                    return current > long.MaxValue ? fix.NaN : fix.Raw((long)current);
                }

                current = next;
            }
#else
            fix x = value > fix.One ? value : fix.One;
            long scaled = value.value << fix.PRECISION;

            while (true)
            {
                long current = x.value;
                long next = (current + scaled / current) >> 1;
                if (next >= current)
                {
                    return x;
                }

                x.value = next;
            }
#endif
        }

#if NET7_0_OR_GREATER
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static fix FromCheckedRaw(Int128 raw)
        {
            if (raw <= long.MinValue || raw > long.MaxValue)
            {
                return fix.NaN;
            }

            return fix.Raw((long)raw);
        }
#endif
    }
}
