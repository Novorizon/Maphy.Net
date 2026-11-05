using System;
using System.Runtime.CompilerServices;

namespace Maphy.Mathematics
{
    internal static class fixWide
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix FromInteger(long value)
        {
            long limit = long.MaxValue >> fix.PRECISION;
            if (value > limit || value < -limit)
            {
                return fix.NaN;
            }

            return fix.Raw(value << fix.PRECISION);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix Add(fix a, fix b)
        {
            if (fix.IsNaN(a) || fix.IsNaN(b))
            {
                return fix.NaN;
            }

            long x = a.value;
            long y = b.value;
            long sum = unchecked(x + y);
            if (((~(x ^ y) & (x ^ sum)) & long.MinValue) != 0 || sum == long.MinValue)
            {
                return fix.NaN;
            }

            return fix.Raw(sum);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix Sub(fix a, fix b)
        {
            if (fix.IsNaN(a) || fix.IsNaN(b))
            {
                return fix.NaN;
            }

            long x = a.value;
            long y = b.value;
            long diff = unchecked(x - y);
            if ((((x ^ y) & (x ^ diff)) & long.MinValue) != 0 || diff == long.MinValue)
            {
                return fix.NaN;
            }

            return fix.Raw(diff);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix Mul(fix a, fix b)
        {
            if (fix.IsNaN(a) || fix.IsNaN(b))
            {
                return fix.NaN;
            }

#if NET7_0_OR_GREATER && !MAPHY_FORCE_PORTABLE
            return FromCheckedRaw(((Int128)a.value * b.value) >> fix.PRECISION);
#else
            bool negative = (a.value < 0) != (b.value < 0);
            MultiplyUnsigned(Magnitude(a.value), Magnitude(b.value), out ulong high, out ulong low);
            // After removing the fractional bits, the result must fit in 63 bits.
            if (high >= (1UL << (fix.PRECISION - 1)))
            {
                return fix.NaN;
            }

            ulong magnitude = (high << (64 - fix.PRECISION)) | (low >> fix.PRECISION);
            // Match the arithmetic right shift in the Int128 path: negative
            // products with discarded bits round down, rather than toward zero.
            if (negative && (low & (fix.ONE - 1UL)) != 0)
            {
                magnitude++;
            }

            return FromMagnitude(magnitude, negative);
#endif
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix Div(fix a, fix b)
        {
            if (fix.IsNaN(a) || fix.IsNaN(b) || b.value == 0)
            {
                return fix.NaN;
            }

#if NET7_0_OR_GREATER && !MAPHY_FORCE_PORTABLE
            return FromCheckedRaw(((Int128)a.value << fix.PRECISION) / b.value);
#else
            ulong numerator = Magnitude(a.value);
            ulong denominator = Magnitude(b.value);
            ulong quotient = numerator / denominator;
            if (quotient > (ulong)(long.MaxValue >> fix.PRECISION))
            {
                return fix.NaN;
            }

            ulong remainder = numerator % denominator;
            quotient <<= fix.PRECISION;
            // Generate the fractional bits without shifting the full numerator.
            // remainder < denominator <= long.MaxValue, so doubling fits ulong.
            for (int bit = fix.PRECISION - 1; bit >= 0; bit--)
            {
                remainder <<= 1;
                if (remainder >= denominator)
                {
                    remainder -= denominator;
                    quotient |= 1UL << bit;
                }
            }

            return FromMagnitude(quotient, (a.value < 0) != (b.value < 0));
#endif
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fix ShiftLeft(fix value, int amount)
        {
            if (fix.IsNaN(value))
            {
                return fix.NaN;
            }

            // Match the count masking of the 64-bit right-shift operator.
            int shift = amount & 63;
            long limit = long.MaxValue >> shift;
            if (value.value > limit || value.value < -limit)
            {
                return fix.NaN;
            }

            return fix.Raw(unchecked(value.value << shift));
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

#if NET7_0_OR_GREATER && !MAPHY_FORCE_PORTABLE
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
            // Digit-by-digit integer sqrt of the 79-bit scaled input. The root
            // and remainder fit in ulong, even when the input is fix.Max.
            ulong high = (ulong)value.value >> (64 - fix.PRECISION);
            ulong low = (ulong)value.value << fix.PRECISION;
            ulong root = 0;
            ulong remainder = 0;
            for (int pair = (64 + fix.PRECISION) / 2 - 1; pair >= 0; pair--)
            {
                int shift = pair * 2;
                ulong bits = shift >= 64 ? high >> (shift - 64) : low >> shift;
                remainder = (remainder << 2) | (bits & 3UL);
                root <<= 1;
                ulong trial = (root << 1) | 1UL;
                if (remainder >= trial)
                {
                    remainder -= trial;
                    root++;
                }
            }

            return fix.Raw((long)root);
#endif
        }

#if NET7_0_OR_GREATER && !MAPHY_FORCE_PORTABLE
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static fix FromCheckedRaw(Int128 raw)
        {
            if (raw <= long.MinValue || raw > long.MaxValue)
            {
                return fix.NaN;
            }

            return fix.Raw((long)raw);
        }
#else
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static ulong Magnitude(long value)
        {
            return (ulong)(value < 0 ? -value : value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static fix FromMagnitude(ulong magnitude, bool negative)
        {
            if (magnitude > long.MaxValue)
            {
                return fix.NaN;
            }

            return fix.Raw(negative ? -(long)magnitude : (long)magnitude);
        }

        private static void MultiplyUnsigned(ulong x, ulong y, out ulong high, out ulong low)
        {
            ulong xLow = unchecked((uint)x);
            ulong xHigh = x >> 32;
            ulong yLow = unchecked((uint)y);
            ulong yHigh = y >> 32;
            ulong lowProduct = xLow * yLow;
            ulong middle = xHigh * yLow + (lowProduct >> 32);
            ulong carry = middle >> 32;
            middle = unchecked((uint)middle) + xLow * yHigh;
            high = xHigh * yHigh + carry + (middle >> 32);
            low = unchecked((middle << 32) | (uint)lowProduct);
        }
#endif
    }
}

