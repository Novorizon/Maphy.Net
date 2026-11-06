using System;
using System.Runtime.InteropServices;

namespace Maphy.Mathematics
{
    internal static class fixConvert
    {
        [StructLayout(LayoutKind.Explicit)]
        private struct SingleBits
        {
            [FieldOffset(0)] public float number;
            [FieldOffset(0)] public uint bits;
        }

        public static fix FromSingle(float value)
        {
            uint bits = new SingleBits { number = value }.bits;
            int exponent = (int)((bits >> 23) & 255);
            if (exponent == 255) return fix.NaN;
            ulong significand = bits & 0x7FFFFF;
            if (exponent != 0) significand |= 1UL << 23;
            return FromBinary(significand, exponent == 0 ? -133 : exponent - 134, (bits >> 31) != 0, true);
        }

        public static fix FromDouble(double value)
        {
            ulong bits = unchecked((ulong)BitConverter.DoubleToInt64Bits(value));
            int exponent = (int)((bits >> 52) & 2047);
            if (exponent == 2047) return fix.NaN;
            ulong significand = bits & 0xFFFFFFFFFFFFFUL;
            if (exponent != 0) significand |= 1UL << 52;
            return FromBinary(significand, exponent == 0 ? -1058 : exponent - 1059, (bits >> 63) != 0, false);
        }

        public static fix FromDecimal(decimal value)
        {
            const decimal limit = 140737488355328m;
            if (value >= limit || value <= -limit) return fix.NaN;
            decimal integral = decimal.Truncate(value);
            long raw = (long)integral * fix.ONE;
            decimal fraction = value - integral;
            bool negative = fraction < 0;
            if (negative) fraction = -fraction;
            // Extract fractional bits while the decimal magnitude stays below 2.
            // Scaling the full decimal can round across a raw integer boundary.
            long fractionalRaw = 0;
            for (int bit = 0; bit < fix.PRECISION; bit++)
            {
                fraction *= 2;
                fractionalRaw <<= 1;
                if (fraction >= 1) { fractionalRaw++; fraction -= 1; }
            }
            return fix.Raw(raw + (negative ? -fractionalRaw : fractionalRaw));
        }

        private static fix FromBinary(ulong significand, int shift, bool negative, bool round)
        {
            if (significand == 0) return fix.Zero;
            ulong magnitude;
            if (shift >= 0)
            {
                if (shift >= 63 || significand > ((ulong)long.MaxValue >> shift)) return fix.NaN;
                magnitude = significand << shift;
            }
            else
            {
                int right = -shift;
                if (right >= 64) return fix.Zero;
                magnitude = significand >> right;
                if (round && (significand & (1UL << (right - 1))) != 0) magnitude++;
                if (magnitude > long.MaxValue) return fix.NaN;
            }
            return fix.Raw(negative ? -(long)magnitude : (long)magnitude);
        }
    }
}
