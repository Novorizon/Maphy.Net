
namespace Maphy.Mathematics
{
    public static partial class trigonometric
    {
        public const int PRECISION = fix.PRECISION;
        public const int SHIFT = PRECISION - 9;
        public const long ONE = 1 << PRECISION;

        public static long sin(long value)
        {
            int sign = (int)(value >> 63) | 1;
            value = (value + (value >> 63)) ^ (value >> 63);

            int index = (int)(value >> SHIFT);
            long fraction = (value - (index << SHIFT)) << 9;
            int a = SinLut[index];
            int b = SinLut[index + 1];
            long v2 = a + (((b - a) * fraction) >> PRECISION);
            return v2 * sign;
        }

        public static long cos(long value)
        {
            value = (value + (value >> 63)) ^ (value >> 63);

            value += fix._0_25.value;

            if (value >= ONE)
            {
                value -= ONE;
            }

            int index = (int)(value >> SHIFT);
            long fraction = (value - (index << SHIFT)) << 9;
            int a = SinLut[index];
            int b = SinLut[index + 1];
            long v2 = a + (((b - a) * fraction) >> PRECISION);
            return v2;
        }


        public static long tan(long value)
        {
            int sign = (int)(value >> 63) | 1;
            value = (value + (value >> 63)) ^ (value >> 63);

            int index = (int)(value >> SHIFT);
            long fraction = (value - (index << SHIFT)) << 9;
            int a = TanLut[index];
            int b = TanLut[index + 1];
            long v2 = a + (((b - a) * fraction) >> PRECISION);
            return v2 * sign;
        }

        public static void sin_cos(long value, out long sin, out long cos)
        {
            int sign = (int)(value >> 63) | 1;
            value = (value + (value >> 63)) ^ (value >> 63);

            int index = (int)(value >> SHIFT);
            int doubleIndex = index * 2;
            long fractions = (value - (index << SHIFT)) << 9;

            int sinA = SinCosLut[doubleIndex];
            int cosA = SinCosLut[doubleIndex + 1];
            int sinB = SinCosLut[doubleIndex + 2];
            int cosB = SinCosLut[doubleIndex + 3];

            sin = (sinA + (((sinB - sinA) * fractions) >> PRECISION)) * sign;
            cos = cosA + (((cosB - cosA) * fractions) >> PRECISION);
        }

        public static void sin_cos_tan(long value, out long sin, out long cos, out long tan)
        {
            int sign = (int)(value >> 63) | 1;
            value = (value + (value >> 63)) ^ (value >> 63);

            int index = (int)(value >> SHIFT);
            int doubleIndex = index * 2;
            long fractions = (value - (index << SHIFT)) << 9;

            int sinA = SinCosLut[doubleIndex];
            int cosA = SinCosLut[doubleIndex + 1];
            int sinB = SinCosLut[doubleIndex + 2];
            int cosB = SinCosLut[doubleIndex + 3];

            sin = (sinA + (((sinB - sinA) * fractions) >> PRECISION)) * sign;
            cos = cosA + (((cosB - cosA) * fractions) >> PRECISION);
            tan = (sin << PRECISION) / cos;
        }

        public static long acos(long value)
        {
            int index = (int)(value >> SHIFT);
            var fraction = (value - (index << SHIFT)) << 9;
            int a = AcosLut[index];
            int b = AcosLut[index + 1];
            long v2 = a + (((b - a) * fraction) >> PRECISION);
            return v2;
        }

        public static long asin(long value)
        {
            int index = (int)(value >> SHIFT);
            long fraction = (value - (index << SHIFT)) << 9;
            int a = AsinLut[index];
            int b = AsinLut[index + 1];
            long v2 = a + (((b - a) * fraction) >> PRECISION);
            return v2;
        }
    }
}