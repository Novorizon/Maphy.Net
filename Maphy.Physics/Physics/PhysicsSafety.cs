using Maphy.Mathematics;

namespace Maphy.Physics
{
    public static class PhysicsSafety
    {
        public static bool IsFinite(fix value)
        {
            return value != fix.NaN;
        }

        public static bool IsFinite(fix3 value)
        {
            return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
        }

        public static fix Sanitize(fix value)
        {
            return IsFinite(value) ? value : fix.Zero;
        }

        public static fix3 Sanitize(fix3 value)
        {
            return new fix3(Sanitize(value.x), Sanitize(value.y), Sanitize(value.z));
        }

        public static fix3 ClampVectorMagnitude(fix3 value, fix maxMagnitude)
        {
            value = Sanitize(value);
            if (maxMagnitude <= fix.Zero)
            {
                return value;
            }

            value = new fix3(
                Clamp(value.x, -maxMagnitude, maxMagnitude),
                Clamp(value.y, -maxMagnitude, maxMagnitude),
                Clamp(value.z, -maxMagnitude, maxMagnitude));

            fix maxMagnitudeSq = maxMagnitude * maxMagnitude;
            fix lengthSq = math.lengthsq(value);
            if (lengthSq <= maxMagnitudeSq || lengthSq <= math.Epsilon)
            {
                return value;
            }

            return value * (maxMagnitude / math.sqrt(lengthSq));
        }

        public static fix SafeSqrt(fix value)
        {
            value = Sanitize(value);
            return value <= fix.Zero ? fix.Zero : math.sqrt(value);
        }

        public static fix SafeDiv(fix numerator, fix denominator, fix fallback = default)
        {
            numerator = Sanitize(numerator);
            denominator = Sanitize(denominator);
            if (math.abs(denominator) <= math.Epsilon)
            {
                return fallback;
            }

            fix result = numerator / denominator;
            return IsFinite(result) ? result : fallback;
        }

        public static fix3 SafeNormalize(fix3 value, fix3 fallback)
        {
            value = Sanitize(value);
            fix lengthSq = math.lengthsq(value);
            if (lengthSq <= math.Epsilon || !IsFinite(lengthSq))
            {
                return fallback;
            }

            fix length = math.sqrt(lengthSq);
            return length > math.Epsilon ? value / length : fallback;
        }

        public static fix Clamp(fix value, fix min, fix max)
        {
            value = Sanitize(value);
            if (value < min)
            {
                return min;
            }

            if (value > max)
            {
                return max;
            }

            return value;
        }
    }
}
