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
