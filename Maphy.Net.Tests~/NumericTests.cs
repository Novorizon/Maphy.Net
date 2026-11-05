using System;
using System.Numerics;
using Maphy.Mathematics;

internal static class NumericTests
{
    private static ulong digest = 14695981039346656037UL;
    private static readonly long[] BoundaryRawValues =
    {
        long.MinValue, long.MinValue + 1, long.MinValue + 2,
        -(1L << 62), -(1L << 48), -(1L << 32),
        -131073, -131072, -98305, -98304, -98303,
        -65537, -65536, -65535, -32769, -32768, -32767, -1,
        0, 1, 32767, 32768, 32769, 65535, 65536, 65537,
        98303, 98304, 98305, 131072, 131073,
        1L << 32, 1L << 48, 1L << 62, long.MaxValue - 1, long.MaxValue
    };

    public static void WideArithmeticRegressions()
    {
        AssertRaw(4294967296L * fix.ONE, new fix(65536) * new fix(65536), "wide multiply");
        AssertRaw(10000000000L * fix.ONE, new fix(10000000000L) / fix.One, "wide divide");
        AssertRaw(long.MinValue, fix.Max * 2, "multiply overflow");
        AssertRaw(long.MinValue, fix.Min / fix.Raw(-1), "divide overflow");
        AssertRaw(-1, fix.Raw(-1) * fix.Raw(1), "negative product rounds down");
        AssertRaw(0, fix.Raw(-1) / new fix(2), "negative quotient truncates toward zero");
        AssertRaw(0, math.trunc(new fix(-1)) + fix.One, "negative integer truncation");
        AssertRaw(fix.ONE, math.round((fix)1.3m), "positive rounding");
        AssertRaw(-fix.ONE, math.round((fix)(-1.3m)), "negative rounding");
    }

    public static void OperatorsMatchBigInteger()
    {
        foreach (long a in BoundaryRawValues)
        {
            foreach (long b in BoundaryRawValues)
            {
                CheckOperators(a, b);
            }
        }

        ulong seed = 0xD1B54A32D192ED03UL;
        for (int i = 0; i < 4000; i++)
        {
            long a = NextRaw(ref seed);
            long b = NextRaw(ref seed);
            if (i % 4 == 1) { a >>= 32; b >>= 32; }
            if (i % 4 == 2) b >>= 48;
            CheckOperators(a, b);
        }
    }

    private static void CheckOperators(long a, long b)
    {
        fix x = fix.Raw(a);
        fix y = fix.Raw(b);
        bool invalid = a == long.MinValue || b == long.MinValue;
        BigInteger wideA = a;
        BigInteger wideB = b;
        long sum = invalid ? long.MinValue : ExpectedRaw(wideA + wideB);
        long difference = invalid ? long.MinValue : ExpectedRaw(wideA - wideB);
        long product = invalid ? long.MinValue : ExpectedRaw((wideA * wideB) >> fix.PRECISION);
        long quotient = invalid || b == 0 ? long.MinValue : ExpectedRaw((wideA << fix.PRECISION) / wideB);
        long remainder = invalid || b == 0 ? long.MinValue : ExpectedRaw(wideA % wideB);
        AssertRaw(sum, x + y, "add");
        AssertRaw(sum, math.add(x, y), "math.add");
        AssertRaw(difference, x - y, "subtract");
        AssertRaw(difference, math.sub(x, y), "math.sub");
        AssertRaw(product, x * y, "multiply");
        AssertRaw(quotient, x / y, "divide");
        AssertRaw(remainder, x % y, "remainder");
        AssertRaw(remainder, math.mod(x, y), "math.mod");
    }

    public static void IntegerConversionsAndShifts()
    {
        long limit = long.MaxValue >> fix.PRECISION;
        long[] integers = { long.MinValue, -limit - 1, -limit, -1, 0, 1, limit, limit + 1, long.MaxValue };
        foreach (long value in integers)
        {
            AssertRaw(ExpectedRaw((BigInteger)value << fix.PRECISION), new fix(value), "integer conversion");
        }

        int[] shifts = { int.MinValue, -129, -65, -64, -1, 0, 1, 16, 31, 47, 48, 62, 63, 64, 65, 127, 128, int.MaxValue };
        foreach (long value in BoundaryRawValues)
        {
            foreach (int shift in shifts)
            {
                int masked = shift & 63;
                long expectedLeft = value == long.MinValue ? long.MinValue : ExpectedRaw((BigInteger)value << masked);
                long expectedRight = value == long.MinValue ? long.MinValue : ExpectedRaw((BigInteger)value >> masked);
                AssertRaw(expectedLeft, fix.Raw(value) << shift, "left shift");
                AssertRaw(expectedRight, fix.Raw(value) >> shift, "right shift");
            }
        }
    }

    public static void SquareRootBounds()
    {
        foreach (long value in BoundaryRawValues) CheckSquareRoot(value);
        ulong seed = 0x94D049BB133111EBUL;
        for (int i = 0; i < 4000; i++)
        {
            long value = NextRaw(ref seed) & long.MaxValue;
            CheckSquareRoot(i % 2 == 0 ? value : value >> 32);
        }
        // Exercise values immediately around representable perfect squares.
        for (long n = 1; n <= 64; n++)
        {
            long square = n * n * fix.ONE;
            CheckSquareRoot(square - 1);
            CheckSquareRoot(square);
            CheckSquareRoot(square + 1);
        }
    }

    private static void CheckSquareRoot(long value)
    {
        fix root = math.sqrt(fix.Raw(value));
        if (value < 0)
        {
            AssertRaw(long.MinValue, root, "negative sqrt");
            return;
        }

        BigInteger scaled = (BigInteger)value << fix.PRECISION;
        BigInteger rawRoot = root.value;
        if (rawRoot < 0 || rawRoot * rawRoot > scaled || (rawRoot + 1) * (rawRoot + 1) <= scaled)
            throw new InvalidOperationException($"sqrt raw {value}: {root.value} is not the integer floor root");
        AddDigest(root.value);
    }

    public static void RoundingMatchesDecimal()
    {
        foreach (long raw in BoundaryRawValues) CheckRounding(raw);
        for (long integer = -4; integer <= 4; integer++)
        {
            for (long fraction = 0; fraction <= fix.ONE; fraction += fix.ONE / 4)
            {
                long value = integer * fix.ONE + fraction;
                CheckRounding(value - 1);
                CheckRounding(value);
                CheckRounding(value + 1);
            }
        }
        ulong seed = 0x9E3779B97F4A7C15UL;
        for (int i = 0; i < 4000; i++) CheckRounding(NextRaw(ref seed));
    }

    private static void CheckRounding(long raw)
    {
        fix value = fix.Raw(raw);
        if (raw == long.MinValue)
        {
            AssertRaw(raw, math.trunc(value), "trunc NaN");
            AssertRaw(raw, math.frac(value), "frac NaN");
            AssertRaw(raw, math.floor(value), "floor NaN");
            AssertRaw(raw, math.ceil(value), "ceil NaN");
            AssertRaw(raw, math.round(value), "round NaN");
            return;
        }

        decimal exact = (decimal)value;
        AssertRaw(ExpectedRaw(new BigInteger(decimal.Truncate(exact) * fix.ONE)), math.trunc(value), "trunc");
        AssertRaw(ExpectedRaw(new BigInteger(decimal.Floor(exact) * fix.ONE)), math.floor(value), "floor");
        AssertRaw(ExpectedRaw(new BigInteger(decimal.Ceiling(exact) * fix.ONE)), math.ceil(value), "ceil");
        AssertRaw(ExpectedRaw(new BigInteger(decimal.Round(exact, 0, MidpointRounding.AwayFromZero) * fix.ONE)), math.round(value), "round");
        // Decimal has insufficient fractional precision near fix.Max to recover
        // every raw bit after subtracting the integral part. Use exact integers.
        AssertRaw(ExpectedRaw(BigInteger.Abs((BigInteger)raw % fix.ONE)), math.frac(value), "fraction magnitude");
    }

    public static void VectorRounding()
    {
        fix4 rounded = math.round(new fix4((fix)1.3m, (fix)(-1.3m), (fix)1.5m, (fix)(-1.5m)));
        AssertRaw(fix.ONE, rounded.x, "round vector x");
        AssertRaw(-fix.ONE, rounded.y, "round vector y");
        AssertRaw(2 * fix.ONE, rounded.z, "round vector z");
        AssertRaw(-2 * fix.ONE, rounded.w, "round vector w");
    }

    public static void VectorSplats()
    {
        foreach (long raw in BoundaryRawValues)
        {
            fix value = fix.Raw(raw);
            fix2 pair = new fix2(value);
            fix3 triple = new fix3(value);
            fix4 quad = new fix4(value);
            fix2 implicitPair = value;
            fix3 implicitTriple = value;
            fix4 implicitQuad = value;
            AssertRaw(raw, pair.x, "fix2 splat x");
            AssertRaw(raw, pair.y, "fix2 splat y");
            AssertRaw(raw, triple.x, "fix3 splat x");
            AssertRaw(raw, triple.y, "fix3 splat y");
            AssertRaw(raw, triple.z, "fix3 splat z");
            AssertRaw(raw, quad.x, "fix4 splat x");
            AssertRaw(raw, quad.y, "fix4 splat y");
            AssertRaw(raw, quad.z, "fix4 splat z");
            AssertRaw(raw, quad.w, "fix4 splat w");
            AssertRaw(raw, implicitPair.x, "implicit fix2 splat");
            AssertRaw(raw, implicitTriple.x, "implicit fix3 splat");
            AssertRaw(raw, implicitQuad.x, "implicit fix4 splat");
        }
        AssertRaw(long.MinValue, fix3.NaN.x, "fix3.NaN x");
        AssertRaw(long.MinValue, fix3.NaN.y, "fix3.NaN y");
        AssertRaw(long.MinValue, fix3.NaN.z, "fix3.NaN z");
    }

    public static void VectorArithmetic()
    {
        fix2 pair = new fix2(new fix(65536));
        fix2 product = pair * pair;
        AssertRaw(4294967296L * fix.ONE, product.x, "fix2 wide multiply x");
        AssertRaw(4294967296L * fix.ONE, product.y, "fix2 wide multiply y");
        AssertRaw(4294967296L * fix.ONE, (pair * new fix(65536)).x, "fix2 scalar multiply");
        AssertRaw(4294967296L * fix.ONE, (new fix(65536) * pair).y, "scalar fix2 multiply");
        fix2 large = new fix2(new fix(10000000000L));
        AssertRaw(10000000000L * fix.ONE, (large / fix2.one).x, "fix2 wide divide");
        AssertRaw(10000000000L * fix.ONE, (large / fix.One).y, "fix2 scalar divide");
        AssertRaw(10000000000L * fix.ONE, (new fix(10000000000L) / fix2.one).x, "scalar fix2 divide");
        AssertRaw(long.MinValue, (new fix2(fix.Max) + fix2.one).x, "fix2 add overflow");
        AssertRaw(long.MinValue, (new fix2(fix.Min) - fix2.one).y, "fix2 subtract overflow");
        AssertRaw(long.MinValue, (new fix2(fix.Min) - fix.One).x, "fix2 scalar subtract overflow");
        AssertRaw(long.MinValue, (fix.Min - fix2.one).y, "scalar fix2 subtract overflow");
        AssertRaw(long.MinValue, (-new fix2(fix.NaN)).x, "fix2 invalid negation");
        AssertRaw(long.MinValue, (pair / fix.Zero).y, "fix2 zero divisor");
        fix4 quad = new fix4(new fix(65536));
        fix4 quadProduct = quad * quad;
        AssertRaw(4294967296L * fix.ONE, quadProduct.x, "fix4 wide multiply x");
        AssertRaw(4294967296L * fix.ONE, quadProduct.y, "fix4 wide multiply y");
        AssertRaw(4294967296L * fix.ONE, quadProduct.z, "fix4 wide multiply z");
        AssertRaw(4294967296L * fix.ONE, quadProduct.w, "fix4 wide multiply w");
        long outOfRange = (long.MaxValue >> fix.PRECISION) + 1;
        AssertRaw(long.MinValue, new fix2(outOfRange, outOfRange).x, "fix2 long construction");
    }

    public static void AngleEdgeCases()
    {
        AssertRaw(0, Geometry.Angle(fix3.zero, fix3.zero), "zero angle");
        AssertRaw(0, Geometry.Angle(fix3.zero, fix3.right), "zero from vector");
        AssertRaw(0, Geometry.Angle(fix3.right, fix3.zero), "zero to vector");
        AssertRaw(0, Geometry.Angle(fix3.right, fix3.right), "parallel angle");
        AssertRaw(long.MinValue, Geometry.Angle(new fix3(fix.NaN), fix3.right), "invalid angle");
        fix opposite = Geometry.Angle(fix3.left, fix3.right);
        if (fix.IsNaN(opposite) || math.abs(opposite - new fix(180)) > (fix)0.01m)
            throw new InvalidOperationException("opposite vectors should form a 180 degree angle");
        AddDigest(opposite.value);
        if (Geometry.kEpsilonNormalSqrt <= fix.Zero)
            throw new InvalidOperationException("angle epsilon must be representable and positive");
    }

    public static void ArithmeticDoesNotAllocate()
    {
        fix a = fix.Raw(long.MaxValue / 3);
        fix b = fix.Raw(fix.ONE + 1);
        long checksum = 0;
        for (int i = 0; i < 32; i++) checksum ^= ExerciseArithmetic(a, b);
        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 1000; i++) checksum ^= ExerciseArithmetic(a, b);
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;
        if (allocated != 0) throw new InvalidOperationException($"arithmetic allocated {allocated} bytes");
        // Keep all results observable without allocating in the measured section.
        AddDigest(checksum);
    }

    private static long ExerciseArithmetic(fix a, fix b)
    {
        return (a * b).value ^ (a / b).value ^ math.sqrt(a).value
            ^ (a + b).value ^ (a - b).value ^ (a << 1).value;
    }

    private static long ExpectedRaw(BigInteger value)
    {
        return value <= long.MinValue || value > long.MaxValue ? long.MinValue : (long)value;
    }

    private static long NextRaw(ref ulong state)
    {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        return unchecked((long)state);
    }

    private static void AssertRaw(long expected, fix actual, string operation)
    {
        if (expected != actual.value)
            throw new InvalidOperationException($"{operation}: expected raw {expected}, got {actual.value}");
        AddDigest(actual.value);
    }

    private static void AddDigest(long raw)
    {
        digest = unchecked((digest ^ (ulong)raw) * 1099511628211UL);
    }

    public static void PrintDigest()
    {
        Console.WriteLine($"Numeric corpus hash: {digest:X16}");
    }
}
