using System;
using System.Numerics;
using Maphy.Mathematics;
using Maphy.Physics;
using PhysicsApi = Maphy.Physics.Physics;

internal static class RuntimeTests
{
    // Must be the first test in a fresh process, before any trig lookup occurs.
    public static void ReservedFirstRotation()
    {
        var world = new PhysicsWorld(new PhysicsWorldSettings(false));
        world.Reserve(1, 1, 4, 4);
        world.CreateBody(fix3.zero, quaternion.identity, RigidType.Dynamic, out BodyHandle body);
        world.AddOBB(body, fix3.zero, new fix3(1), quaternion.identity, out _);
        world.SetBodyAngularVelocity(body, fix3.up);
        long before = GC.GetAllocatedBytesForCurrentThread();
        world.Step(fix.One / 60);
        long bytes = GC.GetAllocatedBytesForCurrentThread() - before;
        if (bytes != 0) throw new InvalidOperationException($"first reserved rotation allocated {bytes} bytes");
    }

    public static void BinaryIngress()
    {
        uint[] singles = { 0, 0x80000000, 1, 0x80000001, 0x007FFFFF, 0x00800000,
            0x37000000, 0xB7000000, 0x37800000, 0x3F800000, 0x7F7FFFFF,
            0x7F800000, 0xFF800000, 0x7FC00001, 0x57000000, 0xD7000000 };
        foreach (uint bits in singles) CheckSingle(bits);
        ulong[] doubles = { 0, 0x8000000000000000, 1, 0x8000000000000001,
            0x0010000000000000, 0x3EE0000000000000, 0xBEE0000000000000,
            0x3FF0000000000000, 0x42DFFFFFFFFFFFFF, 0x42E0000000000000,
            0xC2E0000000000000, 0x7FEFFFFFFFFFFFFF, 0x7FF0000000000000,
            0xFFF0000000000000, 0x7FF8000000000001 };
        foreach (ulong bits in doubles) CheckDouble(bits);
        ulong state = 0x94D049BB133111EB;
        for (int i = 0; i < 4000; i++)
        {
            state ^= state << 13; state ^= state >> 7; state ^= state << 17;
            CheckSingle(unchecked((uint)state));
            CheckDouble(state);
        }
        foreach (float value in new[] { float.NaN, float.PositiveInfinity, -float.MaxValue, 0.1f, -0.1f, 65536f })
        {
            long expected = new fix(value).value;
            Equal(expected, new fix2(value).x);
            Equal(expected, new fix3(value).y);
            Equal(expected, new fix4(value).w);
            Equal(expected, new fix4(value, value, value, value).z);
        }
    }

    private static void CheckSingle(uint bits)
    {
        int exponent = (int)((bits >> 23) & 255);
        ulong fraction = bits & 0x7FFFFF;
        long expected = exponent == 255 ? long.MinValue : RationalRaw(
            exponent == 0 ? fraction : fraction + (1UL << 23),
            (exponent == 0 ? -126 : exponent - 127) - 23, (bits >> 31) != 0, true);
        Equal(expected, new fix(BitConverter.Int32BitsToSingle(unchecked((int)bits))));
    }

    private static void CheckDouble(ulong bits)
    {
        int exponent = (int)((bits >> 52) & 2047);
        ulong fraction = bits & 0xFFFFFFFFFFFFF;
        long expected = exponent == 2047 ? long.MinValue : RationalRaw(
            exponent == 0 ? fraction : fraction + (1UL << 52),
            (exponent == 0 ? -1022 : exponent - 1023) - 52, (bits >> 63) != 0, false);
        Equal(expected, new fix(BitConverter.Int64BitsToDouble(unchecked((long)bits))));
    }

    private static long RationalRaw(ulong significand, int exponent, bool negative, bool round)
    {
        BigInteger numerator = (BigInteger)significand * fix.ONE;
        BigInteger denominator = BigInteger.One;
        if (exponent >= 0) numerator *= BigInteger.Pow(2, exponent);
        else denominator = BigInteger.Pow(2, -exponent);
        BigInteger quotient = BigInteger.DivRem(numerator, denominator, out BigInteger remainder);
        if (round && remainder * 2 >= denominator) quotient++;
        if (negative) quotient = -quotient;
        return quotient <= long.MinValue || quotient > long.MaxValue ? long.MinValue : (long)quotient;
    }

    public static void DecimalIngress()
    {
        decimal[] values = { decimal.MaxValue, decimal.MinValue, 140737488355328m,
            -140737488355328m, 140737488355327.99999999999999m,
            -140737488355327.99999999999999m, 0m, 0.1m, -0.1m, 1m / 131072m };
        foreach (decimal value in values)
            CheckDecimal(value);
        var random = new System.Random(7123);
        for (int i = 0; i < 4000; i++)
            CheckDecimal(new decimal(random.Next(), random.Next(), random.Next(), (i & 1) != 0, (byte)(i % 29)));
        Equal(1, fix.Epsilon);
        Equal(0, fix._0_00001); // Below Q16 resolution: not a usable tolerance.
    }

    private static void CheckDecimal(decimal value)
    {
        int[] bits = decimal.GetBits(value);
        BigInteger coefficient = (uint)bits[0] + ((BigInteger)(uint)bits[1] << 32) + ((BigInteger)(uint)bits[2] << 64);
        BigInteger raw = coefficient * fix.ONE / BigInteger.Pow(10, (bits[3] >> 16) & 255);
        if (bits[3] < 0) raw = -raw;
        Equal(raw <= long.MinValue || raw > long.MaxValue ? long.MinValue : (long)raw, new fix(value));
    }

    public static void CapabilityContract()
    {
        if (!World.Capabilities.SupportsAlgorithm(NarrowPhaseAlgorithm.GJK)
            || PhysicsWorld.Capabilities.SupportsAlgorithm(NarrowPhaseAlgorithm.GJK)
            || !World.Capabilities.SupportsConstraint(ConstraintType.Distance)
            || PhysicsWorld.Capabilities.SupportsConstraint(ConstraintType.Distance)
            || World.Capabilities.SupportsConstraint(ConstraintType.Gear)
            || !World.Capabilities.SupportsCollisionCallbacks
            || PhysicsWorld.Capabilities.SupportsCollisionCallbacks)
            throw new InvalidOperationException("capability descriptions do not match the public entry points");
        var settings = new PhysicsWorldSettings(false);
        settings.narrowPhaseAlgorithm = NarrowPhaseAlgorithm.GJK;
        Throws<NotSupportedException>(() => new PhysicsWorld(settings));
        var core = new PhysicsWorld(new PhysicsWorldSettings(false));
        Throws<NotSupportedException>(() => core.Settings = settings);
        if (core.Settings.narrowPhaseAlgorithm != NarrowPhaseAlgorithm.Auto)
            throw new InvalidOperationException("rejected settings must not change the world");
        var shape = PhysicsShapeData.Sphere(fix3.zero, fix.One);
        Throws<NotSupportedException>(() => PhysicsApi.TryComputeContact(shape, shape, NarrowPhaseAlgorithm.GJK, out _));
        Throws<ArgumentOutOfRangeException>(() => PhysicsApi.TryComputeContact(shape, shape, (NarrowPhaseAlgorithm)99, out _));
    }

    public static void ContactEntryPoints()
    {
        PhysicsShapeData[] values = { PhysicsShapeData.AABB(fix3.zero, new fix3(2)),
            PhysicsShapeData.OBB(fix3.zero, new fix3(2), quaternion.identity),
            PhysicsShapeData.Sphere(fix3.zero, fix.One),
            PhysicsShapeData.Capsule(fix3.zero, fix._0_5, new fix(2), quaternion.identity) };
        Shape[] objects = { values[0].aabb, values[1].obb, values[2].sphere, values[3].capsule };
        foreach (NarrowPhaseAlgorithm algorithm in new[] { NarrowPhaseAlgorithm.Auto, NarrowPhaseAlgorithm.SAT })
        {
            for (int i = 0; i < values.Length; i++)
            for (int j = 0; j < values.Length; j++)
            {
                bool core = PhysicsApi.TryComputeContact(values[i], values[j], algorithm, out var a);
                bool obj = PhysicsApi.TryComputeContact(objects[i], objects[j], algorithm, out var b);
                if (!core || !obj || a.penetrationDepth != b.penetrationDepth)
                    throw new InvalidOperationException($"contact entry points differ for {i}/{j}/{algorithm}");
            }
        }
    }

    private static void Equal(long expected, fix actual)
    {
        if (expected != actual.value) throw new InvalidOperationException($"expected raw {expected}, got {actual.value}");
    }
    private static void Throws<T>(Action action) where T : Exception
    {
        try { action(); } catch (T) { return; }
        throw new InvalidOperationException($"expected {typeof(T).Name}");
    }
}
