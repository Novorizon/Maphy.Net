using System;
using Maphy.Mathematics;
using Maphy.Physics;
using PhysicsApi = Maphy.Physics.Physics;

internal static class BoxSATTests
{
    public static void FrozenContacts()
    {
        // Includes contact order, anchors, feature IDs and invalid-range fallback.
        // Frozen using the original 15-axis box SAT, before the canonical shortcut.
        const ulong expected = 0xA30F46C553E109EFUL;
        ulong actual = Fingerprint();
        if (actual != expected) throw new InvalidOperationException($"box SAT fingerprint {actual:X16}");
    }

    internal static ulong Fingerprint()
    {
        new PhysicsWorld().Reserve(1, 1, 1, 1);
        ulong hash = 14695981039346656037UL;
        uint state = 719;
        for (int i = 0; i < 4096; i++)
        {
            fix3 center = new fix3(Next(ref state, 8), Next(ref state, 8), Next(ref state, 8));
            fix3 size0 = new fix3(math.abs(Next(ref state, 4)), math.abs(Next(ref state, 4)), math.abs(Next(ref state, 4)));
            fix3 size1 = new fix3(math.abs(Next(ref state, 4)), math.abs(Next(ref state, 4)), math.abs(Next(ref state, 4)));
            fix3 delta = new fix3(Next(ref state, 3), Next(ref state, 3), Next(ref state, 3));
            Sample(ref hash, PhysicsShapeData.AABB(center, size0), PhysicsShapeData.AABB(center + delta, size1));
        }
        // Tolerance ties, coincident centers, containment, degeneracy and touching.
        for (int raw = -16; raw <= 16; raw++)
        {
            var a = PhysicsShapeData.AABB(fix3.zero, new fix3(2));
            var b = PhysicsShapeData.AABB(new fix3(fix.One, fix.One + fix.Raw(raw), fix.One), new fix3(2));
            Sample(ref hash, a, b);
            Sample(ref hash, b, a);
            Sample(ref hash, a, PhysicsShapeData.AABB(fix3.zero, new fix3(fix.Raw(raw))));
            Sample(ref hash, a, PhysicsShapeData.AABB(new fix3(fix._2 + fix.Raw(raw)), new fix3(2)));
            Sample(ref hash, a, PhysicsShapeData.OBB(b.aabb.center, new fix3(2), quaternion.identity));
        }
        foreach (fix extreme in new[] { fix.Max, fix.Min, fix.NaN, fix.Raw(long.MaxValue - 4) })
        {
            Sample(ref hash, PhysicsShapeData.AABB(new fix3(extreme), fix3.one), PhysicsShapeData.AABB(fix3.zero, new fix3(2)));
            Sample(ref hash, PhysicsShapeData.AABB(fix3.zero, new fix3(extreme)), PhysicsShapeData.AABB(fix3.zero, new fix3(2)));
        }
        return hash;
    }

    private static fix Next(ref uint state, int range)
    {
        state = unchecked(state * 1664525U + 1013904223U);
        return fix.Raw((long)(state % (uint)(range * 2 * fix.ONE)) - range * fix.ONE);
    }

    private static void Sample(ref ulong hash, PhysicsShapeData a, PhysicsShapeData b)
    {
        bool hit = PhysicsApi.TryComputeContact(a, b, out CollisionInfo c);
        Add(ref hash, hit ? 1 : 0);
        Add(ref hash, c.penetrationDepth.value);
        Add(ref hash, c.normal);
        Add(ref hash, c.contactCount);
        for (int j = 0; j < c.contactCount; j++)
        {
            CollisionContact point = c[j];
            Add(ref hash, point.pointOnCollider0);
            Add(ref hash, point.pointOnCollider1);
            Add(ref hash, point.penetrationDepth.value);
            Add(ref hash, point.featureId);
        }
    }

    private static void Add(ref ulong hash, fix3 v)
    {
        Add(ref hash, v.x.value); Add(ref hash, v.y.value); Add(ref hash, v.z.value);
    }
    private static void Add(ref ulong hash, long raw)
        => hash = unchecked((hash ^ (ulong)raw) * 1099511628211UL);
}
