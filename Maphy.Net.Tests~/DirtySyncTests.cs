using System;
using Maphy.Mathematics;
using Maphy.Physics;

internal static class DirtySyncTests
{
    public static void SparseLifecycle()
    {
        const int count = 4097;
        var world = new PhysicsWorld(new PhysicsWorldSettings(false));
        var bodies = new BodyHandle[count];
        var colliders = new ColliderHandle[count];
        var positions = new fix3[count];
        world.Reserve(64, 64, 0, 0);
        for (int i = 0; i < count; i++)
        {
            if (i == 64)
            {
                // Preserve pending marks while growing from one word to three levels.
                Move(world, bodies, positions, 0);
                Move(world, bodies, positions, 31);
                Move(world, bodies, positions, 63);
                world.Reserve(count, count, 0, 0);
            }
            positions[i] = new fix3(i * 4, 0, 0);
            Require(world.CreateBody(positions[i], quaternion.identity, RigidType.Static, out bodies[i]));
            Require(world.AddSphere(bodies[i], fix3.zero, fix._0_5, out colliders[i]));
        }
        // Boundary bits, sparse words, reverse insertion and duplicate marks.
        foreach (int i in new[] { 4096, 4095, 2048, 255, 128, 127, 65, 64, 63, 0, 4096 })
            Move(world, bodies, positions, i);

        BodyHandle staleBody = bodies[63];
        ColliderHandle staleCollider = colliders[63];
        Require(world.DestroyBody(staleBody));
        positions[63] = new fix3(252, 9, 0);
        Require(world.CreateBody(positions[63], quaternion.identity, RigidType.Static, out bodies[63]));
        Require(world.AddSphere(bodies[63], fix3.zero, fix._0_5, out colliders[63]));
        Require(bodies[63].Index == staleBody.Index && bodies[63] != staleBody);
        Require(!world.TryGetCollider(staleCollider, out _));

        // Collider order differs from body order after slot reuse into a compound.
        Require(world.DestroyCollider(colliders[0]));
        Require(world.AddSphere(bodies[64], fix3.up, fix._0_5, out colliders[0]));
        Require(world.SetBodyEnabled(bodies[65], false));
        Require(world.SetColliderEnabled(colliders[128], false));
        Require(world.SetColliderTrigger(colliders[4096], true));
        world.Reserve(count + 64, count + 64, 0, 0);

        long before = GC.GetAllocatedBytesForCurrentThread();
        Require(world.TryGetColliderBounds(colliders[4096], out _));
        Require(GC.GetAllocatedBytesForCurrentThread() == before);
        for (int i = 0; i < count; i++)
        {
            Require(world.TryGetColliderBounds(colliders[i], out AABB bounds));
            fix3 expected = i == 0 ? positions[64] + fix3.up : positions[i];
            if (bounds.center != expected) throw new InvalidOperationException("stale collider at index " + i);
        }

        // Drain-to-empty and re-add to an emptied leaf and its summary ancestors.
        world.Update(fix.Zero);
        Move(world, bodies, positions, 4096);
        before = GC.GetAllocatedBytesForCurrentThread();
        world.Update(fix.Zero);
        Require(GC.GetAllocatedBytesForCurrentThread() == before);
        Require(world.LastStepStats.bodySyncCount == 1 && world.LastStepStats.colliderSyncCount == 1);
        Require(!world.LastStepStats.HasOverflow);
    }

    private static void Move(PhysicsWorld world, BodyHandle[] bodies, fix3[] positions, int index)
    {
        positions[index] += fix3.up;
        Require(world.SetBodyTransform(bodies[index], positions[index], quaternion.identity));
    }

    public static void PendingCCDReplay()
    {
        // Frozen against the ascending full-scan dirty synchronizer before replacement.
        const ulong expected = 0x72FF59AB79CD3A51UL;
        ulong indexed = PendingCCDFingerprint(true);
        ulong reference = PendingCCDFingerprint(false);
        if (indexed != expected || reference != expected)
            throw new InvalidOperationException($"pending CCD snapshots changed: {indexed:X16}, {reference:X16}");
    }

    internal static ulong PendingCCDFingerprint(bool indexed)
    {
        const int count = 130;
        var settings = new PhysicsWorldSettings(false)
        {
            enableCCD = true, enableDynamicCCD = true, enableSleeping = false,
            ccdMinVelocity = fix.Zero, ccdMaxIterations = 4,
            positionCorrectionPercent = fix.Zero,
        };
        var world = new PhysicsWorld(settings) { EnableCCDBroadphase = indexed };
        world.Reserve(count, count * 2, count * 16, count * 16);
        var bodies = new BodyHandle[count];
        var colliders = new ColliderHandle[count];
        var positions = new fix3[count];
        for (int i = 0; i < count; i++)
        {
            int z = (i / 5) * 8;
            int kind = i % 5;
            positions[i] = kind == 0 ? new fix3(-5, 0, z - 3)
                : kind == 1 ? new fix3(-4, 0, z + 3)
                : kind == 2 ? new fix3(0, 0, z)
                : kind == 3 ? new fix3(5, 5, z) : new fix3(0, 8, z);
            Require(world.CreateBody(positions[i], quaternion.identity,
                kind >= 3 ? RigidType.Static : kind == 1 ? RigidType.Kinematic : RigidType.Dynamic, out bodies[i]));
            if (kind == 0) Require(world.SetBodyCCD(bodies[i], false));
        }
        // Interleave ownership so a body-by-body collider flush would reorder refits.
        for (int n = 0; n < count; n++)
        {
            int i = (n * 37) % count;
            if (i % 5 >= 3)
                Require(world.AddAABB(bodies[i], fix3.zero, i % 5 == 3 ? new fix3(1, 20, 1) : new fix3(20, 1, 1), out colliders[i]));
            else Require(world.AddSphere(bodies[i], fix3.zero, fix._0_5, out colliders[i]));
        }
        for (int i = 2; i < count; i += 5)
            Require(world.AddSphere(bodies[i], new fix3(fix._0_25, fix.Zero, fix.Zero), fix._0_25, out _));

        ulong hash = 14695981039346656037UL;
        for (int frame = 0; frame < 64; frame++)
        {
            for (int i = count - 1; i >= 0; i--)
            {
                Require(world.SetBodyTransform(bodies[i], positions[i], quaternion.identity));
                if (i % 5 < 3) Require(world.SetBodyVelocity(bodies[i], i % 5 == 2 ? new fix3(10, 10, 0) : fix3.right));
            }
            if (frame == 10) world.Reserve(count + 64, count * 3, count * 16, count * 16);
            if (frame == 20) Require(world.SetBodyEnabled(bodies[7], false));
            if (frame == 21) Require(world.SetBodyEnabled(bodies[7], true));
            if (frame == 30)
            {
                Require(world.DestroyCollider(colliders[8]));
                Require(world.AddAABB(bodies[8], fix3.zero, new fix3(1, 20, 1), out colliders[8]));
            }
            if (frame == 40) Require(world.SetColliderTrigger(colliders[9], true));
            if (frame == 41) Require(world.SetColliderTrigger(colliders[9], false));
            world.Update(fix.One);
            Require(!world.LastStepStats.HasOverflow && !world.LastCCDStats.indexOverflowed);
            unchecked { hash = (hash ^ world.ComputeStateHash()) * 1099511628211UL; }
        }
        return hash;
    }

    private static void Require(bool success)
    {
        if (!success) throw new InvalidOperationException("dirty synchronization regression");
    }
}
