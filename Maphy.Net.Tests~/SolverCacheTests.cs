using System;
using Maphy.Mathematics;
using Maphy.Physics;

internal static class SolverCacheTests
{
    public static void DifferentialLifecycle()
    {
        for (int seed = 0; seed < 4; seed++)
        {
            PhysicsWorld fast = Build(seed, true, out BodyHandle[] a, out ColliderHandle[] ca);
            PhysicsWorld reference = Build(seed, false, out BodyHandle[] b, out ColliderHandle[] cb);
            for (int frame = 0; frame < 96; frame++)
            {
                Mutate(fast, a, ca, frame); Mutate(reference, b, cb, frame);
                fast.Update(fix.One / 60); reference.Update(fix.One / 60);
                Require(fast.ComputeStateHash() == reference.ComputeStateHash());
                Require(!fast.LastStepStats.HasOverflow);
            }
            long bytes = GC.GetAllocatedBytesForCurrentThread();
            fast.Update(fix.One / 60);
            Require(GC.GetAllocatedBytesForCurrentThread() == bytes);
            reference.Update(fix.One / 60);
            Require(fast.ComputeStateHash() == reference.ComputeStateHash());
        }
    }

    private static PhysicsWorld Build(int seed, bool cache, out BodyHandle[] bodies, out ColliderHandle[] colliders)
    {
        var settings = new PhysicsWorldSettings(true)
        {
            solverIterations = 4, positionIterations = 3,
            warmStartScale = seed % 2 == 0 ? fix.One : fix.Zero,
            sleepTime = fix.Zero, contactVelocityBiasFactor = fix._0_1,
        };
        var w = new PhysicsWorld(settings) { EnableSolverMassCache = cache };
        w.Reserve(48, 64, 2048, 2048);
        bodies = new BodyHandle[33]; colliders = new ColliderHandle[33];
        for (int i = 0; i < bodies.Length; i++)
        {
            var type = i == 0 ? RigidType.Static : i % 9 == 0 ? RigidType.Kinematic : RigidType.Dynamic;
            fix3 position = i == 0 ? new fix3(0, -1, 0)
                : new fix3(((i - 1) % 4) * fix._0_75, ((i - 1) / 4) * fix._0_75, fix.Zero);
            Require(w.CreateBody(position, quaternion.identity, type, out bodies[i]));
            Require(w.AddAABB(bodies[i], fix3.zero, i == 0 ? new fix3(20, 1, 20) : fix3.one, out colliders[i]));
            Require(w.SetColliderMaterial(colliders[i], new Material(fix.One, fix._0_75, fix._0_1)));
            if (i % 4 == 0) Require(w.SetBodyMass(bodies[i], new fix(i + 1)));
            if (i % 5 == 0) Require(w.SetBodyInertia(bodies[i], new fix3(fix.Zero, fix._2, fix._0_5)));
        }
        return w;
    }

    private static void Mutate(PhysicsWorld w, BodyHandle[] b, ColliderHandle[] c, int frame)
    {
        if (frame == 5) Require(w.SetBodyMass(b[2], fix._0_5));
        if (frame == 10) Require(w.SetBodyInertia(b[3], new fix3(0, 0, 7)));
        if (frame == 15) Require(w.SetBodyEnabled(b[4], false));
        if (frame == 16) Require(w.SetBodyEnabled(b[4], true));
        if (frame == 20) Require(w.SetBodyType(b[5], RigidType.Kinematic));
        if (frame == 21) Require(w.SetBodyType(b[5], RigidType.Dynamic));
        if (frame == 25) Require(w.SetBodyAutoMassProperties(b[6], true, true));
        if (frame == 30) w.Reserve(96, 192, 4096, 4096);
        if (frame == 35) Require(w.SetColliderMaterial(c[7], new Material(fix._2, fix.Zero, fix._0_5)));
        if (frame == 40) { Require(w.DestroyBody(b[8])); Require(w.CreateBody(new fix3(0, 1, 0), quaternion.identity, RigidType.Dynamic, out b[8])); Require(w.AddSphere(b[8], fix3.zero, fix.One, out c[8])); }
        if (frame == 50) Require(w.SetBodyVelocity(b[9], new fix3(3, -2, 1)));
        if (frame == 60) Require(w.SetBodyAngularVelocity(b[10], new fix3(1, 2, 3)));
        if (frame == 70) w.EnableSolverMassCache = !w.EnableSolverMassCache;
    }

    private static void Require(bool success)
    {
        if (!success) throw new InvalidOperationException("solver mass cache diverged from live reciprocals");
    }
}
