using System;
using Maphy.Mathematics;
using Maphy.Physics;

internal static class CCDReuseTests
{
    public static void ProxyLifecycle()
    {
        var settings = new PhysicsWorldSettings(false) { enableCCD = true };
        var world = new PhysicsWorld(settings);
        world.Reserve(48, 64, 256, 256);
        var bodies = new BodyHandle[32]; var colliders = new ColliderHandle[32];
        for (int i = 0; i < bodies.Length; i++)
        {
            Require(world.CreateBody(new fix3(i * 3, 0, 0), quaternion.identity, RigidType.Static, out bodies[i]));
            Require(world.AddSphere(bodies[i], fix3.zero, fix._0_5, out colliders[i]));
        }
        world.Update(fix.One / 60);
        if (world.LastCCDStats.proxiesCreated != 32) throw new InvalidOperationException("initial index was not populated");
        world.Update(fix.One / 60);
        CheckReuse(world, 32);
        world.DestroyCollider(colliders[0]);
        world.AddSphere(bodies[1], fix3.right, fix._0_25, out colliders[0]);
        world.Update(fix.One / 60);
        if (world.LastCCDStats.proxiesCreated != 1 || world.LastCCDStats.proxiesRemoved != 1)
            throw new InvalidOperationException("recycled collider version kept a stale CCD proxy");
        world.SetColliderTrigger(colliders[2], true);
        world.Update(fix.One / 60);
        if (world.LastCCDStats.proxyCount != 31 || world.LastCCDStats.proxiesRemoved != 1)
            throw new InvalidOperationException("trigger proxy was not removed");
        world.SetColliderTrigger(colliders[2], false);
        world.Update(fix.One / 60);
        if (world.LastCCDStats.proxyCount != 32 || world.LastCCDStats.proxiesCreated != 1)
            throw new InvalidOperationException("trigger proxy was not restored");
        world.SetBodyEnabled(bodies[3], false); world.Update(fix.One / 60);
        if (world.LastCCDStats.proxyCount != 31) throw new InvalidOperationException("disabled body retained its proxy");
        world.SetBodyEnabled(bodies[3], true); world.Update(fix.One / 60);
        world.Reserve(96, 192, 512, 512); world.Update(fix.One / 60);
        CheckReuse(world, 32);
        world.EnableCCDBroadphase = false;
        world.DestroyBody(bodies[4]);
        world.CreateBody(new fix3(12, 0, 0), quaternion.identity, RigidType.Static, out bodies[4]);
        world.AddSphere(bodies[4], fix3.zero, fix._0_5, out colliders[4]);
        world.Update(fix.One / 60);
        world.EnableCCDBroadphase = true; world.Update(fix.One / 60);
        if (world.LastCCDStats.proxiesCreated != 1 || world.LastCCDStats.proxiesRemoved != 1)
            throw new InvalidOperationException("re-enabling CCD did not reconcile stale versions");
    }

    private static void CheckReuse(PhysicsWorld world, int count)
    {
        PhysicsWorldCCDStats stats = world.LastCCDStats;
        if (stats.proxyCount != count || stats.proxiesCreated != 0 || stats.proxiesUpdated != 0
            || stats.proxiesReused < count || stats.indexRebuilds != 0)
            throw new InvalidOperationException("unchanged CCD proxies were rebuilt instead of reused");
    }

    public static void MotionEnvelopeChanges()
    {
        var fast = Build(true, false, out var mover, out var target);
        var slow = Build(false, false, out var referenceMover, out var referenceTarget);
        fix[] steps = { fix.One / 60, fix._0_1, fix.Zero, fix.One / 120, fix._0_5, fix.One / 30 };
        foreach (fix dt in steps)
        {
            ResetMotion(fast, mover, target); ResetMotion(slow, referenceMover, referenceTarget);
            fast.Update(dt); slow.Update(dt);
            Same(fast, slow, "variable motion horizon");
        }
        var wide = Build(true, true, out var wideMover, out var remote);
        var wideReference = Build(false, true, out var refMover, out var refRemote);
        wide.Update(fix.One); wideReference.Update(fix.One);
        Same(wide, wideReference, "initial wide bounds");
        wide.SetBodyTransform(remote, new fix3(new fix(70000000000000L), fix.Zero, fix.Zero), quaternion.identity);
        wideReference.SetBodyTransform(refRemote, new fix3(new fix(70000000000000L), fix.Zero, fix.Zero), quaternion.identity);
        wide.SetBodyTransform(wideMover, fix3.zero, quaternion.identity);
        wideReference.SetBodyTransform(refMover, fix3.zero, quaternion.identity);
        wide.SetBodyVelocity(wideMover, new fix3(10, 0, 0));
        wideReference.SetBodyVelocity(refMover, new fix3(10, 0, 0));
        wide.Update(fix.One); wideReference.Update(fix.One);
        Same(wide, wideReference, "large teleport with incompatible old and new tree span");
        if (wide.LastCCDStats.indexRebuilds != 1 || wide.LastCCDStats.fallbackScans != 0)
            throw new InvalidOperationException("large teleport did not recover with one safe index rebuild");
    }

    private static PhysicsWorld Build(bool accelerate, bool wide, out BodyHandle mover, out BodyHandle target)
    {
        var settings = new PhysicsWorldSettings(false) { enableCCD = true, enableDynamicCCD = true, ccdMinVelocity = fix.Zero, ccdMaxIterations = 3 };
        var world = new PhysicsWorld(settings) { EnableCCDBroadphase = accelerate };
        world.Reserve(4, 4, 16, 16);
        world.CreateBody(fix3.zero, quaternion.identity, RigidType.Dynamic, out mover);
        world.AddSphere(mover, fix3.zero, fix._0_5, out _);
        world.SetBodyVelocity(mover, new fix3(10, 0, 0));
        if (wide)
        {
            world.CreateBody(new fix3(5, 0, 0), quaternion.identity, RigidType.Static, out var wall);
            world.AddAABB(wall, fix3.zero, new fix3(1, 2, 1), out _);
            world.CreateBody(new fix3(new fix(-70000000000000L), fix.Zero, fix.Zero), quaternion.identity, RigidType.Static, out target);
            world.AddAABB(target, fix3.zero, new fix3(new fix(20000000000000L), fix.One, fix.One), out var masked);
            world.SetColliderCollisionMask(masked, 0);
        }
        else
        {
            world.CreateBody(new fix3(10, 0, 0), quaternion.identity, RigidType.Kinematic, out target);
            world.AddSphere(target, fix3.zero, fix.One, out _);
            world.SetBodyVelocity(target, new fix3(-20, 0, 0));
        }
        return world;
    }

    private static void ResetMotion(PhysicsWorld world, BodyHandle mover, BodyHandle target)
    {
        world.SetBodyTransform(mover, fix3.zero, quaternion.identity);
        world.SetBodyTransform(target, new fix3(10, 0, 0), quaternion.identity);
        world.SetBodyVelocity(mover, new fix3(50, 0, 0));
        world.SetBodyVelocity(target, new fix3(-20, 0, 0));
    }

    public static void IslandSleepLifecycle()
    {
        var world = new PhysicsWorld(new PhysicsWorldSettings(false));
        world.Reserve(1000, 1000, 2000, 2000);
        var bodies = new BodyHandle[1000];
        for (int i = 0; i < bodies.Length; i++)
        {
            Require(world.CreateBody(new fix3(i * 3, 0, 0), quaternion.identity, RigidType.Dynamic, out bodies[i]));
            Require(world.AddSphere(bodies[i], fix3.zero, fix._0_5, out _));
        }
        for (int frame = 0; frame < 30; frame++) world.Step(fix.One / 60);
        if (world.SleepingIslandCount != 0) throw new InvalidOperationException("islands slept before the fixed-point duration elapsed");
        long before = GC.GetAllocatedBytesForCurrentThread();
        world.Step(fix.One / 60);
        long bytes = GC.GetAllocatedBytesForCurrentThread() - before;
        if (bytes != 0 || world.SleepingIslandCount != 1000 || world.LastStepStats.sleepingBodyCount != 1000)
            throw new InvalidOperationException("independent islands did not sleep without allocation");
        world.SetBodyVelocity(bodies[123], fix3.right); world.Step(fix.One / 60);
        if (world.SleepingIslandCount != 999 || world.LastStepStats.awakeDynamicBodyCount != 1)
            throw new InvalidOperationException("waking one island changed other islands");
        world.DestroyBody(bodies[7]);
        world.CreateBody(new fix3(21, 0, 0), quaternion.identity, RigidType.Dynamic, out bodies[7]);
        world.AddSphere(bodies[7], fix3.zero, fix._0_5, out _); world.Step(fix.One / 60);
        if (world.IslandCount != 1000 || world.LastStepStats.awakeDynamicBodyCount != 2)
            throw new InvalidOperationException("recycled body corrupted island member lists");
        world.SetBodyType(bodies[8], RigidType.Static); world.Step(fix.One / 60);
        if (world.IslandCount != 999) throw new InvalidOperationException("body type change left a stale island member");
    }

    private static void Same(PhysicsWorld a, PhysicsWorld b, string context)
    {
        if (a.ComputeStateHash() != b.ComputeStateHash()) throw new InvalidOperationException("CCD reuse diverged: " + context);
    }
    private static void Require(bool success)
    {
        if (!success) throw new InvalidOperationException("reuse scene setup failed");
    }
}
