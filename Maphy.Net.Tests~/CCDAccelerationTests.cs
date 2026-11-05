using System;
using Maphy.Mathematics;
using Maphy.Physics;

internal static class CCDAccelerationTests
{
    public static void DifferentialReplay()
    {
        for (int seed = 0; seed < 12; seed++)
        {
            PhysicsWorld fast = BuildMixed(seed, true, out BodyHandle[] fastBodies, out ColliderHandle[] fastColliders);
            PhysicsWorld slow = BuildMixed(seed, false, out BodyHandle[] slowBodies, out ColliderHandle[] slowColliders);
            fast.EnableProfiling = true;
            for (int frame = 0; frame < 160; frame++)
            {
                Mutate(fast, fastBodies, fastColliders, frame);
                Mutate(slow, slowBodies, slowColliders, frame);
                if (frame == 130) fast.EnableCCDBroadphase = false;
                if (frame == 135) fast.EnableCCDBroadphase = true;
                fast.Step(fix.One / 60);
                slow.Step(fix.One / 60);
                Same(fast, slow, $"seed {seed}, frame {frame}");
                if (fast.LastCCDStats.indexOverflowed || fast.LastStepStats.HasOverflow)
                    throw new InvalidOperationException("mixed CCD corpus overflowed");
            }
        }
    }

    private static PhysicsWorld BuildMixed(int seed, bool accelerate, out BodyHandle[] bodies, out ColliderHandle[] colliders)
    {
        var settings = new PhysicsWorldSettings((seed & 1) != 0);
        settings.enableCCD = true;
        settings.enableDynamicCCD = (seed & 2) != 0;
        settings.ccdMinVelocity = fix.Zero;
        settings.ccdMaxIterations = 3;
        settings.ccdSkin = fix._0_01;
        settings.solverIterations = 2;
        settings.positionIterations = 2;
        var world = new PhysicsWorld(settings) { EnableCCDBroadphase = accelerate };
        world.Reserve(40, 80, 2048, 2048);
        bodies = new BodyHandle[24]; colliders = new ColliderHandle[24];
        var random = new System.Random(seed + 913);
        for (int i = 0; i < bodies.Length; i++)
        {
            var type = i % 7 == 0 ? RigidType.Static : i % 11 == 0 ? RigidType.Kinematic : RigidType.Dynamic;
            fix3 position = new fix3(random.Next(-6, 7), random.Next(-3, 7), random.Next(-3, 4));
            Require(world.CreateBody(position, quaternion.identity, type, out bodies[i]));
            PhysicsShapeData shape;
            switch (i % 4)
            {
                case 0: shape = PhysicsShapeData.AABB(fix3.zero, new fix3(1)); break;
                case 1: shape = PhysicsShapeData.Sphere(fix3.zero, fix._0_5); break;
                case 2: shape = PhysicsShapeData.OBB(fix3.zero, new fix3(1, 2, 1), quaternion.RotateY(fix._0_25)); break;
                default: shape = PhysicsShapeData.Capsule(fix3.zero, fix._0_25, new fix(2), quaternion.identity); break;
            }
            Require(world.AddCollider(bodies[i], shape, out colliders[i]));
            if (i % 5 == 0) Require(world.AddSphere(bodies[i], fix3.right, fix._0_25, out _));
            world.SetBodyVelocity(bodies[i], new fix3(random.Next(-90, 91), random.Next(-30, 31), random.Next(-20, 21)));
            world.SetBodyAngularVelocity(bodies[i], new fix3(fix._0_1, fix._0_25, fix.Zero));
            if (i % 9 == 0) world.SetColliderTrigger(colliders[i], true);
            if (i % 8 == 0) world.SetColliderCollisionMask(colliders[i], 0);
        }
        return world;
    }

    private static void Mutate(PhysicsWorld world, BodyHandle[] bodies, ColliderHandle[] colliders, int frame)
    {
        if (frame == 20) world.SetBodyEnabled(bodies[3], false);
        if (frame == 25) world.SetBodyEnabled(bodies[3], true);
        if (frame == 35)
        {
            Require(world.DestroyCollider(colliders[5]));
            Require(world.AddSphere(bodies[7], fix3.right, fix._0_25, out colliders[5]));
        }
        if (frame == 50)
        {
            Require(world.DestroyBody(bodies[10]));
            Require(world.CreateBody(new fix3(-3, 2, 0), quaternion.identity, RigidType.Dynamic, out bodies[10]));
            Require(world.AddCapsule(bodies[10], fix3.zero, fix._0_25, new fix(2), quaternion.identity, out colliders[10]));
        }
        if (frame == 60) world.Reserve(96, 192, 4096, 4096);
        if (frame == 65) world.SetBodyVelocity(bodies[10], new fix3(1000, -50, 0));
        if (frame == 80) world.SetColliderTrigger(colliders[2], true);
        if (frame == 95) world.SetColliderTrigger(colliders[2], false);
        if (frame == 100) world.SetBodyTransform(bodies[11], new fix3(0, 2, 0), quaternion.RotateZ(fix._0_5));
        if (frame == 110) world.SetBodyType(bodies[12], RigidType.Kinematic);
        if (frame == 120) world.SetBodyVelocity(bodies[12], new fix3(-1500, 0, 0));
    }

    public static void FastTargetsAndTies()
    {
        foreach (RigidType targetType in new[] { RigidType.Dynamic, RigidType.Kinematic })
        {
            var fast = BuildFastTarget(targetType, true);
            var slow = BuildFastTarget(targetType, false);
            fast.Step(fix.One / 60); slow.Step(fix.One / 60);
            Same(fast, slow, "fast crossing target " + targetType);
            if (fast.LastCCDStats.shapeTests == 0) throw new InvalidOperationException("fast target was not tested");
        }
        var a = BuildTie(true); var b = BuildTie(false);
        a.Update(fix.One); b.Update(fix.One);
        Same(a, b, "equal fraction targets preserve collider-index tie break");
        var overflow = BuildUnrepresentableEnvelope(true);
        var overflowReference = BuildUnrepresentableEnvelope(false);
        overflow.Update(fix.One); overflowReference.Update(fix.One);
        Same(overflow, overflowReference, "unrepresentable swept bounds use the reference scan");
        if (overflow.LastCCDStats.fallbackScans == 0)
            throw new InvalidOperationException("invalid CCD envelope did not select the reference scan");
        var wide = BuildUnrepresentableUnion(true); var wideReference = BuildUnrepresentableUnion(false);
        wide.Update(fix.One); wideReference.Update(fix.One);
        Same(wide, wideReference, "finite leaves with unrepresentable tree union");
        if (wide.LastCCDStats.fallbackScans == 0)
            throw new InvalidOperationException("unrepresentable tree union did not select the reference scan");
    }

    private static PhysicsWorld BuildUnrepresentableUnion(bool accelerate)
    {
        var world = BuildTie(accelerate);
        world.Reserve(5, 5, 16, 16);
        foreach (long sign in new long[] { -1, 1 })
        {
            world.CreateBody(new fix3(new fix(sign * 70000000000000L), fix.Zero, fix.Zero), quaternion.identity, RigidType.Static, out var body);
            world.AddAABB(body, fix3.zero, new fix3(new fix(20000000000000L), fix.One, fix.One), out var collider);
            world.SetColliderCollisionMask(collider, 0);
        }
        return world;
    }

    private static PhysicsWorld BuildUnrepresentableEnvelope(bool accelerate)
    {
        var world = BuildTie(accelerate);
        world.Reserve(4, 4, 8, 8);
        world.CreateBody(new fix3(100, 0, 0), quaternion.identity, RigidType.Kinematic, out var remote);
        world.AddSphere(remote, fix3.zero, fix.One, out var collider);
        world.SetColliderCollisionMask(collider, 0);
        world.SetBodyVelocity(remote, new fix3(fix.Max, fix.Zero, fix.Zero));
        return world;
    }

    private static PhysicsWorld BuildFastTarget(RigidType targetType, bool accelerate)
    {
        var settings = new PhysicsWorldSettings(false) { enableCCD = true, enableDynamicCCD = true, ccdMinVelocity = fix.Zero };
        var world = new PhysicsWorld(settings) { EnableCCDBroadphase = accelerate };
        world.Reserve(2, 2, 4, 4);
        world.CreateBody(fix3.zero, quaternion.identity, RigidType.Dynamic, out var moving);
        world.AddSphere(moving, fix3.zero, fix.One, out _);
        world.SetBodyVelocity(moving, new fix3(1200, 0, 0));
        world.CreateBody(new fix3(30, 0, 0), quaternion.identity, targetType, out var target);
        world.AddSphere(target, fix3.zero, fix.One, out _);
        world.SetBodyVelocity(target, new fix3(-1200, 0, 0));
        return world;
    }

    private static PhysicsWorld BuildTie(bool accelerate)
    {
        var settings = new PhysicsWorldSettings(false) { enableCCD = true, ccdMinVelocity = fix.Zero };
        var world = new PhysicsWorld(settings) { EnableCCDBroadphase = accelerate };
        world.Reserve(3, 3, 8, 8);
        world.CreateBody(fix3.zero, quaternion.identity, RigidType.Dynamic, out var moving);
        world.AddSphere(moving, fix3.zero, fix.One, out _);
        world.SetBodyVelocity(moving, new fix3(10, 0, 0));
        for (int sign = 1; sign >= -1; sign -= 2)
        {
            world.CreateBody(new fix3(5, sign, 0), quaternion.identity, RigidType.Static, out var target);
            world.AddSphere(target, fix3.zero, fix.One, out _);
        }
        return world;
    }

    public static void SparsePruning()
    {
        PhysicsWorld fast = BuildSparse(true), slow = BuildSparse(false);
        fast.EnableProfiling = true;
        long before = GC.GetAllocatedBytesForCurrentThread();
        fast.Update(fix.One);
        long bytes = GC.GetAllocatedBytesForCurrentThread() - before;
        slow.Update(fix.One);
        Same(fast, slow, "sparse CCD scene");
        if (bytes != 0) throw new InvalidOperationException($"profiled CCD allocated {bytes} bytes");
        if (fast.LastCCDStats.fallbackScans != 0 || fast.LastCCDStats.candidates * 10 >= slow.LastCCDStats.fallbackCandidates)
            throw new InvalidOperationException("sparse CCD did not prune the reference candidate set");
    }

    private static PhysicsWorld BuildSparse(bool accelerate)
    {
        var settings = new PhysicsWorldSettings(false) { enableCCD = true, enableDynamicCCD = true, ccdMinVelocity = fix.Zero };
        var world = new PhysicsWorld(settings) { EnableCCDBroadphase = accelerate };
        world.Reserve(256, 256, 1024, 1024);
        for (int i = 0; i < 128; i++)
        {
            world.CreateBody(new fix3(0, 0, i * 3), quaternion.identity, RigidType.Dynamic, out var moving);
            world.AddSphere(moving, fix3.zero, fix._0_5, out _);
            world.SetBodyVelocity(moving, new fix3(10, 0, 0));
            world.CreateBody(new fix3(5, 0, i * 3), quaternion.identity, RigidType.Static, out var wall);
            world.AddAABB(wall, fix3.zero, new fix3(1, 2, 1), out _);
        }
        return world;
    }

    public static void ProfileAggregation()
    {
        var fast = BuildFastTarget(RigidType.Dynamic, true);
        var slow = BuildFastTarget(RigidType.Dynamic, true);
        fast.EnableProfiling = true;
        fix dt = fix.One / 60;
        fast.Step(dt * 3, dt, 8); slow.Step(dt * 3, dt, 8);
        Same(fast, slow, "profiling does not affect state");
        if (fast.LastStepProfile.updateCount != 3 || fast.LastStepProfile.TotalTicks <= 0)
            throw new InvalidOperationException("profiling did not aggregate substeps");
        fast.Step(fix.Zero, dt, 8);
        if (fast.LastStepProfile.updateCount != 0 || fast.LastCCDStats.queries != 0)
            throw new InvalidOperationException("zero-substep diagnostics are stale");
        fast.EnableProfiling = false;
        fast.Step(dt);
        if (fast.LastStepProfile.TotalTicks != 0) throw new InvalidOperationException("disabled profiler retained timings");
    }

    private static void Same(PhysicsWorld a, PhysicsWorld b, string context)
    {
        if (a.ComputeStateHash() != b.ComputeStateHash())
            throw new InvalidOperationException("CCD reference diverged: " + context);
    }
    private static void Require(bool success)
    {
        if (!success) throw new InvalidOperationException("CCD scene setup failed");
    }
}
