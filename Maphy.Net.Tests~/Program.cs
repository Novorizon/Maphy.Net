using System;
using Maphy.Mathematics;
using Maphy.Physics;
using PhysicsApi = Maphy.Physics.Physics;

internal static class Program
{
    private static int passed;

    private static void Main()
    {
        Run("fix4 csum includes w", TestFix4Csum);
        Run("fix sqrt converges below one", TestFixSqrt);
        Run("fix3x3 mul is matrix multiplication", TestFix3x3MatrixMul);
        Run("fix4x4 mul preserves TRS with identity", TestFix4x4MatrixMul);
        Run("capsule capsule overlap uses segment distance", TestCapsuleCapsuleOverlap);
        Run("box overlap handles OBB separation", TestBoxOverlap);
        Run("sphere capsule touching counts as contact", TestSphereCapsuleTouching);
        Run("OBB support point uses oriented box", TestOBBSupportPoint);
        Run("sphere AABB contact reports penetration", TestSphereAABBContact);
        Run("world update syncs collider transform", TestWorldTransformSync);
        Run("world builds contact manifolds", TestWorldContactManifolds);
        Run("world integrates linear velocity", TestWorldIntegratesLinearVelocity);
        Run("world resolves collision impulse", TestWorldResolvesCollisionImpulse);
        Run("world applies position correction", TestWorldAppliesPositionCorrection);
        Run("world dispatches collision callbacks", TestWorldDispatchesCollisionCallbacks);
        Run("trigger reports contact without solver response", TestTriggerReportsContactWithoutSolverResponse);
        Run("static rigid ignores motion and forces", TestStaticRigidIgnoresMotionAndForces);
        Run("dynamic body resolves against static body", TestDynamicBodyResolvesAgainstStaticBody);
        Run("kinematic rigid moves without force integration", TestKinematicRigidMovesWithoutForceIntegration);
        Run("kinematic body pushes dynamic body", TestKinematicBodyPushesDynamicBody);
        Run("solver applies linear friction", TestSolverAppliesLinearFriction);
        Run("world integrates angular velocity", TestWorldIntegratesAngularVelocity);
        Run("world integrates torque", TestWorldIntegratesTorque);
        Run("off center contact applies angular impulse", TestOffCenterContactAppliesAngularImpulse);

        Console.WriteLine($"Passed {passed} tests.");
    }

    private static void TestFix4Csum()
    {
        AssertEqual(new fix(10), math.csum(new fix4(1, 2, 3, 4)));
    }

    private static void TestFixSqrt()
    {
        AssertEqual(fix._0_5, math.sqrt(fix._0_25));
        AssertEqual(new fix(2), math.sqrt(new fix(4)));
    }

    private static void TestFix3x3MatrixMul()
    {
        fix3x3 a = new fix3x3(
            1, 2, 3,
            4, 5, 6,
            7, 8, 9);

        fix3x3 b = new fix3x3(
            9, 8, 7,
            6, 5, 4,
            3, 2, 1);

        fix3x3 result = math.mul(a, b);

        AssertEqual(new fix3(30, 84, 138), result.c0);
        AssertEqual(new fix3(24, 69, 114), result.c1);
        AssertEqual(new fix3(18, 54, 90), result.c2);
    }

    private static void TestFix4x4MatrixMul()
    {
        fix4x4 trs = math.TRS(new fix3(1, 2, 3), quaternion.identity, new fix3(2, 3, 4));

        AssertEqual(trs, math.mul(fix4x4.identity, trs));
        AssertEqual(trs, math.mul(trs, fix4x4.identity));
        AssertEqual(new fix3(3, 5, 7), math.transform(trs, fix3.one));
    }

    private static void TestCapsuleCapsuleOverlap()
    {
        Capsule vertical = new Capsule(fix3.zero, fix._0_5, 4, quaternion.identity, fix3.up);
        Capsule horizontal = new Capsule(fix3.zero, fix._0_5, 4, quaternion.RotateZ(math.PI * fix._0_5), fix3.up);
        Capsule far = new Capsule(new fix3(3, 0, 0), fix._0_5, 4, quaternion.identity, fix3.up);

        AssertTrue(PhysicsApi.Overlaps(vertical, horizontal), "crossing capsules should overlap");
        AssertFalse(PhysicsApi.Overlaps(vertical, far), "parallel capsules outside combined radius should not overlap");
    }

    private static void TestBoxOverlap()
    {
        OBB a = new OBB(fix3.zero, new fix3(2, 2, 2), quaternion.identity);
        OBB touching = new OBB(new fix3(1, 0, 0), new fix3(2, 2, 2), quaternion.RotateZ(math.PI * fix._0_25));
        OBB separated = new OBB(new fix3(5, 0, 0), new fix3(2, 2, 2), quaternion.RotateZ(math.PI * fix._0_25));

        AssertTrue(PhysicsApi.Overlaps(a, touching), "near OBBs should overlap");
        AssertFalse(PhysicsApi.Overlaps(a, separated), "separated OBBs should not overlap");
    }

    private static void TestSphereCapsuleTouching()
    {
        Sphere sphere = new Sphere(new fix3(1, 0, 0), fix._0_5);
        Capsule capsule = new Capsule(fix3.zero, fix._0_5, 4, quaternion.identity, fix3.up);

        AssertTrue(PhysicsApi.Overlaps(sphere, capsule), "touching sphere capsule pair should overlap");
        AssertTrue(PhysicsApi.TryComputeContact(sphere, capsule, out CollisionInfo collision), "touching pair should produce contact");
        AssertEqual(fix.Zero, collision.penetrationDepth);
    }

    private static void TestOBBSupportPoint()
    {
        quaternion rotation = quaternion.RotateZ(math.PI * fix._0_25);
        OBB obb = new OBB(fix3.zero, new fix3(2, 2, 2), rotation);

        fix3 expected = rotation * new fix3(fix.One, -fix.One, fix.One);
        AssertEqual(expected, PhysicsApi.GetSupportPoint(obb, fix3.right));
    }

    private static void TestSphereAABBContact()
    {
        Sphere sphere = new Sphere(new fix3(fix._1_5, fix.Zero, fix.Zero), fix.One);
        AABB aabb = new AABB(fix3.zero, new fix3(2, 2, 2));

        AssertTrue(PhysicsApi.TryComputeContact(sphere, aabb, out CollisionInfo collision), "sphere should overlap AABB");
        AssertEqual(fix._0_5, collision.penetrationDepth);
        AssertEqual(fix3.left, collision.normal);
    }

    private static void TestWorldTransformSync()
    {
        World world = new World(new WorldSettings(false));
        Rigid rigid = world.CreateRigid(new fix3(10, 0, 0), quaternion.identity);
        Collider collider = world.AddSphereCollider(rigid.id, new fix3(1, 0, 0), fix.One);

        world.Update();

        AssertTrue(world.TryGetCollider(collider.id, out Collider synced), "collider should remain registered");
        Sphere sphere = (Sphere)synced.shape;
        AssertEqual(new fix3(11, 0, 0), sphere.Center);
    }

    private static void TestWorldContactManifolds()
    {
        World world = new World(new WorldSettings(false));
        Rigid rigid0 = world.CreateRigid(fix3.zero, quaternion.identity);
        Rigid rigid1 = world.CreateRigid(new fix3(fix._1_5, fix.Zero, fix.Zero), quaternion.identity);
        Collider collider0 = world.AddSphereCollider(rigid0.id, fix3.zero, fix.One);
        Collider collider1 = world.AddSphereCollider(rigid1.id, fix3.zero, fix.One);

        world.Update();

        AssertEqual(1, world.ContactManifolds.Count);
        ContactManifold manifold = world.ContactManifolds[0];
        AssertTrue(manifold.key == new BroadCollisionPairKey(collider0.id, collider1.id), "manifold should keep the broadphase pair key");
        AssertEqual(1, manifold.contactCount);
        AssertEqual(fix._0_5, manifold[0].penetrationDepth);
        AssertEqual(1, manifold[0].lifetime);

        world.Update();

        AssertEqual(1, world.ContactManifolds.Count);
        AssertEqual(2, world.ContactManifolds[0][0].lifetime);

        world.SetTranslation(rigid1.id, new fix3(4, 0, 0));
        world.Update();

        AssertEqual(0, world.ContactManifolds.Count);
    }

    private static void TestWorldIntegratesLinearVelocity()
    {
        World world = new World(new WorldSettings(false));
        Rigid rigid = world.CreateRigid(fix3.zero, quaternion.identity);

        world.SetVelocity(rigid.id, new fix3(2, 0, 0));
        world.Update(fix._0_5);

        AssertTrue(world.TryGetEntity(rigid.id, out Entity entity), "entity should exist");
        AssertEqual(new fix3(1, 0, 0), entity.translation);
        AssertTrue(world.TryGetRigid(rigid.id, out Rigid syncedRigid), "rigid should exist");
        AssertEqual(new fix3(2, 0, 0), syncedRigid.velocity);
    }

    private static void TestWorldResolvesCollisionImpulse()
    {
        WorldSettings settings = new WorldSettings(false);
        settings.positionCorrectionPercent = fix.Zero;
        World world = new World(settings);
        Rigid rigid0 = world.CreateRigid(fix3.zero, quaternion.identity);
        Rigid rigid1 = world.CreateRigid(new fix3(fix._1_5, fix.Zero, fix.Zero), quaternion.identity);
        world.AddSphereCollider(rigid0.id, fix3.zero, fix.One);
        world.AddSphereCollider(rigid1.id, fix3.zero, fix.One);

        world.SetVelocity(rigid0.id, fix3.right);
        world.SetVelocity(rigid1.id, fix3.left);
        world.Update(fix.Zero);

        AssertTrue(world.TryGetRigid(rigid0.id, out Rigid synced0), "first rigid should exist");
        AssertTrue(world.TryGetRigid(rigid1.id, out Rigid synced1), "second rigid should exist");
        AssertEqual(fix3.zero, synced0.velocity);
        AssertEqual(fix3.zero, synced1.velocity);
    }

    private static void TestWorldAppliesPositionCorrection()
    {
        WorldSettings settings = new WorldSettings(false);
        settings.positionCorrectionPercent = fix.One;
        settings.penetrationSlop = fix.Zero;
        World world = new World(settings);
        Rigid rigid0 = world.CreateRigid(fix3.zero, quaternion.identity);
        Rigid rigid1 = world.CreateRigid(new fix3(fix._1_5, fix.Zero, fix.Zero), quaternion.identity);
        world.AddSphereCollider(rigid0.id, fix3.zero, fix.One);
        world.AddSphereCollider(rigid1.id, fix3.zero, fix.One);

        world.Update(fix.Zero);

        AssertTrue(world.TryGetEntity(rigid0.id, out Entity entity0), "first entity should exist");
        AssertTrue(world.TryGetEntity(rigid1.id, out Entity entity1), "second entity should exist");
        AssertEqual(new fix3(-fix._0_25, fix.Zero, fix.Zero), entity0.translation);
        AssertEqual(new fix3(fix._1_5 + fix._0_25, fix.Zero, fix.Zero), entity1.translation);
    }

    private static void TestWorldDispatchesCollisionCallbacks()
    {
        World world = new World(new WorldSettings(false));
        Rigid rigid0 = world.CreateRigid(fix3.zero, quaternion.identity);
        Rigid rigid1 = world.CreateRigid(new fix3(fix._1_5, fix.Zero, fix.Zero), quaternion.identity);
        Collider collider0 = world.AddSphereCollider(rigid0.id, fix3.zero, fix.One);
        Collider collider1 = world.AddSphereCollider(rigid1.id, fix3.zero, fix.One);
        int collider0Count = 0;
        int collider1Count = 0;
        int rigid0Count = 0;
        int rigid1Count = 0;
        CollisionInfo collider0Collision = default;
        CollisionInfo collider1Collision = default;

        collider0.OnCollision += collision =>
        {
            collider0Count++;
            collider0Collision = collision;
        };
        collider1.OnCollision += collision =>
        {
            collider1Count++;
            collider1Collision = collision;
        };
        rigid0.OnCollision += collision => rigid0Count++;
        rigid1.OnCollision += collision => rigid1Count++;

        world.Update(fix.Zero);

        AssertEqual(1, collider0Count);
        AssertEqual(1, collider1Count);
        AssertEqual(1, rigid0Count);
        AssertEqual(1, rigid1Count);
        AssertEqual(collider0.id, collider0Collision.id);
        AssertEqual(collider1.id, collider0Collision.otherId);
        AssertEqual(collider1.id, collider1Collision.id);
        AssertEqual(collider0.id, collider1Collision.otherId);
        AssertEqual(-collider0Collision.normal, collider1Collision.normal);
    }

    private static void TestTriggerReportsContactWithoutSolverResponse()
    {
        WorldSettings settings = new WorldSettings(false);
        settings.positionCorrectionPercent = fix.One;
        settings.penetrationSlop = fix.Zero;
        World world = new World(settings);
        Rigid rigid0 = world.CreateRigid(fix3.zero, quaternion.identity);
        Rigid rigid1 = world.CreateRigid(new fix3(fix._1_5, fix.Zero, fix.Zero), quaternion.identity);
        Collider collider0 = world.AddSphereCollider(rigid0.id, fix3.zero, fix.One);
        Collider collider1 = world.AddSphereCollider(rigid1.id, fix3.zero, fix.One);
        int triggerCount = 0;

        collider1.OnCollision += collision => triggerCount++;
        world.SetColliderTrigger(collider1.id, true);
        world.SetVelocity(rigid0.id, fix3.right);
        world.SetVelocity(rigid1.id, fix3.left);
        world.Update(fix.Zero);

        AssertEqual(1, triggerCount);
        AssertEqual(1, world.ContactManifolds.Count);
        AssertTrue(world.ContactManifolds[0].isTrigger, "trigger manifold should be marked");
        AssertTrue(world.TryGetRigid(rigid0.id, out Rigid synced0), "first rigid should exist");
        AssertTrue(world.TryGetRigid(rigid1.id, out Rigid synced1), "second rigid should exist");
        AssertEqual(fix3.right, synced0.velocity);
        AssertEqual(fix3.left, synced1.velocity);
        AssertTrue(world.TryGetEntity(rigid0.id, out Entity entity0), "first entity should exist");
        AssertTrue(world.TryGetEntity(rigid1.id, out Entity entity1), "second entity should exist");
        AssertEqual(fix3.zero, entity0.translation);
        AssertEqual(new fix3(fix._1_5, fix.Zero, fix.Zero), entity1.translation);
        AssertTrue(collider0.isTrigger == false, "first collider should not be trigger");
        AssertTrue(collider1.isTrigger, "second collider should be trigger");
    }

    private static void TestStaticRigidIgnoresMotionAndForces()
    {
        World world = new World(new WorldSettings(true, -10));
        Rigid rigid = world.CreateRigid(fix3.zero, quaternion.identity);
        Collider collider = world.AddSphereCollider(rigid.id, fix3.zero, fix.One);

        world.SetRigidType(rigid.id, RigidType.Static);
        world.SetVelocity(rigid.id, new fix3(3, 0, 0));
        world.SetAngularVelocity(rigid.id, new fix3(fix.Zero, fix.Zero, math.PI));
        world.SetAcceleration(rigid.id, new fix3(0, 5, 0));
        world.SetAngularAcceleration(rigid.id, new fix3(fix.Zero, fix.Zero, fix.One));
        world.AddForce(rigid.id, new fix3(10, 0, 0));
        world.AddTorque(rigid.id, new fix3(fix.Zero, fix.Zero, fix.One));
        world.Update(fix.One);

        AssertTrue(world.TryGetEntity(rigid.id, out Entity entity), "static entity should exist");
        AssertEqual(fix3.zero, entity.translation);
        AssertEqual(quaternion.identity, entity.orientation);
        AssertTrue(world.TryGetRigid(rigid.id, out Rigid syncedRigid), "static rigid should exist");
        AssertEqual(fix3.zero, syncedRigid.force);
        AssertEqual(fix3.zero, syncedRigid.torque);
        AssertTrue(world.TryGetCollider(collider.id, out Collider syncedCollider), "static collider should exist");
        Sphere sphere = (Sphere)syncedCollider.shape;
        AssertEqual(fix3.zero, sphere.Center);
    }

    private static void TestDynamicBodyResolvesAgainstStaticBody()
    {
        WorldSettings settings = new WorldSettings(false);
        settings.positionCorrectionPercent = fix.One;
        settings.penetrationSlop = fix.Zero;
        World world = new World(settings);
        Rigid staticRigid = world.CreateRigid(fix3.zero, quaternion.identity);
        Rigid dynamicRigid = world.CreateRigid(new fix3(fix._1_5, fix.Zero, fix.Zero), quaternion.identity);
        world.AddSphereCollider(staticRigid.id, fix3.zero, fix.One);
        world.AddSphereCollider(dynamicRigid.id, fix3.zero, fix.One);

        world.SetRigidType(staticRigid.id, RigidType.Static);
        world.SetVelocity(dynamicRigid.id, fix3.left);
        world.Update(fix.Zero);

        AssertTrue(world.TryGetEntity(staticRigid.id, out Entity staticEntity), "static entity should exist");
        AssertTrue(world.TryGetEntity(dynamicRigid.id, out Entity dynamicEntity), "dynamic entity should exist");
        AssertEqual(fix3.zero, staticEntity.translation);
        AssertEqual(new fix3(2, 0, 0), dynamicEntity.translation);
        AssertTrue(world.TryGetRigid(dynamicRigid.id, out Rigid syncedDynamic), "dynamic rigid should exist");
        AssertEqual(fix3.zero, syncedDynamic.velocity);
    }

    private static void TestKinematicRigidMovesWithoutForceIntegration()
    {
        World world = new World(new WorldSettings(true, -10));
        Rigid rigid = world.CreateRigid(fix3.zero, quaternion.identity);

        world.SetRigidType(rigid.id, RigidType.Kinematic);
        world.SetVelocity(rigid.id, new fix3(2, 0, 0));
        world.SetAngularVelocity(rigid.id, new fix3(fix.Zero, fix.Zero, math.PI));
        world.SetAcceleration(rigid.id, new fix3(0, 5, 0));
        world.SetAngularAcceleration(rigid.id, new fix3(fix.Zero, fix.Zero, fix.One));
        world.AddForce(rigid.id, new fix3(10, 0, 0));
        world.AddTorque(rigid.id, new fix3(fix.Zero, fix.Zero, fix.One));
        world.Update(fix._0_5);

        AssertTrue(world.TryGetEntity(rigid.id, out Entity entity), "kinematic entity should exist");
        AssertEqual(new fix3(1, 0, 0), entity.translation);
        AssertNear(fix3.up, entity.orientation * fix3.right, fix._0_01);
        AssertTrue(world.TryGetRigid(rigid.id, out Rigid syncedRigid), "kinematic rigid should exist");
        AssertEqual(new fix3(2, 0, 0), syncedRigid.velocity);
        AssertEqual(new fix3(fix.Zero, fix.Zero, math.PI), syncedRigid.angularVelocity);
        AssertEqual(fix3.zero, syncedRigid.force);
        AssertEqual(fix3.zero, syncedRigid.torque);
    }

    private static void TestKinematicBodyPushesDynamicBody()
    {
        WorldSettings settings = new WorldSettings(false);
        settings.positionCorrectionPercent = fix.Zero;
        World world = new World(settings);
        Rigid kinematicRigid = world.CreateRigid(fix3.zero, quaternion.identity);
        Rigid dynamicRigid = world.CreateRigid(new fix3(fix._1_5, fix.Zero, fix.Zero), quaternion.identity);
        world.AddSphereCollider(kinematicRigid.id, fix3.zero, fix.One);
        world.AddSphereCollider(dynamicRigid.id, fix3.zero, fix.One);

        world.SetRigidType(kinematicRigid.id, RigidType.Kinematic);
        world.SetVelocity(kinematicRigid.id, fix3.right);
        world.Update(fix.Zero);

        AssertTrue(world.TryGetRigid(kinematicRigid.id, out Rigid syncedKinematic), "kinematic rigid should exist");
        AssertTrue(world.TryGetRigid(dynamicRigid.id, out Rigid syncedDynamic), "dynamic rigid should exist");
        AssertEqual(fix3.right, syncedKinematic.velocity);
        AssertEqual(fix3.right, syncedDynamic.velocity);
    }

    private static void TestSolverAppliesLinearFriction()
    {
        WorldSettings settings = new WorldSettings(false);
        settings.friction = fix.One;
        settings.positionCorrectionPercent = fix.Zero;
        World world = new World(settings);
        Rigid dynamicRigid = world.CreateRigid(fix3.zero, quaternion.identity);
        Rigid staticRigid = world.CreateRigid(new fix3(fix.Zero, fix._1_5, fix.Zero), quaternion.identity);
        world.AddSphereCollider(dynamicRigid.id, fix3.zero, fix.One);
        world.AddSphereCollider(staticRigid.id, fix3.zero, fix.One);

        world.SetRigidType(staticRigid.id, RigidType.Static);
        world.SetInertia(dynamicRigid.id, fix3.zero);
        world.SetVelocity(dynamicRigid.id, new fix3(1, 1, 0));
        world.Update(fix.Zero);

        AssertTrue(world.TryGetRigid(dynamicRigid.id, out Rigid syncedDynamic), "dynamic rigid should exist");
        AssertEqual(fix3.zero, syncedDynamic.velocity);
    }

    private static void TestWorldIntegratesAngularVelocity()
    {
        World world = new World(new WorldSettings(false));
        Rigid rigid = world.CreateRigid(fix3.zero, quaternion.identity);

        world.SetAngularVelocity(rigid.id, new fix3(fix.Zero, fix.Zero, math.PI));
        world.Update(fix._0_5);

        AssertTrue(world.TryGetEntity(rigid.id, out Entity entity), "entity should exist");
        AssertNear(fix3.up, entity.orientation * fix3.right, fix._0_01);
    }

    private static void TestWorldIntegratesTorque()
    {
        World world = new World(new WorldSettings(false));
        Rigid rigid = world.CreateRigid(fix3.zero, quaternion.identity);

        world.SetInertia(rigid.id, new fix3(2, 2, 2));
        world.AddTorque(rigid.id, new fix3(fix.Zero, fix.Zero, fix._2));
        world.Update(fix.One);

        AssertTrue(world.TryGetRigid(rigid.id, out Rigid syncedRigid), "rigid should exist");
        AssertEqual(new fix3(fix.Zero, fix.Zero, fix.One), syncedRigid.angularVelocity);
        AssertEqual(fix3.zero, syncedRigid.torque);
    }

    private static void TestOffCenterContactAppliesAngularImpulse()
    {
        WorldSettings settings = new WorldSettings(false);
        settings.friction = fix.Zero;
        settings.positionCorrectionPercent = fix.Zero;
        World world = new World(settings);
        Rigid boxRigid = world.CreateRigid(fix3.zero, quaternion.identity);
        Rigid sphereRigid = world.CreateRigid(new fix3(fix._1_5, fix._0_5, fix.Zero), quaternion.identity);
        world.AddAABBCollider(boxRigid.id, fix3.zero, new fix3(2, 2, 2));
        world.AddSphereCollider(sphereRigid.id, fix3.zero, fix.One);

        world.SetRigidType(sphereRigid.id, RigidType.Static);
        world.SetVelocity(boxRigid.id, fix3.right);
        world.Update(fix.Zero);

        AssertTrue(world.TryGetRigid(boxRigid.id, out Rigid syncedBox), "box rigid should exist");
        AssertTrue(syncedBox.angularVelocity.z > fix.Zero, "off center normal impulse should spin the dynamic body");
    }

    private static void Run(string name, Action test)
    {
        try
        {
            test();
            passed++;
            Console.WriteLine($"PASS {name}");
        }
        catch (Exception ex)
        {
            throw new InvalidOperationException($"FAIL {name}: {ex.Message}", ex);
        }
    }

    private static void AssertTrue(bool condition, string message)
    {
        if (!condition)
        {
            throw new InvalidOperationException(message);
        }
    }

    private static void AssertFalse(bool condition, string message)
    {
        if (condition)
        {
            throw new InvalidOperationException(message);
        }
    }

    private static void AssertEqual(fix expected, fix actual)
    {
        if (expected != actual)
        {
            throw new InvalidOperationException($"Expected {expected}, got {actual}");
        }
    }

    private static void AssertEqual(fix3 expected, fix3 actual)
    {
        if (expected != actual)
        {
            throw new InvalidOperationException($"Expected {expected}, got {actual}");
        }
    }

    private static void AssertEqual(fix4x4 expected, fix4x4 actual)
    {
        if (!expected.Equals(actual))
        {
            throw new InvalidOperationException($"Expected {expected}, got {actual}");
        }
    }

    private static void AssertEqual(quaternion expected, quaternion actual)
    {
        if (expected != actual)
        {
            throw new InvalidOperationException($"Expected {expected.value}, got {actual.value}");
        }
    }

    private static void AssertNear(fix3 expected, fix3 actual, fix tolerance)
    {
        fix3 delta = math.abs(expected - actual);
        if (delta.x > tolerance || delta.y > tolerance || delta.z > tolerance)
        {
            throw new InvalidOperationException($"Expected {expected}, got {actual}, tolerance {tolerance}");
        }
    }

    private static void AssertEqual(int expected, int actual)
    {
        if (expected != actual)
        {
            throw new InvalidOperationException($"Expected {expected}, got {actual}");
        }
    }

    private static void AssertEqual(ulong expected, ulong actual)
    {
        if (expected != actual)
        {
            throw new InvalidOperationException($"Expected {expected}, got {actual}");
        }
    }
}
