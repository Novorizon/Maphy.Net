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
        World world = new World();
        Rigid rigid = world.CreateRigid(new fix3(10, 0, 0), quaternion.identity);
        Collider collider = world.AddSphereCollider(rigid.id, new fix3(1, 0, 0), fix.One);

        world.Update();

        AssertTrue(world.TryGetCollider(collider.id, out Collider synced), "collider should remain registered");
        Sphere sphere = (Sphere)synced.shape;
        AssertEqual(new fix3(11, 0, 0), sphere.Center);
    }

    private static void TestWorldContactManifolds()
    {
        World world = new World();
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

    private static void AssertEqual(int expected, int actual)
    {
        if (expected != actual)
        {
            throw new InvalidOperationException($"Expected {expected}, got {actual}");
        }
    }
}
