using System;
using System.Collections.Generic;
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
        Run("fix arithmetic reports overflow safely", TestFixArithmeticReportsOverflowSafely);
        Run("fix3x3 mul is matrix multiplication", TestFix3x3MatrixMul);
        Run("fix4x4 mul preserves TRS with identity", TestFix4x4MatrixMul);
        Run("capsule capsule overlap uses segment distance", TestCapsuleCapsuleOverlap);
        Run("box overlap handles OBB separation", TestBoxOverlap);
        Run("sphere capsule touching counts as contact", TestSphereCapsuleTouching);
        Run("OBB support point uses oriented box", TestOBBSupportPoint);
        Run("sphere AABB contact reports penetration", TestSphereAABBContact);
        Run("dynamic AABB tree query updates moved proxy", TestDynamicAABBTreeUpdatesMovedProxy);
        Run("dynamic AABB tree keeps height bounded", TestDynamicAABBTreeKeepsHeightBounded);
        Run("world update syncs collider transform", TestWorldTransformSync);
        Run("physics world core integrates like object world", TestPhysicsWorldCoreIntegratesLikeObjectWorld);
        Run("physics world core builds contact manifolds", TestPhysicsWorldCoreBuildsContactManifolds);
        Run("physics world core resolves contact velocity", TestPhysicsWorldCoreResolvesContactVelocity);
        Run("physics world core corrects contact positions", TestPhysicsWorldCoreCorrectsContactPositions);
        Run("physics world core broadphase tree updates moving pairs", TestPhysicsWorldCoreBroadphaseTreeUpdatesMovingPairs);
        Run("physics world core broadphase tree keeps height bounded", TestPhysicsWorldCoreBroadphaseTreeKeepsHeightBounded);
        Run("physics world core raycast non alloc returns sorted hits", TestPhysicsWorldCoreRaycastNonAllocReturnsSortedHits);
        Run("physics world core shape cast non alloc reports sorted hits", TestPhysicsWorldCoreShapeCastNonAllocReportsSortedHits);
        Run("physics world core query and broadphase do not allocate", TestPhysicsWorldCoreQueryAndBroadphaseDoNotAllocate);
        Run("physics world core large broadphase query does not allocate", TestPhysicsWorldCoreLargeBroadphaseQueryDoesNotAllocate);
        Run("physics world core removes stale persistent broadphase proxies", TestPhysicsWorldCoreRemovesStalePersistentBroadphaseProxies);
        Run("physics world core replay hash is deterministic", TestPhysicsWorldCoreReplayHashIsDeterministic);
        Run("physics world core compound mass aggregates colliders", TestPhysicsWorldCoreCompoundMassAggregatesColliders);
        Run("physics world core builds islands and sleeps", TestPhysicsWorldCoreBuildsIslandsAndSleeps);
        Run("physics world core CCD stops fast body", TestPhysicsWorldCoreCCDStopsFastBody);
        Run("physics world core dynamic CCD handles dynamic target", TestPhysicsWorldCoreDynamicCCDHandlesDynamicTarget);
        Run("physics world core material affects solver", TestPhysicsWorldCoreMaterialAffectsSolver);
        Run("physics world core CCD and sleep do not allocate", TestPhysicsWorldCoreCCDAndSleepDoNotAllocate);
        Run("physics world core exposes step stats and island ids", TestPhysicsWorldCoreExposesStepStatsAndIslandIds);
        Run("physics world core exposes capacity and memory budget", TestPhysicsWorldCoreExposesCapacityAndMemoryBudget);
        Run("physics world core reports capacity overflow in stats", TestPhysicsWorldCoreReportsCapacityOverflowInStats);
        Run("physics world core reports performance counters", TestPhysicsWorldCoreReportsPerformanceCounters);
        Run("physics world core dirty sync updates changed colliders only", TestPhysicsWorldCoreDirtySyncUpdatesChangedCollidersOnly);
        Run("physics world core auto mass can be re-enabled", TestPhysicsWorldCoreAutoMassCanBeReenabled);
        Run("physics world core long replay with lifecycle is deterministic", TestPhysicsWorldCoreLongReplayWithLifecycleIsDeterministic);
        Run("physics world core clamps unsafe motion values", TestPhysicsWorldCoreClampsUnsafeMotionValues);
        Run("physics world core stacked boxes stay separated", TestPhysicsWorldCoreStackedBoxesStaySeparated);
        Run("physics world core CCD sweeps capsule and compound colliders", TestPhysicsWorldCoreCCDSweepsCapsuleAndCompoundColliders);
        Run("physics world core contact lifetime persists", TestPhysicsWorldCoreContactLifetimePersists);
        Run("broadphase tree updates moving pairs", TestBroadphaseTreeUpdatesMovingPairs);
        Run("broadphase removes stale tree proxies", TestBroadphaseRemovesStaleTreeProxies);
        Run("world AABB query uses broadphase tree", TestWorldAABBQueryUsesBroadphaseTree);
        Run("collision filtering uses layer masks", TestCollisionFilteringUsesLayerMasks);
        Run("queries respect layer masks", TestQueriesRespectLayerMasks);
        Run("physics raycast hits supported shapes", TestPhysicsRaycastHitsSupportedShapes);
        Run("world raycast returns nearest hit", TestWorldRaycastReturnsNearestHit);
        Run("world builds contact manifolds", TestWorldContactManifolds);
        Run("contact manifold matches persistent anchors", TestContactManifoldMatchesPersistentAnchors);
        Run("contact solver preserves warm start impulses", TestContactSolverPreservesWarmStartImpulses);
        Run("box face contact builds multi point manifold", TestBoxFaceContactBuildsMultiPointManifold);
        Run("OBB contact uses oriented SAT normal", TestOBBContactUsesOrientedSATNormal);
        Run("SAT narrow phase can be selected", TestSATNarrowPhaseCanBeSelected);
        Run("GJK narrow phase can be selected", TestGJKNarrowPhaseCanBeSelected);
        Run("GJK EPA handles rotated box boundary", TestGJKEPAHandlesRotatedBoxBoundary);
        Run("GJK EPA warm path does not allocate", TestGJKEPAWarmPathDoesNotAllocate);
        Run("capsule OBB contact uses segment box distance", TestCapsuleOBBContactUsesSegmentBoxDistance);
        Run("world integrates linear velocity", TestWorldIntegratesLinearVelocity);
        Run("world fixed step accumulator advances deterministically", TestWorldFixedStepAccumulatorAdvancesDeterministically);
        Run("world state hash is replay deterministic", TestWorldStateHashIsReplayDeterministic);
        Run("world state hash includes constraint parameters", TestWorldStateHashIncludesConstraintParameters);
        Run("world long replay hash is deterministic", TestWorldLongReplayHashIsDeterministic);
        Run("world resolves collision impulse", TestWorldResolvesCollisionImpulse);
        Run("world applies position correction", TestWorldAppliesPositionCorrection);
        Run("position correction can be clamped", TestPositionCorrectionCanBeClamped);
        Run("stacked boxes stay separated under gravity", TestStackedBoxesStaySeparatedUnderGravity);
        Run("world dispatches collision callbacks", TestWorldDispatchesCollisionCallbacks);
        Run("trigger reports contact without solver response", TestTriggerReportsContactWithoutSolverResponse);
        Run("collision events report enter stay exit", TestCollisionEventsReportEnterStayExit);
        Run("trigger events report enter stay exit", TestTriggerEventsReportEnterStayExit);
        Run("disabled and removed objects clear collision state", TestDisabledAndRemovedObjectsClearCollisionState);
        Run("disabled rigid clears collision state", TestDisabledRigidClearsCollisionState);
        Run("removing rigid removes colliders and collision state", TestRemovingRigidRemovesCollidersAndCollisionState);
        Run("distance constraint preserves rigid distance", TestDistanceConstraintPreservesRigidDistance);
        Run("disabled rigid disables distance constraint", TestDisabledRigidDisablesDistanceConstraint);
        Run("removing rigid removes constraints", TestRemovingRigidRemovesConstraints);
        Run("point constraint pins anchors", TestPointConstraintPinsAnchors);
        Run("spring distance constraint pulls toward rest length", TestSpringDistanceConstraintPullsTowardRestLength);
        Run("hinge constraint pins anchors", TestHingeConstraintPinsAnchors);
        Run("fixed constraint locks relative pose", TestFixedConstraintLocksRelativePose);
        Run("slider constraint preserves slide axis", TestSliderConstraintPreservesSlideAxis);
        Run("static rigid ignores motion and forces", TestStaticRigidIgnoresMotionAndForces);
        Run("dynamic body resolves against static body", TestDynamicBodyResolvesAgainstStaticBody);
        Run("kinematic rigid moves without force integration", TestKinematicRigidMovesWithoutForceIntegration);
        Run("kinematic body pushes dynamic body", TestKinematicBodyPushesDynamicBody);
        Run("solver applies linear friction", TestSolverAppliesLinearFriction);
        Run("solver applies two axis friction", TestSolverAppliesTwoAxisFriction);
        Run("restitution threshold suppresses low speed bounce", TestRestitutionThresholdSuppressesLowSpeedBounce);
        Run("material bounciness affects restitution", TestMaterialBouncinessAffectsRestitution);
        Run("material friction affects solver", TestMaterialFrictionAffectsSolver);
        Run("world integrates angular velocity", TestWorldIntegratesAngularVelocity);
        Run("world integrates torque", TestWorldIntegratesTorque);
        Run("shape and constraint frameworks expose capabilities", TestShapeAndConstraintFrameworksExposeCapabilities);
        Run("physics shape cast reports time of impact", TestPhysicsShapeCastReportsTimeOfImpact);
        Run("physics shape cast sweeps capsule targets", TestPhysicsShapeCastSweepsCapsuleTargets);
        Run("physics shape cast sweeps OBB with SAT", TestPhysicsShapeCastSweepsOBBWithSAT);
        Run("world clamps unsafe motion values", TestWorldClampsUnsafeMotionValues);
        Run("capsule zero axis stays numerically safe", TestCapsuleZeroAxisStaysNumericallySafe);
        Run("solver clamps contact impulses", TestSolverClampsContactImpulses);
        Run("solver contact velocity bias separates penetration", TestSolverContactVelocityBiasSeparatesPenetration);
        Run("contact manifold settings control persistence", TestContactManifoldSettingsControlPersistence);
        Run("world reports step stats after update", TestWorldReportsStepStatsAfterUpdate);
        Run("world warm update allocation baseline stays low", TestWorldWarmUpdateAllocationBaselineStaysLow);
        Run("collision callback lifecycle changes are deferred safely", TestCollisionCallbackLifecycleChangesAreDeferredSafely);
        Run("collision callback can create bodies safely", TestCollisionCallbackCanCreateBodiesSafely);
        Run("collision callback exceptions can be captured", TestCollisionCallbackExceptionsCanBeCaptured);
        Run("CCD stops fast body before static collider", TestCCDStopsFastBodyBeforeStaticCollider);
        Run("CCD sweeps capsule against static collider", TestCCDSweepsCapsuleAgainstStaticCollider);
        Run("CCD uses earliest compound collider hit", TestCCDUsesEarliestCompoundColliderHit);
        Run("CCD sweeps sphere against sphere", TestCCDSweepsSphereAgainstSphere);
        Run("CCD sweeps dynamic sphere against dynamic sphere", TestCCDSweepsDynamicSphereAgainstDynamicSphere);
        Run("off center contact applies angular impulse", TestOffCenterContactAppliesAngularImpulse);
        Run("collider initializes rigid mass properties", TestColliderInitializesRigidMassProperties);
        Run("collider density updates automatic mass properties", TestColliderDensityUpdatesAutomaticMassProperties);
        Run("manual mass properties survive density changes", TestManualMassPropertiesSurviveDensityChanges);
        Run("rigid supports multiple colliders", TestRigidSupportsMultipleColliders);
        Run("compound body aggregates automatic mass properties", TestCompoundBodyAggregatesAutomaticMassProperties);
        Run("world builds islands from contacts and constraints", TestWorldBuildsIslandsFromContactsAndConstraints);
        Run("sleeping island skips integration and wakes", TestSleepingIslandSkipsIntegrationAndWakes);

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

    private static void TestFixArithmeticReportsOverflowSafely()
    {
        fix large = fix.Raw(long.MaxValue / 2);

        AssertEqual(fix.NaN, fix.Max + fix.One);
        AssertEqual(fix.NaN, large * new fix(3));
        AssertEqual(fix.NaN, large / fix._0_0001);
        AssertFalse(fix.IsNaN(math.sqrt(fix.Max)), "sqrt should not overflow its fixed-point scaling path");
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

    private static void TestDynamicAABBTreeUpdatesMovedProxy()
    {
        DynamicAABBTree tree = new DynamicAABBTree(fix._0_1);
        Collider collider = new Collider();
        collider.AddSphereCollider(1, fix3.zero, fix.One);
        BroadphaseProxy proxy = new BroadphaseProxy(collider, new AABB(fix3.zero, new fix3(2, 2, 2)));
        List<BroadphaseProxy> results = new List<BroadphaseProxy>();

        tree.CreateProxy(proxy);
        tree.Query(new AABB(fix3.zero, new fix3(1, 1, 1)), results);

        AssertEqual(1, results.Count);
        AssertEqual(collider.id, results[0].colliderId);

        proxy = new BroadphaseProxy(collider, new AABB(new fix3(5, 0, 0), new fix3(2, 2, 2)));
        tree.MoveProxy(proxy);

        results.Clear();
        tree.Query(new AABB(fix3.zero, new fix3(1, 1, 1)), results);
        AssertEqual(0, results.Count);

        results.Clear();
        tree.Query(new AABB(new fix3(5, 0, 0), new fix3(1, 1, 1)), results);
        AssertEqual(1, results.Count);
        AssertEqual(collider.id, results[0].colliderId);
    }

    private static void TestDynamicAABBTreeKeepsHeightBounded()
    {
        DynamicAABBTree tree = new DynamicAABBTree(fix._0_1);

        for (int i = 0; i < 32; i++)
        {
            Collider collider = new Collider();
            collider.AddSphereCollider((ulong)(i + 1), new fix3(i * 3, 0, 0), fix._0_5);
            AABB bounds = new AABB(new fix3(i * 3, 0, 0), new fix3(1, 1, 1));
            tree.CreateProxy(new BroadphaseProxy(collider, bounds));
        }

        AssertEqual(32, tree.ProxyCount);
        AssertTrue(tree.Height <= 8, "balanced tree height should stay bounded for ordered inserts");
        AssertTrue(tree.MaxBalance <= 1, "balanced tree should not contain a heavily skewed internal node");
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

    private static void TestPhysicsWorldCoreIntegratesLikeObjectWorld()
    {
        World objectWorld = new World(new WorldSettings(false));
        Rigid objectRigid = objectWorld.CreateRigid(fix3.zero, quaternion.identity);
        Collider objectCollider = objectWorld.AddSphereCollider(objectRigid.id, fix3.right, fix.One);
        objectWorld.SetVelocity(objectRigid.id, new fix3(2, 0, 0));

        PhysicsWorld coreWorld = new PhysicsWorld(new PhysicsWorldSettings(false));
        coreWorld.Reserve(1, 1, 1);
        AssertTrue(coreWorld.CreateBody(fix3.zero, quaternion.identity, RigidType.Dynamic, out BodyHandle body), "core body should be created");
        AssertTrue(coreWorld.AddSphere(body, fix3.right, fix.One, out ColliderHandle collider), "core collider should be created");
        AssertTrue(coreWorld.SetBodyVelocity(body, new fix3(2, 0, 0)), "core velocity should be set");

        objectWorld.Update(fix._0_5);
        coreWorld.Update(fix._0_5);

        AssertTrue(objectWorld.TryGetEntity(objectRigid.id, out Entity objectEntity), "object world entity should exist");
        AssertTrue(objectWorld.TryGetCollider(objectCollider.id, out Collider syncedObjectCollider), "object world collider should exist");
        AssertTrue(coreWorld.TryGetBody(body, out PhysicsWorldBody coreBody), "core body should exist");
        AssertTrue(coreWorld.TryGetCollider(collider, out PhysicsWorldCollider coreCollider), "core collider should exist");

        Sphere objectSphere = (Sphere)syncedObjectCollider.shape;
        AssertEqual(objectEntity.translation, coreBody.position);
        AssertEqual(objectSphere.Center, coreCollider.worldShape.sphere.Center);
    }

    private static void TestPhysicsWorldCoreBuildsContactManifolds()
    {
        PhysicsWorld world = new PhysicsWorld(new PhysicsWorldSettings(false));
        world.Reserve(2, 2, 4, 4);
        AssertTrue(world.CreateBody(fix3.zero, quaternion.identity, RigidType.Dynamic, out BodyHandle body0), "first body should be created");
        AssertTrue(world.CreateBody(new fix3(fix._1_5, fix.Zero, fix.Zero), quaternion.identity, RigidType.Dynamic, out BodyHandle body1), "second body should be created");
        AssertTrue(world.AddAABB(body0, fix3.zero, new fix3(2, 2, 2), out ColliderHandle collider0), "first AABB should be created");
        AssertTrue(world.AddAABB(body1, fix3.zero, new fix3(2, 2, 2), out ColliderHandle collider1), "second AABB should be created");

        world.Update(fix.Zero);

        AssertEqual(1, world.PairCount);
        AssertEqual(1, world.ContactManifoldCount);
        AssertFalse(world.ContactOverflowed, "core contact manifold buffer should not overflow");
        AssertTrue(world.TryGetContactManifold(0, out PhysicsWorldManifold manifold), "core contact manifold should be readable");
        AssertTrue(manifold.collider0 == collider0 && manifold.collider1 == collider1, "manifold should keep stable collider handles");
        AssertEqual(fix3.right, manifold.normal);
        AssertEqual(4, manifold.contactCount);
        AssertEqual(fix._0_5, manifold[0].penetrationDepth);

        AssertTrue(world.SetColliderEnabled(collider1, false), "disabling collider should succeed");
        AssertEqual(0, world.ContactManifoldCount);
        world.Update(fix.Zero);

        AssertEqual(0, world.PairCount);
        AssertEqual(0, world.ContactManifoldCount);
    }

    private static void TestPhysicsWorldCoreResolvesContactVelocity()
    {
        PhysicsWorldSettings settings = new PhysicsWorldSettings(false);
        settings.positionCorrectionPercent = fix.Zero;
        settings.friction = fix.Zero;
        PhysicsWorld world = new PhysicsWorld(settings);
        world.Reserve(2, 2, 4, 4);
        AssertTrue(world.CreateBody(fix3.zero, quaternion.identity, RigidType.Dynamic, out BodyHandle dynamicBody), "dynamic body should be created");
        AssertTrue(world.CreateBody(new fix3(fix._1_5, fix.Zero, fix.Zero), quaternion.identity, RigidType.Static, out BodyHandle staticBody), "static body should be created");
        AssertTrue(world.AddSphere(dynamicBody, fix3.zero, fix.One, out ColliderHandle _), "dynamic sphere should be created");
        AssertTrue(world.AddSphere(staticBody, fix3.zero, fix.One, out ColliderHandle _), "static sphere should be created");
        AssertTrue(world.SetBodyVelocity(dynamicBody, fix3.right), "dynamic body velocity should be set");

        world.Update(fix.Zero);

        AssertTrue(world.TryGetBody(dynamicBody, out PhysicsWorldBody body), "dynamic body should still exist");
        AssertNear(fix3.zero, body.velocity, fix._0_0001);
        AssertEqual(1, world.ContactManifoldCount);
        AssertTrue(world.TryGetContactManifold(0, out PhysicsWorldManifold manifold), "contact manifold should be readable");
        AssertTrue(manifold[0].normalImpulse > fix.Zero, "normal impulse should be accumulated for warm start");
    }

    private static void TestPhysicsWorldCoreCorrectsContactPositions()
    {
        PhysicsWorldSettings settings = new PhysicsWorldSettings(false);
        settings.penetrationSlop = fix.Zero;
        settings.positionCorrectionPercent = fix.One;
        settings.friction = fix.Zero;
        PhysicsWorld world = new PhysicsWorld(settings);
        world.Reserve(2, 2, 4, 4);
        AssertTrue(world.CreateBody(fix3.zero, quaternion.identity, RigidType.Dynamic, out BodyHandle dynamicBody), "dynamic body should be created");
        AssertTrue(world.CreateBody(new fix3(fix._1_5, fix.Zero, fix.Zero), quaternion.identity, RigidType.Static, out BodyHandle staticBody), "static body should be created");
        AssertTrue(world.AddSphere(dynamicBody, fix3.zero, fix.One, out ColliderHandle _), "dynamic sphere should be created");
        AssertTrue(world.AddSphere(staticBody, fix3.zero, fix.One, out ColliderHandle _), "static sphere should be created");

        world.Update(fix.Zero);

        AssertTrue(world.TryGetBody(dynamicBody, out PhysicsWorldBody body), "dynamic body should still exist");
        AssertNear(new fix3(-fix._0_5, fix.Zero, fix.Zero), body.position, fix._0_0001);
    }

    private static void TestPhysicsWorldCoreBroadphaseTreeUpdatesMovingPairs()
    {
        PhysicsWorldSettings settings = new PhysicsWorldSettings(false);
        settings.positionCorrectionPercent = fix.Zero;
        PhysicsWorld world = new PhysicsWorld(settings);
        world.Reserve(4, 4, 8, 8);
        AssertTrue(world.CreateBody(fix3.zero, quaternion.identity, RigidType.Static, out BodyHandle body0), "first body should be created");
        AssertTrue(world.CreateBody(new fix3(5, 0, 0), quaternion.identity, RigidType.Static, out BodyHandle body1), "second body should be created");
        AssertTrue(world.AddSphere(body0, fix3.zero, fix.One, out ColliderHandle collider0), "first collider should be created");
        AssertTrue(world.AddSphere(body1, fix3.zero, fix.One, out ColliderHandle collider1), "second collider should be created");
        AssertTrue(world.SetColliderTrigger(collider0, true), "first collider should be trigger");
        AssertTrue(world.SetColliderTrigger(collider1, true), "second collider should be trigger");

        world.Update(fix.Zero);

        AssertEqual(2, world.BroadphaseTreeProxyCount);
        AssertTrue(world.BroadphaseTreeHeight > 0, "core broadphase tree should be populated");
        AssertEqual(0, world.PairCount);

        AssertTrue(world.SetBodyTransform(body1, new fix3(fix._1_5, fix.Zero, fix.Zero), quaternion.identity), "moving body should update transform");
        world.Update(fix.Zero);

        AssertEqual(2, world.BroadphaseTreeProxyCount);
        AssertEqual(1, world.PairCount);
        AssertTrue(world.TryGetPair(0, out PhysicsWorldPair pair), "moving pair should be generated");
        AssertTrue(pair.collider0 == collider0 && pair.collider1 == collider1, "core pair order should stay stable after tree query");

        AssertTrue(world.SetBodyTransform(body1, new fix3(5, 0, 0), quaternion.identity), "moving body should update transform again");
        world.Update(fix.Zero);

        AssertEqual(2, world.BroadphaseTreeProxyCount);
        AssertEqual(0, world.PairCount);
        AssertEqual(0, world.ContactManifoldCount);
    }

    private static void TestPhysicsWorldCoreBroadphaseTreeKeepsHeightBounded()
    {
        PhysicsWorld world = new PhysicsWorld(new PhysicsWorldSettings(false));
        world.Reserve(32, 32, 64, 64);

        for (int i = 0; i < 32; i++)
        {
            AssertTrue(world.CreateBody(new fix3(i * 3, 0, 0), quaternion.identity, RigidType.Static, out BodyHandle body), "body should be created");
            AssertTrue(world.AddSphere(body, fix3.zero, fix._0_5, out ColliderHandle _), "collider should be created");
        }

        world.Update(fix.Zero);

        AssertEqual(32, world.BroadphaseTreeProxyCount);
        AssertTrue(world.BroadphaseTreeHeight <= 8, "core broadphase tree height should stay bounded for ordered inserts");
        AssertTrue(world.BroadphaseTreeMaxBalance <= 1, "core broadphase tree should stay locally balanced");
        AssertFalse(world.BroadphaseTreeOverflowed, "core broadphase tree should fit in reserved capacity");
    }

    private static void TestPhysicsWorldCoreRaycastNonAllocReturnsSortedHits()
    {
        PhysicsWorld world = new PhysicsWorld(new PhysicsWorldSettings(false));
        world.Reserve(3, 3, 8, 8);
        AssertTrue(world.CreateBody(new fix3(3, 0, 0), quaternion.identity, RigidType.Static, out BodyHandle body0), "first body should be created");
        AssertTrue(world.CreateBody(new fix3(6, 0, 0), quaternion.identity, RigidType.Static, out BodyHandle body1), "second body should be created");
        AssertTrue(world.CreateBody(new fix3(0, 3, 0), quaternion.identity, RigidType.Static, out BodyHandle body2), "third body should be created");
        AssertTrue(world.AddSphere(body0, fix3.zero, fix.One, out ColliderHandle collider0), "first sphere should be created");
        AssertTrue(world.AddSphere(body1, fix3.zero, fix.One, out ColliderHandle collider1), "second sphere should be created");
        AssertTrue(world.AddSphere(body2, fix3.zero, fix.One, out ColliderHandle collider2), "third sphere should be created");
        AssertTrue(world.SetColliderLayer(collider1, 2), "second collider layer should be set");
        PhysicsWorldRaycastHit[] hits = new PhysicsWorldRaycastHit[2];

        world.Update(fix.Zero);

        int count = world.RaycastNonAlloc(new Ray(fix3.zero, fix3.right), new fix(10), hits);
        AssertEqual(2, count);
        AssertFalse(world.RaycastOverflowed, "raycast result buffer should not overflow");
        AssertTrue(hits[0].collider == collider0, "nearest raycast hit should be first");
        AssertTrue(hits[1].collider == collider1, "farther raycast hit should be second");
        AssertEqual(new fix(2), hits[0].distance);
        AssertEqual(new fix(5), hits[1].distance);
        AssertTrue(hits[0].body == body0, "raycast hit should carry the body handle");

        count = world.RaycastNonAlloc(new Ray(fix3.zero, fix3.right), new fix(10), Collider.GetLayerBit(2), hits);
        AssertEqual(1, count);
        AssertTrue(hits[0].collider == collider1, "layer-filtered raycast should return only layer 2");

        count = world.RaycastNonAlloc(new Ray(fix3.zero, fix3.up), new fix(10), hits);
        AssertEqual(1, count);
        AssertTrue(hits[0].collider == collider2, "vertical raycast should hit the third collider");
    }

    private static void TestPhysicsWorldCoreShapeCastNonAllocReportsSortedHits()
    {
        PhysicsWorld world = new PhysicsWorld(new PhysicsWorldSettings(false));
        world.Reserve(3, 3, 8, 8);
        AssertTrue(world.CreateBody(fix3.zero, quaternion.identity, RigidType.Static, out BodyHandle movingBody), "moving body should be created");
        AssertTrue(world.CreateBody(new fix3(3, 0, 0), quaternion.identity, RigidType.Static, out BodyHandle targetBody0), "first target should be created");
        AssertTrue(world.CreateBody(new fix3(6, 0, 0), quaternion.identity, RigidType.Static, out BodyHandle targetBody1), "second target should be created");
        AssertTrue(world.AddSphere(movingBody, fix3.zero, fix._0_5, out ColliderHandle movingCollider), "moving sphere should be created");
        AssertTrue(world.AddAABB(targetBody0, fix3.zero, new fix3(1, 1, 1), out ColliderHandle target0), "first target AABB should be created");
        AssertTrue(world.AddAABB(targetBody1, fix3.zero, new fix3(1, 1, 1), out ColliderHandle target1), "second target AABB should be created");
        PhysicsWorldShapeCastHit[] hits = new PhysicsWorldShapeCastHit[2];

        world.Update(fix.Zero);

        int count = world.ShapeCastNonAlloc(movingCollider, new fix3(8, 0, 0), hits);
        AssertEqual(2, count);
        AssertFalse(world.ShapeCastOverflowed, "shape cast result buffer should not overflow");
        AssertTrue(hits[0].collider == target0, "first shape cast hit should be nearest");
        AssertTrue(hits[1].collider == target1, "second shape cast hit should be farther");
        AssertEqual(fix._0_25, hits[0].fraction);
        AssertEqual(new fix(5) / 8, hits[1].fraction);
        AssertTrue(hits[0].body == targetBody0, "shape cast hit should carry the body handle");
    }

    private static void TestPhysicsWorldCoreQueryAndBroadphaseDoNotAllocate()
    {
        PhysicsWorldSettings settings = new PhysicsWorldSettings(false);
        settings.positionCorrectionPercent = fix.Zero;
        PhysicsWorld world = new PhysicsWorld(settings);
        world.Reserve(4, 4, 8);
        AssertTrue(world.CreateBody(fix3.zero, quaternion.identity, RigidType.Dynamic, out BodyHandle body0), "first body should be created");
        AssertTrue(world.CreateBody(new fix3(fix._1_5, fix.Zero, fix.Zero), quaternion.identity, RigidType.Dynamic, out BodyHandle body1), "second body should be created");
        AssertTrue(world.AddSphere(body0, fix3.zero, fix.One, out ColliderHandle collider0), "first collider should be created");
        AssertTrue(world.AddSphere(body1, fix3.zero, fix.One, out ColliderHandle collider1), "second collider should be created");
        ColliderHandle[] results = new ColliderHandle[4];
        PhysicsWorldRaycastHit[] rayResults = new PhysicsWorldRaycastHit[4];
        PhysicsWorldShapeCastHit[] shapeCastResults = new PhysicsWorldShapeCastHit[4];
        AABB queryBounds = new AABB(fix3.zero, new fix3(4, 4, 4));
        Ray ray = new Ray(new fix3(-5, 0, 0), fix3.right);
        PhysicsShapeData movingShape = PhysicsShapeData.Sphere(new fix3(-4, 0, 0), fix._0_5);

        for (int i = 0; i < 8; i++)
        {
            world.Update(fix.Zero);
            AssertEqual(2, world.QueryAABBNonAlloc(queryBounds, results));
            AssertEqual(2, world.RaycastNonAlloc(ray, new fix(10), rayResults));
            AssertEqual(2, world.ShapeCastNonAlloc(movingShape, new fix3(8, 0, 0), shapeCastResults));
        }

        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 16; i++)
        {
            world.Update(fix.Zero);
            AssertEqual(1, world.PairCount);
            AssertEqual(1, world.ContactManifoldCount);
            AssertEqual(2, world.QueryAABBNonAlloc(queryBounds, results));
            AssertEqual(2, world.RaycastNonAlloc(ray, new fix(10), rayResults));
            AssertEqual(2, world.ShapeCastNonAlloc(movingShape, new fix3(8, 0, 0), shapeCastResults));
        }

        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;
        AssertTrue(allocated == 0, $"core warm update/query allocated {allocated} bytes");
        AssertFalse(world.PairOverflowed, "core broadphase pair buffer should not overflow");
        AssertFalse(world.ContactOverflowed, "core contact manifold buffer should not overflow");
        AssertFalse(world.QueryOverflowed, "core query buffer should not overflow");
        AssertFalse(world.RaycastOverflowed, "core raycast buffer should not overflow");
        AssertFalse(world.ShapeCastOverflowed, "core shape cast buffer should not overflow");
        AssertTrue(world.TryGetPair(0, out PhysicsWorldPair pair), "core broadphase pair should be readable");
        AssertTrue(world.TryGetContactManifold(0, out PhysicsWorldManifold manifold), "core contact manifold should be readable");
        AssertTrue(pair.collider0 == collider0 && pair.collider1 == collider1, "core broadphase pair should be stable by creation order");
        AssertTrue(manifold[0].lifetime > 1, "core contact manifold should preserve point lifetime across frames");
        AssertTrue(results[0] == collider0 && results[1] == collider1, "query results should be stable by storage order");
        AssertTrue(rayResults[0].collider == collider0 && rayResults[1].collider == collider1, "raycast results should be stable by distance");
        AssertTrue(shapeCastResults[0].collider == collider0 && shapeCastResults[1].collider == collider1, "shape cast results should be stable by fraction");
    }

    private static void TestPhysicsWorldCoreLargeBroadphaseQueryDoesNotAllocate()
    {
        const int Count = 64;
        PhysicsWorld world = new PhysicsWorld(new PhysicsWorldSettings(false));
        world.Reserve(PhysicsWorldCapacity.ForBodies(Count, 1, 2));
        for (int i = 0; i < Count; i++)
        {
            int x = i % 8;
            int z = i / 8;
            AssertTrue(world.CreateBody(new fix3(x * 4, 0, z * 4), quaternion.identity, RigidType.Static, out BodyHandle body), "body should be created");
            AssertTrue(world.AddAABB(body, fix3.zero, new fix3(1, 1, 1), out ColliderHandle _), "collider should be created");
        }

        ColliderHandle[] results = new ColliderHandle[Count];
        AABB queryBounds = new AABB(new fix3(14, 0, 14), new fix3(40, 4, 40));
        for (int i = 0; i < 4; i++)
        {
            world.Update(fix.Zero);
            AssertEqual(Count, world.QueryAABBNonAlloc(queryBounds, results));
        }

        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 16; i++)
        {
            world.Update(fix.Zero);
            AssertEqual(Count, world.QueryAABBNonAlloc(queryBounds, results));
        }

        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;
        AssertTrue(allocated == 0, $"core large broadphase/query warm path allocated {allocated} bytes");
        AssertFalse(world.QueryOverflowed, "large query result buffer should fit");
        AssertFalse(world.BroadphaseTreeOverflowed, "large broadphase tree should fit reserved capacity");
        AssertTrue(world.BroadphaseTreeHeight <= 12, "large broadphase tree height should stay bounded");
        AssertTrue(world.LastStepStats.broadphaseCandidateCount >= Count, "broadphase should report visited candidates");
        AssertTrue(world.LastStepStats.broadphaseFilteredCandidateCount >= Count, "broadphase should report filtered candidates");
        AssertEqual(0, world.LastStepStats.narrowPhaseTestCount);
    }

    private static void TestPhysicsWorldCoreRemovesStalePersistentBroadphaseProxies()
    {
        PhysicsWorld world = new PhysicsWorld(new PhysicsWorldSettings(false));
        world.Reserve(3, 3, 8, 8);
        AssertTrue(world.CreateBody(fix3.zero, quaternion.identity, RigidType.Static, out BodyHandle body0), "first body should be created");
        AssertTrue(world.CreateBody(new fix3(3, 0, 0), quaternion.identity, RigidType.Static, out BodyHandle body1), "second body should be created");
        AssertTrue(world.CreateBody(new fix3(6, 0, 0), quaternion.identity, RigidType.Static, out BodyHandle body2), "third body should be created");
        AssertTrue(world.AddSphere(body0, fix3.zero, fix.One, out ColliderHandle collider0), "first collider should be created");
        AssertTrue(world.AddSphere(body1, fix3.zero, fix.One, out ColliderHandle collider1), "second collider should be created");
        AssertTrue(world.AddSphere(body2, fix3.zero, fix.One, out ColliderHandle collider2), "third collider should be created");
        ColliderHandle[] results = new ColliderHandle[4];

        world.Update(fix.Zero);

        AssertEqual(3, world.BroadphaseTreeProxyCount);
        AssertEqual(3, world.QueryAABBNonAlloc(new AABB(new fix3(3, 0, 0), new fix3(10, 2, 2)), results));

        AssertTrue(world.SetColliderEnabled(collider1, false), "disabling collider should remove its proxy");
        AssertEqual(2, world.QueryAABBNonAlloc(new AABB(new fix3(3, 0, 0), new fix3(10, 2, 2)), results));
        AssertEqual(2, world.BroadphaseTreeProxyCount);
        AssertTrue(results[0] == collider0 && results[1] == collider2, "query should skip the disabled collider");

        AssertTrue(world.DestroyCollider(collider2), "destroying collider should remove its proxy");
        AssertEqual(1, world.QueryAABBNonAlloc(new AABB(new fix3(3, 0, 0), new fix3(10, 2, 2)), results));
        AssertEqual(1, world.BroadphaseTreeProxyCount);
        AssertTrue(results[0] == collider0, "query should skip the destroyed collider");
    }

    private static void TestPhysicsWorldCoreReplayHashIsDeterministic()
    {
        PhysicsWorld first = CreateCoreReplayWorld();
        PhysicsWorld second = CreateCoreReplayWorld();

        ulong firstHash = 0;
        ulong secondHash = 0;
        for (int i = 0; i < 64; i++)
        {
            firstHash = first.StepAndComputeStateHash(fix.One / 60);
            secondHash = second.StepAndComputeStateHash(fix.One / 60);
            AssertEqual(firstHash, secondHash);
        }

        AssertEqual(firstHash, first.ComputeStateHash());
        AssertEqual(secondHash, second.ComputeStateHash());
    }

    private static PhysicsWorld CreateCoreReplayWorld()
    {
        PhysicsWorldSettings settings = new PhysicsWorldSettings(false);
        settings.enableCCD = true;
        settings.ccdMinVelocity = fix.Zero;
        settings.solverIterations = 2;
        settings.positionIterations = 2;
        settings.enableSleeping = true;
        settings.sleepTime = new fix(2);
        PhysicsWorld world = new PhysicsWorld(settings);
        world.Reserve(4, 5, 16, 16);
        AssertTrue(world.CreateBody(new fix3(-4, 0, 0), quaternion.identity, RigidType.Dynamic, out BodyHandle moving), "moving body should be created");
        AssertTrue(world.CreateBody(new fix3(0, 0, 0), quaternion.identity, RigidType.Static, out BodyHandle wall), "static body should be created");
        AssertTrue(world.CreateBody(new fix3(3, 0, 0), quaternion.identity, RigidType.Dynamic, out BodyHandle sleeper), "sleep candidate should be created");
        AssertTrue(world.CreateBody(new fix3(6, 0, 0), quaternion.identity, RigidType.Kinematic, out BodyHandle kinematic), "kinematic body should be created");
        AssertTrue(world.AddSphere(moving, fix3.zero, fix._0_5, out ColliderHandle _), "moving sphere should be created");
        AssertTrue(world.AddAABB(wall, fix3.zero, new fix3(1, 4, 4), out ColliderHandle _), "wall collider should be created");
        AssertTrue(world.AddSphere(sleeper, fix3.zero, fix._0_5, out ColliderHandle _), "sleep sphere should be created");
        AssertTrue(world.AddSphere(kinematic, fix3.zero, fix._0_5, out ColliderHandle _), "kinematic sphere should be created");
        AssertTrue(world.SetBodyVelocity(moving, new fix3(8, 0, 0)), "moving velocity should be set");
        AssertTrue(world.SetBodyVelocity(kinematic, new fix3(-fix._0_5, fix.Zero, fix.Zero)), "kinematic velocity should be set");
        return world;
    }

    private static void TestPhysicsWorldCoreCompoundMassAggregatesColliders()
    {
        PhysicsWorld world = new PhysicsWorld(new PhysicsWorldSettings(false));
        world.Reserve(1, 2, 2, 2);
        AssertTrue(world.CreateBody(fix3.zero, quaternion.identity, RigidType.Dynamic, out BodyHandle body), "body should be created");
        AssertTrue(world.AddAABB(body, fix3.zero, new fix3(2, 2, 2), out ColliderHandle collider0), "first box should be created");
        AssertTrue(world.AddAABB(body, new fix3(2, 0, 0), new fix3(2, 2, 2), out ColliderHandle collider1), "second box should be created");

        AssertTrue(world.TryGetBody(body, out PhysicsWorldBody massBody), "body should be readable");
        AssertEqual(new fix(16), massBody.mass);
        AssertTrue(massBody.inertia.y > massBody.inertia.x, "off-center collider should add parallel-axis inertia");

        AssertTrue(world.SetColliderDensity(collider1, new fix(2)), "density update should succeed");
        AssertTrue(world.TryGetBody(body, out massBody), "body should still be readable");
        AssertEqual(new fix(24), massBody.mass);

        AssertTrue(world.SetBodyMass(body, new fix(5)), "manual mass should disable auto mass");
        AssertTrue(world.SetColliderDensity(collider0, new fix(4)), "density update should not override manual mass");
        AssertTrue(world.TryGetBody(body, out massBody), "manual mass body should be readable");
        AssertEqual(new fix(5), massBody.mass);
    }

    private static void TestPhysicsWorldCoreBuildsIslandsAndSleeps()
    {
        PhysicsWorldSettings settings = new PhysicsWorldSettings(false);
        settings.sleepTime = fix.Zero;
        settings.linearSleepThreshold = fix._0_01;
        settings.angularSleepThreshold = fix._0_01;
        PhysicsWorld world = new PhysicsWorld(settings);
        world.Reserve(3, 3, 8, 8);
        AssertTrue(world.CreateBody(fix3.zero, quaternion.identity, RigidType.Dynamic, out BodyHandle body0), "first body should be created");
        AssertTrue(world.CreateBody(new fix3(fix._1_5, fix.Zero, fix.Zero), quaternion.identity, RigidType.Dynamic, out BodyHandle body1), "second body should be created");
        AssertTrue(world.CreateBody(new fix3(6, 0, 0), quaternion.identity, RigidType.Dynamic, out BodyHandle body2), "third body should be created");
        AssertTrue(world.AddSphere(body0, fix3.zero, fix.One, out ColliderHandle _), "first sphere should be created");
        AssertTrue(world.AddSphere(body1, fix3.zero, fix.One, out ColliderHandle _), "second sphere should be created");
        AssertTrue(world.AddSphere(body2, fix3.zero, fix.One, out ColliderHandle _), "third sphere should be created");

        world.Update(fix.One / 60);

        AssertEqual(2, world.IslandCount);
        AssertEqual(2, world.SleepingIslandCount);
        AssertTrue(world.TryGetBody(body0, out PhysicsWorldBody sleeping0), "first body should be readable");
        AssertTrue(world.TryGetBody(body1, out PhysicsWorldBody sleeping1), "second body should be readable");
        AssertTrue(sleeping0.isSleeping && sleeping1.isSleeping, "connected island should sleep together");

        AssertTrue(world.SetBodyVelocity(body0, fix3.right), "setting velocity should wake the body");
        world.Update(fix.One / 60);

        AssertTrue(world.TryGetBody(body0, out PhysicsWorldBody awake0), "woken body should be readable");
        AssertFalse(awake0.isSleeping, "moving body should remain awake");
    }

    private static void TestPhysicsWorldCoreCCDStopsFastBody()
    {
        PhysicsWorldSettings settings = new PhysicsWorldSettings(false);
        settings.enableCCD = true;
        settings.ccdMinVelocity = fix.Zero;
        settings.positionCorrectionPercent = fix.Zero;
        PhysicsWorld world = new PhysicsWorld(settings);
        world.Reserve(2, 2, 4, 4);
        AssertTrue(world.CreateBody(fix3.zero, quaternion.identity, RigidType.Dynamic, out BodyHandle moving), "moving body should be created");
        AssertTrue(world.CreateBody(new fix3(5, 0, 0), quaternion.identity, RigidType.Static, out BodyHandle target), "target body should be created");
        AssertTrue(world.AddSphere(moving, fix3.zero, fix._0_5, out ColliderHandle _), "moving sphere should be created");
        AssertTrue(world.AddSphere(target, fix3.zero, fix._0_5, out ColliderHandle _), "target sphere should be created");
        AssertTrue(world.SetBodyVelocity(moving, new fix3(10, 0, 0)), "fast velocity should be set");

        world.Update(fix.One);

        AssertTrue(world.TryGetBody(moving, out PhysicsWorldBody body), "moving body should be readable");
        AssertNear(new fix3(4, 0, 0), body.position, fix._0_001);
        AssertNear(fix3.zero, body.velocity, fix._0_001);
    }

    private static void TestPhysicsWorldCoreDynamicCCDHandlesDynamicTarget()
    {
        PhysicsWorldSettings settings = new PhysicsWorldSettings(false);
        settings.enableCCD = true;
        settings.enableDynamicCCD = true;
        settings.ccdMinVelocity = fix.Zero;
        settings.positionCorrectionPercent = fix.Zero;
        PhysicsWorld world = new PhysicsWorld(settings);
        world.Reserve(2, 2, 4, 4);
        AssertTrue(world.CreateBody(fix3.zero, quaternion.identity, RigidType.Dynamic, out BodyHandle moving), "moving body should be created");
        AssertTrue(world.CreateBody(new fix3(5, 0, 0), quaternion.identity, RigidType.Dynamic, out BodyHandle target), "target body should be created");
        AssertTrue(world.AddSphere(moving, fix3.zero, fix._0_5, out ColliderHandle _), "moving sphere should be created");
        AssertTrue(world.AddSphere(target, fix3.zero, fix._0_5, out ColliderHandle _), "target sphere should be created");
        AssertTrue(world.SetBodyVelocity(moving, new fix3(10, 0, 0)), "fast velocity should be set");

        world.Update(fix.One);

        AssertTrue(world.TryGetBody(moving, out PhysicsWorldBody movingBody), "moving body should be readable");
        AssertTrue(world.TryGetBody(target, out PhysicsWorldBody targetBody), "target body should be readable");
        AssertNear(new fix3(4, 0, 0), movingBody.position, fix._0_001);
        AssertEqual(new fix3(5, 0, 0), targetBody.position);
        AssertNear(fix3.zero, movingBody.velocity, fix._0_001);
    }

    private static void TestPhysicsWorldCoreMaterialAffectsSolver()
    {
        PhysicsWorldSettings settings = new PhysicsWorldSettings(false);
        settings.positionCorrectionPercent = fix.Zero;
        settings.restitution = fix.Zero;
        settings.friction = fix.Zero;
        PhysicsWorld world = new PhysicsWorld(settings);
        world.Reserve(2, 2, 4, 4);
        AssertTrue(world.CreateBody(fix3.zero, quaternion.identity, RigidType.Dynamic, out BodyHandle dynamicBody), "dynamic body should be created");
        AssertTrue(world.CreateBody(new fix3(fix._1_5, fix.Zero, fix.Zero), quaternion.identity, RigidType.Static, out BodyHandle staticBody), "static body should be created");
        AssertTrue(world.AddSphere(dynamicBody, fix3.zero, fix.One, out ColliderHandle dynamicCollider), "dynamic sphere should be created");
        AssertTrue(world.AddSphere(staticBody, fix3.zero, fix.One, out ColliderHandle staticCollider), "static sphere should be created");
        Material bouncy = Material.Default;
        bouncy.SetBounciness(fix.One);
        AssertTrue(world.SetColliderMaterial(staticCollider, bouncy), "bouncy material should be set");
        AssertTrue(world.SetBodyVelocity(dynamicBody, fix3.right), "dynamic body velocity should be set");

        world.Update(fix.Zero);

        AssertTrue(world.TryGetBody(dynamicBody, out PhysicsWorldBody body), "dynamic body should be readable");
        AssertTrue(body.velocity.x < fix.Zero, "material bounciness should reverse the impact velocity");

        Material sticky = Material.Default;
        sticky.SetFrictionCoefficient(new fix(2));
        AssertTrue(world.SetColliderMaterial(dynamicCollider, sticky), "sticky material should be set");
        AssertTrue(world.SetColliderMaterial(staticCollider, sticky), "sticky static material should be set");
    }

    private static void TestPhysicsWorldCoreCCDAndSleepDoNotAllocate()
    {
        PhysicsWorldSettings settings = new PhysicsWorldSettings(false);
        settings.enableCCD = true;
        settings.enableDynamicCCD = true;
        settings.ccdMinVelocity = fix.Zero;
        settings.sleepTime = new fix(10);
        settings.positionCorrectionPercent = fix.Zero;
        PhysicsWorld world = new PhysicsWorld(settings);
        world.Reserve(2, 2, 4, 4);
        AssertTrue(world.CreateBody(fix3.zero, quaternion.identity, RigidType.Dynamic, out BodyHandle moving), "moving body should be created");
        AssertTrue(world.CreateBody(new fix3(5, 0, 0), quaternion.identity, RigidType.Dynamic, out BodyHandle target), "target body should be created");
        AssertTrue(world.AddSphere(moving, fix3.zero, fix._0_5, out ColliderHandle _), "moving sphere should be created");
        AssertTrue(world.AddSphere(target, fix3.zero, fix._0_5, out ColliderHandle _), "target sphere should be created");

        for (int i = 0; i < 4; i++)
        {
            AssertTrue(world.SetBodyTransform(moving, fix3.zero, quaternion.identity), "moving body should reset");
            AssertTrue(world.SetBodyVelocity(moving, new fix3(10, 0, 0)), "moving velocity should reset");
            world.Update(fix.One);
        }

        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 16; i++)
        {
            AssertTrue(world.SetBodyTransform(moving, fix3.zero, quaternion.identity), "moving body should reset in warm loop");
            AssertTrue(world.SetBodyVelocity(moving, new fix3(10, 0, 0)), "moving velocity should reset in warm loop");
            world.Update(fix.One);
            AssertTrue(world.TryGetBody(moving, out PhysicsWorldBody movingBody), "moving body should be readable");
            AssertNear(new fix3(4, 0, 0), movingBody.position, fix._0_001);
        }

        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;
        AssertTrue(allocated == 0, $"core CCD/sleep warm path allocated {allocated} bytes");
    }

    private static void TestPhysicsWorldCoreExposesStepStatsAndIslandIds()
    {
        PhysicsWorldSettings settings = new PhysicsWorldSettings(false);
        settings.sleepTime = fix.Zero;
        PhysicsWorld world = new PhysicsWorld(settings);
        world.Reserve(2, 2, 4, 4);
        AssertTrue(world.CreateBody(fix3.zero, quaternion.identity, RigidType.Dynamic, out BodyHandle body0), "first body should be created");
        AssertTrue(world.CreateBody(new fix3(fix._1_5, fix.Zero, fix.Zero), quaternion.identity, RigidType.Dynamic, out BodyHandle body1), "second body should be created");
        AssertTrue(world.AddSphere(body0, fix3.zero, fix.One, out ColliderHandle _), "first sphere should be created");
        AssertTrue(world.AddSphere(body1, fix3.zero, fix.One, out ColliderHandle _), "second sphere should be created");

        world.Update(fix.Zero);

        PhysicsWorldStepStats stats = world.LastStepStats;
        AssertEqual(2, stats.activeBodyCount);
        AssertEqual(2, stats.activeColliderCount);
        AssertEqual(2, stats.broadphaseProxyCount);
        AssertEqual(1, stats.pairCount);
        AssertEqual(1, stats.contactManifoldCount);
        AssertEqual(1, stats.islandCount);
        AssertTrue(world.TryGetBodyIslandId(body0, out int island0), "first body should expose an island id");
        AssertTrue(world.TryGetBodyIslandId(body1, out int island1), "second body should expose an island id");
        AssertEqual(island0, island1);
    }

    private static void TestPhysicsWorldCoreExposesCapacityAndMemoryBudget()
    {
        PhysicsWorldCapacity capacity = PhysicsWorldCapacity.ForBodies(8, 2, 3);
        AssertTrue(capacity.IsValid, "capacity helper should produce a valid capacity block");
        AssertEqual(8, capacity.bodyCapacity);
        AssertEqual(16, capacity.colliderCapacity);
        AssertEqual(48, capacity.pairCapacity);
        AssertEqual(48, capacity.contactManifoldCapacity);

        PhysicsWorldMemoryBudget budget = capacity.EstimateMemoryBudget();
        AssertTrue(budget.bodyBytes > 0, "body budget should be reported");
        AssertTrue(budget.colliderBytes > 0, "collider budget should be reported");
        AssertTrue(budget.broadphaseBytes > 0, "broadphase budget should be reported");
        AssertTrue(budget.totalBytes >= budget.bodyBytes + budget.colliderBytes, "total budget should include major components");

        PhysicsWorld world = new PhysicsWorld(new PhysicsWorldSettings(false));
        world.Reserve(capacity);
        AssertEqual(8, world.BodyCapacity);
        AssertEqual(16, world.ColliderCapacity);
        AssertEqual(48, world.PairCapacity);
        AssertEqual(48, world.ContactManifoldCapacity);
        AssertEqual(16, world.Capacity.colliderCapacity);
        AssertEqual(48, world.Capacity.contactManifoldCapacity);
        AssertTrue(world.EstimatedMemoryBudget.totalBytes >= budget.totalBytes, "world budget should reflect reserved buffers");

        world.Update(fix.Zero);
        PhysicsWorldStepStats stats = world.LastStepStats;
        AssertEqual(8, stats.bodyCapacity);
        AssertEqual(16, stats.colliderCapacity);
        AssertEqual(48, stats.pairCapacity);
        AssertEqual(48, stats.contactManifoldCapacity);
    }

    private static void TestPhysicsWorldCoreReportsCapacityOverflowInStats()
    {
        PhysicsWorld pairWorld = new PhysicsWorld(new PhysicsWorldSettings(false));
        pairWorld.Reserve(3, 3, 1, 8);
        for (int i = 0; i < 3; i++)
        {
            AssertTrue(pairWorld.CreateBody(fix3.zero, quaternion.identity, RigidType.Dynamic, out BodyHandle body), "body should be created");
            AssertTrue(pairWorld.AddSphere(body, fix3.zero, fix.One, out ColliderHandle _), "collider should be created");
        }

        pairWorld.Update(fix.Zero);
        AssertEqual(1, pairWorld.PairCount);
        AssertTrue(pairWorld.PairOverflowed, "pair capacity should overflow");
        AssertTrue(pairWorld.LastStepStats.pairOverflowed, "stats should report pair overflow");
        AssertTrue(pairWorld.LastStepStats.HasOverflow, "stats should expose overflow summary");

        PhysicsWorld contactWorld = new PhysicsWorld(new PhysicsWorldSettings(false));
        contactWorld.Reserve(3, 3, 8, 1);
        for (int i = 0; i < 3; i++)
        {
            AssertTrue(contactWorld.CreateBody(fix3.zero, quaternion.identity, RigidType.Dynamic, out BodyHandle body), "body should be created");
            AssertTrue(contactWorld.AddSphere(body, fix3.zero, fix.One, out ColliderHandle _), "collider should be created");
        }

        contactWorld.Update(fix.Zero);
        AssertEqual(3, contactWorld.PairCount);
        AssertEqual(1, contactWorld.ContactManifoldCount);
        AssertTrue(contactWorld.ContactOverflowed, "contact manifold capacity should overflow");
        AssertTrue(contactWorld.LastStepStats.contactOverflowed, "stats should report contact overflow");
        AssertTrue(contactWorld.LastStepStats.HasOverflow, "stats should expose contact overflow summary");
        AssertEqual(3, contactWorld.LastStepStats.narrowPhaseTestCount);
        AssertEqual(1, contactWorld.LastStepStats.contactManifoldNewCount);
        AssertEqual(2, contactWorld.LastStepStats.contactManifoldDroppedCount);
    }

    private static void TestPhysicsWorldCoreReportsPerformanceCounters()
    {
        PhysicsWorldSettings settings = new PhysicsWorldSettings(false);
        settings.positionCorrectionPercent = fix.Zero;
        PhysicsWorld world = new PhysicsWorld(settings);
        world.Reserve(2, 2, 8, 8);
        AssertTrue(world.CreateBody(fix3.zero, quaternion.identity, RigidType.Dynamic, out BodyHandle body0), "first body should be created");
        AssertTrue(world.CreateBody(new fix3(fix._1_5, fix.Zero, fix.Zero), quaternion.identity, RigidType.Dynamic, out BodyHandle body1), "second body should be created");
        AssertTrue(world.AddSphere(body0, fix3.zero, fix.One, out ColliderHandle _), "first sphere should be created");
        AssertTrue(world.AddSphere(body1, fix3.zero, fix.One, out ColliderHandle _), "second sphere should be created");

        world.Update(fix.Zero);

        PhysicsWorldStepStats first = world.LastStepStats;
        AssertTrue(first.broadphaseCandidateCount >= 2, "stats should report broadphase candidates");
        AssertTrue(first.broadphaseFilteredCandidateCount >= 1, "stats should report self/duplicate filtering");
        AssertEqual(1, first.pairCount);
        AssertEqual(1, first.narrowPhaseTestCount);
        AssertEqual(1, first.contactManifoldCount);
        AssertEqual(1, first.contactManifoldNewCount);
        AssertEqual(0, first.contactManifoldReusedCount);
        AssertEqual(0, first.contactManifoldDroppedCount);
        AssertEqual(1, first.solverContactPointCount);
        AssertEqual(0, first.sleepingBodyCount);
        AssertEqual(2, first.awakeDynamicBodyCount);

        world.Update(fix.Zero);

        PhysicsWorldStepStats second = world.LastStepStats;
        AssertEqual(1, second.narrowPhaseTestCount);
        AssertEqual(0, second.contactManifoldNewCount);
        AssertEqual(1, second.contactManifoldReusedCount);
        AssertEqual(1, second.solverContactPointCount);
    }

    private static void TestPhysicsWorldCoreDirtySyncUpdatesChangedCollidersOnly()
    {
        PhysicsWorld world = new PhysicsWorld(new PhysicsWorldSettings(false));
        world.Reserve(4, 4, 8, 8);
        BodyHandle[] bodies = new BodyHandle[4];
        ColliderHandle[] colliders = new ColliderHandle[4];
        for (int i = 0; i < bodies.Length; i++)
        {
            AssertTrue(world.CreateBody(new fix3(i * 4, 0, 0), quaternion.identity, RigidType.Static, out bodies[i]), "body should be created");
            AssertTrue(world.AddSphere(bodies[i], fix3.zero, fix._0_5, out colliders[i]), "collider should be created");
        }

        world.Update(fix.Zero);
        PhysicsWorldStepStats first = world.LastStepStats;
        AssertEqual(0, first.bodySyncCount);
        AssertEqual(0, first.colliderSyncCount);
        AssertEqual(4, first.broadphaseMovedProxyCount);

        world.Update(fix.Zero);
        PhysicsWorldStepStats stable = world.LastStepStats;
        AssertEqual(0, stable.bodySyncCount);
        AssertEqual(0, stable.colliderSyncCount);
        AssertEqual(0, stable.broadphaseMovedProxyCount);

        AssertTrue(world.SetBodyTransform(bodies[2], new fix3(20, 0, 0), quaternion.identity), "moving one body should succeed");
        world.Update(fix.Zero);

        PhysicsWorldStepStats movedOne = world.LastStepStats;
        AssertEqual(1, movedOne.bodySyncCount);
        AssertEqual(1, movedOne.colliderSyncCount);
        AssertEqual(1, movedOne.broadphaseMovedProxyCount);

        AssertTrue(world.SetBodyTransform(bodies[0], new fix3(40, 0, 0), quaternion.identity), "moving body should mark collider dirty");
        AssertTrue(world.TryGetColliderBounds(colliders[0], out AABB bounds), "bounds getter should flush dirty collider state");
        AssertEqual(new fix3(40, 0, 0), bounds.center);
    }

    private static void TestPhysicsWorldCoreAutoMassCanBeReenabled()
    {
        PhysicsWorld world = new PhysicsWorld(new PhysicsWorldSettings(false));
        world.Reserve(1, 1, 2, 2);
        AssertTrue(world.CreateBody(fix3.zero, quaternion.identity, RigidType.Dynamic, out BodyHandle body), "body should be created");
        AssertTrue(world.AddAABB(body, fix3.zero, new fix3(2, 2, 2), out ColliderHandle collider), "collider should be created");

        AssertTrue(world.SetBodyMass(body, new fix(5)), "manual mass should be set");
        AssertTrue(world.SetBodyInertia(body, new fix3(7, 8, 9)), "manual inertia should be set");
        AssertTrue(world.SetColliderDensity(collider, fix._2), "density should update material");
        AssertTrue(world.TryGetBody(body, out PhysicsWorldBody manual), "manual body should be readable");
        AssertEqual(new fix(5), manual.mass);
        AssertEqual(new fix3(7, 8, 9), manual.inertia);

        AssertTrue(world.SetBodyAutoMassProperties(body, true, true), "auto mass properties should be re-enabled");
        AssertTrue(world.TryGetBody(body, out PhysicsWorldBody automatic), "automatic body should be readable");
        fix expectedInertia = new fix(128) / 12;
        AssertEqual(new fix(16), automatic.mass);
        AssertEqual(new fix3(expectedInertia, expectedInertia, expectedInertia), automatic.inertia);
    }

    private static void TestPhysicsWorldCoreLongReplayWithLifecycleIsDeterministic()
    {
        PhysicsWorld first = CreateLifecycleReplayWorld();
        PhysicsWorld second = CreateLifecycleReplayWorld();
        ColliderHandle firstToggled = default;
        ColliderHandle secondToggled = default;
        ColliderHandle firstDestroyed = default;
        ColliderHandle secondDestroyed = default;
        BodyHandle firstTransient = default;
        BodyHandle secondTransient = default;

        ulong firstHash = 0;
        ulong secondHash = 0;
        for (int i = 0; i < 1000; i++)
        {
            ApplyLifecycleReplayInput(first, i, ref firstToggled, ref firstDestroyed, ref firstTransient);
            ApplyLifecycleReplayInput(second, i, ref secondToggled, ref secondDestroyed, ref secondTransient);
            firstHash = first.StepAndComputeStateHash(fix.One / 60);
            secondHash = second.StepAndComputeStateHash(fix.One / 60);
            AssertEqual(firstHash, secondHash);
        }

        AssertEqual(firstHash, first.ComputeStateHash());
        AssertEqual(secondHash, second.ComputeStateHash());
    }

    private static PhysicsWorld CreateLifecycleReplayWorld()
    {
        PhysicsWorldSettings settings = new PhysicsWorldSettings(false);
        settings.enableCCD = true;
        settings.ccdMinVelocity = fix.Zero;
        settings.solverIterations = 2;
        settings.positionIterations = 2;
        settings.sleepTime = new fix(5);
        PhysicsWorld world = new PhysicsWorld(settings);
        world.Reserve(6, 8, 24, 24);
        AssertTrue(world.CreateBody(new fix3(-4, 0, 0), quaternion.identity, RigidType.Dynamic, out BodyHandle moving), "moving body should be created");
        AssertTrue(world.CreateBody(fix3.zero, quaternion.identity, RigidType.Static, out BodyHandle wall), "wall body should be created");
        AssertTrue(world.CreateBody(new fix3(3, 0, 0), quaternion.identity, RigidType.Dynamic, out BodyHandle toggled), "toggled body should be created");
        AssertTrue(world.CreateBody(new fix3(6, 0, 0), quaternion.identity, RigidType.Dynamic, out BodyHandle removed), "removed body should be created");
        AssertTrue(world.AddSphere(moving, fix3.zero, fix._0_5, out ColliderHandle _), "moving sphere should be created");
        AssertTrue(world.AddAABB(wall, fix3.zero, new fix3(1, 4, 4), out ColliderHandle _), "wall collider should be created");
        AssertTrue(world.AddSphere(toggled, fix3.zero, fix._0_5, out ColliderHandle toggleCollider), "toggle collider should be created");
        AssertTrue(world.AddSphere(removed, fix3.zero, fix._0_5, out ColliderHandle removedCollider), "removed collider should be created");
        AssertTrue(world.SetBodyVelocity(moving, new fix3(7, 0, 0)), "moving velocity should be set");
        AssertTrue(world.SetBodyVelocity(toggled, new fix3(-fix._0_5, fix.Zero, fix.Zero)), "toggled velocity should be set");
        AssertTrue(world.SetBodyVelocity(removed, new fix3(-fix.One, fix.Zero, fix.Zero)), "removed velocity should be set");
        return world;
    }

    private static void ApplyLifecycleReplayInput(
        PhysicsWorld world,
        int frame,
        ref ColliderHandle toggledCollider,
        ref ColliderHandle destroyedCollider,
        ref BodyHandle transientBody)
    {
        if (frame == 0)
        {
            for (int i = 0; i < world.PairCount; i++)
            {
                world.TryGetPair(i, out PhysicsWorldPair _);
            }
        }

        if (frame == 30 && !toggledCollider.IsValid)
        {
            AssertTrue(world.CreateBody(new fix3(8, 0, 0), quaternion.identity, RigidType.Dynamic, out transientBody), "transient body should be created");
            AssertTrue(world.AddSphere(transientBody, fix3.zero, fix._0_5, out toggledCollider), "transient collider should be created");
            AssertTrue(world.SetBodyVelocity(transientBody, new fix3(-2, 0, 0)), "transient velocity should be set");
        }

        if (frame == 120 && toggledCollider.IsValid)
        {
            AssertTrue(world.SetColliderEnabled(toggledCollider, false), "transient collider should be disabled");
        }

        if (frame == 180 && toggledCollider.IsValid)
        {
            AssertTrue(world.SetColliderEnabled(toggledCollider, true), "transient collider should be enabled");
        }

        if (frame == 240 && transientBody.IsValid)
        {
            AssertTrue(world.DestroyBody(transientBody), "transient body should be destroyed");
            transientBody = BodyHandle.Invalid;
            toggledCollider = ColliderHandle.Invalid;
        }

        if (frame == 360 && !destroyedCollider.IsValid)
        {
            AssertTrue(world.CreateBody(new fix3(-6, 1, 0), quaternion.identity, RigidType.Dynamic, out BodyHandle body), "late body should be created");
            AssertTrue(world.AddCapsule(body, fix3.zero, fix._0_25, new fix(2), quaternion.identity, out destroyedCollider), "late capsule should be created");
            AssertTrue(world.SetBodyVelocity(body, new fix3(new fix(3), -fix._0_5, fix.Zero)), "late body velocity should be set");
        }

        if (frame == 500 && destroyedCollider.IsValid)
        {
            AssertTrue(world.DestroyCollider(destroyedCollider), "late collider should be destroyed");
            destroyedCollider = ColliderHandle.Invalid;
        }
    }

    private static void TestPhysicsWorldCoreClampsUnsafeMotionValues()
    {
        PhysicsWorldSettings settings = new PhysicsWorldSettings(false);
        settings.maxTranslationPerStep = fix._0_5;
        settings.maxLinearVelocity = fix.Zero;
        PhysicsWorld world = new PhysicsWorld(settings);
        world.Reserve(1, 1, 1, 1);
        AssertTrue(world.CreateBody(fix3.zero, quaternion.identity, RigidType.Dynamic, out BodyHandle body), "body should be created");
        AssertTrue(world.AddSphere(body, fix3.zero, fix.One, out ColliderHandle _), "collider should be created");
        AssertTrue(world.SetBodyVelocity(body, new fix3(100, 0, 0)), "unsafe velocity should be accepted and clamped");

        world.Update(fix.One);

        AssertTrue(world.TryGetBody(body, out PhysicsWorldBody synced), "body should be readable");
        AssertTrue(synced.position.x <= fix._0_5, "per-step translation should be clamped");
        AssertTrue(synced.velocity.x <= fix._0_5, "velocity should be clamped for the step");

        AssertTrue(world.SetBodyVelocity(body, fix3.NaN), "NaN velocity input should be sanitized");
        AssertTrue(world.TryGetBody(body, out synced), "sanitized body should be readable");
        AssertEqual(fix3.zero, synced.velocity);
    }

    private static void TestPhysicsWorldCoreStackedBoxesStaySeparated()
    {
        PhysicsWorldSettings settings = new PhysicsWorldSettings(true);
        settings.gravity = new fix3(fix.Zero, -new fix(10), fix.Zero);
        settings.solverIterations = 8;
        settings.positionIterations = 4;
        settings.positionCorrectionPercent = fix._0_2;
        settings.penetrationSlop = fix._0_01;
        settings.sleepTime = new fix(10);
        PhysicsWorld world = new PhysicsWorld(settings);
        world.Reserve(4, 4, 16, 16);
        AssertTrue(world.CreateBody(new fix3(fix.Zero, -fix._0_5, fix.Zero), quaternion.identity, RigidType.Static, out BodyHandle ground), "ground should be created");
        AssertTrue(world.CreateBody(new fix3(fix.Zero, fix._0_5, fix.Zero), quaternion.identity, RigidType.Dynamic, out BodyHandle box0), "first box should be created");
        AssertTrue(world.CreateBody(new fix3(fix.Zero, fix._1_5, fix.Zero), quaternion.identity, RigidType.Dynamic, out BodyHandle box1), "second box should be created");
        AssertTrue(world.CreateBody(new fix3(fix.Zero, new fix(5) * fix._0_5, fix.Zero), quaternion.identity, RigidType.Dynamic, out BodyHandle box2), "third box should be created");
        AssertTrue(world.AddAABB(ground, fix3.zero, new fix3(8, 1, 8), out ColliderHandle _), "ground collider should be created");
        AssertTrue(world.AddAABB(box0, fix3.zero, new fix3(1, 1, 1), out ColliderHandle _), "first box collider should be created");
        AssertTrue(world.AddAABB(box1, fix3.zero, new fix3(1, 1, 1), out ColliderHandle _), "second box collider should be created");
        AssertTrue(world.AddAABB(box2, fix3.zero, new fix3(1, 1, 1), out ColliderHandle _), "third box collider should be created");

        for (int i = 0; i < 90; i++)
        {
            world.Step(fix.One / 60);
        }

        AssertTrue(world.TryGetBody(box0, out PhysicsWorldBody lower), "lower box should be readable");
        AssertTrue(world.TryGetBody(box1, out PhysicsWorldBody middle), "middle box should be readable");
        AssertTrue(world.TryGetBody(box2, out PhysicsWorldBody upper), "upper box should be readable");
        AssertTrue(lower.position.y >= fix._0_25, $"lower box should stay above the floor, got {lower.position.y}");
        AssertTrue(middle.position.y - lower.position.y >= fix._0_5, $"middle box should stay above lower box, got lower {lower.position.y}, middle {middle.position.y}");
        AssertTrue(upper.position.y - middle.position.y >= fix._0_5, $"upper box should stay above middle box, got middle {middle.position.y}, upper {upper.position.y}");
    }

    private static void TestPhysicsWorldCoreCCDSweepsCapsuleAndCompoundColliders()
    {
        PhysicsWorldSettings settings = new PhysicsWorldSettings(false);
        settings.enableCCD = true;
        settings.ccdMinVelocity = fix.Zero;
        settings.positionCorrectionPercent = fix.Zero;
        PhysicsWorld capsuleWorld = new PhysicsWorld(settings);
        capsuleWorld.Reserve(2, 2, 4, 4);
        AssertTrue(capsuleWorld.CreateBody(fix3.zero, quaternion.identity, RigidType.Dynamic, out BodyHandle capsule), "capsule body should be created");
        AssertTrue(capsuleWorld.CreateBody(new fix3(5, 0, 0), quaternion.identity, RigidType.Static, out BodyHandle wall), "wall body should be created");
        AssertTrue(capsuleWorld.AddCapsule(capsule, fix3.zero, fix._0_5, new fix(3), quaternion.identity, out ColliderHandle _), "capsule collider should be created");
        AssertTrue(capsuleWorld.AddAABB(wall, fix3.zero, new fix3(1, 4, 4), out ColliderHandle _), "wall collider should be created");
        AssertTrue(capsuleWorld.SetBodyVelocity(capsule, new fix3(10, 0, 0)), "capsule velocity should be set");

        capsuleWorld.Update(fix.One);

        AssertTrue(capsuleWorld.TryGetBody(capsule, out PhysicsWorldBody capsuleBody), "capsule body should be readable");
        AssertNear(new fix3(4, 0, 0), capsuleBody.position, fix._0_001);

        PhysicsWorld compoundWorld = new PhysicsWorld(settings);
        compoundWorld.Reserve(2, 3, 8, 8);
        AssertTrue(compoundWorld.CreateBody(fix3.zero, quaternion.identity, RigidType.Dynamic, out BodyHandle compound), "compound body should be created");
        AssertTrue(compoundWorld.CreateBody(new fix3(5, 0, 0), quaternion.identity, RigidType.Static, out BodyHandle target), "target body should be created");
        AssertTrue(compoundWorld.AddSphere(compound, new fix3(1, 0, 0), fix._0_5, out ColliderHandle _), "front sphere should be created");
        AssertTrue(compoundWorld.AddSphere(compound, new fix3(-1, 0, 0), fix._0_5, out ColliderHandle _), "back sphere should be created");
        AssertTrue(compoundWorld.AddAABB(target, fix3.zero, new fix3(1, 4, 4), out ColliderHandle _), "target collider should be created");
        AssertTrue(compoundWorld.SetBodyVelocity(compound, new fix3(10, 0, 0)), "compound velocity should be set");

        compoundWorld.Update(fix.One);

        AssertTrue(compoundWorld.TryGetBody(compound, out PhysicsWorldBody compoundBody), "compound body should be readable");
        AssertNear(new fix3(3, 0, 0), compoundBody.position, fix._0_001);
    }

    private static void TestPhysicsWorldCoreContactLifetimePersists()
    {
        PhysicsWorldSettings settings = new PhysicsWorldSettings(false);
        settings.positionCorrectionPercent = fix.Zero;
        settings.warmStartScale = fix.One;
        PhysicsWorld world = new PhysicsWorld(settings);
        world.Reserve(2, 2, 4, 4);
        AssertTrue(world.CreateBody(fix3.zero, quaternion.identity, RigidType.Dynamic, out BodyHandle body0), "first body should be created");
        AssertTrue(world.CreateBody(new fix3(fix._1_5, fix.Zero, fix.Zero), quaternion.identity, RigidType.Static, out BodyHandle body1), "second body should be created");
        AssertTrue(world.AddAABB(body0, fix3.zero, new fix3(2, 2, 2), out ColliderHandle _), "first box should be created");
        AssertTrue(world.AddAABB(body1, fix3.zero, new fix3(2, 2, 2), out ColliderHandle _), "second box should be created");
        AssertTrue(world.SetBodyVelocity(body0, fix3.right), "first body velocity should be set");

        world.Update(fix.Zero);
        AssertTrue(world.TryGetContactManifold(0, out PhysicsWorldManifold first), "first contact should be readable");
        int firstLifetime = 0;
        fix firstImpulse = fix.Zero;
        for (int i = 0; i < first.contactCount; i++)
        {
            firstLifetime = math.max(firstLifetime, first[i].lifetime);
            firstImpulse = math.max(firstImpulse, first[i].normalImpulse);
        }

        world.Update(fix.Zero);

        AssertTrue(world.TryGetContactManifold(0, out PhysicsWorldManifold second), "second contact should be readable");
        int secondLifetime = 0;
        fix secondImpulse = fix.Zero;
        for (int i = 0; i < second.contactCount; i++)
        {
            secondLifetime = math.max(secondLifetime, second[i].lifetime);
            secondImpulse = math.max(secondImpulse, second[i].normalImpulse);
        }

        AssertTrue(secondLifetime > firstLifetime, "contact lifetime should persist across stable frames");
        AssertTrue(secondImpulse > fix.Zero && firstImpulse > fix.Zero, "warm-start contact should keep a non-zero impulse on at least one persistent point");
    }

    private static void TestBroadphaseTreeUpdatesMovingPairs()
    {
        World world = new World(new WorldSettings(false));
        Rigid rigid0 = world.CreateRigid(fix3.zero, quaternion.identity);
        Rigid rigid1 = world.CreateRigid(new fix3(4, 0, 0), quaternion.identity);
        Collider collider0 = world.AddSphereCollider(rigid0.id, fix3.zero, fix.One);
        Collider collider1 = world.AddSphereCollider(rigid1.id, fix3.zero, fix.One);

        world.Update(fix.Zero);

        AssertEqual(0, world.BroadphasePairs.Count);

        world.SetTranslation(rigid1.id, new fix3(fix._1_5, fix.Zero, fix.Zero));
        world.Update(fix.Zero);

        AssertEqual(1, world.BroadphasePairs.Count);
        AssertTrue(world.BroadphasePairs[0].key == new BroadCollisionPairKey(collider0.id, collider1.id), "moving pair should be generated by tree broadphase");

        world.SetTranslation(rigid1.id, new fix3(4, 0, 0));
        world.Update(fix.Zero);

        AssertEqual(0, world.BroadphasePairs.Count);
    }

    private static void TestBroadphaseRemovesStaleTreeProxies()
    {
        BroadCollisionSystem broadphase = new BroadCollisionSystem();
        Collider collider0 = new Collider();
        Collider collider1 = new Collider();
        collider0.AddSphereCollider(1, fix3.zero, fix.One);
        collider1.AddSphereCollider(2, new fix3(fix._1_5, fix.Zero, fix.Zero), fix.One);

        broadphase.Collision(new Collider[] { collider0, collider1 });

        AssertEqual(2, broadphase.TreeProxyCount);
        AssertEqual(1, broadphase.Pairs.Count);

        broadphase.Collision(new Collider[] { collider0 });

        AssertEqual(1, broadphase.TreeProxyCount);
        AssertEqual(0, broadphase.Pairs.Count);

        broadphase.Collision(Array.Empty<Collider>());

        AssertEqual(0, broadphase.TreeProxyCount);
        AssertEqual(0, broadphase.Pairs.Count);
    }

    private static void TestWorldAABBQueryUsesBroadphaseTree()
    {
        World world = new World(new WorldSettings(false));
        Rigid rigid0 = world.CreateRigid(fix3.zero, quaternion.identity);
        Rigid rigid1 = world.CreateRigid(new fix3(6, 0, 0), quaternion.identity);
        Collider collider0 = world.AddSphereCollider(rigid0.id, fix3.zero, fix.One);
        Collider collider1 = world.AddSphereCollider(rigid1.id, fix3.zero, fix.One);
        List<Collider> results = new List<Collider>();

        world.QueryAABB(new AABB(fix3.zero, new fix3(4, 4, 4)), results);

        AssertEqual(1, results.Count);
        AssertTrue(ContainsCollider(results, collider0.id), "query should include first collider");

        world.SetTranslation(rigid1.id, new fix3(fix._1_5, fix.Zero, fix.Zero));
        world.QueryAABB(new AABB(fix3.zero, new fix3(4, 4, 4)), results);

        AssertEqual(2, results.Count);
        AssertTrue(ContainsCollider(results, collider0.id), "query should include first collider after move");
        AssertTrue(ContainsCollider(results, collider1.id), "query should include moved collider");
    }

    private static void TestCollisionFilteringUsesLayerMasks()
    {
        World world = new World(new WorldSettings(false));
        Rigid rigid0 = world.CreateRigid(fix3.zero, quaternion.identity);
        Rigid rigid1 = world.CreateRigid(new fix3(fix._1_5, fix.Zero, fix.Zero), quaternion.identity);
        Collider collider0 = world.AddSphereCollider(rigid0.id, fix3.zero, fix.One);
        Collider collider1 = world.AddSphereCollider(rigid1.id, fix3.zero, fix.One);

        AssertTrue(world.SetColliderLayer(collider0.id, 1), "first collider layer should be valid");
        AssertTrue(world.SetColliderLayer(collider1.id, 2), "second collider layer should be valid");
        AssertTrue(world.SetColliderCollisionMask(collider0.id, Collider.GetLayerBit(1)), "first collider mask should be set");
        AssertTrue(world.SetColliderCollisionMask(collider1.id, Collider.GetLayerBit(1)), "second collider mask should be set");
        world.Update(fix.Zero);

        AssertEqual(0, world.BroadphasePairs.Count);
        AssertEqual(0, world.ContactManifolds.Count);

        AssertTrue(world.SetColliderCollisionMask(collider0.id, Collider.GetLayerBit(2)), "first collider mask should include second layer");
        world.Update(fix.Zero);

        AssertEqual(1, world.BroadphasePairs.Count);
        AssertEqual(1, world.ContactManifolds.Count);
    }

    private static void TestQueriesRespectLayerMasks()
    {
        World world = new World(new WorldSettings(false));
        Rigid rigid0 = world.CreateRigid(new fix3(5, 0, 0), quaternion.identity);
        Rigid rigid1 = world.CreateRigid(new fix3(8, 0, 0), quaternion.identity);
        Collider collider0 = world.AddSphereCollider(rigid0.id, fix3.zero, fix.One);
        Collider collider1 = world.AddSphereCollider(rigid1.id, fix3.zero, fix.One);
        List<Collider> results = new List<Collider>();

        AssertTrue(world.SetColliderLayer(collider0.id, 3), "near collider layer should be valid");
        AssertTrue(world.SetColliderLayer(collider1.id, 4), "far collider layer should be valid");

        world.QueryAABB(new AABB(new fix3(6, 0, 0), new fix3(8, 4, 4)), Collider.GetLayerBit(4), results);

        AssertEqual(1, results.Count);
        AssertEqual(collider1.id, results[0].id);

        AssertTrue(world.Raycast(fix3.zero, fix3.right, out RaycastHit hit, 10, Collider.GetLayerBit(4)), "raycast should hit masked far collider");
        AssertEqual(collider1.id, hit.colliderId);
        AssertEqual(new fix(7), hit.distance);
    }

    private static void TestPhysicsRaycastHitsSupportedShapes()
    {
        Ray ray = new Ray(fix3.zero, fix3.right);

        AssertTrue(PhysicsApi.TryRaycast(new Sphere(new fix3(5, 0, 0), fix.One), ray, 10, out RaycastHit sphereHit), "ray should hit sphere");
        AssertEqual(new fix(4), sphereHit.distance);
        AssertEqual(new fix3(4, 0, 0), sphereHit.point);

        AssertTrue(PhysicsApi.TryRaycast(new AABB(new fix3(5, 0, 0), new fix3(2, 2, 2)), ray, 10, out RaycastHit aabbHit), "ray should hit AABB");
        AssertEqual(new fix(4), aabbHit.distance);
        AssertEqual(fix3.left, aabbHit.normal);

        AssertTrue(PhysicsApi.TryRaycast(new AABB(new fix3(5, 0, 0), new fix3(2, 2, 2)), new Ray(new fix3(10, 0, 0), fix3.left), 10, out RaycastHit reverseAabbHit), "reverse ray should hit AABB");
        AssertEqual(new fix(4), reverseAabbHit.distance);
        AssertEqual(fix3.right, reverseAabbHit.normal);

        AssertTrue(PhysicsApi.TryRaycast(new OBB(new fix3(5, 0, 0), new fix3(2, 2, 2), quaternion.identity), ray, 10, out RaycastHit obbHit), "ray should hit OBB");
        AssertEqual(new fix(4), obbHit.distance);

        Capsule capsule = new Capsule(new fix3(5, 0, 0), fix.One, 4, quaternion.identity, fix3.up);
        AssertTrue(PhysicsApi.TryRaycast(capsule, ray, 10, out RaycastHit capsuleHit), "ray should hit capsule");
        AssertEqual(new fix(4), capsuleHit.distance);
    }

    private static void TestWorldRaycastReturnsNearestHit()
    {
        World world = new World(new WorldSettings(false));
        Rigid farRigid = world.CreateRigid(new fix3(8, 0, 0), quaternion.identity);
        Rigid nearRigid = world.CreateRigid(new fix3(5, 0, 0), quaternion.identity);
        Collider farCollider = world.AddSphereCollider(farRigid.id, fix3.zero, fix.One);
        Collider nearCollider = world.AddSphereCollider(nearRigid.id, fix3.zero, fix.One);

        AssertTrue(world.Raycast(fix3.zero, fix3.right * fix._2, out RaycastHit hit, 10), "world raycast should hit nearest sphere");
        AssertEqual(nearCollider.id, hit.colliderId);
        AssertEqual(nearRigid.id, hit.rigidId);
        AssertEqual(new fix(4), hit.distance);
        AssertEqual(new fix3(4, 0, 0), hit.point);

        world.SetTranslation(nearRigid.id, new fix3(12, 0, 0));

        AssertTrue(world.Raycast(fix3.zero, fix3.right, out hit, 10), "world raycast should hit far sphere after moving nearest away");
        AssertEqual(farCollider.id, hit.colliderId);
        AssertEqual(new fix(7), hit.distance);
    }

    private static void TestBoxFaceContactBuildsMultiPointManifold()
    {
        WorldSettings settings = new WorldSettings(false);
        settings.positionCorrectionPercent = fix.Zero;
        World world = new World(settings);
        Rigid rigid0 = world.CreateRigid(fix3.zero, quaternion.identity);
        Rigid rigid1 = world.CreateRigid(new fix3(fix._1_5, fix.Zero, fix.Zero), quaternion.identity);
        world.AddAABBCollider(rigid0.id, fix3.zero, new fix3(2, 2, 2));
        world.AddAABBCollider(rigid1.id, fix3.zero, new fix3(2, 2, 2));

        world.Update(fix.Zero);

        AssertEqual(1, world.ContactManifolds.Count);
        AssertEqual(4, world.ContactManifolds[0].contactCount);
        AssertEqual(fix3.right, world.ContactManifolds[0].normal);
        AssertEqual(fix._0_5, world.ContactManifolds[0][0].penetrationDepth);
    }

    private static void TestOBBContactUsesOrientedSATNormal()
    {
        quaternion rotation = quaternion.RotateZ(math.PI * fix._0_25);
        fix3 normal = rotation * fix3.right;
        OBB box0 = new OBB(fix3.zero, new fix3(2, 2, 2), rotation);
        OBB box1 = new OBB(normal * fix._1_5, new fix3(2, 2, 2), rotation);

        AssertTrue(PhysicsApi.TryComputeContact(box0, box1, out CollisionInfo collision), "rotated boxes should produce contact");
        AssertNear(normal, collision.normal, fix._0_01);
        AssertEqual(4, collision.contactCount);
        AssertTrue(math.abs(collision.penetrationDepth - fix._0_5) <= fix._0_0001, "rotated box penetration should be close to expected overlap");
    }

    private static void TestSATNarrowPhaseCanBeSelected()
    {
        OBB box0 = new OBB(fix3.zero, new fix3(2, 2, 2), quaternion.identity);
        OBB box1 = new OBB(new fix3(fix._1_5, fix.Zero, fix.Zero), new fix3(2, 2, 2), quaternion.RotateZ(math.PI * fix._0_25));

        AssertTrue(PhysicsApi.TryComputeContact(box0, box1, NarrowPhaseAlgorithm.SAT, out CollisionInfo collision), "SAT narrow phase should produce box contact");
        AssertTrue(collision.contactCount > 0, "SAT contact should include at least one clipped contact point");
        AssertTrue(collision.penetrationDepth > fix.Zero, "SAT contact should report penetration");
    }

    private static void TestGJKNarrowPhaseCanBeSelected()
    {
        Sphere sphere0 = new Sphere(fix3.zero, fix.One);
        Sphere sphere1 = new Sphere(new fix3(fix._1_5, fix.Zero, fix.Zero), fix.One);
        Sphere separated = new Sphere(new fix3(4, 0, 0), fix.One);
        AABB box0 = new AABB(fix3.zero, new fix3(2, 2, 2));
        AABB box1 = new AABB(new fix3(fix._1_5, fix.Zero, fix.Zero), new fix3(2, 2, 2));
        WorldSettings settings = new WorldSettings(false);
        settings.narrowPhaseAlgorithm = NarrowPhaseAlgorithm.GJK;
        World world = new World(settings);
        Rigid rigid0 = world.CreateRigid(fix3.zero, quaternion.identity);
        Rigid rigid1 = world.CreateRigid(new fix3(fix._1_5, fix.Zero, fix.Zero), quaternion.identity);
        world.AddSphereCollider(rigid0.id, fix3.zero, fix.One);
        world.AddSphereCollider(rigid1.id, fix3.zero, fix.One);

        AssertTrue(PhysicsApi.GJKOverlaps(sphere0, sphere1), "GJK should detect overlapping convex shapes");
        AssertFalse(PhysicsApi.GJKOverlaps(sphere0, separated), "GJK should reject separated convex shapes");
        AssertTrue(PhysicsApi.TryComputeGJKEPAContact(sphere0, sphere1, out CollisionInfo epaCollision), "EPA should produce contact from GJK simplex");
        AssertNear(fix3.right, epaCollision.normal, fix._0_01);
        AssertTrue(math.abs(epaCollision.penetrationDepth - fix._0_5) <= fix._0_01, "EPA should report sphere penetration depth");
        AssertTrue(PhysicsApi.TryComputeGJKEPAContact(box0, box1, out CollisionInfo boxEpaCollision), "EPA should produce box contact from GJK simplex");
        AssertTrue(math.dot(boxEpaCollision.normal, fix3.right) > fix._0_5, "EPA box normal should point from first shape to second");
        AssertTrue(boxEpaCollision.penetrationDepth > fix.Zero, "EPA box contact should report penetration");
        AssertTrue(PhysicsApi.TryComputeContact(sphere0, sphere1, NarrowPhaseAlgorithm.GJK, out CollisionInfo collision), "GJK narrow phase should produce contact through contact fallback");
        AssertEqual(1, collision.contactCount);

        world.Update(fix.Zero);

        AssertEqual(1, world.ContactManifolds.Count);
    }

    private static void TestGJKEPAHandlesRotatedBoxBoundary()
    {
        quaternion rotation0 = quaternion.RotateZ(math.PI * fix._0_25);
        quaternion rotation1 = quaternion.RotateZ(-math.PI * fix._0_25);
        OBB box0 = new OBB(fix3.zero, new fix3(2, 2, 2), rotation0);
        OBB box1 = new OBB(new fix3(fix._1_5, fix.Zero, fix.Zero), new fix3(2, 2, 2), rotation1);
        OBB separated = new OBB(new fix3(5, 0, 0), new fix3(2, 2, 2), rotation1);

        AssertTrue(PhysicsApi.GJKOverlaps(box0, box1), "GJK should detect rotated box overlap");
        AssertFalse(PhysicsApi.GJKOverlaps(box0, separated), "GJK should reject separated rotated boxes");
        AssertTrue(PhysicsApi.TryComputeGJKEPAContact(box0, box1, out CollisionInfo collision), "EPA should generate a rotated box contact");
        AssertTrue(collision.penetrationDepth > fix.Zero, "EPA rotated box contact should report penetration");
        AssertTrue(collision.contactCount > 0, "EPA rotated box contact should include a contact point");
    }

    private static void TestGJKEPAWarmPathDoesNotAllocate()
    {
        quaternion rotation0 = quaternion.RotateZ(math.PI * fix._0_25);
        quaternion rotation1 = quaternion.RotateZ(-math.PI * fix._0_25);
        OBB box0 = new OBB(fix3.zero, new fix3(2, 2, 2), rotation0);
        OBB box1 = new OBB(new fix3(fix._1_5, fix.Zero, fix.Zero), new fix3(2, 2, 2), rotation1);
        Shape shape0 = box0;
        Shape shape1 = box1;

        AssertTrue(PhysicsApi.TryComputeGJKEPAContact(shape0, shape1, out CollisionInfo _), "warmup EPA contact should succeed");
        for (int i = 0; i < 16; i++)
        {
            AssertTrue(PhysicsApi.TryComputeGJKEPAContact(shape0, shape1, out CollisionInfo _), "EPA buffers should warm to steady capacity");
        }

        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 16; i++)
        {
            AssertTrue(PhysicsApi.TryComputeGJKEPAContact(shape0, shape1, out CollisionInfo collision), "warm EPA contact should succeed without allocation");
            AssertTrue(collision.contactCount > 0, "warm EPA contact should keep contact data");
        }

        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;
        AssertTrue(allocated == 0, $"warm EPA path allocated {allocated} bytes");
    }

    private static void TestCapsuleOBBContactUsesSegmentBoxDistance()
    {
        quaternion rotation = quaternion.RotateZ(math.PI * fix._0_25);
        fix3 boxNormal = rotation * fix3.right;
        OBB obb = new OBB(fix3.zero, new fix3(2, 2, 2), rotation);
        Capsule capsule = new Capsule(boxNormal * (fix._1_5 - fix._0_1), fix._0_5, 4, rotation, fix3.up);

        AssertTrue(PhysicsApi.Overlaps(capsule, obb), "capsule parallel to OBB face should overlap");
        AssertTrue(PhysicsApi.TryComputeContact(capsule, obb, out CollisionInfo collision), "capsule OBB pair should produce contact");
        AssertNear(-boxNormal, collision.normal, fix._0_01);
        AssertEqual(1, collision.contactCount);
        AssertTrue(math.abs(collision.penetrationDepth - fix._0_1) <= fix._0_01, "capsule OBB penetration should come from segment box distance");
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

    private static void TestContactManifoldMatchesPersistentAnchors()
    {
        ContactManifold manifold = new ContactManifold();
        CollisionInfo first = new CollisionInfo(1, 2)
        {
            normal = fix3.up,
        };
        first.AddContact(fix3.zero, new fix3(fix.Zero, -fix._0_1, fix.Zero), fix._0_1);

        fix shift = fix._0_01 + fix._0_005;
        CollisionInfo shifted = new CollisionInfo(1, 2)
        {
            normal = fix3.up,
        };
        shifted.AddContact(new fix3(shift, fix.Zero, fix.Zero), new fix3(shift, -fix._0_1, fix.Zero), fix._0_1);

        manifold.Update(first, false, 1);
        manifold.Update(shifted, false, 2);

        AssertEqual(1, manifold.contactCount);
        AssertEqual(2, manifold[0].lifetime);
    }

    private static void TestContactSolverPreservesWarmStartImpulses()
    {
        WorldSettings settings = new WorldSettings(true, -10);
        settings.positionCorrectionPercent = fix.Zero;
        settings.friction = fix.Zero;
        World world = new World(settings);
        Rigid staticRigid = world.CreateRigid(fix3.zero, quaternion.identity);
        Rigid dynamicRigid = world.CreateRigid(new fix3(fix.Zero, fix._1_5, fix.Zero), quaternion.identity);
        world.AddSphereCollider(staticRigid.id, fix3.zero, fix.One);
        world.AddSphereCollider(dynamicRigid.id, fix3.zero, fix.One);

        world.SetRigidType(staticRigid.id, RigidType.Static);
        world.Update();

        AssertEqual(1, world.ContactManifolds.Count);
        fix firstImpulse = world.ContactManifolds[0][0].normalImpulse;
        AssertTrue(firstImpulse > fix.Zero, "first frame should solve a normal impulse");

        world.Update();

        AssertEqual(1, world.ContactManifolds.Count);
        ContactPoint point = world.ContactManifolds[0][0];
        AssertEqual(2, point.lifetime);
        AssertTrue(point.normalImpulse > fix.Zero, "persistent contact should keep a warm-start normal impulse");
        AssertTrue(point.normalImpulse >= firstImpulse - fix._0_01, "warm-start impulse should stay close under constant gravity");
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

    private static void TestWorldFixedStepAccumulatorAdvancesDeterministically()
    {
        WorldSettings settings = new WorldSettings(false);
        settings.timeStep = fix._0_5;
        World world = new World(settings);
        Rigid rigid = world.CreateRigid(fix3.zero, quaternion.identity);

        world.SetVelocity(rigid.id, new fix3(2, 0, 0));

        AssertEqual(0, world.Step(fix._0_25));
        AssertTrue(world.TryGetEntity(rigid.id, out Entity entity), "entity should exist before fixed step");
        AssertEqual(fix3.zero, entity.translation);
        AssertEqual(fix._0_25, world.FixedTimeAccumulator);

        AssertEqual(1, world.Step(fix._0_25));

        AssertTrue(world.TryGetEntity(rigid.id, out entity), "entity should exist after fixed step");
        AssertEqual(new fix3(1, 0, 0), entity.translation);
        AssertEqual(fix.Zero, world.FixedTimeAccumulator);
        AssertEqual(1UL, world.FixedStepCount);
    }

    private static void TestWorldStateHashIsReplayDeterministic()
    {
        WorldSettings settings = new WorldSettings(false);
        settings.timeStep = fix._0_25;
        World world0 = CreateReplayHashWorld(settings);
        World world1 = CreateReplayHashWorld(settings);

        for (int i = 0; i < 4; i++)
        {
            world0.Step(fix._0_25 / 2);
            world0.Step(fix._0_25 / 2);
            world1.Step(fix._0_25);
        }

        AssertEqual(world0.ComputeStateHash(), world1.ComputeStateHash());

        world1.AddForce(10000000, fix3.right);
        world1.Step(fix._0_25);
        AssertTrue(world0.ComputeStateHash() != world1.ComputeStateHash(), "hash should change after divergent replay input");
    }

    private static World CreateReplayHashWorld(WorldSettings settings)
    {
        World world = new World(settings);
        Rigid rigid = world.CreateRigid(fix3.zero, quaternion.identity);
        world.AddSphereCollider(rigid.id, fix3.zero, fix.One);
        world.SetVelocity(rigid.id, new fix3(1, 0, 0));
        return world;
    }

    private static void TestWorldStateHashIncludesConstraintParameters()
    {
        WorldSettings settings = new WorldSettings(false);
        World world0 = CreateConstraintHashWorld(settings, new fix(2));
        World world1 = CreateConstraintHashWorld(settings, new fix(3));

        AssertTrue(world0.ComputeStateHash() != world1.ComputeStateHash(), "state hash should include constraint parameters");
    }

    private static World CreateConstraintHashWorld(WorldSettings settings, fix distance)
    {
        World world = new World(settings);
        Rigid rigid0 = world.CreateRigid(fix3.zero, quaternion.identity);
        Rigid rigid1 = world.CreateRigid(new fix3(4, 0, 0), quaternion.identity);
        world.CreateDistanceConstraint(rigid0.id, rigid1.id, distance);
        return world;
    }

    private static void TestWorldLongReplayHashIsDeterministic()
    {
        WorldSettings settings = new WorldSettings(true, -10);
        settings.timeStep = fix.One / 60;
        settings.solverIterations = 4;
        settings.positionIterations = 2;
        settings.enableCCD = true;
        settings.enableDynamicCCD = true;
        settings.ccdMinVelocity = fix.Zero;
        World world0 = CreateLongReplayWorld(settings);
        World world1 = CreateLongReplayWorld(settings);

        ulong hash0 = 0;
        ulong hash1 = 0;
        for (int frame = 0; frame < 120; frame++)
        {
            ApplyReplayInputs(world0, frame);
            ApplyReplayInputs(world1, frame);
            hash0 = world0.StepAndComputeStateHash(settings.timeStep);
            hash1 = world1.StepAndComputeStateHash(settings.timeStep);
            AssertEqual(hash0, hash1);
        }

        AssertTrue(hash0 != 0 && hash1 != 0, "long replay hash should be non-zero");
    }

    private static World CreateLongReplayWorld(WorldSettings settings)
    {
        World world = new World(settings);
        Rigid floor = world.CreateRigid(new fix3(fix.Zero, -fix._0_5, fix.Zero), quaternion.identity);
        Rigid box = world.CreateRigid(new fix3(0, 2, 0), quaternion.identity);
        Rigid sphere = world.CreateRigid(new fix3(-2, 3, 0), quaternion.identity);
        Rigid slider = world.CreateRigid(new fix3(2, 2, 0), quaternion.identity);
        world.AddAABBCollider(floor.id, fix3.zero, new fix3(8, 1, 8));
        world.AddAABBCollider(box.id, fix3.zero, new fix3(1, 1, 1));
        world.AddSphereCollider(sphere.id, fix3.zero, fix._0_5);
        world.AddSphereCollider(slider.id, fix3.zero, fix._0_5);
        world.SetRigidType(floor.id, RigidType.Static);
        world.CreateDistanceConstraint(box.id, sphere.id, new fix(2));
        world.CreateSliderConstraint(floor.id, slider.id, new fix3(2, 0, 0), fix3.zero, fix3.right, fix3.right);
        return world;
    }

    private static void ApplyReplayInputs(World world, int frame)
    {
        if (frame % 15 == 0)
        {
            world.AddForce(10000001, new fix3(4, 0, 0));
        }

        if (frame % 20 == 5)
        {
            world.AddForce(10000002, new fix3(-3, 2, 0));
        }

        if (frame == 40)
        {
            world.SetVelocity(10000003, new fix3(-8, 0, 0));
        }
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
        AssertNear(fix3.zero, synced0.velocity, fix._0_0001);
        AssertNear(fix3.zero, synced1.velocity, fix._0_0001);
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
        world.SetMass(rigid0.id, fix.One);
        world.SetMass(rigid1.id, fix.One);

        world.Update(fix.Zero);

        AssertTrue(world.TryGetEntity(rigid0.id, out Entity entity0), "first entity should exist");
        AssertTrue(world.TryGetEntity(rigid1.id, out Entity entity1), "second entity should exist");
        AssertNear(new fix3(-fix._0_25, fix.Zero, fix.Zero), entity0.translation, fix._0_0001);
        AssertNear(new fix3(fix._1_5 + fix._0_25, fix.Zero, fix.Zero), entity1.translation, fix._0_0001);
    }

    private static void TestPositionCorrectionCanBeClamped()
    {
        WorldSettings settings = new WorldSettings(false);
        settings.positionCorrectionPercent = fix.One;
        settings.maxPositionCorrection = fix._0_1;
        settings.penetrationSlop = fix.Zero;
        World world = new World(settings);
        Rigid rigid0 = world.CreateRigid(fix3.zero, quaternion.identity);
        Rigid rigid1 = world.CreateRigid(new fix3(fix._1_5, fix.Zero, fix.Zero), quaternion.identity);
        world.AddSphereCollider(rigid0.id, fix3.zero, fix.One);
        world.AddSphereCollider(rigid1.id, fix3.zero, fix.One);
        world.SetMass(rigid0.id, fix.One);
        world.SetMass(rigid1.id, fix.One);

        world.Update(fix.Zero);

        AssertTrue(world.TryGetEntity(rigid0.id, out Entity entity0), "first clamped entity should exist");
        AssertTrue(world.TryGetEntity(rigid1.id, out Entity entity1), "second clamped entity should exist");
        AssertNear(new fix3(-fix._0_1, fix.Zero, fix.Zero), entity0.translation, fix._0_0001);
        AssertNear(new fix3(fix._1_5 + fix._0_1, fix.Zero, fix.Zero), entity1.translation, fix._0_0001);
    }

    private static void TestStackedBoxesStaySeparatedUnderGravity()
    {
        WorldSettings settings = new WorldSettings(true, -10);
        settings.timeStep = fix.One / 60;
        settings.solverIterations = 8;
        settings.positionIterations = 4;
        settings.positionCorrectionPercent = fix._0_2;
        settings.penetrationSlop = fix._0_01;
        World world = new World(settings);

        Rigid floor = world.CreateRigid(new fix3(fix.Zero, -fix._0_5, fix.Zero), quaternion.identity);
        Rigid box0 = world.CreateRigid(new fix3(fix.Zero, fix._0_5, fix.Zero), quaternion.identity);
        Rigid box1 = world.CreateRigid(new fix3(fix.Zero, fix._1_5, fix.Zero), quaternion.identity);
        Rigid box2 = world.CreateRigid(new fix3(fix.Zero, new fix(5) * fix._0_5, fix.Zero), quaternion.identity);
        world.AddAABBCollider(floor.id, fix3.zero, new fix3(8, 1, 8));
        world.AddAABBCollider(box0.id, fix3.zero, new fix3(1, 1, 1));
        world.AddAABBCollider(box1.id, fix3.zero, new fix3(1, 1, 1));
        world.AddAABBCollider(box2.id, fix3.zero, new fix3(1, 1, 1));
        world.SetRigidType(floor.id, RigidType.Static);

        for (int i = 0; i < 90; i++)
        {
            world.Step(settings.timeStep);
        }

        AssertTrue(world.TryGetEntity(box0.id, out Entity entity0), "bottom box should exist");
        AssertTrue(world.TryGetEntity(box1.id, out Entity entity1), "middle box should exist");
        AssertTrue(world.TryGetEntity(box2.id, out Entity entity2), "top box should exist");
        AssertTrue(entity0.translation.y >= fix._0_25, "bottom box should stay above the floor");
        AssertTrue(entity1.translation.y - entity0.translation.y >= fix._0_5, "middle box should stay above bottom box");
        AssertTrue(entity2.translation.y - entity1.translation.y >= fix._0_5, "top box should stay above middle box");
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

    private static void TestCollisionEventsReportEnterStayExit()
    {
        WorldSettings settings = new WorldSettings(false);
        settings.positionCorrectionPercent = fix.Zero;
        World world = new World(settings);
        Rigid rigid0 = world.CreateRigid(fix3.zero, quaternion.identity);
        Rigid rigid1 = world.CreateRigid(new fix3(fix._1_5, fix.Zero, fix.Zero), quaternion.identity);
        Collider collider0 = world.AddSphereCollider(rigid0.id, fix3.zero, fix.One);
        world.AddSphereCollider(rigid1.id, fix3.zero, fix.One);
        int legacyCount = 0;
        int enterCount = 0;
        int stayCount = 0;
        int exitCount = 0;
        int rigidEnterCount = 0;
        int rigidStayCount = 0;
        int rigidExitCount = 0;

        collider0.OnCollision += collision => legacyCount++;
        collider0.OnCollisionEnter += collision => enterCount++;
        collider0.OnCollisionStay += collision => stayCount++;
        collider0.OnCollisionExit += collision => exitCount++;
        rigid0.OnCollisionEnter += collision => rigidEnterCount++;
        rigid0.OnCollisionStay += collision => rigidStayCount++;
        rigid0.OnCollisionExit += collision => rigidExitCount++;

        world.Update(fix.Zero);
        world.Update(fix.Zero);
        world.SetTranslation(rigid1.id, new fix3(4, 0, 0));
        world.Update(fix.Zero);

        AssertEqual(2, legacyCount);
        AssertEqual(1, enterCount);
        AssertEqual(1, stayCount);
        AssertEqual(1, exitCount);
        AssertEqual(1, rigidEnterCount);
        AssertEqual(1, rigidStayCount);
        AssertEqual(1, rigidExitCount);
        AssertEqual(0, world.ContactManifolds.Count);
    }

    private static void TestTriggerEventsReportEnterStayExit()
    {
        WorldSettings settings = new WorldSettings(false);
        settings.positionCorrectionPercent = fix.Zero;
        World world = new World(settings);
        Rigid rigid0 = world.CreateRigid(fix3.zero, quaternion.identity);
        Rigid rigid1 = world.CreateRigid(new fix3(fix._1_5, fix.Zero, fix.Zero), quaternion.identity);
        Collider collider0 = world.AddSphereCollider(rigid0.id, fix3.zero, fix.One);
        Collider collider1 = world.AddSphereCollider(rigid1.id, fix3.zero, fix.One);
        int legacyCount = 0;
        int enterCount = 0;
        int stayCount = 0;
        int exitCount = 0;

        world.SetColliderTrigger(collider1.id, true);
        collider0.OnCollision += collision => legacyCount++;
        collider0.OnTriggerEnter += collision => enterCount++;
        collider0.OnTriggerStay += collision => stayCount++;
        collider0.OnTriggerExit += collision => exitCount++;

        world.Update(fix.Zero);
        world.Update(fix.Zero);
        world.SetTranslation(rigid1.id, new fix3(4, 0, 0));
        world.Update(fix.Zero);

        AssertEqual(2, legacyCount);
        AssertEqual(1, enterCount);
        AssertEqual(1, stayCount);
        AssertEqual(1, exitCount);
        AssertEqual(0, world.ContactManifolds.Count);
    }

    private static void TestDisabledAndRemovedObjectsClearCollisionState()
    {
        WorldSettings settings = new WorldSettings(false);
        settings.positionCorrectionPercent = fix.Zero;
        World world = new World(settings);
        Rigid rigid0 = world.CreateRigid(fix3.zero, quaternion.identity);
        Rigid rigid1 = world.CreateRigid(new fix3(fix._1_5, fix.Zero, fix.Zero), quaternion.identity);
        Collider collider0 = world.AddSphereCollider(rigid0.id, fix3.zero, fix.One);
        Collider collider1 = world.AddSphereCollider(rigid1.id, fix3.zero, fix.One);
        List<Collider> results = new List<Collider>();
        int enterCount = 0;
        int exitCount = 0;

        collider0.OnCollisionEnter += collision => enterCount++;
        collider0.OnCollisionExit += collision => exitCount++;

        world.Update(fix.Zero);

        AssertEqual(1, enterCount);
        AssertEqual(1, world.BroadphasePairs.Count);
        AssertEqual(1, world.ContactManifolds.Count);

        AssertTrue(world.SetColliderEnabled(collider1.id, false), "collider disable should succeed");

        AssertEqual(1, exitCount);
        AssertEqual(0, world.ContactManifolds.Count);
        world.QueryAABB(new AABB(new fix3(fix._1_5, fix.Zero, fix.Zero), new fix3(4, 4, 4)), results);
        AssertFalse(ContainsCollider(results, collider1.id), "disabled collider should be excluded from queries");

        world.Update(fix.Zero);

        AssertEqual(1, exitCount);
        AssertEqual(0, world.BroadphasePairs.Count);
        AssertEqual(0, world.ContactManifolds.Count);

        AssertTrue(world.SetColliderEnabled(collider1.id, true), "collider enable should succeed");
        world.Update(fix.Zero);

        AssertEqual(2, enterCount);
        AssertEqual(1, world.ContactManifolds.Count);
        AssertTrue(world.RemoveCollider(collider1.id), "collider removal should succeed");
        AssertEqual(2, exitCount);
        AssertFalse(world.TryGetCollider(collider1.id, out Collider _), "removed collider should not be registered");
        AssertEqual(0, world.ContactManifolds.Count);
    }

    private static void TestDisabledRigidClearsCollisionState()
    {
        WorldSettings settings = new WorldSettings(false);
        settings.positionCorrectionPercent = fix.Zero;
        World world = new World(settings);
        Rigid rigid0 = world.CreateRigid(fix3.zero, quaternion.identity);
        Rigid rigid1 = world.CreateRigid(new fix3(fix._1_5, fix.Zero, fix.Zero), quaternion.identity);
        Collider collider0 = world.AddSphereCollider(rigid0.id, fix3.zero, fix.One);
        Collider collider1 = world.AddSphereCollider(rigid1.id, fix3.zero, fix.One);
        List<Collider> results = new List<Collider>();
        int enterCount = 0;
        int exitCount = 0;

        collider0.OnCollisionEnter += collision => enterCount++;
        collider0.OnCollisionExit += collision => exitCount++;

        world.Update(fix.Zero);

        AssertEqual(1, enterCount);
        AssertEqual(1, world.ContactManifolds.Count);
        AssertTrue(world.SetRigidEnabled(rigid1.id, false), "rigid disable should succeed");

        AssertEqual(1, exitCount);
        AssertEqual(0, world.ContactManifolds.Count);
        world.QueryAABB(new AABB(new fix3(fix._1_5, fix.Zero, fix.Zero), new fix3(4, 4, 4)), results);
        AssertFalse(ContainsCollider(results, collider1.id), "disabled rigid collider should be excluded from queries");

        world.Update(fix.Zero);

        AssertEqual(0, world.BroadphasePairs.Count);
        AssertEqual(0, world.ContactManifolds.Count);
        AssertEqual(1, exitCount);

        AssertTrue(world.SetRigidEnabled(rigid1.id, true), "rigid enable should succeed");
        world.Update(fix.Zero);

        AssertEqual(2, enterCount);
        AssertEqual(1, world.ContactManifolds.Count);
        AssertTrue(world.TryGetCollider(collider1.id, out Collider _), "enabled rigid collider should remain registered");
    }

    private static void TestRemovingRigidRemovesCollidersAndCollisionState()
    {
        WorldSettings settings = new WorldSettings(false);
        settings.positionCorrectionPercent = fix.Zero;
        World world = new World(settings);
        Rigid rigid0 = world.CreateRigid(fix3.zero, quaternion.identity);
        Rigid rigid1 = world.CreateRigid(new fix3(fix._1_5, fix.Zero, fix.Zero), quaternion.identity);
        Collider collider0 = world.AddSphereCollider(rigid0.id, fix3.zero, fix.One);
        Collider collider1 = world.AddSphereCollider(rigid1.id, fix3.zero, fix.One);
        int exitCount = 0;

        collider0.OnCollisionExit += collision => exitCount++;
        world.Update(fix.Zero);

        AssertEqual(1, world.ContactManifolds.Count);
        AssertTrue(world.RemoveRigid(rigid1.id), "rigid removal should succeed");

        AssertEqual(1, exitCount);
        AssertFalse(world.TryGetRigid(rigid1.id, out Rigid _), "removed rigid should not be registered");
        AssertFalse(world.TryGetEntity(rigid1.id, out Entity _), "removed rigid entity should not be registered");
        AssertFalse(world.TryGetCollider(collider1.id, out Collider _), "removed rigid collider should not be registered");
        AssertEqual(1, world.Colliders.Count);
        AssertTrue(world.TryGetCollider(collider0.id, out Collider _), "unrelated collider should stay registered");
        AssertEqual(0, world.ContactManifolds.Count);

        world.Update(fix.Zero);

        AssertEqual(0, world.BroadphasePairs.Count);
        AssertEqual(0, world.ContactManifolds.Count);
        AssertEqual(1, exitCount);
    }

    private static void TestDistanceConstraintPreservesRigidDistance()
    {
        WorldSettings settings = new WorldSettings(false);
        settings.positionCorrectionPercent = fix.Zero;
        World world = new World(settings);
        Rigid rigid0 = world.CreateRigid(fix3.zero, quaternion.identity);
        Rigid rigid1 = world.CreateRigid(new fix3(4, 0, 0), quaternion.identity);
        World autoDistanceWorld = new World(settings);
        Rigid autoRigid0 = autoDistanceWorld.CreateRigid(fix3.zero, quaternion.identity);
        Rigid autoRigid1 = autoDistanceWorld.CreateRigid(new fix3(3, 0, 0), quaternion.identity);

        DistanceConstraint constraint = world.CreateDistanceConstraint(rigid0.id, rigid1.id, new fix(2));
        DistanceConstraint autoDistanceConstraint = autoDistanceWorld.CreateDistanceConstraint(autoRigid0.id, autoRigid1.id);

        AssertTrue(constraint != null, "distance constraint should be created");
        AssertTrue(autoDistanceConstraint != null, "auto distance constraint should be created");
        AssertEqual(new fix(3), autoDistanceConstraint.distance);
        AssertEqual(1, world.Constraints.Count);
        AssertTrue(world.TryGetDistanceConstraint(constraint.id, out DistanceConstraint _), "distance constraint should be queryable");

        world.Update(fix.Zero);

        AssertTrue(world.TryGetEntity(rigid0.id, out Entity entity0), "first constrained entity should exist");
        AssertTrue(world.TryGetEntity(rigid1.id, out Entity entity1), "second constrained entity should exist");
        AssertTrue(math.abs(math.distance(entity0.translation, entity1.translation) - new fix(2)) <= fix._0_0001, "distance constraint should correct current distance");

        world.SetVelocity(rigid0.id, fix3.left);
        world.SetVelocity(rigid1.id, fix3.right);
        world.Update(fix.One);

        AssertTrue(world.TryGetEntity(rigid0.id, out entity0), "first constrained entity should still exist");
        AssertTrue(world.TryGetEntity(rigid1.id, out entity1), "second constrained entity should still exist");
        AssertTrue(math.abs(math.distance(entity0.translation, entity1.translation) - new fix(2)) <= fix._0_0001, "distance constraint should preserve distance after integration");
    }

    private static void TestDisabledRigidDisablesDistanceConstraint()
    {
        World world = new World(new WorldSettings(false));
        Rigid rigid0 = world.CreateRigid(fix3.zero, quaternion.identity);
        Rigid rigid1 = world.CreateRigid(new fix3(4, 0, 0), quaternion.identity);
        DistanceConstraint constraint = world.CreateDistanceConstraint(rigid0.id, rigid1.id, new fix(2));

        AssertTrue(constraint != null, "distance constraint should be created");
        AssertTrue(world.SetRigidEnabled(rigid1.id, false), "rigid disable should succeed");
        world.Update(fix.Zero);

        AssertTrue(world.TryGetEntity(rigid0.id, out Entity entity0), "first constrained entity should exist");
        AssertTrue(world.TryGetEntity(rigid1.id, out Entity entity1), "second constrained entity should exist");
        AssertEqual(new fix(4), math.distance(entity0.translation, entity1.translation));

        AssertTrue(world.SetRigidEnabled(rigid1.id, true), "rigid enable should succeed");
        world.Update(fix.Zero);

        AssertTrue(world.TryGetEntity(rigid0.id, out entity0), "first constrained entity should still exist");
        AssertTrue(world.TryGetEntity(rigid1.id, out entity1), "second constrained entity should still exist");
        AssertTrue(math.abs(math.distance(entity0.translation, entity1.translation) - new fix(2)) <= fix._0_0001, "re-enabled rigid should participate in constraint solving");
    }

    private static void TestRemovingRigidRemovesConstraints()
    {
        World world = new World(new WorldSettings(false));
        Rigid rigid0 = world.CreateRigid(fix3.zero, quaternion.identity);
        Rigid rigid1 = world.CreateRigid(new fix3(4, 0, 0), quaternion.identity);
        DistanceConstraint constraint = world.CreateDistanceConstraint(rigid0.id, rigid1.id, new fix(2));

        AssertTrue(constraint != null, "distance constraint should be created");
        AssertEqual(1, world.Constraints.Count);
        AssertTrue(world.RemoveRigid(rigid1.id), "rigid removal should succeed");

        AssertEqual(0, world.Constraints.Count);
        AssertFalse(world.TryGetConstraint(constraint.id, out Constraint _), "constraint connected to removed rigid should be removed");
        AssertTrue(world.TryGetRigid(rigid0.id, out Rigid _), "unrelated rigid should stay registered");
        AssertFalse(world.TryGetRigid(rigid1.id, out Rigid _), "removed rigid should not be registered");
    }

    private static void TestPointConstraintPinsAnchors()
    {
        World world = new World(new WorldSettings(false));
        Rigid rigid0 = world.CreateRigid(fix3.zero, quaternion.identity);
        Rigid rigid1 = world.CreateRigid(new fix3(4, 0, 0), quaternion.identity);

        PointConstraint constraint = world.CreatePointConstraint(rigid0.id, rigid1.id);
        AssertTrue(constraint != null, "point constraint should be created");
        AssertTrue(world.TryGetPointConstraint(constraint.id, out PointConstraint _), "point constraint should be queryable");

        world.Update(fix.Zero);

        AssertTrue(world.TryGetEntity(rigid0.id, out Entity entity0), "first point entity should exist");
        AssertTrue(world.TryGetEntity(rigid1.id, out Entity entity1), "second point entity should exist");
        AssertTrue(math.distance(entity0.translation, entity1.translation) <= fix._0_0001, "point constraint should pin anchors together");
    }

    private static void TestSpringDistanceConstraintPullsTowardRestLength()
    {
        World world = new World(new WorldSettings(false));
        Rigid rigid0 = world.CreateRigid(fix3.zero, quaternion.identity);
        Rigid rigid1 = world.CreateRigid(new fix3(4, 0, 0), quaternion.identity);

        SpringDistanceConstraint constraint = world.CreateSpringDistanceConstraint(
            rigid0.id,
            rigid1.id,
            fix3.zero,
            fix3.zero,
            new fix(2),
            fix.One,
            fix.One);
        AssertTrue(constraint != null, "spring distance constraint should be created");
        AssertTrue(world.TryGetSpringDistanceConstraint(constraint.id, out SpringDistanceConstraint _), "spring distance constraint should be queryable");

        world.Update(fix.Zero);

        AssertTrue(world.TryGetEntity(rigid0.id, out Entity entity0), "first spring entity should exist");
        AssertTrue(world.TryGetEntity(rigid1.id, out Entity entity1), "second spring entity should exist");
        AssertTrue(math.abs(math.distance(entity0.translation, entity1.translation) - new fix(2)) <= fix._0_0001, "spring should correct toward rest length");
    }

    private static void TestHingeConstraintPinsAnchors()
    {
        World world = new World(new WorldSettings(false));
        Rigid rigid0 = world.CreateRigid(fix3.zero, quaternion.identity);
        Rigid rigid1 = world.CreateRigid(new fix3(4, 0, 0), quaternion.identity);

        HingeConstraint constraint = world.CreateHingeConstraint(rigid0.id, rigid1.id, fix3.zero, fix3.zero, fix3.up, fix3.up);
        AssertTrue(constraint != null, "hinge constraint should be created");
        AssertTrue(world.TryGetHingeConstraint(constraint.id, out HingeConstraint _), "hinge constraint should be queryable");

        world.Update(fix.Zero);

        AssertTrue(world.TryGetEntity(rigid0.id, out Entity entity0), "first hinge entity should exist");
        AssertTrue(world.TryGetEntity(rigid1.id, out Entity entity1), "second hinge entity should exist");
        AssertTrue(math.distance(entity0.translation, entity1.translation) <= fix._0_0001, "hinge should pin anchors together");
    }

    private static void TestFixedConstraintLocksRelativePose()
    {
        WorldSettings settings = new WorldSettings(false);
        settings.positionIterations = 8;
        World world = new World(settings);
        quaternion targetRotation = quaternion.RotateZ(math.PI * fix._0_25);
        Rigid rigid0 = world.CreateRigid(fix3.zero, quaternion.identity);
        Rigid rigid1 = world.CreateRigid(new fix3(2, 0, 0), targetRotation);
        world.SetRigidType(rigid0.id, RigidType.Static);

        FixedConstraint constraint = world.CreateFixedConstraint(rigid0.id, rigid1.id, new fix3(2, 0, 0), fix3.zero);
        AssertTrue(constraint != null, "fixed constraint should be created");
        AssertTrue(world.TryGetFixedConstraint(constraint.id, out FixedConstraint _), "fixed constraint should be queryable");

        world.SetTransform(rigid1.id, new fix3(4, 0, 0), quaternion.identity);
        world.Update(fix.Zero);

        AssertTrue(world.TryGetEntity(rigid1.id, out Entity entity1), "fixed body should exist");
        AssertNear(new fix3(2, 0, 0), entity1.translation, fix._0_01);
        AssertNear(targetRotation * fix3.right, entity1.orientation * fix3.right, fix._0_1);
    }

    private static void TestSliderConstraintPreservesSlideAxis()
    {
        WorldSettings settings = new WorldSettings(false);
        settings.positionIterations = 4;
        World world = new World(settings);
        Rigid rail = world.CreateRigid(fix3.zero, quaternion.identity);
        Rigid slider = world.CreateRigid(new fix3(4, 2, 0), quaternion.RotateZ(math.PI * fix._0_25));
        world.SetRigidType(rail.id, RigidType.Static);

        SliderConstraint constraint = world.CreateSliderConstraint(rail.id, slider.id, fix3.zero, fix3.zero, fix3.right, fix3.right);
        AssertTrue(constraint != null, "slider constraint should be created");
        AssertTrue(world.TryGetSliderConstraint(constraint.id, out SliderConstraint _), "slider constraint should be queryable");

        world.Update(fix.Zero);

        AssertTrue(world.TryGetEntity(slider.id, out Entity sliderEntity), "slider entity should exist");
        AssertNear(new fix3(4, 0, 0), sliderEntity.translation, fix._0_01);
        AssertNear(fix3.right, sliderEntity.orientation * fix3.right, fix._0_1);
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
        AssertNear(new fix3(2, 0, 0), dynamicEntity.translation, fix._0_0001);
        AssertTrue(world.TryGetRigid(dynamicRigid.id, out Rigid syncedDynamic), "dynamic rigid should exist");
        AssertNear(fix3.zero, syncedDynamic.velocity, fix._0_0001);
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
        AssertNear(fix3.right, syncedDynamic.velocity, fix._0_0001);
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
        AssertNear(fix3.zero, syncedDynamic.velocity, fix._0_0001);
    }

    private static void TestSolverAppliesTwoAxisFriction()
    {
        WorldSettings settings = new WorldSettings(false);
        settings.friction = fix._2;
        settings.positionCorrectionPercent = fix.Zero;
        World world = new World(settings);
        Rigid dynamicRigid = world.CreateRigid(fix3.zero, quaternion.identity);
        Rigid staticRigid = world.CreateRigid(new fix3(fix.Zero, fix._1_5, fix.Zero), quaternion.identity);
        world.AddSphereCollider(dynamicRigid.id, fix3.zero, fix.One);
        world.AddSphereCollider(staticRigid.id, fix3.zero, fix.One);

        world.SetRigidType(staticRigid.id, RigidType.Static);
        world.SetInertia(dynamicRigid.id, fix3.zero);
        world.SetVelocity(dynamicRigid.id, new fix3(1, 1, 1));
        world.Update(fix.Zero);

        AssertTrue(world.TryGetRigid(dynamicRigid.id, out Rigid syncedDynamic), "dynamic rigid should exist");
        AssertNear(fix3.zero, syncedDynamic.velocity, fix._0_0001);
        AssertEqual(1, world.ContactManifolds.Count);
        ContactPoint point = world.ContactManifolds[0][0];
        AssertTrue(math.abs(point.tangentImpulse0) > math.Epsilon, "first tangent impulse should be used");
        AssertTrue(math.abs(point.tangentImpulse1) > math.Epsilon, "second tangent impulse should be used");
    }

    private static void TestRestitutionThresholdSuppressesLowSpeedBounce()
    {
        WorldSettings settings = new WorldSettings(false);
        settings.friction = fix.Zero;
        settings.restitution = fix.One;
        settings.restitutionVelocityThreshold = fix._0_5;
        settings.positionCorrectionPercent = fix.Zero;
        World world = new World(settings);
        Rigid staticRigid = world.CreateRigid(fix3.zero, quaternion.identity);
        Rigid dynamicRigid = world.CreateRigid(new fix3(fix._1_5, fix.Zero, fix.Zero), quaternion.identity);
        world.AddSphereCollider(staticRigid.id, fix3.zero, fix.One);
        world.AddSphereCollider(dynamicRigid.id, fix3.zero, fix.One);

        world.SetRigidType(staticRigid.id, RigidType.Static);
        world.SetVelocity(dynamicRigid.id, fix3.left * fix._0_25);
        world.Update(fix.Zero);

        AssertTrue(world.TryGetRigid(dynamicRigid.id, out Rigid syncedDynamic), "dynamic threshold rigid should exist");
        AssertNear(fix3.zero, syncedDynamic.velocity, fix._0_0001);
    }

    private static void TestMaterialBouncinessAffectsRestitution()
    {
        WorldSettings settings = new WorldSettings(false);
        settings.friction = fix.Zero;
        settings.restitution = fix.Zero;
        settings.positionCorrectionPercent = fix.Zero;
        World world = new World(settings);
        Rigid staticRigid = world.CreateRigid(fix3.zero, quaternion.identity);
        Rigid dynamicRigid = world.CreateRigid(new fix3(fix._1_5, fix.Zero, fix.Zero), quaternion.identity);
        Collider staticCollider = world.AddSphereCollider(staticRigid.id, fix3.zero, fix.One);
        Collider dynamicCollider = world.AddSphereCollider(dynamicRigid.id, fix3.zero, fix.One);

        world.SetRigidType(staticRigid.id, RigidType.Static);
        world.SetColliderMaterial(staticCollider.id, new Material(fix.One, fix.Zero, fix.One));
        world.SetColliderMaterial(dynamicCollider.id, new Material(fix.One, fix.Zero, fix.One));
        world.SetVelocity(dynamicRigid.id, fix3.left);
        world.Update(fix.Zero);

        AssertTrue(world.TryGetRigid(dynamicRigid.id, out Rigid syncedDynamic), "dynamic rigid should exist");
        AssertNear(fix3.right, syncedDynamic.velocity, fix._0_0001);
    }

    private static void TestMaterialFrictionAffectsSolver()
    {
        WorldSettings settings = new WorldSettings(false);
        settings.friction = fix.Zero;
        settings.positionCorrectionPercent = fix.Zero;
        World world = new World(settings);
        Rigid dynamicRigid = world.CreateRigid(fix3.zero, quaternion.identity);
        Rigid staticRigid = world.CreateRigid(new fix3(fix.Zero, fix._1_5, fix.Zero), quaternion.identity);
        Collider dynamicCollider = world.AddSphereCollider(dynamicRigid.id, fix3.zero, fix.One);
        Collider staticCollider = world.AddSphereCollider(staticRigid.id, fix3.zero, fix.One);

        world.SetRigidType(staticRigid.id, RigidType.Static);
        world.SetInertia(dynamicRigid.id, fix3.zero);
        world.SetColliderMaterial(dynamicCollider.id, new Material(fix.One, fix.One, fix.Zero));
        world.SetColliderMaterial(staticCollider.id, new Material(fix.One, fix.One, fix.Zero));
        world.SetVelocity(dynamicRigid.id, new fix3(1, 1, 0));
        world.Update(fix.Zero);

        AssertTrue(world.TryGetRigid(dynamicRigid.id, out Rigid syncedDynamic), "dynamic rigid should exist");
        AssertNear(fix3.zero, syncedDynamic.velocity, fix._0_0001);
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

    private static void TestShapeAndConstraintFrameworksExposeCapabilities()
    {
        ShapeDescriptor sphere = PhysicsApi.GetShapeDescriptor(ShapeType.Sphere);
        ShapeDescriptor obb = PhysicsApi.GetShapeDescriptor(ShapeType.OBB);
        ShapeDescriptor triangleMesh = PhysicsApi.GetShapeDescriptor(ShapeType.TriangleMesh);
        ConstraintDescriptor hinge = ConstraintRegistry.GetDescriptor(ConstraintType.Hinge);
        ConstraintDescriptor sixDof = ConstraintRegistry.GetDescriptor(ConstraintType.SixDof);

        AssertTrue(sphere.implemented, "sphere should be an implemented primitive shape");
        AssertTrue((sphere.capabilities & ShapeCapabilities.ShapeCast) != 0, "sphere descriptor should expose shape cast support");
        AssertTrue((obb.capabilities & ShapeCapabilities.ShapeCast) != 0, "OBB descriptor should expose SAT shape cast support");
        AssertFalse(triangleMesh.implemented, "triangle mesh should be framework-only for now");
        AssertEqual((int)ShapeCategory.SceneGeometry, (int)triangleMesh.category);
        AssertTrue(hinge.implemented, "hinge descriptor should be implemented");
        AssertFalse(sixDof.implemented, "six dof descriptor should be framework-only for now");
        AssertTrue(World.IsConstraintTypeImplemented(ConstraintType.Fixed), "world should expose implemented constraint types");
    }

    private static void TestPhysicsShapeCastReportsTimeOfImpact()
    {
        Sphere moving = new Sphere(fix3.zero, fix.One);
        AABB target = new AABB(new fix3(5, 0, 0), new fix3(2, 2, 2));

        bool hit = PhysicsApi.TryShapeCast(moving, target, new fix3(10, 0, 0), out ShapeCastHit hitInfo);

        AssertTrue(hit, "shape cast should hit expanded target");
        AssertEqual(new fix(3) / 10, hitInfo.fraction);
        AssertEqual(fix3.right, hitInfo.normal);
    }

    private static void TestPhysicsShapeCastSweepsCapsuleTargets()
    {
        Sphere movingSphere = new Sphere(fix3.zero, fix._0_5);
        Capsule targetCapsule = new Capsule(new fix3(5, 0, 0), fix._0_5, new fix(3), quaternion.identity, fix3.up);
        Capsule movingCapsule = new Capsule(fix3.zero, fix._0_5, new fix(3), quaternion.identity, fix3.up);
        AABB targetBox = new AABB(new fix3(5, 0, 0), new fix3(2, 2, 2));

        AssertTrue(PhysicsApi.TryShapeCast(movingSphere, targetCapsule, new fix3(10, 0, 0), out ShapeCastHit sphereHit), "sphere should sweep against capsule");
        AssertEqual(new fix(4) / 10, sphereHit.fraction);
        AssertEqual(fix3.right, sphereHit.normal);

        AssertTrue(PhysicsApi.TryShapeCast(movingCapsule, targetBox, new fix3(10, 0, 0), out ShapeCastHit capsuleHit), "capsule should sweep against box");
        AssertEqual(new fix(7) / 20, capsuleHit.fraction);
        AssertEqual(fix3.right, capsuleHit.normal);
    }

    private static void TestPhysicsShapeCastSweepsOBBWithSAT()
    {
        OBB moving = new OBB(fix3.zero, new fix3(2, 2, 2), quaternion.RotateZ(math.PI * fix._0_25));
        AABB target = new AABB(new fix3(5, 0, 0), new fix3(2, 2, 2));

        AssertTrue(PhysicsApi.TryShapeCast(moving, target, new fix3(10, 0, 0), out ShapeCastHit hit), "OBB should use swept SAT shape cast");
        AssertTrue(hit.fraction > fix.Zero && hit.fraction < fix.One, "swept SAT shape cast should report a valid time of impact");
        AssertEqual(fix3.right, hit.normal);
    }

    private static void TestWorldClampsUnsafeMotionValues()
    {
        WorldSettings settings = new WorldSettings(false);
        settings.maxLinearVelocity = fix.One;
        settings.maxTranslationPerStep = fix._0_5;
        World world = new World(settings);
        Rigid rigid = world.CreateRigid(fix3.zero, quaternion.identity);

        world.SetVelocity(rigid.id, new fix3(10, 0, 0));
        world.Update(fix.One);

        AssertTrue(world.TryGetEntity(rigid.id, out Entity entity), "entity should exist");
        AssertTrue(entity.translation.x <= fix._0_5, "translation should be clamped by per-step limit");
        AssertTrue(world.TryGetRigid(rigid.id, out Rigid syncedRigid), "rigid should exist");
        AssertTrue(syncedRigid.velocity.x <= fix._0_5, "velocity should be clamped for the step");

        world.SetVelocity(rigid.id, new fix3(fix.NaN, fix.NaN, fix.NaN));
        AssertTrue(world.TryGetRigid(rigid.id, out syncedRigid), "rigid should still exist");
        AssertEqual(fix3.zero, syncedRigid.velocity);
    }

    private static void TestCapsuleZeroAxisStaysNumericallySafe()
    {
        Capsule capsule = new Capsule(fix3.zero, fix._0_5, new fix(3), quaternion.identity, fix3.zero);

        AssertEqual(fix3.up, capsule.Axis);
        AssertTrue(PhysicsSafety.IsFinite(capsule.Center1), "capsule endpoint should be finite");
        AssertTrue(PhysicsSafety.IsFinite(capsule.Center2), "capsule endpoint should be finite");
    }

    private static void TestSolverClampsContactImpulses()
    {
        WorldSettings settings = new WorldSettings(false);
        settings.positionCorrectionPercent = fix.Zero;
        settings.maxContactImpulse = fix._0_5;
        World world = new World(settings);
        Rigid dynamicRigid = world.CreateRigid(fix3.zero, quaternion.identity);
        Rigid staticRigid = world.CreateRigid(new fix3(fix._1_5, fix.Zero, fix.Zero), quaternion.identity);
        world.AddSphereCollider(dynamicRigid.id, fix3.zero, fix.One);
        world.AddSphereCollider(staticRigid.id, fix3.zero, fix.One);

        world.SetRigidType(staticRigid.id, RigidType.Static);
        world.SetVelocity(dynamicRigid.id, new fix3(10, 0, 0));
        world.Update(fix.Zero);

        AssertTrue(world.TryGetRigid(dynamicRigid.id, out Rigid syncedRigid), "dynamic rigid should exist");
        AssertTrue(syncedRigid.velocity.x >= new fix(9), "contact impulse clamp should limit solver velocity correction");
        AssertEqual(1, world.ContactManifolds.Count);
        AssertTrue(world.ContactManifolds[0][0].normalImpulse <= fix._0_5, "stored normal impulse should be clamped");
    }

    private static void TestSolverContactVelocityBiasSeparatesPenetration()
    {
        WorldSettings settings = new WorldSettings(false);
        settings.positionCorrectionPercent = fix.Zero;
        settings.contactVelocityBiasFactor = fix._0_2;
        settings.maxContactBiasVelocity = fix.One;
        World world = new World(settings);
        Rigid rigid0 = world.CreateRigid(fix3.zero, quaternion.identity);
        Rigid rigid1 = world.CreateRigid(new fix3(fix._1_5, fix.Zero, fix.Zero), quaternion.identity);
        world.AddSphereCollider(rigid0.id, fix3.zero, fix.One);
        world.AddSphereCollider(rigid1.id, fix3.zero, fix.One);

        world.Update(fix.One);

        AssertTrue(world.TryGetRigid(rigid0.id, out Rigid synced0), "first rigid should exist");
        AssertTrue(world.TryGetRigid(rigid1.id, out Rigid synced1), "second rigid should exist");
        AssertTrue(synced0.velocity.x < fix.Zero, "bias should push first body away");
        AssertTrue(synced1.velocity.x > fix.Zero, "bias should push second body away");
    }

    private static void TestContactManifoldSettingsControlPersistence()
    {
        ContactManifold manifold = new ContactManifold();
        ContactManifoldSettings strictSettings = ContactManifoldSettings.Default;
        strictSettings.anchorMatchDistance = fix._0_001;
        strictSettings.positionMatchDistance = fix._0_001;

        CollisionInfo first = new CollisionInfo(1, 2)
        {
            normal = fix3.up,
        };
        first.AddContact(fix3.zero, new fix3(fix.Zero, -fix._0_1, fix.Zero), fix._0_1);

        CollisionInfo shifted = new CollisionInfo(1, 2)
        {
            normal = fix3.up,
        };
        shifted.AddContact(new fix3(fix._0_01, fix.Zero, fix.Zero), new fix3(fix._0_01, -fix._0_1, fix.Zero), fix._0_1);

        manifold.Update(first, false, 1, strictSettings);
        manifold.Update(shifted, false, 2, strictSettings);

        AssertEqual(1, manifold.contactCount);
        AssertEqual(1, manifold[0].lifetime);
    }

    private static void TestWorldReportsStepStatsAfterUpdate()
    {
        World world = new World(new WorldSettings(false));
        world.Reserve(4, 4, 2);
        Rigid rigid0 = world.CreateRigid(fix3.zero, quaternion.identity);
        Rigid rigid1 = world.CreateRigid(new fix3(fix._1_5, fix.Zero, fix.Zero), quaternion.identity);
        world.AddSphereCollider(rigid0.id, fix3.zero, fix.One);
        world.AddSphereCollider(rigid1.id, fix3.zero, fix.One);

        world.Update(fix.Zero);

        AssertEqual(2, world.LastStepStats.activeColliderCount);
        AssertEqual(1, world.LastStepStats.broadphasePairCount);
        AssertEqual(2, world.LastStepStats.broadphaseProxyCount);
        AssertTrue(world.LastStepStats.broadphaseTreeHeight > 0, "broadphase tree height should be reported");
        AssertEqual(1, world.LastStepStats.contactManifoldCount);
        AssertEqual(1, world.LastStepStats.islandCount);
    }

    private static void TestWorldWarmUpdateAllocationBaselineStaysLow()
    {
        WorldSettings settings = new WorldSettings(false);
        settings.positionCorrectionPercent = fix.Zero;
        World world = new World(settings);
        world.Reserve(4, 4, 1);
        Rigid rigid0 = world.CreateRigid(fix3.zero, quaternion.identity);
        Rigid rigid1 = world.CreateRigid(new fix3(3, 0, 0), quaternion.identity);
        world.AddSphereCollider(rigid0.id, fix3.zero, fix.One);
        world.AddSphereCollider(rigid1.id, fix3.zero, fix.One);

        world.Update(fix.Zero);
        world.Update(fix.Zero);
        long before = GC.GetAllocatedBytesForCurrentThread();
        world.Update(fix.Zero);
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;

        AssertTrue(allocated <= 1024, $"warm stationary update allocation baseline regressed, got {allocated}");
    }

    private static void TestCollisionCallbackLifecycleChangesAreDeferredSafely()
    {
        WorldSettings settings = new WorldSettings(false);
        settings.positionCorrectionPercent = fix.Zero;
        World world = new World(settings);
        Rigid rigid0 = world.CreateRigid(fix3.zero, quaternion.identity);
        Rigid rigid1 = world.CreateRigid(new fix3(fix._1_5, fix.Zero, fix.Zero), quaternion.identity);
        Collider collider0 = world.AddSphereCollider(rigid0.id, fix3.zero, fix.One);
        Collider collider1 = world.AddSphereCollider(rigid1.id, fix3.zero, fix.One);
        int enterCount = 0;

        collider0.OnCollisionEnter += collision =>
        {
            enterCount++;
            AssertTrue(world.TryGetRigid(rigid1.id, out Rigid _), "rigid should still exist during callback");
            AssertTrue(world.RemoveRigid(rigid1.id), "callback rigid removal should be queued");
            AssertTrue(world.TryGetRigid(rigid1.id, out Rigid _), "queued rigid removal should not mutate during callback");
        };

        world.Update(fix.Zero);

        AssertEqual(1, enterCount);
        AssertFalse(world.TryGetRigid(rigid1.id, out Rigid _), "queued rigid should be removed after callbacks");
        AssertFalse(world.TryGetCollider(collider1.id, out Collider _), "queued rigid collider should be removed after callbacks");
        AssertTrue(world.TryGetCollider(collider0.id, out Collider _), "unrelated collider should remain");
        AssertEqual(1, world.LastStepStats.deferredLifecycleOperationCount);
    }

    private static void TestCollisionCallbackCanCreateBodiesSafely()
    {
        WorldSettings settings = new WorldSettings(false);
        settings.positionCorrectionPercent = fix.Zero;
        World world = new World(settings);
        Rigid rigid0 = world.CreateRigid(fix3.zero, quaternion.identity);
        Rigid rigid1 = world.CreateRigid(new fix3(fix._1_5, fix.Zero, fix.Zero), quaternion.identity);
        Collider collider0 = world.AddSphereCollider(rigid0.id, fix3.zero, fix.One);
        world.AddSphereCollider(rigid1.id, fix3.zero, fix.One);
        ulong createdRigidId = 0;
        int enterCount = 0;

        collider0.OnCollisionEnter += collision =>
        {
            enterCount++;
            Rigid created = world.CreateRigid(new fix3(10, 0, 0), quaternion.identity);
            world.AddSphereCollider(created.id, fix3.zero, fix.One);
            createdRigidId = created.id;
        };

        world.Update(fix.Zero);

        AssertEqual(1, enterCount);
        AssertTrue(createdRigidId != 0, "callback should create a body");
        AssertTrue(world.TryGetRigid(createdRigidId, out Rigid _), "created rigid should be registered after callback");
    }

    private static void TestCollisionCallbackExceptionsCanBeCaptured()
    {
        WorldSettings settings = new WorldSettings(false);
        settings.positionCorrectionPercent = fix.Zero;
        settings.catchCallbackExceptions = true;
        World world = new World(settings);
        Rigid rigid0 = world.CreateRigid(fix3.zero, quaternion.identity);
        Rigid rigid1 = world.CreateRigid(new fix3(fix._1_5, fix.Zero, fix.Zero), quaternion.identity);
        Collider collider0 = world.AddSphereCollider(rigid0.id, fix3.zero, fix.One);
        world.AddSphereCollider(rigid1.id, fix3.zero, fix.One);
        int rigidCallbackCount = 0;

        collider0.OnCollisionEnter += collision => throw new InvalidOperationException("callback failure");
        rigid0.OnCollisionEnter += collision => rigidCallbackCount++;

        world.Update(fix.Zero);

        AssertEqual(1, world.LastStepStats.callbackExceptionCount);
        AssertTrue(world.LastCallbackException is InvalidOperationException, "last callback exception should be captured");
        AssertEqual(1, rigidCallbackCount);
    }

    private static void TestCCDStopsFastBodyBeforeStaticCollider()
    {
        WorldSettings settings = new WorldSettings(false);
        settings.enableCCD = true;
        settings.ccdMinVelocity = fix.Zero;
        settings.positionCorrectionPercent = fix.Zero;
        World world = new World(settings);
        Rigid dynamicRigid = world.CreateRigid(fix3.zero, quaternion.identity);
        Rigid staticRigid = world.CreateRigid(new fix3(5, 0, 0), quaternion.identity);
        world.AddSphereCollider(dynamicRigid.id, fix3.zero, fix._0_5);
        world.AddAABBCollider(staticRigid.id, fix3.zero, new fix3(1, 4, 4));

        world.SetRigidType(staticRigid.id, RigidType.Static);
        world.SetVelocity(dynamicRigid.id, new fix3(10, 0, 0));
        world.Update(fix.One);

        AssertTrue(world.TryGetEntity(dynamicRigid.id, out Entity entity), "dynamic entity should exist");
        AssertNear(new fix3(4, 0, 0), entity.translation, fix._0_0001);
        AssertTrue(world.TryGetRigid(dynamicRigid.id, out Rigid syncedRigid), "dynamic rigid should exist");
        AssertEqual(fix.Zero, syncedRigid.velocity.x);
        AssertTrue(entity.translation.x < new fix(4) + fix._0_5, "CCD should keep the dynamic body before the wall");
    }

    private static void TestCCDSweepsCapsuleAgainstStaticCollider()
    {
        WorldSettings settings = new WorldSettings(false);
        settings.enableCCD = true;
        settings.ccdMinVelocity = fix.Zero;
        settings.positionCorrectionPercent = fix.Zero;
        World world = new World(settings);
        Rigid dynamicRigid = world.CreateRigid(fix3.zero, quaternion.identity);
        Rigid staticRigid = world.CreateRigid(new fix3(5, 0, 0), quaternion.identity);
        world.AddCapsuleCollider(dynamicRigid.id, fix3.zero, fix._0_5, new fix(3), quaternion.identity);
        world.AddAABBCollider(staticRigid.id, fix3.zero, new fix3(1, 4, 4));

        world.SetRigidType(staticRigid.id, RigidType.Static);
        world.SetVelocity(dynamicRigid.id, new fix3(10, 0, 0));
        world.Update(fix.One);

        AssertTrue(world.TryGetEntity(dynamicRigid.id, out Entity entity), "dynamic capsule entity should exist");
        AssertNear(new fix3(4, 0, 0), entity.translation, fix._0_0001);
        AssertTrue(world.TryGetRigid(dynamicRigid.id, out Rigid syncedRigid), "dynamic capsule rigid should exist");
        AssertEqual(fix.Zero, syncedRigid.velocity.x);
    }

    private static void TestCCDUsesEarliestCompoundColliderHit()
    {
        WorldSettings settings = new WorldSettings(false);
        settings.enableCCD = true;
        settings.ccdMinVelocity = fix.Zero;
        settings.positionCorrectionPercent = fix.Zero;
        World world = new World(settings);
        Rigid compoundRigid = world.CreateRigid(fix3.zero, quaternion.identity);
        Rigid staticRigid = world.CreateRigid(new fix3(5, 0, 0), quaternion.identity);
        world.AddSphereCollider(compoundRigid.id, new fix3(1, 0, 0), fix._0_5);
        world.AddSphereCollider(compoundRigid.id, new fix3(-1, 0, 0), fix._0_5);
        world.AddAABBCollider(staticRigid.id, fix3.zero, new fix3(1, 4, 4));

        world.SetRigidType(staticRigid.id, RigidType.Static);
        world.SetVelocity(compoundRigid.id, new fix3(10, 0, 0));
        world.Update(fix.One);

        AssertTrue(world.TryGetEntity(compoundRigid.id, out Entity entity), "compound rigid entity should exist");
        AssertNear(new fix3(3, 0, 0), entity.translation, fix._0_001);
    }

    private static void TestCCDSweepsSphereAgainstSphere()
    {
        WorldSettings settings = new WorldSettings(false);
        settings.enableCCD = true;
        settings.ccdMinVelocity = fix.Zero;
        settings.positionCorrectionPercent = fix.Zero;
        World world = new World(settings);
        Rigid dynamicRigid = world.CreateRigid(fix3.zero, quaternion.identity);
        Rigid staticRigid = world.CreateRigid(new fix3(4, 3, 0), quaternion.identity);
        world.AddSphereCollider(dynamicRigid.id, fix3.zero, fix._0_5);
        world.AddSphereCollider(staticRigid.id, fix3.zero, fix._0_5);

        world.SetRigidType(staticRigid.id, RigidType.Static);
        world.SetVelocity(dynamicRigid.id, new fix3(8, 6, 0));
        world.Update(fix.One);

        AssertTrue(world.TryGetEntity(dynamicRigid.id, out Entity entity), "dynamic sphere entity should exist");
        AssertNear(new fix3(new fix(16) / 5, new fix(12) / 5, fix.Zero), entity.translation, fix._0_01);
        AssertTrue(world.TryGetRigid(dynamicRigid.id, out Rigid syncedRigid), "dynamic sphere rigid should exist");
        AssertNear(fix3.zero, syncedRigid.velocity, fix._0_01);
    }

    private static void TestCCDSweepsDynamicSphereAgainstDynamicSphere()
    {
        WorldSettings settings = new WorldSettings(false);
        settings.enableCCD = true;
        settings.enableDynamicCCD = true;
        settings.ccdMinVelocity = fix.Zero;
        settings.positionCorrectionPercent = fix.Zero;
        World world = new World(settings);
        Rigid movingRigid = world.CreateRigid(fix3.zero, quaternion.identity);
        Rigid targetRigid = world.CreateRigid(new fix3(5, 0, 0), quaternion.identity);
        world.AddSphereCollider(movingRigid.id, fix3.zero, fix._0_5);
        world.AddSphereCollider(targetRigid.id, fix3.zero, fix._0_5);

        world.SetVelocity(movingRigid.id, new fix3(10, 0, 0));
        world.Update(fix.One);

        AssertTrue(world.TryGetEntity(movingRigid.id, out Entity movingEntity), "moving dynamic CCD entity should exist");
        AssertNear(new fix3(4, 0, 0), movingEntity.translation, fix._0_0001);
        AssertTrue(world.TryGetEntity(targetRigid.id, out Entity targetEntity), "target dynamic CCD entity should exist");
        AssertEqual(new fix3(5, 0, 0), targetEntity.translation);
        AssertTrue(world.TryGetRigid(movingRigid.id, out Rigid syncedRigid), "moving dynamic CCD rigid should exist");
        AssertEqual(fix.Zero, syncedRigid.velocity.x);
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

    private static void TestColliderInitializesRigidMassProperties()
    {
        World world = new World(new WorldSettings(false));
        Rigid rigid = world.CreateRigid(fix3.zero, quaternion.identity);

        world.AddAABBCollider(rigid.id, fix3.zero, new fix3(2, 2, 2));

        AssertTrue(world.TryGetRigid(rigid.id, out Rigid syncedRigid), "rigid should exist");
        fix expectedInertia = new fix(64) / 12;
        AssertEqual(new fix(8), syncedRigid.mass);
        AssertEqual(new fix3(expectedInertia, expectedInertia, expectedInertia), syncedRigid.inertia);
    }

    private static void TestColliderDensityUpdatesAutomaticMassProperties()
    {
        World world = new World(new WorldSettings(false));
        Rigid rigid = world.CreateRigid(fix3.zero, quaternion.identity);
        Collider collider = world.AddAABBCollider(rigid.id, fix3.zero, new fix3(2, 2, 2));

        world.SetColliderDensity(collider.id, fix._2);

        AssertTrue(world.TryGetRigid(rigid.id, out Rigid syncedRigid), "rigid should exist");
        fix expectedInertia = new fix(128) / 12;
        AssertEqual(new fix(16), syncedRigid.mass);
        AssertEqual(new fix3(expectedInertia, expectedInertia, expectedInertia), syncedRigid.inertia);
    }

    private static void TestManualMassPropertiesSurviveDensityChanges()
    {
        World world = new World(new WorldSettings(false));
        Rigid rigid = world.CreateRigid(fix3.zero, quaternion.identity);
        Collider collider = world.AddAABBCollider(rigid.id, fix3.zero, new fix3(2, 2, 2));
        fix3 manualInertia = new fix3(7, 8, 9);

        world.SetMass(rigid.id, 5);
        world.SetInertia(rigid.id, manualInertia);
        world.SetColliderDensity(collider.id, fix._2);

        AssertTrue(world.TryGetRigid(rigid.id, out Rigid syncedRigid), "rigid should exist");
        AssertEqual(new fix(5), syncedRigid.mass);
        AssertEqual(manualInertia, syncedRigid.inertia);
    }

    private static void TestRigidSupportsMultipleColliders()
    {
        World world = new World(new WorldSettings(false));
        Rigid compoundRigid = world.CreateRigid(fix3.zero, quaternion.identity);
        Rigid otherRigid = world.CreateRigid(new fix3(3, 0, 0), quaternion.identity);
        Collider left = world.AddSphereCollider(compoundRigid.id, new fix3(-2, 0, 0), fix.One);
        Collider right = world.AddSphereCollider(compoundRigid.id, new fix3(2, 0, 0), fix.One);
        Collider other = world.AddSphereCollider(otherRigid.id, fix3.zero, fix.One);

        world.Update(fix.Zero);

        AssertTrue(world.TryGetRigid(compoundRigid.id, out Rigid syncedRigid), "compound rigid should exist");
        AssertEqual(2, syncedRigid.colliderCount);
        AssertEqual(left.id, syncedRigid.colliderId);
        AssertTrue(syncedRigid.ContainsCollider(left.id), "left collider should be registered on rigid");
        AssertTrue(syncedRigid.ContainsCollider(right.id), "right collider should be registered on rigid");
        AssertEqual(1, world.BroadphasePairs.Count);
        AssertEqual(right.id, world.BroadphasePairs[0].colliderId0);
        AssertEqual(other.id, world.BroadphasePairs[0].colliderId1);

        AssertTrue(world.RemoveCollider(left.id), "first collider removal should succeed");
        AssertTrue(world.TryGetRigid(compoundRigid.id, out syncedRigid), "compound rigid should still exist");
        AssertEqual(1, syncedRigid.colliderCount);
        AssertEqual(right.id, syncedRigid.colliderId);
        AssertFalse(world.TryGetCollider(left.id, out Collider _), "removed collider should not be registered");
        AssertTrue(world.TryGetCollider(right.id, out Collider _), "remaining collider should stay registered");
    }

    private static void TestCompoundBodyAggregatesAutomaticMassProperties()
    {
        World world = new World(new WorldSettings(false));
        Rigid rigid = world.CreateRigid(fix3.zero, quaternion.identity);
        Collider left = world.AddAABBCollider(rigid.id, new fix3(-1, 0, 0), new fix3(2, 2, 2));
        Collider right = world.AddAABBCollider(rigid.id, new fix3(1, 0, 0), new fix3(2, 2, 2));

        AssertTrue(world.TryGetRigid(rigid.id, out Rigid syncedRigid), "compound rigid should exist");
        fix baseInertia = new fix(64) / 12;
        AssertEqual(new fix(16), syncedRigid.mass);
        AssertEqual(new fix3(baseInertia * 2, baseInertia * 2 + 16, baseInertia * 2 + 16), syncedRigid.inertia);

        AssertTrue(world.RemoveCollider(left.id), "left collider removal should succeed");
        AssertTrue(world.TryGetRigid(rigid.id, out syncedRigid), "compound rigid should still exist");
        AssertEqual(new fix(8), syncedRigid.mass);
        AssertEqual(new fix3(baseInertia, baseInertia + 8, baseInertia + 8), syncedRigid.inertia);

        world.SetColliderDensity(right.id, fix._2);
        AssertTrue(world.TryGetRigid(rigid.id, out syncedRigid), "compound rigid should still exist after density change");
        AssertEqual(new fix(16), syncedRigid.mass);
        AssertEqual(new fix3(baseInertia * 2, baseInertia * 2 + 16, baseInertia * 2 + 16), syncedRigid.inertia);
    }

    private static void TestWorldBuildsIslandsFromContactsAndConstraints()
    {
        World world = new World(new WorldSettings(false));
        Rigid rigid0 = world.CreateRigid(fix3.zero, quaternion.identity);
        Rigid rigid1 = world.CreateRigid(new fix3(fix._1_5, fix.Zero, fix.Zero), quaternion.identity);
        Rigid rigid2 = world.CreateRigid(new fix3(5, 0, 0), quaternion.identity);
        world.AddSphereCollider(rigid0.id, fix3.zero, fix.One);
        world.AddSphereCollider(rigid1.id, fix3.zero, fix.One);
        world.AddSphereCollider(rigid2.id, fix3.zero, fix.One);

        DistanceConstraint constraint = world.CreateDistanceConstraint(rigid1.id, rigid2.id);
        AssertTrue(constraint != null, "distance constraint should be created");

        world.Update(fix.Zero);

        AssertEqual(1, world.Islands.Count);
        AssertEqual(3, world.Islands[0].RigidIds.Count);
        AssertTrue(ContainsRigidId(world.Islands[0].RigidIds, rigid0.id), "contact body should be in island");
        AssertTrue(ContainsRigidId(world.Islands[0].RigidIds, rigid1.id), "shared body should be in island");
        AssertTrue(ContainsRigidId(world.Islands[0].RigidIds, rigid2.id), "constraint body should be in island");
    }

    private static void TestSleepingIslandSkipsIntegrationAndWakes()
    {
        WorldSettings settings = new WorldSettings(false);
        settings.sleepTime = fix.Zero;
        World world = new World(settings);
        Rigid rigid = world.CreateRigid(fix3.zero, quaternion.identity);
        world.AddSphereCollider(rigid.id, fix3.zero, fix.One);

        world.Update(fix.Zero);

        AssertTrue(world.TryGetRigid(rigid.id, out Rigid sleepingRigid), "rigid should exist");
        AssertTrue(sleepingRigid.isSleeping, "resting rigid should sleep immediately when sleepTime is zero");
        AssertEqual(1, world.Islands.Count);
        AssertTrue(world.Islands[0].sleeping, "single body island should be marked sleeping");

        world.Update(fix.One);

        AssertTrue(world.TryGetEntity(rigid.id, out Entity entity), "sleeping entity should exist");
        AssertEqual(fix3.zero, entity.translation);

        world.AddForce(rigid.id, fix3.right);
        world.Update(fix.One);

        AssertTrue(world.TryGetRigid(rigid.id, out Rigid awakeRigid), "rigid should still exist");
        AssertFalse(awakeRigid.isSleeping, "external force should wake a sleeping rigid");
        AssertTrue(awakeRigid.velocity.x > fix.Zero, "force should affect velocity after waking");
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

    private static bool ContainsCollider(List<Collider> colliders, ulong colliderId)
    {
        for (int i = 0; i < colliders.Count; i++)
        {
            if (colliders[i].id == colliderId)
            {
                return true;
            }
        }

        return false;
    }

    private static bool ContainsRigidId(IReadOnlyList<ulong> rigidIds, ulong rigidId)
    {
        for (int i = 0; i < rigidIds.Count; i++)
        {
            if (rigidIds[i] == rigidId)
            {
                return true;
            }
        }

        return false;
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
