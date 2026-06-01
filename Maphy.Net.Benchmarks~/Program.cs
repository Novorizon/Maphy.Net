using System;
using System.Diagnostics;
using System.IO;
using System.Text;
using Maphy.Mathematics;
using Maphy.Physics;

internal static class Program
{
    private const int WarmupIterations = 8;
    private const string Header = "Name,Iterations,ElapsedMs,MicrosecondsPerIteration,AllocatedBytes,Hash,Bodies,Colliders,Pairs,Contacts,BodySyncs,ColliderSyncs,BroadphaseMovedProxies,BroadphaseCandidates,NarrowphaseTests,ManifoldNew,ManifoldReused,ManifoldDropped,SolverPoints,SleepingBodies,AwakeDynamicBodies,Overflow";

    private static int Main(string[] args)
    {
        if (HasArg(args, "--help") || HasArg(args, "-h"))
        {
            WriteUsage(Console.Out);
            return 0;
        }

        int iterations = ReadIntArg(args, "--iterations", 120);
        bool quick = HasArg(args, "--quick");
        int countOverride = ReadIntArg(args, "--count", -1);
        string scenarioFilter = ReadStringArg(args, "--scenario", "all");
        string outputPath = ReadStringArg(args, "--out", null);

        StreamWriter fileWriter = null;
        TextWriter output = Console.Out;
        try
        {
            if (!string.IsNullOrEmpty(outputPath))
            {
                string fullPath = Path.GetFullPath(outputPath);
                string directory = Path.GetDirectoryName(fullPath);
                if (!string.IsNullOrEmpty(directory))
                {
                    Directory.CreateDirectory(directory);
                }

                fileWriter = new StreamWriter(fullPath, false, new UTF8Encoding(false));
                output = fileWriter;
            }

            output.WriteLine(Header);

            if (ShouldRunScenario(scenarioFilter, "StaticQuery"))
            {
                if (countOverride > 0)
                {
                    RunStaticQuery(countOverride, iterations, output);
                }
                else
                {
                    RunStaticQuery(quick ? 100 : 100, iterations, output);
                    RunStaticQuery(quick ? 200 : 500, iterations, output);
                    RunStaticQuery(quick ? 300 : 1000, iterations, output);
                }
            }

            if (ShouldRunScenario(scenarioFilter, "DynamicBroadphase"))
            {
                if (countOverride > 0)
                {
                    RunDynamicBroadphase(countOverride, iterations, output);
                }
                else
                {
                    RunDynamicBroadphase(quick ? 100 : 100, iterations, output);
                    RunDynamicBroadphase(quick ? 200 : 500, iterations, output);
                }
            }

            if (ShouldRunScenario(scenarioFilter, "StackSolver"))
            {
                RunStackSolver(countOverride > 0 ? countOverride : quick ? 16 : 48, iterations, output);
            }

            if (ShouldRunScenario(scenarioFilter, "SleepingBodies"))
            {
                RunSleepingBodies(countOverride > 0 ? countOverride : quick ? 100 : 500, iterations, output);
            }

            if (ShouldRunScenario(scenarioFilter, "CCD"))
            {
                RunCCD(countOverride > 0 ? countOverride : quick ? 16 : 64, iterations, output);
            }

            output.Flush();
        }
        finally
        {
            if (fileWriter != null)
            {
                fileWriter.Dispose();
            }
        }

        return 0;
    }

    private static void RunStaticQuery(int count, int iterations, TextWriter output)
    {
        PhysicsWorld world = new PhysicsWorld(new PhysicsWorldSettings(false));
        world.Reserve(PhysicsWorldCapacity.ForBodies(count, 1, 2));
        int side = CeilingSqrt(count);
        for (int i = 0; i < count; i++)
        {
            int x = i % side;
            int z = i / side;
            Require(world.CreateBody(new fix3(x * 4, 0, z * 4), quaternion.identity, RigidType.Static, out BodyHandle body), "body");
            Require(world.AddAABB(body, fix3.zero, new fix3(1, 1, 1), out ColliderHandle _), "collider");
        }

        ColliderHandle[] results = new ColliderHandle[count];
        AABB bounds = new AABB(new fix3(side * 2, 0, side * 2), new fix3(side * 5, 4, side * 5));
        Measure($"StaticQuery/{count}", iterations, world, output, () =>
        {
            world.Update(fix.Zero);
            int hitCount = world.QueryAABBNonAlloc(bounds, results);
            if (hitCount != count)
            {
                throw new InvalidOperationException($"Expected {count} query hits, got {hitCount}");
            }
        });
    }

    private static void RunDynamicBroadphase(int count, int iterations, TextWriter output)
    {
        PhysicsWorldSettings settings = new PhysicsWorldSettings(false);
        settings.positionCorrectionPercent = fix.Zero;
        PhysicsWorld world = new PhysicsWorld(settings);
        world.Reserve(PhysicsWorldCapacity.ForBodies(count, 1, 2));
        for (int i = 0; i < count; i++)
        {
            Require(world.CreateBody(new fix3(i * 3, 0, 0), quaternion.identity, RigidType.Dynamic, out BodyHandle body), "body");
            Require(world.AddSphere(body, fix3.zero, fix._0_5, out ColliderHandle _), "collider");
            fix direction = (i & 1) == 0 ? fix.One : -fix.One;
            Require(world.SetBodyVelocity(body, new fix3(direction, fix.Zero, fix.Zero)), "velocity");
        }

        Measure($"DynamicBroadphase/{count}", iterations, world, output, () =>
        {
            world.Step(fix.One / 60);
        });
    }

    private static void RunStackSolver(int count, int iterations, TextWriter output)
    {
        PhysicsWorldSettings settings = new PhysicsWorldSettings(true);
        settings.solverIterations = 4;
        settings.positionIterations = 4;
        settings.enableSleeping = false;
        PhysicsWorld world = new PhysicsWorld(settings);
        world.Reserve(count + 1, count + 1, count * 8, count * 8);
        Require(world.CreateBody(new fix3(0, -1, 0), quaternion.identity, RigidType.Static, out BodyHandle floor), "floor");
        Require(world.AddAABB(floor, fix3.zero, new fix3(50, 1, 50), out ColliderHandle _), "floor collider");
        for (int i = 0; i < count; i++)
        {
            Require(world.CreateBody(new fix3(0, i + 1, 0), quaternion.identity, RigidType.Dynamic, out BodyHandle body), "body");
            Require(world.AddAABB(body, fix3.zero, new fix3(1, 1, 1), out ColliderHandle _), "collider");
        }

        Measure($"StackSolver/{count}", iterations, world, output, () =>
        {
            world.Step(fix.One / 60);
        });
    }

    private static void RunSleepingBodies(int count, int iterations, TextWriter output)
    {
        PhysicsWorldSettings settings = new PhysicsWorldSettings(false);
        settings.sleepTime = fix.Zero;
        settings.linearSleepThreshold = fix._0_01;
        settings.angularSleepThreshold = fix._0_01;
        PhysicsWorld world = new PhysicsWorld(settings);
        world.Reserve(PhysicsWorldCapacity.ForBodies(count, 1, 1));
        for (int i = 0; i < count; i++)
        {
            Require(world.CreateBody(new fix3(i * 4, 0, 0), quaternion.identity, RigidType.Dynamic, out BodyHandle body), "body");
            Require(world.AddSphere(body, fix3.zero, fix._0_5, out ColliderHandle _), "collider");
        }

        world.Update(fix.One / 60);
        Measure($"SleepingBodies/{count}", iterations, world, output, () =>
        {
            world.Step(fix.One / 60);
        });
    }

    private static void RunCCD(int count, int iterations, TextWriter output)
    {
        PhysicsWorldSettings settings = new PhysicsWorldSettings(false);
        settings.enableCCD = true;
        settings.enableDynamicCCD = true;
        settings.ccdMinVelocity = fix.Zero;
        settings.positionCorrectionPercent = fix.Zero;
        PhysicsWorld world = new PhysicsWorld(settings);
        world.Reserve(count * 2, count * 2, count * 8, count * 8);
        BodyHandle[] movers = new BodyHandle[count];
        for (int i = 0; i < count; i++)
        {
            fix z = i * 3;
            Require(world.CreateBody(new fix3(fix.Zero, fix.Zero, z), quaternion.identity, RigidType.Dynamic, out movers[i]), "mover");
            Require(world.AddSphere(movers[i], fix3.zero, fix._0_5, out ColliderHandle _), "mover collider");
            Require(world.CreateBody(new fix3(new fix(5), fix.Zero, z), quaternion.identity, RigidType.Static, out BodyHandle wall), "wall");
            Require(world.AddAABB(wall, fix3.zero, new fix3(1, 2, 1), out ColliderHandle _), "wall collider");
        }

        Measure($"CCD/{count}", iterations, world, output, () =>
        {
            for (int i = 0; i < movers.Length; i++)
            {
                fix z = i * 3;
                world.SetBodyTransform(movers[i], new fix3(fix.Zero, fix.Zero, z), quaternion.identity);
                world.SetBodyVelocity(movers[i], new fix3(10, 0, 0));
            }

            world.Update(fix.One);
        });
    }

    private static void Measure(string name, int iterations, PhysicsWorld world, TextWriter output, Action step)
    {
        for (int i = 0; i < WarmupIterations; i++)
        {
            step();
        }

        long beforeAlloc = GC.GetAllocatedBytesForCurrentThread();
        long startTimestamp = Stopwatch.GetTimestamp();
        ulong hash = 0;
        for (int i = 0; i < iterations; i++)
        {
            step();
            hash = world.ComputeStateHash();
        }

        long endTimestamp = Stopwatch.GetTimestamp();
        long allocated = GC.GetAllocatedBytesForCurrentThread() - beforeAlloc;
        PhysicsWorldStepStats stats = world.LastStepStats;
        double elapsedMs = (endTimestamp - startTimestamp) * 1000.0 / Stopwatch.Frequency;
        double microseconds = elapsedMs * 1000.0 / Math.Max(1, iterations);
        output.WriteLine(
            $"{name},{iterations},{elapsedMs:F3},{microseconds:F3},{allocated},{hash},{stats.activeBodyCount},{stats.activeColliderCount},{stats.pairCount},{stats.contactManifoldCount},{stats.bodySyncCount},{stats.colliderSyncCount},{stats.broadphaseMovedProxyCount},{stats.broadphaseCandidateCount},{stats.narrowPhaseTestCount},{stats.contactManifoldNewCount},{stats.contactManifoldReusedCount},{stats.contactManifoldDroppedCount},{stats.solverContactPointCount},{stats.sleepingBodyCount},{stats.awakeDynamicBodyCount},{stats.HasOverflow}");
    }

    private static int CeilingSqrt(int value)
    {
        int side = 1;
        while (side * side < value)
        {
            side++;
        }

        return side;
    }

    private static bool HasArg(string[] args, string name)
    {
        for (int i = 0; i < args.Length; i++)
        {
            if (string.Equals(args[i], name, StringComparison.OrdinalIgnoreCase))
            {
                return true;
            }
        }

        return false;
    }

    private static int ReadIntArg(string[] args, string name, int fallback)
    {
        for (int i = 0; i < args.Length - 1; i++)
        {
            if (string.Equals(args[i], name, StringComparison.OrdinalIgnoreCase)
                && int.TryParse(args[i + 1], out int value)
                && value > 0)
            {
                return value;
            }
        }

        return fallback;
    }

    private static string ReadStringArg(string[] args, string name, string fallback)
    {
        for (int i = 0; i < args.Length - 1; i++)
        {
            if (string.Equals(args[i], name, StringComparison.OrdinalIgnoreCase))
            {
                return args[i + 1];
            }
        }

        return fallback;
    }

    private static bool ShouldRunScenario(string filter, string scenario)
    {
        if (string.IsNullOrEmpty(filter)
            || string.Equals(filter, "all", StringComparison.OrdinalIgnoreCase))
        {
            return true;
        }

        string[] parts = filter.Split(new[] { ',', ';', '|' }, StringSplitOptions.RemoveEmptyEntries);
        for (int i = 0; i < parts.Length; i++)
        {
            string part = parts[i].Trim();
            if (string.Equals(part, scenario, StringComparison.OrdinalIgnoreCase))
            {
                return true;
            }
        }

        return false;
    }

    private static void WriteUsage(TextWriter output)
    {
        output.WriteLine("Usage:");
        output.WriteLine("  dotnet run --project Maphy.Net.Benchmarks~/Maphy.Net.Benchmarks.csproj -- [options]");
        output.WriteLine("Options:");
        output.WriteLine("  --quick                  Use smaller default scenario sizes.");
        output.WriteLine("  --iterations N           Number of measured iterations per scenario.");
        output.WriteLine("  --scenario A,B           Run selected scenarios: StaticQuery, DynamicBroadphase, StackSolver, SleepingBodies, CCD.");
        output.WriteLine("  --count N                Override scenario object count.");
        output.WriteLine("  --out path.csv           Write CSV output to a file.");
    }

    private static void Require(bool condition, string label)
    {
        if (!condition)
        {
            throw new InvalidOperationException($"Failed to create benchmark {label}");
        }
    }
}
