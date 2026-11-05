using System;
using System.Globalization;
using System.IO;
using System.Runtime.InteropServices;
using Maphy.Mathematics;
using Maphy.Physics;

namespace Maphy.Validation
{
    /// <summary>The same source is compiled by .NET and the Unity player builders.</summary>
    public static class DeterminismCorpus
    {
        [StructLayout(LayoutKind.Explicit)]
        private struct Single { [FieldOffset(0)] public uint bits; [FieldOffset(0)] public float number; }

        public static void Write(TextWriter output)
        {
            output.WriteLine("Maphy deterministic corpus v1");
            uint[] singles = { 0, 0x80000000, 1, 0x80000001, 0x37000000, 0xB7000000,
                0x37800000, 0x3DCCCCCD, 0xBDCCCCCD, 0x3F800000, 0x47800000,
                0x56FFFFFF, 0x57000000, 0xD7000000, 0x7F7FFFFF, 0x7F800000, 0xFF800000, 0x7FC00001 };
            foreach (uint bits in singles)
                output.WriteLine("float," + bits.ToString("X8", CultureInfo.InvariantCulture) + "," + new fix(new Single { bits = bits }.number).value.ToString(CultureInfo.InvariantCulture));
            ulong[] doubles = { 0, 0x8000000000000000, 1, 0x3EE0000000000000,
                0xBEE0000000000000, 0x3FB999999999999A, 0x3FF0000000000000,
                0x42DFFFFFFFFFFFFF, 0x42E0000000000000, 0xC2E0000000000000,
                0x7FEFFFFFFFFFFFFF, 0x7FF0000000000000, 0xFFF0000000000000, 0x7FF8000000000001 };
            foreach (ulong bits in doubles)
                output.WriteLine("double," + bits.ToString("X16", CultureInfo.InvariantCulture) + "," + new fix(BitConverter.Int64BitsToDouble(unchecked((long)bits))).value.ToString(CultureInfo.InvariantCulture));
            decimal[] decimals = { 0m, 0.1m, -0.1m, decimal.MaxValue, decimal.MinValue,
                140737488355328m, -140737488355328m, 140737488355327.99999999999999m,
                -140737488355327.99999999999999m, 1m / 131072m };
            for (int i = 0; i < decimals.Length; i++)
                output.WriteLine(FormattableString.Invariant($"decimal,{i},{new fix(decimals[i]).value}"));
            long[] raw = { long.MinValue, long.MinValue + 1, -4294967296, -65537, -65536, -32768, -1,
                0, 1, 32768, 65536, 65537, 4294967296, long.MaxValue };
            foreach (long a in raw)
            foreach (long b in raw)
            {
                fix x = fix.Raw(a), y = fix.Raw(b);
                output.WriteLine(FormattableString.Invariant($"raw,{a},{b},{(x + y).value},{(x - y).value},{(x * y).value},{(x / y).value},{math.sqrt(x).value},{math.round(x).value}"));
            }
            WriteReplay(output);
        }

        private static void WriteReplay(TextWriter output)
        {
            var settings = new PhysicsWorldSettings(true);
            settings.enableCCD = true;
            settings.enableDynamicCCD = true;
            settings.ccdMinVelocity = fix.Zero;
            settings.solverIterations = 4;
            settings.positionIterations = 2;
            var core = new PhysicsWorld(settings);
            core.Reserve(32, 32, 256, 256);
            Require(core.CreateBody(new fix3(0, -1, 0), quaternion.identity, RigidType.Static, out var floor));
            Require(core.AddAABB(floor, fix3.zero, new fix3(20, 1, 20), out _));
            Require(core.CreateBody(new fix3(5, 3, 0), quaternion.identity, RigidType.Static, out var wall));
            Require(core.AddAABB(wall, fix3.zero, new fix3(fix._0_1, new fix(8), new fix(8)), out _));
            Require(core.CreateBody(new fix3(-4, 3, 0), quaternion.identity, RigidType.Dynamic, out var sphere));
            Require(core.AddSphere(sphere, fix3.zero, fix._0_5, out _));
            Require(core.CreateBody(new fix3(0, 4, 0), quaternion.identity, RigidType.Dynamic, out var box));
            Require(core.AddOBB(box, fix3.zero, new fix3(1), quaternion.RotateZ(fix._0_25), out _));
            Require(core.CreateBody(new fix3(0, 2, 3), quaternion.identity, RigidType.Dynamic, out var capsule));
            Require(core.AddCapsule(capsule, fix3.zero, fix._0_5, new fix(2), quaternion.identity, out _));
            core.SetBodyAngularVelocity(box, fix3.up);

            var objects = new World(new WorldSettings(true));
            var ground = objects.CreateRigid(new fix3(0, -1, 0), quaternion.identity);
            objects.SetRigidType(ground.id, RigidType.Static);
            objects.AddAABBCollider(ground.id, fix3.zero, new fix3(20, 1, 20));
            var a = objects.CreateRigid(new fix3(0, 3, 0), quaternion.identity);
            var b = objects.CreateRigid(new fix3(2, 3, 0), quaternion.identity);
            objects.AddSphereCollider(a.id, fix3.zero, fix._0_5);
            objects.AddSphereCollider(b.id, fix3.zero, fix._0_5);
            objects.CreateDistanceConstraint(a.id, b.id, new fix(2));
            for (int frame = 0; frame < 1000; frame++)
            {
                if (frame % 100 == 0)
                {
                    core.SetBodyTransform(sphere, new fix3(-4, 3, 0), quaternion.identity);
                    core.SetBodyVelocity(sphere, new fix3(600, 0, 0));
                    objects.AddForce(a.id, new fix3(4, 2, 0));
                }
                if (frame == 200) core.SetBodyEnabled(box, false);
                if (frame == 250) core.SetBodyEnabled(box, true);
                if (frame == 400) Require(core.DestroyBody(capsule));
                if (frame == 420)
                {
                    Require(core.CreateBody(new fix3(0, 2, 3), quaternion.identity, RigidType.Dynamic, out capsule));
                    Require(core.AddCapsule(capsule, fix3.zero, fix._0_5, new fix(2), quaternion.identity, out _));
                }
                core.Step(fix.One / 60);
                objects.Step(fix.One / 60);
                if (core.LastStepStats.HasOverflow) throw new InvalidOperationException("Replay corpus capacity overflow");
                output.WriteLine(FormattableString.Invariant($"frame,{frame},{core.ComputeStateHash():X16},{objects.ComputeStateHash():X16}"));
            }
        }

        private static void Require(bool success)
        {
            if (!success) throw new InvalidOperationException("Could not construct replay corpus");
        }
    }
}
