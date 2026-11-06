using System;
using System.IO;
using Maphy.Mathematics;
using Maphy.Physics;
using Maphy.Unity;
using UnityEngine;

namespace Maphy.Validation
{
    public sealed class PlayerEntry : MonoBehaviour
    {
        private void Start()
        {
            try
            {
                VerifyAdapters();
                string[] args = Environment.GetCommandLineArgs();
                int index = Array.IndexOf(args, "--corpus-out");
                if (index < 0 || index + 1 >= args.Length) throw new ArgumentException("--corpus-out path is required");
                using (var output = new StreamWriter(args[index + 1], false, new System.Text.UTF8Encoding(false)))
                    DeterminismCorpus.Write(output);
                Debug.Log("Maphy validation succeeded on Unity " + Application.unityVersion);
                Application.Quit(0);
            }
            catch (Exception error)
            {
                Debug.LogException(error);
                Application.Quit(1);
            }
        }

        private static void VerifyAdapters()
        {
            fix3 converted = MaphyUnityConvert.ToFix3(new Vector3(0.1f, -0.1f, float.NaN));
            if (converted.x != new fix(0.1f) || converted.y != new fix(-0.1f) || !fix.IsNaN(converted.z))
                throw new InvalidOperationException("Unity Vector3 conversion differs from scalar conversion");
            var instance = new GameObject("Maphy adapter smoke test");
            instance.transform.position = new Vector3(0.1f, 0f, 0f);
            var world = new PhysicsWorld(new PhysicsWorldSettings(false));
            world.Reserve(1, 1, 1, 1);
            if (!world.CreateBody(instance.transform, RigidType.Dynamic, out BodyHandle body))
                throw new InvalidOperationException("Unity body creation failed");
            world.SetBodyVelocity(body, fix3.right);
            world.Step(fix.One / 60);
            if (!world.PullTransform(body, instance.transform) || !world.TryGetBody(body, out var state)
                || new fix(instance.transform.position.x) != state.position.x)
                throw new InvalidOperationException("Unity Transform round trip failed");
            Destroy(instance);
        }
    }
}
