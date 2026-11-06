using System;
using System.IO;
using Maphy.Validation;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEditor.SceneManagement;
using UnityEngine;

public static class MaphyValidationBuild
{
    public static void Mono() { Build(ScriptingImplementation.Mono2x); }
    public static void IL2CPP() { Build(ScriptingImplementation.IL2CPP); }

    private static void Build(ScriptingImplementation backend)
    {
        try
        {
            if (!Application.unityVersion.StartsWith("2022.3.", StringComparison.Ordinal))
                throw new InvalidOperationException("Unity 2022.3 is required");
            string[] args = Environment.GetCommandLineArgs();
            int index = Array.IndexOf(args, "--build-output");
            if (index < 0 || index + 1 >= args.Length) throw new ArgumentException("--build-output path is required");
            string path = Path.GetFullPath(args[index + 1]);
            Directory.CreateDirectory(Path.GetDirectoryName(path));
            var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            new GameObject("Validation").AddComponent<PlayerEntry>();
            EditorSceneManager.SaveScene(scene, "Assets/Validation.unity");
            PlayerSettings.SetScriptingBackend(BuildTargetGroup.Standalone, backend);
            PlayerSettings.SetApiCompatibilityLevel(BuildTargetGroup.Standalone, ApiCompatibilityLevel.NET_Standard);
            var report = BuildPipeline.BuildPlayer(new BuildPlayerOptions
            {
                scenes = new[] { "Assets/Validation.unity" },
                locationPathName = path,
                target = BuildTarget.StandaloneLinux64,
                options = BuildOptions.None,
            });
            if (report.summary.result != BuildResult.Succeeded) throw new InvalidOperationException("Player build failed");
            EditorApplication.Exit(0);
        }
        catch (Exception error)
        {
            Debug.LogException(error);
            EditorApplication.Exit(1);
        }
    }
}
