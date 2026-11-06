using System;
using System.IO;
using Maphy.Validation;

internal static class Program
{
    private static void Main(string[] args)
    {
        if (args.Length == 0) { DeterminismCorpus.Write(Console.Out); return; }
        if (args.Length != 2 || args[0] != "--out") throw new ArgumentException("Usage: --out path.csv");
        using (var writer = new StreamWriter(args[1], false, new System.Text.UTF8Encoding(false)))
            DeterminismCorpus.Write(writer);
    }
}
