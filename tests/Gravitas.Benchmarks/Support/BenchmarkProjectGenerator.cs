using BenchmarkDotNet.Loggers;
using BenchmarkDotNet.Running;
using BenchmarkDotNet.Toolchains;
using BenchmarkDotNet.Toolchains.CsProj;
using System;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Xml.Linq;

namespace Gravitas.Benchmarks;

internal sealed class BenchmarkProjectGenerator : CsProjGenerator
{
    internal BenchmarkProjectGenerator(string cliPath, string packagesPath)
        : base("net10.0", cliPath, packagesPath, runtimeFrameworkVersion: null) { }

    protected override FileInfo GetProjectFilePath(Type benchmarkTarget, ILogger logger) =>
        new(typeof(Program).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>()
            .Single(attribute => attribute.Key == "BenchmarkProjectPath").Value);

#if USE_LOCAL_LSF_STACK
    protected override void GenerateProject(BuildPartition buildPartition, ArtifactsPaths artifactsPaths, ILogger logger)
    {
        base.GenerateProject(buildPartition, artifactsPaths, logger);
        var project = XDocument.Load(artifactsPaths.ProjectFilePath);
        var references = new XElement("ItemGroup");
        // Generated signatures need dependency types even when implicit source edges are disabled.
        foreach (AssemblyName name in typeof(Program).Assembly.GetReferencedAssemblies())
        {
            if (name.Name == "System" || name.Name == "netstandard"
                || name.Name.StartsWith("System.", StringComparison.Ordinal)
                || name.Name.StartsWith("BenchmarkDotNet", StringComparison.Ordinal)) continue;
            references.Add(new XElement("Reference", new XAttribute("Include", name.FullName),
                new XElement("HintPath", Assembly.Load(name).Location), new XElement("Private", "true")));
        }
        project.Root.Add(references);
        project.Save(artifactsPaths.ProjectFilePath);
    }
#endif
}
