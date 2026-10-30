using BenchmarkDotNet.Characteristics;
using BenchmarkDotNet.Running;
using BenchmarkDotNet.Toolchains;
using BenchmarkDotNet.Toolchains.CsProj;
using BenchmarkDotNet.Toolchains.DotNetCli;
using BenchmarkDotNet.Validators;
using System.Collections.Generic;

namespace Gravitas.Benchmarks;

internal sealed class BenchmarkProjectToolchain : Toolchain
{
    private readonly IToolchain standard;

    internal BenchmarkProjectToolchain(string cliPath, string packagesPath)
        : this(CsProjCoreToolchain.From(new NetCoreAppSettings("net10.0", null, ".NET 10.0", cliPath, packagesPath)),
            cliPath, packagesPath) { }

    private BenchmarkProjectToolchain(IToolchain standard, string cliPath, string packagesPath)
        : base(standard.Name, new BenchmarkProjectGenerator(cliPath, packagesPath), standard.Builder, standard.Executor)
        => this.standard = standard;

    public override IEnumerable<ValidationError> Validate(BenchmarkCase benchmarkCase, IResolver resolver) =>
        standard.Validate(benchmarkCase, resolver);
}
