using BenchmarkDotNet.Running;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using BenchmarkDotNet.Configs;
using BenchmarkDotNet.ConsoleArguments;
using BenchmarkDotNet.ConsoleArguments.ListBenchmarks;
using BenchmarkDotNet.Jobs;
using BenchmarkDotNet.Loggers;
using BenchmarkDotNet.Reports;

namespace Gravitas.Benchmarks;

internal static class Program
{
    private static readonly BenchmarkCatalog _catalog = BenchmarkCatalog.Create(typeof(Program).Assembly);

    private static int Main(string[] args)
    {
        if (args.Length == 0)
        {
            return RunBenchmarks(BenchmarkSwitcher.FromAssembly(typeof(Program).Assembly), args);
        }

        string command = args[0];

        if ((string.Equals(command, "list", StringComparison.OrdinalIgnoreCase) ||
             string.Equals(command, "ls", StringComparison.OrdinalIgnoreCase)) &&
            args.Length == 1)
        {
            _catalog.WriteAvailableSelections(Console.Out);
            return 0;
        }

        if (string.Equals(command, "help", StringComparison.OrdinalIgnoreCase) && args.Length == 1)
        {
            WriteUsage();
            return 0;
        }

        if (string.Equals(command, "all", StringComparison.OrdinalIgnoreCase))
        {
            if (args.Length > 1 && !args[1].StartsWith("-", StringComparison.Ordinal))
            {
                Console.Error.WriteLine("The 'all' selection cannot be combined with other benchmark aliases.");
                Console.Error.WriteLine();
                WriteUsage();
                return 1;
            }

            return RunBenchmarks(BenchmarkSwitcher.FromAssembly(typeof(Program).Assembly),
                EnsureAllBenchmarksSelected(args.Skip(1).ToArray()));
        }

        int aliasCount = 0;
        while (aliasCount < args.Length && !args[aliasCount].StartsWith("-", StringComparison.Ordinal))
            aliasCount++;

        if (aliasCount == 0)
        {
            return RunBenchmarks(BenchmarkSwitcher.FromAssembly(typeof(Program).Assembly), args);
        }

        Type[] selectedTypes = _catalog.Resolve(args.Take(aliasCount).ToArray(), out string unknownAlias);
        if (unknownAlias != null)
        {
            Console.Error.WriteLine($"Unknown benchmark selection '{unknownAlias}'.");
            Console.Error.WriteLine();
            WriteUsage();
            return 1;
        }

        return RunBenchmarks(BenchmarkSwitcher.FromTypes(selectedTypes),
            EnsureAllBenchmarksSelected(args.Skip(aliasCount).ToArray()));
    }

    private static int RunBenchmarks(BenchmarkSwitcher switcher, string[] args)
    {
        bool helpOrVersion = args.Length > 0 &&
            (string.Equals(args[0], "--help", StringComparison.OrdinalIgnoreCase) ||
             string.Equals(args[0], "--version", StringComparison.OrdinalIgnoreCase));

        // Empty summaries also represent parse failures. BDN's leading help/version
        // handler ignores later arguments, so validate those before allowing success.
        var (parsed, _, options) = ConfigParser.Parse(
            helpOrVersion ? args.Skip(1).ToArray() : args, ConsoleLogger.Default);
        if (!parsed)
            return 1;

        var build = Job.Default.WithCustomBuildConfiguration(
            typeof(Program).Assembly.GetCustomAttribute<AssemblyConfigurationAttribute>().Configuration);
#if USE_LOCAL_LSF_STACK
        // The generated executable otherwise adds unversioned transitive project
        // references, bypassing the explicit versions on our source-stack edges.
        build = build.WithMsBuildArguments("/p:UseLocalLsfStack=true",
            "/p:DisableTransitiveProjectReferences=true", "/p:BuildInParallel=false");
#endif
        // Mutate build settings only: a custom default job would override --job Dry.
        var config = DefaultConfig.Instance.AddJob(build.AsMutator());
        return GetExitCode(switcher.Run(args, config), helpOrVersion || options.PrintInformation ||
            options.ListBenchmarkCaseMode != ListBenchmarkCaseMode.Disabled);
    }

    private static int GetExitCode(IEnumerable<Summary> summaries, bool allowEmpty)
    {
        bool hasSummary = false;
        foreach (Summary summary in summaries)
        {
            hasSummary = true;
            // A successful early launch can leave results after a later child fails.
            // Inspect every execution, not only the aggregate result statistics.
            if (summary.HasCriticalValidationErrors || summary.Reports.Any(report =>
                    !report.Success || report.ExecuteResults.Any(execution =>
                        !execution.IsSuccess || execution.ExitCode != 0)))
                return 1;
        }

        return hasSummary || allowEmpty ? 0 : 1;
    }

    private static string[] EnsureAllBenchmarksSelected(string[] benchmarkArgs)
    {
        if (benchmarkArgs.Any(IsBenchmarkSelectionArgument))
            return benchmarkArgs;

        return new[] { "--filter", "*" }.Concat(benchmarkArgs).ToArray();
    }

    private static bool IsBenchmarkSelectionArgument(string argument)
    {
        return string.Equals(argument, "--filter", StringComparison.OrdinalIgnoreCase)
            || string.Equals(argument, "-f", StringComparison.OrdinalIgnoreCase)
            || string.Equals(argument, "--list", StringComparison.OrdinalIgnoreCase)
            || string.Equals(argument, "--help", StringComparison.OrdinalIgnoreCase)
            || string.Equals(argument, "--version", StringComparison.OrdinalIgnoreCase);
    }

    private static void WriteUsage()
    {
        Console.WriteLine("Usage:");
        Console.WriteLine("  dotnet build tests/Gravitas.Benchmarks/Gravitas.Benchmarks.csproj -c Release -f net8.0");
        Console.WriteLine("  dotnet tests/Gravitas.Benchmarks/bin/Release/net8.0/Gravitas.Benchmarks.dll all --list flat");
        Console.WriteLine();
        Console.WriteLine("Leading arguments that do not start with '-' are treated as benchmark selections.");
        Console.WriteLine("Remaining arguments are forwarded to BenchmarkDotNet.");
        Console.WriteLine();
        _catalog.WriteAvailableSelections(Console.Out);
    }
}
