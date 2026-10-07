using Chronicler.Hashing;
using FixedMathSharp;
using FixedMathSharp.Chronicler;
using GridForge.Grids;
using SwiftCollections;
using SwiftCollections.Query;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Security.Cryptography;
using System.Text.Json;

namespace Gravitas.Tests.Determinism;

internal static class ReplayCapture
{
    public static void Write(string directory, ReplayFixture fixture, ReplayFrame[] frames)
    {
        Directory.CreateDirectory(directory);
        File.WriteAllText(Path.Combine(directory, fixture.Name + ".actual.json"), ReplayFixture.Serialize(frames));

        // Read the actual executing process and loaded dependency files. Runner
        // labels alone cannot distinguish native ARM64 from x64 emulation.
        Assembly[] dependencies =
        {
            typeof(GravitasWorldContext).Assembly, typeof(Fixed64).Assembly,
            typeof(FixedChronicleHashWriterExtensions).Assembly, typeof(SwiftList<int>).Assembly,
            typeof(FixedBoundVolume).Assembly, typeof(GridWorld).Assembly, typeof(ChronicleHashWriter).Assembly
        };
        string fixtureDirectory = Path.Combine(AppContext.BaseDirectory, "Determinism", "Fixtures");
        var fixtures = Directory.GetFiles(fixtureDirectory, "*.json").OrderBy(path => path, StringComparer.Ordinal)
            .ToDictionary(path => Path.GetFileNameWithoutExtension(path)!, path =>
            {
                using JsonDocument document = JsonDocument.Parse(File.ReadAllText(path));
                return new { Version = document.RootElement.GetProperty("Version").GetInt32(), Sha256 = HashFile(path) };
            });
        var metadata = new
        {
            Version = 1,
            OS = OperatingSystem.IsWindows() ? "Windows" : OperatingSystem.IsLinux() ? "Linux" : "Other",
            RuntimeInformation.OSDescription,
            OSArchitecture = RuntimeInformation.OSArchitecture.ToString(),
            ProcessArchitecture = RuntimeInformation.ProcessArchitecture.ToString(),
            Configuration = typeof(ReplayCapture).Assembly.GetCustomAttribute<AssemblyConfigurationAttribute>()!.Configuration,
            Framework = RuntimeInformation.FrameworkDescription,
            RuntimeVersion = Environment.Version.ToString(),
            // Missing provenance remains explicit for ad-hoc diagnostic capture;
            // the conformance comparator rejects it instead of inferring a pass.
            SDKVersion = Environment.GetEnvironmentVariable("GRAVITAS_REPLAY_SDK_VERSION") ?? "unrecorded",
            SourceRevision = Environment.GetEnvironmentVariable("GRAVITAS_REPLAY_SOURCE_REVISION") ?? "unrecorded",
            UseLocalLsfStack = string.Equals(Environment.GetEnvironmentVariable("UseLocalLsfStack"), "true", StringComparison.OrdinalIgnoreCase),
            SourceDependencies = JsonSerializer.Deserialize<Dictionary<string, string>>(
                Environment.GetEnvironmentVariable("GRAVITAS_REPLAY_DEPENDENCY_REVISIONS") ?? "{}"),
            Dependencies = dependencies.OrderBy(assembly => assembly.GetName().Name, StringComparer.Ordinal).Select(assembly => new
            {
                Name = assembly.GetName().Name!,
                Version = assembly.GetName().Version!.ToString(),
                InformationalVersion = assembly.GetCustomAttribute<AssemblyInformationalVersionAttribute>()!.InformationalVersion,
                Sha256 = HashFile(assembly.Location)
            }).ToArray(),
            Fixtures = fixtures
        };
        File.WriteAllText(Path.Combine(directory, "provenance.json"), ReplayFixture.Serialize(metadata));
    }

    private static string HashFile(string path) => Convert.ToHexString(SHA256.HashData(File.ReadAllBytes(path))).ToLowerInvariant();
}
