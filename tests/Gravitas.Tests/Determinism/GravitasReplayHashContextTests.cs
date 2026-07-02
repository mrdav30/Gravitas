using Chronicler;
using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Support;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.Determinism;

public sealed class GravitasReplayHashContextTests
{
    [Fact]
    public void ComputeReplayHash_ShouldMatchForRepeatedEquivalent3DRuns()
    {
        ChronicleHash[] first = Run3DTrace();
        ChronicleHash[] second = Run3DTrace();

        second.Should().Equal(first);
    }

    [Fact]
    public void ComputeReplayHash_ShouldChangeWhenAuthoritativeBodyStateChanges()
    {
        using PhysicsScenarioBuilder scenario = Create3DScenario();
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);

        ChronicleHash before = scenario.Context.ComputeReplayHash();

        body.Body.AddForce(Vector3d.Right);
        scenario.Context.LateSimulate();

        scenario.Context.ComputeReplayHash().Should().NotBe(before);
    }

    [Fact]
    public void ComputeReplayHash_ShouldIgnoreQueryCacheMutationInAuthoritativeMode()
    {
        using PhysicsScenarioBuilder first = Create3DScenario();
        first.CreateSphere(Vector3d.Zero);
        ChronicleHash beforeQuery = first.Context.ComputeReplayHash();

        using PhysicsScenarioBuilder second = Create3DScenario();
        second.CreateSphere(Vector3d.Zero);
        second.Context.Query3D.Raycast(
            Vector3d.Left,
            Vector3d.Right,
            (Fixed64)8,
            out _,
            PhysicsLayerMask.All);

        second.Context.ComputeReplayHash(GravitasReplayHashMode.Authoritative).Should().Be(beforeQuery);
    }

    private static ChronicleHash[] Run3DTrace()
    {
        using PhysicsScenarioBuilder scenario = Create3DScenario();
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);
        body.Body.AddForce(new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero));

        var hashes = new ChronicleHash[8];
        for (int frame = 0; frame < hashes.Length; frame++)
        {
            scenario.Context.LateSimulate();
            hashes[frame] = scenario.Context.ComputeReplayHash();
        }

        return hashes;
    }

    private static PhysicsScenarioBuilder Create3DScenario()
    {
        PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.SetFrameRate(8);
        scenario.Context.Settings.RuntimeMode = PhysicsRuntimeMode.ThreeD;
        return scenario;
    }
}
