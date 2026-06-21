using FixedMathSharp;
using FluentAssertions;
using Gravitas.Support;
using System.Text.Json;
using Xunit;

namespace Gravitas.Tests.Serialization;

public sealed class PhysicsSettingsSerializationTests
{
    [Fact]
    public void SettingsSaverJsonRoundTrip_ShouldPreserveRuntimeAndMixedSettings()
    {
        var source = new PhysicsSettingsSaver
        {
            FrameRate = 60,
            CollisionMatrix = new[]
            {
                new MatrixRow { row = new[] { true, false } },
                new MatrixRow { row = new[] { false, true } }
            },
            GroundCheckLayerMaskBits = PhysicsLayerMask.FromLayer(2).Bits,
            DefaultContinuousCollisionMode = ContinuousCollisionMode.Auto,
            ContinuousCollisionMaxSubsteps = 7,
            DiscreteSolverIterations = 11,
            RetainedPartitionTimeToKillFrames = 120,
            RetainedPartitionRetirementSweepBudget = 8,
            RuntimeMode = PhysicsRuntimeMode.Mixed,
            Mixed2DHalfThickness = Fixed64.FromFraction(3, 2)
        };

        string json = JsonSerializer.Serialize(source);
        PhysicsSettingsSaver? clone = JsonSerializer.Deserialize<PhysicsSettingsSaver>(json);

        clone.Should().NotBeNull();
        PhysicsSettings settings = clone!.CreateSettings();
        settings.FrameRate.Should().Be(60);
        settings.CollisionMatrix[0, 1].Should().BeFalse();
        settings.GroundCheckLayerMask.Should().Be(PhysicsLayerMask.FromLayer(2));
        settings.DefaultContinuousCollisionMode.Should().Be(ContinuousCollisionMode.Auto);
        settings.ContinuousCollisionMaxSubsteps.Should().Be(7);
        settings.DiscreteSolverIterations.Should().Be(11);
        settings.RetainedPartitionTimeToKillFrames.Should().Be(120);
        settings.RetainedPartitionRetirementSweepBudget.Should().Be(8);
        settings.RuntimeMode.Should().Be(PhysicsRuntimeMode.Mixed);
        settings.Mixed2DHalfThickness.Should().Be(Fixed64.FromFraction(3, 2));
    }
}
