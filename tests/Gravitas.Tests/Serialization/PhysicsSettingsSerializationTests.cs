using FixedMathSharp;
using FluentAssertions;
using Gravitas.Support;
#if !GRAVITAS_DISABLE_MEMORYPACK
using MemoryPack;
#endif
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
            ContinuousCollisionMaxToiIterations = 7,
            DiscreteSolverIterations = 11,
            RestitutionVelocityThreshold = Fixed64.FromFraction(7, 8),
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
        settings.ContinuousCollisionMaxToiIterations.Should().Be(7);
        settings.DiscreteSolverIterations.Should().Be(11);
        settings.RestitutionVelocityThreshold.Should().Be(Fixed64.FromFraction(7, 8));
        settings.RetainedPartitionTimeToKillFrames.Should().Be(120);
        settings.RetainedPartitionRetirementSweepBudget.Should().Be(8);
        settings.RuntimeMode.Should().Be(PhysicsRuntimeMode.Mixed);
        settings.Mixed2DHalfThickness.Should().Be(Fixed64.FromFraction(3, 2));
    }

#if !GRAVITAS_DISABLE_MEMORYPACK
    [Fact]
    public void SettingsSaverMemoryPackRoundTrip_ShouldPreserveRestitutionThreshold()
    {
        var source = new PhysicsSettingsSaver
        {
            FrameRate = 48,
            RestitutionVelocityThreshold = Fixed64.FromFraction(9, 16)
        };

        byte[] payload = MemoryPackSerializer.Serialize(source);
        PhysicsSettingsSaver? clone = MemoryPackSerializer.Deserialize<PhysicsSettingsSaver>(payload);

        clone.Should().NotBeNull();
        PhysicsSettings settings = clone!.CreateSettings();
        settings.FrameRate.Should().Be(48);
        settings.RestitutionVelocityThreshold.Should().Be(Fixed64.FromFraction(9, 16));
    }
#endif
}
