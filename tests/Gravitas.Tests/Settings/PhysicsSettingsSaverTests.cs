using FixedMathSharp;
using FluentAssertions;
using Gravitas.Support;
using System;
using Xunit;

namespace Gravitas.Tests.Settings;

public sealed class PhysicsSettingsSaverTests
{
    [Fact]
    public void ApplyTo_ShouldApplySettingsOnlyToTargetContext()
    {
        using GravitasWorldContext contextA = GravitasWorldContext.CreateOwned();
        using GravitasWorldContext contextB = GravitasWorldContext.CreateOwned();
        var saver = new PhysicsSettingsSaver
        {
            FrameRate = 20,
            CollisionMatrix = new[]
            {
                new MatrixRow { row = new[] { true, false } },
                new MatrixRow { row = new[] { false, true } }
            },
            GroundCheckLayerMaskBits = PhysicsLayerMask.FromLayer(1).Bits,
            DefaultContinuousCollisionMode = ContinuousCollisionMode.Auto,
            ContinuousCollisionMaxToiIterations = 6,
            DiscreteSolverIterations = 9,
            RestitutionVelocityThreshold = Fixed64.FromFraction(5, 8),
            RetainedPartitionTimeToKillFrames = 12,
            RetainedPartitionRetirementSweepBudget = 3
        };

        saver.ApplyTo(contextA);

        contextA.Settings.FrameRate.Should().Be(20);
        contextA.FrameRate.Should().Be(20);
        contextA.DeltaTime.Should().Be(Fixed64.One / (Fixed64)20);
        contextA.Settings.CollisionMatrix[0, 0].Should().BeTrue();
        contextA.Settings.CollisionMatrix[0, 1].Should().BeFalse();
        contextA.Settings.CollisionMatrix[1, 0].Should().BeFalse();
        contextA.Settings.CollisionMatrix[1, 1].Should().BeTrue();
        contextA.Settings.GroundCheckLayerMask.Should().Be(PhysicsLayerMask.FromLayer(1));
        contextA.Settings.DefaultContinuousCollisionMode.Should().Be(ContinuousCollisionMode.Auto);
        contextA.Settings.ContinuousCollisionMaxToiIterations.Should().Be(6);
        contextA.Settings.DiscreteSolverIterations.Should().Be(9);
        contextA.Settings.RestitutionVelocityThreshold.Should().Be(Fixed64.FromFraction(5, 8));
        contextA.Settings.RetainedPartitionTimeToKillFrames.Should().Be(12);
        contextA.Settings.RetainedPartitionRetirementSweepBudget.Should().Be(3);

        contextB.Settings.FrameRate.Should().Be(PhysicsSettings.DefaultFrameRate);
        contextB.FrameRate.Should().Be(PhysicsSettings.DefaultFrameRate);
        contextB.DeltaTime.Should().Be(Fixed64.One / (Fixed64)PhysicsSettings.DefaultFrameRate);
        contextB.Settings.DefaultContinuousCollisionMode.Should().Be(ContinuousCollisionMode.Discrete);
        contextB.Settings.ContinuousCollisionMaxToiIterations.Should().Be(PhysicsSettings.DefaultContinuousCollisionMaxToiIterations);
        contextB.Settings.DiscreteSolverIterations.Should().Be(PhysicsSettings.DefaultDiscreteSolverIterations);
        contextB.Settings.RestitutionVelocityThreshold.Should().Be(PhysicsSettings.DefaultRestitutionVelocityThreshold);
        contextB.Settings.RetainedPartitionTimeToKillFrames.Should().Be(PhysicsSettings.DefaultRetainedPartitionTimeToKillFrames);
        contextB.Settings.RetainedPartitionRetirementSweepBudget.Should().Be(PhysicsSettings.DefaultRetainedPartitionRetirementSweepBudget);
    }

    [Fact]
    public void EarlyApply_WithoutBoundContext_ShouldReject()
    {
        var saver = new PhysicsSettingsSaver();

        Action act = saver.EarlyApply;

        act.Should().Throw<InvalidOperationException>()
            .WithMessage("*explicit GravitasWorldContext*");
    }

    [Fact]
    public void EarlyApply_WithBoundContext_ShouldApplySettings()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var saver = new PhysicsSettingsSaver
        {
            FrameRate = 20,
            RuntimeMode = PhysicsRuntimeMode.TwoD,
            Mixed2DHalfThickness = Fixed64.FromFraction(1, 8)
        };
        saver.BindContext(context);

        saver.EarlyApply();

        context.Settings.FrameRate.Should().Be(20);
        context.FrameRate.Should().Be(20);
        context.Settings.RuntimeMode.Should().Be(PhysicsRuntimeMode.TwoD);
        context.Settings.Mixed2DHalfThickness.Should().Be(Fixed64.FromFraction(1, 8));
    }

    [Fact]
    public void CreateSettings_WithNonSquareCollisionMatrix_ShouldReject()
    {
        var saver = new PhysicsSettingsSaver
        {
            CollisionMatrix = new[]
            {
                new MatrixRow { row = new[] { true, false } },
                new MatrixRow { row = new[] { true } }
            }
        };

        Action act = () => _ = saver.CreateSettings();

        act.Should().Throw<InvalidOperationException>()
            .WithMessage("*square*");
    }

    [Fact]
    public void CreateSettings_WithOverlongCollisionMatrixRow_ShouldReject()
    {
        var saver = new PhysicsSettingsSaver
        {
            CollisionMatrix = new[]
            {
                new MatrixRow { row = new[] { true, false } },
                new MatrixRow { row = new[] { false, true, false } }
            }
        };

        Action act = () => _ = saver.CreateSettings();

        act.Should().Throw<InvalidOperationException>()
            .WithMessage("*square*");
    }

    [Fact]
    public void CreateSettings_WithMissingCollisionMatrixRow_ShouldRejectDeterministically()
    {
        var saver = new PhysicsSettingsSaver
        {
            CollisionMatrix = new[] { default(MatrixRow) }
        };

        Action act = () => _ = saver.CreateSettings();

        act.Should().Throw<InvalidOperationException>()
            .WithMessage("*square*");
    }

    [Fact]
    public void CreateSettings_WithNoCollisionMatrix_ShouldUseDefaultLayerRules()
    {
        var saver = new PhysicsSettingsSaver();
        bool[,] expected = PhysicsSettings.GetRegisteredCollisionMatrix();

        PhysicsSettings settings = saver.CreateSettings();

        settings.CollisionMatrix.GetLength(0).Should().Be(expected.GetLength(0));
        settings.CollisionMatrix.GetLength(1).Should().Be(expected.GetLength(1));
        for (int y = 0; y < expected.GetLength(1); y++)
        {
            for (int x = 0; x < expected.GetLength(0); x++)
                settings.CollisionMatrix[x, y].Should().Be(expected[x, y]);
        }
    }
}
