using Chronicler;
using FixedMathSharp;
using FluentAssertions;
using Gravitas.Support;
using System;
using System.Collections.Generic;
using Xunit;

namespace Gravitas.Tests.Settings;

public sealed class PhysicsSettingsTests
{
    [Fact]
    public void ApplySettings_ShouldKeepFrameRateAndCollisionMatrixContextLocal()
    {
        using GravitasWorldContext contextA = GravitasWorldContext.CreateOwned();
        using GravitasWorldContext contextB = GravitasWorldContext.CreateOwned();
        var matrixA = new[,]
        {
            { true, false },
            { false, true }
        };
        var matrixB = new[,]
        {
            { false, true },
            { true, false }
        };

        contextA.ApplySettings(new PhysicsSettings(20, matrixA) { PoolingEnabled = false });
        contextB.ApplySettings(new PhysicsSettings(64, matrixB) { PoolingEnabled = true });

        contextA.Settings.FrameRate.Should().Be(20);
        contextB.Settings.FrameRate.Should().Be(64);
        contextA.FrameRate.Should().Be(20);
        contextB.FrameRate.Should().Be(64);
        contextA.DeltaTime.Should().Be(Fixed64.One / (Fixed64)20);
        contextB.DeltaTime.Should().Be(Fixed64.One / (Fixed64)64);
        contextA.Settings.CollisionMatrix.Should().BeSameAs(matrixA);
        contextB.Settings.CollisionMatrix.Should().BeSameAs(matrixB);
        contextA.Settings.PoolingEnabled.Should().BeFalse();
        contextB.Settings.PoolingEnabled.Should().BeTrue();
        contextA.Settings.DefaultContinuousCollisionMode.Should().Be(ContinuousCollisionMode.Discrete);
        contextB.Settings.DefaultContinuousCollisionMode.Should().Be(ContinuousCollisionMode.Discrete);
        contextA.Settings.ContinuousCollisionMaxToiIterations.Should().Be(PhysicsSettings.DefaultContinuousCollisionMaxToiIterations);
        contextB.Settings.ContinuousCollisionMaxToiIterations.Should().Be(PhysicsSettings.DefaultContinuousCollisionMaxToiIterations);
        contextA.Settings.DiscreteSolverIterations.Should().Be(PhysicsSettings.DefaultDiscreteSolverIterations);
        contextB.Settings.DiscreteSolverIterations.Should().Be(PhysicsSettings.DefaultDiscreteSolverIterations);
        contextA.Settings.RestitutionVelocityThreshold.Should().Be(PhysicsSettings.DefaultRestitutionVelocityThreshold);
        contextB.Settings.RestitutionVelocityThreshold.Should().Be(PhysicsSettings.DefaultRestitutionVelocityThreshold);
    }

    [Fact]
    public void SetFrameRate_ShouldUpdateSettingsAndClockTogether()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();

        context.SetFrameRate(48);

        context.Settings.FrameRate.Should().Be(48);
        context.FrameRate.Should().Be(48);
        context.DeltaTime.Should().Be(Fixed64.One / (Fixed64)48);
        context.InvDeltaTime.Should().Be(Fixed64.One / context.DeltaTime);
    }

    [Fact]
    public void SetFrameRate_ShouldKeepDeltaTimeAboveFixed64Epsilon()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();

        context.SetFrameRate(PhysicsSettings.MaxResolvableFrameRate);

        context.Settings.FrameRate.Should().Be(PhysicsSettings.MaxResolvableFrameRate);
        context.FrameRate.Should().Be(PhysicsSettings.MaxResolvableFrameRate);
        context.DeltaTime.Should().BeGreaterThan(Fixed64.Epsilon);
    }

    [Fact]
    public void SetFrameRate_ShouldRejectRatesThatQuantizeDeltaTimeAtOrBelowEpsilon()
    {
        var settings = PhysicsSettings.DefaultSettings();
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();

        Action construct = () => _ = new PhysicsSettings(PhysicsSettings.MaxResolvableFrameRate + 1, null);
        Action setSettings = () => settings.SetFrameRate(PhysicsSettings.MaxResolvableFrameRate + 1);
        Action setContext = () => context.SetFrameRate(PhysicsSettings.MaxResolvableFrameRate + 1);

        construct.Should().Throw<ArgumentOutOfRangeException>().WithParameterName("frameRate");
        setSettings.Should().Throw<ArgumentOutOfRangeException>().WithParameterName("frameRate");
        setContext.Should().Throw<ArgumentOutOfRangeException>().WithParameterName("frameRate");
    }

    [Fact]
    public void GetRegisteredCollisionMatrix_ShouldReflectRegisteredLayerNames()
    {
        var previousLayers = new List<KeyValuePair<int, string>>();
        foreach (var pair in PhysicsLayer.LayerNamesCache)
            previousLayers.Add(new KeyValuePair<int, string>(pair.Key, pair.Value));

        try
        {
            PhysicsLayer.LayerNamesCache.Clear();

            PhysicsSettings.GetRegisteredCollisionMatrix().Length.Should().Be(0);

            _ = new PhysicsLayer(1, "Players");
            _ = new PhysicsLayer(5, "World");

            bool[,] matrix = PhysicsSettings.GetRegisteredCollisionMatrix();

            matrix.GetLength(0).Should().Be(2);
            matrix.GetLength(1).Should().Be(2);
            matrix[0, 0].Should().BeTrue();
            matrix[0, 1].Should().BeTrue();
            matrix[1, 0].Should().BeTrue();
            matrix[1, 1].Should().BeTrue();
        }
        finally
        {
            PhysicsLayer.LayerNamesCache.Clear();
            for (int i = 0; i < previousLayers.Count; i++)
                PhysicsLayer.LayerNamesCache[previousLayers[i].Key] = previousLayers[i].Value;
        }
    }

    [Theory]
    [InlineData(0)]
    [InlineData(-1)]
    public void ContinuousCollisionMaxToiIterations_ShouldRejectNonPositiveValues(int value)
    {
        var settings = PhysicsSettings.DefaultSettings();

        Action action = () => settings.ContinuousCollisionMaxToiIterations = value;

        action.Should().Throw<ArgumentException>().WithParameterName("value");
    }

    [Theory]
    [InlineData(0)]
    [InlineData(-1)]
    public void DiscreteSolverIterations_ShouldRejectNonPositiveValues(int value)
    {
        var settings = PhysicsSettings.DefaultSettings();

        Action action = () => settings.DiscreteSolverIterations = value;

        action.Should().Throw<ArgumentException>().WithParameterName("value");
    }

    [Theory]
    [InlineData((byte)4)]
    [InlineData(byte.MaxValue)]
    public void DefaultContinuousCollisionMode_ShouldRejectUndefinedValues(byte rawValue)
    {
        var settings = PhysicsSettings.DefaultSettings();
        settings.DefaultContinuousCollisionMode = ContinuousCollisionMode.Auto;

        Action action = () => settings.DefaultContinuousCollisionMode = (ContinuousCollisionMode)rawValue;

        action.Should().Throw<ArgumentOutOfRangeException>().WithParameterName("value");
        settings.DefaultContinuousCollisionMode.Should().Be(ContinuousCollisionMode.Auto);
    }

    [Fact]
    public void RestitutionVelocityThreshold_ShouldStoreNonNegativeValues()
    {
        var settings = PhysicsSettings.DefaultSettings();

        settings.RestitutionVelocityThreshold.Should().Be((Fixed64)0.25f);
        settings.RestitutionVelocityThreshold = Fixed64.Zero;
        settings.RestitutionVelocityThreshold.Should().Be(Fixed64.Zero);
        settings.RestitutionVelocityThreshold = Fixed64.FromFraction(3, 2);
        settings.RestitutionVelocityThreshold.Should().Be(Fixed64.FromFraction(3, 2));
    }

    [Fact]
    public void RestitutionVelocityThreshold_ShouldRejectNegativeValues()
    {
        var settings = PhysicsSettings.DefaultSettings();

        Action action = () => settings.RestitutionVelocityThreshold = -Fixed64.Epsilon;

        action.Should().Throw<ArgumentException>().WithParameterName("value");
    }

    [Fact]
    public void ContributeReplayHash_ShouldEncodeAuthoritativeSettingsAndMatrixShape()
    {
        PhysicsSettings baseline = CreateReplayHashSettings();
        ChronicleHash baselineHash = HashSettings(baseline);

        HashSettings(CreateReplayHashSettings(frameRate: 21))
            .Should().NotBe(baselineHash);
        HashSettings(CreateReplayHashSettings(mutate: settings => settings.PoolingEnabled = !settings.PoolingEnabled))
            .Should().NotBe(baselineHash);
        HashSettings(CreateReplayHashSettings(mutate: settings => settings.GroundCheckLayerMask = PhysicsLayerMask.None))
            .Should().NotBe(baselineHash);
        HashSettings(CreateReplayHashSettings(mutate: settings => settings.RetainedPartitionTimeToKillFrames++))
            .Should().NotBe(baselineHash);
        HashSettings(CreateReplayHashSettings(mutate: settings => settings.RetainedPartitionRetirementSweepBudget++))
            .Should().NotBe(baselineHash);
        HashSettings(CreateReplayHashSettings(mutate: settings => settings.DefaultContinuousCollisionMode = ContinuousCollisionMode.Continuous))
            .Should().NotBe(baselineHash);
        HashSettings(CreateReplayHashSettings(mutate: settings => settings.ContinuousCollisionMaxToiIterations++))
            .Should().NotBe(baselineHash);
        HashSettings(CreateReplayHashSettings(mutate: settings => settings.DiscreteSolverIterations++))
            .Should().NotBe(baselineHash);
        HashSettings(CreateReplayHashSettings(mutate: settings => settings.RestitutionVelocityThreshold += Fixed64.Epsilon))
            .Should().NotBe(baselineHash);
        HashSettings(CreateReplayHashSettings(mutate: settings => settings.Mixed2DHalfThickness += Fixed64.Epsilon))
            .Should().NotBe(baselineHash);
        HashSettings(CreateReplayHashSettings(mutate: settings => settings.RuntimeMode = PhysicsRuntimeMode.Mixed))
            .Should().NotBe(baselineHash);

        bool[,] matrixValueChange =
        {
            { true, false },
            { false, false }
        };
        HashSettings(CreateReplayHashSettings(collisionMatrix: matrixValueChange)).Should().NotBe(baselineHash);
        bool[,] matrixShapeChange =
        {
            { true, false, false, true }
        };
        HashSettings(CreateReplayHashSettings(collisionMatrix: matrixShapeChange)).Should().NotBe(baselineHash);
        HashSettings(CreateReplayHashSettings(collisionMatrix: new bool[0, 0])).Should().NotBe(baselineHash);
    }

    private static PhysicsSettings CreateReplayHashSettings(
        int frameRate = 20,
        Action<PhysicsSettings>? mutate = null,
        bool[,]? collisionMatrix = null)
    {
        collisionMatrix ??= new[,]
        {
            { true, false },
            { false, true }
        };
        var settings = new PhysicsSettings(frameRate, collisionMatrix, PhysicsLayerMask.FromLayer(0))
        {
            PoolingEnabled = true,
            RetainedPartitionTimeToKillFrames = 3,
            RetainedPartitionRetirementSweepBudget = 4,
            DefaultContinuousCollisionMode = ContinuousCollisionMode.Auto,
            ContinuousCollisionMaxToiIterations = 5,
            DiscreteSolverIterations = 6,
            RestitutionVelocityThreshold = Fixed64.FromFraction(1, 3),
            Mixed2DHalfThickness = Fixed64.FromFraction(2, 3),
            RuntimeMode = PhysicsRuntimeMode.Both
        };

        mutate?.Invoke(settings);
        return settings;
    }

    private static ChronicleHash HashSettings(PhysicsSettings settings)
    {
        var writer = new ChronicleHashWriter();
        settings.ContributeReplayHash(ref writer);
        return writer.ToHash();
    }
}
