using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using System;
using System.Collections.Generic;
using Xunit;

namespace Gravitas.Tests.Settings;

public sealed class SettingsGeometryRangeTests
{
    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void GeometrySettings_ShouldChangeOnlyBetweenFixedSteps(int route)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Simulate();
        // Assigning an unchanged value performs no transaction.
        context.Settings.RuntimeMode = context.Settings.RuntimeMode;
        context.Settings.Mixed2DHalfThickness = context.Settings.Mixed2DHalfThickness;
        Action change = route switch
        {
            0 => () => context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed,
            1 => () => context.Settings.Mixed2DHalfThickness = Fixed64.One,
            _ => () => context.ApplySettings(PhysicsSettings.DefaultSettings())
        };
        change.Should().Throw<InvalidOperationException>();
        context.LateSimulate();
        change.Should().NotThrow();
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void MixedModePreflight_ShouldIncludeCompleteBodyAndCompoundAttachment(bool compound)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        LSCollider2D collider = compound
            ? new LSCompoundCollider2D(CompoundColliderPart2D.Circle(Fixed64.One, new Vector2d(4095, 0)))
            : new LSCircleCollider2D(Fixed64.One) { LocalOffset = new Vector2d(4095, 0) };
        var body = new SolidBody2D(new TestMatterAgent(context), collider) { Mass = (Fixed64)2 };
        body.Initialize(Vector2d.Zero);
        body.Sleep();
        var hash = context.ComputeReplayHash();

        Action change = () => context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;

        change.Should().Throw<ArgumentException>();
        body.IsSleeping.Should().BeTrue();
        context.ComputeReplayHash().Should().Be(hash);
    }

    [Fact]
    public void UnusedDefaultThickness_ShouldNotRestrictPure2DOrAnOverriddenMixedCollider()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        context.Settings.Mixed2DHalfThickness = Fixed64.MaxValue;
        var circle = new LSCircleCollider2D(Fixed64.One) { MixedHalfThicknessOverride = Fixed64.Half };
        circle.InitializeWithNoBody(new TestMatterAgent(context));
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        context.Settings.Mixed2DHalfThickness = (Fixed64)100000;
        circle.MixedHalfThicknessOverride.Should().Be(Fixed64.Half);
    }

    [Fact]
    public void RuntimeModeChange_ShouldPreflightExistingPlanarGeometry()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        var circle = new LSCircleCollider2D((Fixed64)4096);
        circle.InitializeWithNoBody(new TestMatterAgent(context));
        uint version = circle.RuntimeShapeVersion;
        var bounds = circle.Bounds;

        Action change = () => context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;

        change.Should().Throw<ArgumentException>().WithMessage("*4,096*");
        context.Settings.RuntimeMode.Should().Be(PhysicsRuntimeMode.TwoD);
        circle.RuntimeShapeVersion.Should().Be(version);
        circle.Bounds.Should().Be(bounds);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void MixedThicknessChange_ShouldRejectBeforeSettingsOrClockPublication(int route)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        var circle = new LSCircleCollider2D((Fixed64)4000);
        circle.InitializeWithNoBody(new TestMatterAgent(context));
        PhysicsSettings original = context.Settings;
        int rate = context.FrameRate;
        uint version = circle.RuntimeShapeVersion;
        Action change = route switch
        {
            0 => () => context.Settings.Mixed2DHalfThickness = (Fixed64)1000,
            1 => () => context.ApplySettings(new PhysicsSettings(60, null)
                { RuntimeMode = PhysicsRuntimeMode.Mixed, Mixed2DHalfThickness = (Fixed64)1000 }),
            _ => () => new PhysicsSettingsSaver
                { FrameRate = 60, RuntimeMode = PhysicsRuntimeMode.Mixed, Mixed2DHalfThickness = (Fixed64)1000 }
                .ApplyTo(context)
        };

        change.Should().Throw<ArgumentException>().WithMessage("*4,096*");
        context.Settings.Should().BeSameAs(original);
        context.FrameRate.Should().Be(rate);
        context.Settings.Mixed2DHalfThickness.Should().Be(PhysicsSettings.DefaultMixed2DHalfThickness);
        circle.RuntimeShapeVersion.Should().Be(version);
    }

    [Fact]
    public void SettingsInstance_ShouldHaveOneContextOwnerAndReleaseItOnReplacementOrDisposal()
    {
        using GravitasWorldContext first = GravitasWorldContext.CreateOwned();
        using GravitasWorldContext second = GravitasWorldContext.CreateOwned();
        PhysicsSettings settings = first.Settings;
        Action share = () => second.ApplySettings(settings);
        share.Should().Throw<InvalidOperationException>();
        first.ApplySettings(settings);
        first.ApplySettings(PhysicsSettings.DefaultSettings());
        second.ApplySettings(settings);
        second.Dispose();
        first.ApplySettings(settings);
        first.Settings.Should().BeSameAs(settings);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(-1)]
    public void LoadedMixedOverride_ShouldRejectNonpositiveThickness(int value)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        var circle = new LSCircleCollider2D(Fixed64.One);
        circle.InitializeWithNoBody(new TestMatterAgent(context));
        uint version = circle.RuntimeShapeVersion;
        var bounds = circle.Bounds;
        var record = new InvalidRecordPayloadChronicler(new Dictionary<string, object>
        {
            ["MixedHalfThicknessOverride"] = (Fixed64)value
        });

        Action load = () => circle.RecordData(record);

        load.Should().Throw<ArgumentException>();
        circle.RuntimeShapeVersion.Should().Be(version);
        circle.Bounds.Should().Be(bounds);
    }
}
