using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed class MixedEmbeddingContractTests
{
    [Fact]
    public void Collider2D_WithContextDefaultMixedThickness_ShouldBuildDeterministic3DBounds()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.Mixed2DHalfThickness = (Fixed64)2;
        var transform = new FixedTransform(
            new Vector3d((Fixed64)3, (Fixed64)7, (Fixed64)5),
            FixedQuaternion.Identity,
            Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var collider = new LSCircleCollider2D(Fixed64.One);
        var body = new SolidBody2D(agent, collider)
        {
            Mass = Fixed64.One
        };

        body.Initialize(transform.WorldPositionXZ);

        collider.MixedHalfThickness.Should().Be((Fixed64)2);
        collider.MixedSlabCenterY.Should().Be((Fixed64)7);
        collider.MixedBounds3D.Min.Should().Be(new Vector3d((Fixed64)2, (Fixed64)5, (Fixed64)4));
        collider.MixedBounds3D.Max.Should().Be(new Vector3d((Fixed64)4, (Fixed64)9, (Fixed64)6));
    }

    [Fact]
    public void Collider2D_WithMixedThicknessOverride_ShouldUseOverrideOverContextDefault()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.Mixed2DHalfThickness = (Fixed64)2;
        var transform = new FixedTransform(
            new Vector3d((Fixed64)3, (Fixed64)7, (Fixed64)5),
            FixedQuaternion.Identity,
            Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var collider = new LSAABBoxCollider2D(new Vector2d((Fixed64)4, (Fixed64)6))
        {
            MixedHalfThicknessOverride = (Fixed64)3
        };

        collider.InitializeWithNoBody(agent);

        collider.MixedHalfThickness.Should().Be((Fixed64)3);
        collider.MixedSlabCenterY.Should().Be((Fixed64)7);
        collider.MixedBounds3D.Min.Should().Be(new Vector3d((Fixed64)1, (Fixed64)4, (Fixed64)2));
        collider.MixedBounds3D.Max.Should().Be(new Vector3d((Fixed64)5, (Fixed64)10, (Fixed64)8));
    }

    [Fact]
    public void Collider2D_RebuildAfterHostYMove_ShouldRefreshMixedSlabCenterAnd3DBounds()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.Mixed2DHalfThickness = Fixed64.Half;
        var transform = new FixedTransform(
            new Vector3d((Fixed64)4, Fixed64.One, (Fixed64)6),
            FixedQuaternion.Identity,
            Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var collider = new LSCircleCollider2D(Fixed64.One);
        collider.InitializeWithNoBody(agent);

        transform.LocalPosition = new Vector3d((Fixed64)4, (Fixed64)9, (Fixed64)6);

        collider.Rebuild();
        collider.MixedSlabCenterY.Should().Be((Fixed64)9);
        collider.MixedBounds3D.Min.Y.Should().Be((Fixed64)9 - Fixed64.Half);
        collider.MixedBounds3D.Max.Y.Should().Be((Fixed64)9 + Fixed64.Half);
    }

    [Fact]
    public void MixedHalfThicknessOverrideMutation_ShouldDirtyOnlyWhenEffectiveOverrideChanges()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.Mixed2DHalfThickness = (Fixed64)2;
        var transform = new FixedTransform(
            new Vector3d((Fixed64)3, (Fixed64)7, (Fixed64)5),
            FixedQuaternion.Identity,
            Vector3d.One);
        var collider = new LSCircleCollider2D(Fixed64.One);
        collider.InitializeWithNoBody(new TestMatterAgent(context, transform));
        uint initialRuntimeVersion = collider.RuntimeShapeVersion;
        uint initialBroadPhaseVersion = collider.BroadPhaseVersion;

        collider.MixedHalfThicknessOverride = null;
        collider.Simulate();

        collider.RuntimeShapeVersion.Should().Be(initialRuntimeVersion);
        collider.BroadPhaseVersion.Should().Be(initialBroadPhaseVersion);

        collider.MixedHalfThicknessOverride = (Fixed64)3;
        collider.Simulate();
        uint overrideRuntimeVersion = collider.RuntimeShapeVersion;
        uint overrideBroadPhaseVersion = collider.BroadPhaseVersion;

        overrideRuntimeVersion.Should().Be(initialRuntimeVersion + 1);
        overrideBroadPhaseVersion.Should().Be(initialBroadPhaseVersion);
        collider.MixedHalfThickness.Should().Be((Fixed64)3);
        collider.MixedBounds3D.Min.Y.Should().Be((Fixed64)4);
        collider.MixedBounds3D.Max.Y.Should().Be((Fixed64)10);

        collider.MixedHalfThicknessOverride = (Fixed64)3;
        collider.Simulate();

        collider.RuntimeShapeVersion.Should().Be(overrideRuntimeVersion);
        collider.BroadPhaseVersion.Should().Be(overrideBroadPhaseVersion);

        collider.MixedHalfThicknessOverride = null;
        collider.Simulate();

        collider.MixedHalfThickness.Should().Be((Fixed64)2);
        collider.RuntimeShapeVersion.Should().Be(overrideRuntimeVersion + 1);
        collider.BroadPhaseVersion.Should().Be(overrideBroadPhaseVersion);
        collider.MixedBounds3D.Min.Y.Should().Be((Fixed64)5);
        collider.MixedBounds3D.Max.Y.Should().Be((Fixed64)9);
    }

    [Fact]
    public void MixedHalfThicknessSettingsAndOverride_ShouldRejectNonPositiveValues()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var collider = new LSCircleCollider2D(Fixed64.One);

        Action zeroDefault = () => context.Settings.Mixed2DHalfThickness = Fixed64.Zero;
        Action negativeDefault = () => context.Settings.Mixed2DHalfThickness = -Fixed64.One;
        Action zeroOverride = () => collider.MixedHalfThicknessOverride = Fixed64.Zero;
        Action negativeOverride = () => collider.MixedHalfThicknessOverride = -Fixed64.One;

        zeroDefault.Should().Throw<ArgumentException>().WithParameterName("value");
        negativeDefault.Should().Throw<ArgumentException>().WithParameterName("value");
        zeroOverride.Should().Throw<ArgumentException>().WithParameterName("value");
        negativeOverride.Should().Throw<ArgumentException>().WithParameterName("value");
    }
}
