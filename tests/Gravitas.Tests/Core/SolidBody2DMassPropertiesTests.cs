using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.Core;

public sealed class SolidBody2DMassPropertiesTests
{
    [Fact]
    public void EffectiveMassHelpers_ShouldSeparateTranslationAndRotationPolicy()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D body = CreateBody(context, new LSCircleCollider2D(Fixed64.One), mass: (Fixed64)2);

        body.CanTranslate.Should().BeTrue();
        body.CanRotate.Should().BeTrue();
        body.EffectiveInverseMass.Should().Be(Fixed64.Half);
        body.MomentOfInertia.Should().Be(Fixed64.One);
        body.EffectiveInverseMomentOfInertia.Should().Be(Fixed64.One);

        body.PreventAngularForces = true;

        body.CanTranslate.Should().BeTrue();
        body.CanRotate.Should().BeFalse();
        body.EffectiveInverseMass.Should().Be(Fixed64.Half);
        body.EffectiveInverseMomentOfInertia.Should().Be(Fixed64.Zero);
    }

    [Theory]
    [InlineData(false, true)]
    [InlineData(true, false)]
    public void EffectiveMassHelpers_ForKinematicOrImmovableBody_ShouldExposeInfiniteSolverMass(
        bool immovable,
        bool isKinematic)
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D body = CreateBody(
            context,
            new LSCircleCollider2D(Fixed64.One),
            mass: (Fixed64)2,
            immovable: immovable,
            isKinematic: isKinematic);

        body.InverseMass.Should().Be(Fixed64.Half);
        body.InverseMomentOfInertia.Should().Be(Fixed64.One);
        body.CanTranslate.Should().BeFalse();
        body.CanRotate.Should().BeFalse();
        body.EffectiveInverseMass.Should().Be(Fixed64.Zero);
        body.EffectiveInverseMomentOfInertia.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void EffectiveMassHelpers_ForZeroMassBody_ShouldDisableSolverMotion()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D body = CreateBody(context, new LSCircleCollider2D(Fixed64.One), mass: Fixed64.Zero);

        body.InverseMass.Should().Be(Fixed64.Zero);
        body.MomentOfInertia.Should().Be(Fixed64.Zero);
        body.CanTranslate.Should().BeFalse();
        body.CanRotate.Should().BeFalse();
        body.EffectiveInverseMass.Should().Be(Fixed64.Zero);
        body.EffectiveInverseMomentOfInertia.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void MassSetter_ShouldRefreshScalarMomentFromColliderShape()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D body = CreateBody(context, new LSCircleCollider2D((Fixed64)2), mass: (Fixed64)2);

        body.MomentOfInertia.Should().Be((Fixed64)4);

        body.Mass = (Fixed64)4;

        body.MomentOfInertia.Should().Be((Fixed64)8);
        body.InverseMomentOfInertia.Should().Be(Fixed64.FromFraction(1, 8));
    }

    [Fact]
    public void Initialize_WithOffsetCollider_ShouldUseColliderCenterAsDefaultCenterOfMass()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var collider = new LSCircleCollider2D(Fixed64.One)
        {
            LocalOffset = new Vector2d((Fixed64)2, Fixed64.Half)
        };

        SolidBody2D body = CreateBody(context, collider, mass: Fixed64.One);

        body.LocalCenterOfMassOffset.Should().Be(collider.ScaledLocalOffset);
        body.WorldCenterOfMass.Should().Be(collider.ScaledLocalOffset);
    }

    [Fact]
    public void WorldCenterOfMass_ShouldRotateLocalOffsetAroundBodyPosition()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D body = CreateBody(context, new LSCircleCollider2D(Fixed64.One), mass: Fixed64.One);

        body.SetPosition(new Vector2d((Fixed64)3, (Fixed64)4));
        body.SetRotation(FixedMath.DegToRad((Fixed64)90));
        body.LocalCenterOfMassOffset = new Vector2d((Fixed64)2, Fixed64.Zero);

        body.WorldCenterOfMass.Should().Be(new Vector2d((Fixed64)3, (Fixed64)6));
    }

    [Fact]
    public void ResetCenterOfMassFromCollider_ShouldReturnExplicitOverrideToDerivedColliderCenter()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var collider = new LSCircleCollider2D(Fixed64.One)
        {
            LocalOffset = new Vector2d(Fixed64.Half, Fixed64.Zero)
        };
        SolidBody2D body = CreateBody(context, collider, mass: Fixed64.One);
        body.LocalCenterOfMassOffset = new Vector2d(Fixed64.Zero, (Fixed64)3);

        body.ResetCenterOfMassFromCollider();

        body.LocalCenterOfMassOffset.Should().Be(collider.ScaledLocalOffset);
    }

    private static SolidBody2D CreateBody(
        GravitasWorldContext context,
        LSCollider2D collider,
        Fixed64 mass,
        bool immovable = false,
        bool isKinematic = false,
        bool isDynamic = true)
    {
        var body = new SolidBody2D(new TestMatterAgent(context), collider)
        {
            Mass = mass,
            Immovable = immovable,
            IsKinematic = isKinematic
        };

        body.Initialize(Vector2d.Zero, Fixed64.Zero, isDynamic);
        return body;
    }
}
