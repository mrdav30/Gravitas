using FixedMathSharp;
using FixedMathSharp.Geometry;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class ConeGeometryBoundaryTests
{
    [Theory]
    [InlineData(4294967296L, 4294967296L, 4497679235L)]
    [InlineData(2199023255552L, 2199023255552L, 603668288149738837L)]
    [InlineData(4398046511104L, 4398046511104L, 4829346305197910699L)]
    [InlineData(8796093022208L, 8796093022208L, long.MaxValue)]
    [InlineData(8589934592000L, 2621440000L, 10980662194010417L)]
    [InlineData(2147483648L, 2147483648L, 562209904L)]
    [InlineData(4294967297L, 4294967299L, 4497679240L)]
    [InlineData(1073741824L, 12884901888L, 843314856L)]
    [InlineData(1073741824L, 38654705664L, 2529944570L)]
    [InlineData(16384L, 17592186044416L, 268L)]
    [InlineData(1L, 1L, 0L)]
    public void ConeVolume_ShouldRoundOnlyTheFinalProductRatio(
        long radiusRaw, long heightRaw, long expectedRaw)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var cone = new LSConeCollider(ColliderShapeDefinition.Cone(
            Fixed64.FromRaw(radiusRaw), Fixed64.FromRaw(heightRaw)));

        cone.InitializeWithNoBody(new TestMatterAgent(context));

        // Independent integer oracle: round-even(PiRaw*rRaw*rRaw*hRaw/(3*2^96)),
        // saturating only the final positive result to long.MaxValue.
        cone.Volume.Should().Be(Fixed64.FromRaw(expectedRaw));
        cone.Area.Should().Be(cone.Volume);
        cone.ScaledRadius.Should().Be(Fixed64.FromRaw(radiusRaw));
        cone.Height.Should().Be(Fixed64.FromRaw(heightRaw));
    }

    [Fact]
    public void ConeVolume_ShouldFollowEffectiveDimensionsAndPreserveFailedPreparation()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var transform = new FixedTransform(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One * Fixed64.Half);
        var cone = new LSConeCollider(ColliderShapeDefinition.Cone((Fixed64)2048, (Fixed64)2048));
        cone.InitializeWithNoBody(new TestMatterAgent(context, transform));
        Fixed64 expected = Fixed64.FromRaw(4829346305197910699L);
        cone.Volume.Should().Be(expected);
        cone.WorldAxis.Should().Be(Vector3d.Up);

        cone.Position = Vector3d.Right;
        cone.Rotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)19, (Fixed64)31, (Fixed64)7);
        cone.Volume.Should().Be(expected);
        Vector3d rotatedAxis = (cone.CanonicalRotation * Vector3d.Up).Normalized;
        cone.WorldAxis.Should().Be(rotatedAxis);
        rotatedAxis.Should().NotBe(Vector3d.Up);

        transform.LocalScale = Vector3d.One;
        cone.Simulate();
        cone.Volume.Should().Be(Fixed64.MaxValue);
        cone.WorldAxis.Should().Be((cone.CanonicalRotation * Vector3d.Up).Normalized);
        transform.LocalScale = Vector3d.One * Fixed64.Half;
        cone.Simulate();
        cone.Volume.Should().Be(expected);
        cone.WorldAxis.Should().Be((cone.CanonicalRotation * Vector3d.Up).Normalized);

        transform.LocalScale = Vector3d.Zero;
        Action invalidPrepare = () => cone.Simulate();
        invalidPrepare.Should().Throw<ArgumentException>();
        cone.Volume.Should().Be(expected);
        cone.Area.Should().Be(expected);

        transform.LocalScale = Vector3d.One * Fixed64.Half;
        cone.Radius = (Fixed64)1024;
        Vector3d committedAxis = cone.WorldAxis;
        cone.Simulate();
        cone.Volume.Should().Be(Fixed64.FromRaw(1207336576299477675L));
        cone.WorldAxis.Should().Be(committedAxis);
        cone.Size = new Vector3d((Fixed64)2048, (Fixed64)1024, (Fixed64)2048);
        cone.Simulate();
        cone.Volume.Should().Be(Fixed64.FromRaw(603668288149738837L));

        // Fail after the candidate volume has been calculated, before publication.
        cone.Rotation = FixedQuaternion.Identity;
        cone.WorldAxis.Should().Be(Vector3d.Up);
        transform.LocalScale = Vector3d.One;
        cone.LocalOffset = new Vector3d(Fixed64.Zero, Fixed64.MinValue, Fixed64.Zero);
        invalidPrepare.Should().Throw<InvalidOperationException>()
            .WithMessage("*Prepared collider mass-property point*");
        cone.Volume.Should().Be(Fixed64.FromRaw(603668288149738837L));
        cone.Area.Should().Be(cone.Volume);
        cone.LocalOffset = Vector3d.Zero;
        cone.Simulate();
        cone.Volume.Should().Be(Fixed64.FromRaw(4829346305197910699L));
    }

    [Fact]
    public void BodyBoundConeVolume_ShouldRejectUnsupportedAttachmentBeforePublication()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var cone = new LSConeCollider(ColliderShapeDefinition.Cone(Fixed64.One, (Fixed64)4));
        var body = new SolidBody(new TestMatterAgent(context), cone);
        body.Initialize(Vector3d.Zero, FixedQuaternion.Identity);
        Fixed64 volume = cone.Volume;
        FixedBoundBox bounds = cone.Bounds;

        cone.Radius = Fixed64.Two;
        Action invalidOffset = () => cone.LocalOffset = new Vector3d(Fixed64.Zero, Fixed64.MinValue, Fixed64.Zero);

        invalidOffset.Should().Throw<ArgumentException>();
        cone.LocalOffset.Should().Be(Vector3d.Zero);
        cone.Volume.Should().Be(volume);
        cone.Area.Should().Be(volume);
        cone.ScaledRadius.Should().Be(Fixed64.One);
        cone.Bounds.Should().Be(bounds);

        cone.LocalOffset = Vector3d.Zero;
        cone.Simulate();
        cone.Volume.Should().Be(Fixed64.FromRaw(71962867755L));
        cone.ScaledRadius.Should().Be(Fixed64.Two);
    }

    [Fact]
    public void CompoundConeVolume_ShouldUseOwnerAndPartScaleWithoutNarrowingMassWeights()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var compound = new LSCompoundCollider(
            CompoundColliderPart.Cone(
                (Fixed64)2048, (Fixed64)2048, Vector3d.Zero,
                FixedQuaternion.Identity, Vector3d.One * Fixed64.Half));
        var transform = new FixedTransform(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        compound.InitializeWithNoBody(new TestMatterAgent(context, transform));
        var cone = (LSConeCollider)compound.GetPartCollider(0);

        cone.Volume.Should().Be(Fixed64.FromRaw(4829346305197910699L));
        compound.Area.Should().Be(cone.Volume);
        transform.LocalScale = Vector3d.One * Fixed64.Two;
        compound.Simulate();
        cone.Volume.Should().Be(Fixed64.MaxValue);
        compound.Area.Should().Be(Fixed64.MaxValue);
        ExactMassWeight weight = cone.CalculateMassPropertyWeight();
        weight.TryGetMeasure(out _).Should().BeFalse();
        weight.TryGetProportionalShare(Fixed64.One, weight.Add(weight), out Fixed64 share)
            .Should().BeTrue();
        share.Should().Be(Fixed64.Half);
    }

    [Fact]
    public void DetachedCone_ShouldDeriveCenterOfMassFromAuthoredHeight()
    {
        var cone = new LSConeCollider
        {
            Radius = Fixed64.One,
            Size = new Vector3d(
                Fixed64.Two,
                (Fixed64)4,
                Fixed64.Two)
        };

        cone.CalculateLocalCenterOfMassOffset()
            .Should().Be(-Vector3d.Up);
        cone.CalculateMassPropertyWeight()
            .TryGetMeasure(out Fixed64 volume)
            .Should().BeTrue();
        FixedMath.Abs(
                volume
                - Fixed64.FromFraction(4, 3) * Fixed64.Pi)
            .Should().BeLessThanOrEqualTo(
                Fixed64.MinIncrement * Fixed64.Two);
    }

    [Fact]
    public void StandaloneCone_ShouldRejectAnUnrepresentableAuthoredCenterOfMass()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var cone = new LSConeCollider
        {
            Radius = Fixed64.One,
            Size = new Vector3d(Fixed64.Two, (Fixed64)4, Fixed64.Two),
            LocalOffset = new Vector3d(
                Fixed64.Zero,
                Fixed64.MinValue,
                Fixed64.Zero)
        };
        var transform = new FixedTransform(
            new Vector3d(Fixed64.Zero, Fixed64.MaxValue, Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.One);

        Action initialize = () =>
            cone.InitializeWithNoBody(new TestMatterAgent(context, transform));

        initialize.Should().Throw<InvalidOperationException>()
            .WithMessage("*Prepared collider mass-property point*");
    }

    [Fact]
    public void CompoundCone_ShouldRejectAnUnrepresentableAuthoredCenterOfMass()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var compound = new LSCompoundCollider(
            CompoundColliderPart.Cone(
                Fixed64.One,
                (Fixed64)4,
                new Vector3d(
                    Fixed64.Zero,
                    Fixed64.MinValue,
                    Fixed64.Zero)))
        {
            LocalOffset = Vector3d.Up * Fixed64.Half
        };

        Action initialize = () =>
            compound.InitializeWithNoBody(new TestMatterAgent(context));

        initialize.Should().Throw<InvalidOperationException>()
            .WithMessage("*Prepared compound mass-property point*");
    }

    [Fact]
    public void ContainsWorldPoint_AxialToleranceShouldApplyAtBothCaps()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSConeCollider cone = scenario.CreateBody(
            new LSConeCollider(ColliderShapeDefinition.Cone(Fixed64.One, Fixed64.Two)),
            Vector3d.Zero,
            FixedQuaternion.Identity).Collider;
        Fixed64 outsideOffset = Fixed64.Epsilon + Fixed64.FromRaw(1L);

        cone.ContainsWorldPoint(new Vector3d(Fixed64.Zero, -Fixed64.One - Fixed64.Epsilon, Fixed64.Zero), Fixed64.Epsilon)
            .Should().BeTrue();
        cone.ContainsWorldPoint(new Vector3d(Fixed64.Zero, -Fixed64.One - outsideOffset, Fixed64.Zero), Fixed64.Epsilon)
            .Should().BeFalse();
        cone.ContainsWorldPoint(new Vector3d(Fixed64.Zero, Fixed64.One + Fixed64.Epsilon, Fixed64.Zero), Fixed64.Epsilon)
            .Should().BeTrue();
        cone.ContainsWorldPoint(new Vector3d(Fixed64.Zero, Fixed64.One + outsideOffset, Fixed64.Zero), Fixed64.Epsilon)
            .Should().BeFalse();
        cone.ContainsWorldPoint(
                new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero),
                Fixed64.Epsilon)
            .Should().BeFalse();
    }

    [Fact]
    public void ContainsWorldPoint_AcrossTheScalarDomain_ShouldDeclineAnUnmaterializableWitness()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSConeCollider cone = scenario.CreateBody(
            new LSConeCollider(
                ColliderShapeDefinition.Cone(
                    Fixed64.One,
                    Fixed64.Two)),
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity).Collider;

        cone.ContainsWorldPoint(
                new Vector3d(
                    Fixed64.MinValue,
                    Fixed64.Zero,
                    Fixed64.Zero))
            .Should()
            .BeFalse();
    }

    [Fact]
    public void CreateFiniteConeBounds_DegenerateAxis_ShouldUseStableUpFallback()
    {
        ConeGeometry.CreateFiniteConeBounds(
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Zero,
            Fixed64.Two,
            out Vector3d min,
            out Vector3d max);

        min.Should().Be(new Vector3d(-Fixed64.Two, Fixed64.Zero, -Fixed64.Two));
        max.Should().Be(new Vector3d(Fixed64.Two, Fixed64.Zero, Fixed64.Two));
    }

    [Fact]
    public void CreateFiniteConeBounds_NearUnitAxis_ShouldContainExactBaseDiskSupport()
    {
        var axis = new Vector3d(
            Fixed64.FromFraction(3, 5),
            Fixed64.FromFraction(4, 5) + Fixed64.MinIncrement,
            Fixed64.Zero);
        var baseCenter = new Vector3d(Fixed64.One, Fixed64.Two, (Fixed64)3);

        ConeGeometry.CreateFiniteConeBounds(
            Vector3d.Zero,
            baseCenter,
            axis,
            Fixed64.One,
            out Vector3d min,
            out Vector3d max);

        Vector3d support = baseCenter
            + Vector3d.GetNormalizedProjectionOnPlane(Vector3d.Right, axis);
        min.X.Should().BeLessThanOrEqualTo(support.X);
        max.X.Should().BeGreaterThanOrEqualTo(support.X);
        min.Y.Should().BeLessThanOrEqualTo(support.Y);
        max.Y.Should().BeGreaterThanOrEqualTo(support.Y);
        min.Z.Should().BeLessThanOrEqualTo(support.Z);
        max.Z.Should().BeGreaterThanOrEqualTo(support.Z);
        max.X.Should().BeGreaterThan(baseCenter.X + Fixed64.FromFraction(4, 5));
    }

}
