using FixedMathSharp;
using FixedMathSharp.Geometry;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class Collider2DMassPropertyTests
{
    [Fact]
    public void DetachedCompound_ShouldAggregateAnUnrepresentableChildCenter()
    {
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Right),
            CompoundColliderPart2D.Circle(Fixed64.Half, -Vector2d.Right))
        {
            LocalOffset = new Vector2d(Fixed64.MaxValue, Fixed64.Zero)
        };
        LSCollider2D unrepresentablePart = compound.GetPartCollider(0);

        Action calculatePart =
            () => unrepresentablePart.CalculateLocalCenterOfMassOffset();
        Action calculatePartMoment =
            () => unrepresentablePart.CalculateMomentOfInertia(
                Fixed64.One,
                Vector2d.Zero);
        Vector2d center = compound.CalculateLocalCenterOfMassOffset();
        Fixed64 moment = compound.CalculateMomentOfInertia(
            Fixed64.One,
            center);

        calculatePart.Should().Throw<InvalidOperationException>()
            .WithMessage("*outside*coordinate*domain*");
        calculatePartMoment.Should().Throw<InvalidOperationException>()
            .WithMessage("*outside the Fixed64 scalar domain*");
        center.Should().Be(new Vector2d(
            Fixed64.MaxValue,
            Fixed64.Zero));
        moment.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void Compound_ShouldRejectAnUnrepresentableAggregateCenterBeforeAdmission()
    {
        using GravitasWorldContext context =
            Physics2DTestWorld.CreateContext();
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Right),
            CompoundColliderPart2D.Circle(
                Fixed64.Half,
                new Vector2d((Fixed64)3, Fixed64.Zero)))
        {
            LocalOffset = new Vector2d(
                Fixed64.MaxValue,
                Fixed64.Zero)
        };

        Action calculate = () =>
            compound.CalculateLocalCenterOfMassOffset();
        var transform = new FixedTransform(
            new Vector3d(
                Fixed64.MinValue,
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.One);
        Action initialize = () =>
            compound.InitializeWithNoBody(
                new TestMatterAgent(context, transform));

        calculate.Should().Throw<InvalidOperationException>()
            .WithMessage("*2D compound collider's center of mass*");
        initialize.Should().Throw<InvalidOperationException>()
            .WithMessage("*Prepared 2D compound mass-property point*");
    }

    [Fact]
    public void InitializedCompound_ShouldAggregateAnUnrepresentableBodyLocalChildCenter()
    {
        using GravitasWorldContext context =
            Physics2DTestWorld.CreateContext();
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Right),
            CompoundColliderPart2D.Circle(Fixed64.Half, -Vector2d.Right))
        {
            LocalOffset = new Vector2d(
                Fixed64.MaxValue,
                Fixed64.Zero)
        };
        var body = new SolidBody2D(
            new TestMatterAgent(context),
            compound)
        {
            Mass = Fixed64.One
        };

        body.Initialize(
            new Vector2d(Fixed64.MinValue, Fixed64.Zero),
            Fixed64.Zero);

        compound.CalculateLocalCenterOfMassOffset().Should().Be(
            new Vector2d(
                Fixed64.MaxValue,
                Fixed64.Zero));
    }

    [Fact]
    public void Compound_ShouldPreserveWideAreaRatios()
    {
        Fixed64 extent = (Fixed64)1_000_000_000;
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.AABBox(
                new Vector2d(extent, extent),
                Vector2d.Zero),
            CompoundColliderPart2D.AABBox(
                new Vector2d(extent * Fixed64.Two, extent),
                new Vector2d((Fixed64)3, Fixed64.Zero)));

        compound.CalculateLocalCenterOfMassOffset().Should().Be(
            new Vector2d(Fixed64.Two, Fixed64.Zero));
    }

    [Fact]
    public void Circle_ShouldCalculateMomentAroundRequestedLocalReference()
    {
        var collider = new LSCircleCollider2D((Fixed64)2)
        {
            LocalOffset = new Vector2d((Fixed64)3, Fixed64.Zero)
        };

        Fixed64 momentAboutCom = collider.CalculateMomentOfInertia(
            (Fixed64)4,
            collider.CalculateLocalCenterOfMassOffset());
        Fixed64 momentAboutOrigin = collider.CalculateMomentOfInertia((Fixed64)4, Vector2d.Zero);

        GetMeasure(collider.CalculateAreaForMassProperties())
            .Should().Be(Fixed64.Pi * (Fixed64)4);
        collider.CalculateLocalCenterOfMassOffset().Should().Be(new Vector2d((Fixed64)3, Fixed64.Zero));
        momentAboutCom.Should().Be((Fixed64)8);
        momentAboutOrigin.Should().Be((Fixed64)44);
        collider.CalculateMomentOfInertia(
                Fixed64.MaxValue,
                collider.CalculateLocalCenterOfMassOffset())
            .Should().Be(Fixed64.MaxValue);
    }

    [Fact]
    public void Primitive2DGuards_ShouldHandleSameValueSettersZeroMassAndFallbackDirections()
    {
        var circle = new LSCircleCollider2D(Fixed64.One);
        var box = new LSAABBoxCollider2D(Vector2d.One);
        var capsule = new LSCapsuleCollider2D(Fixed64.One, (Fixed64)2);

        circle.Radius = Fixed64.One;
        box.Size = Vector2d.One;
        capsule.Radius = Fixed64.One;
        capsule.Height = (Fixed64)2;

        circle.GetClosestPoint(Vector2d.Zero).Should().Be(Vector2d.Right);
        circle.GetSupportPoint(Vector2d.Zero).Should().Be(Vector2d.Right);
        circle.CalculateMomentOfInertia(Fixed64.Zero, Vector2d.Zero).Should().Be(Fixed64.Zero);
        box.CalculateMomentOfInertia(Fixed64.Zero, Vector2d.Zero).Should().Be(Fixed64.Zero);
        capsule.AxisLength.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void Aabb_ShouldCalculateMomentAroundRequestedLocalReference()
    {
        var collider = new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)4))
        {
            LocalOffset = new Vector2d(Fixed64.One, (Fixed64)(-2))
        };

        Fixed64 momentAboutCom = collider.CalculateMomentOfInertia(
            (Fixed64)6,
            collider.CalculateLocalCenterOfMassOffset());
        Fixed64 momentAboutOrigin = collider.CalculateMomentOfInertia((Fixed64)6, Vector2d.Zero);

        GetMeasure(collider.CalculateAreaForMassProperties())
            .Should().Be((Fixed64)8);
        collider.CalculateLocalCenterOfMassOffset().Should().Be(new Vector2d(Fixed64.One, (Fixed64)(-2)));
        momentAboutCom.Should().Be((Fixed64)10);
        momentAboutOrigin.Should().Be((Fixed64)40);
    }

    [Fact]
    public void Capsule_ShouldCalculateAreaCenterAndMomentAroundRequestedLocalReference()
    {
        var collider = new LSCapsuleCollider2D(Fixed64.One, (Fixed64)4)
        {
            LocalOffset = new Vector2d((Fixed64)3, -Fixed64.One)
        };

        Fixed64 mass = (Fixed64)12;
        Fixed64 momentAboutCom = collider.CalculateMomentOfInertia(
            mass,
            collider.CalculateLocalCenterOfMassOffset());
        Fixed64 momentAboutOrigin = collider.CalculateMomentOfInertia(mass, Vector2d.Zero);
        Fixed64 expectedArea = (Fixed64)4 + Fixed64.Pi;

        collider.CalculateAreaForMassProperties()
            .TryGetMeasure(out Fixed64 area)
            .Should()
            .BeTrue();
        AssertNear(area, expectedArea);
        collider.CalculateLocalCenterOfMassOffset().Should().Be(new Vector2d((Fixed64)3, -Fixed64.One));
        collider.CalculateMomentOfInertia(Fixed64.Zero, Vector2d.Zero).Should().Be(Fixed64.Zero);
        momentAboutCom.Should().BeGreaterThan(mass * Fixed64.Half);
        momentAboutOrigin.Should().Be(momentAboutCom + mass * (Fixed64)10);
    }

    [Fact]
    public void Capsule_WithHeightEqualDiameter_ShouldBehaveLikeCircleForAreaAndMoment()
    {
        var capsule = new LSCapsuleCollider2D(Fixed64.One, (Fixed64)2);
        var circle = new LSCircleCollider2D(Fixed64.One);

        GetMeasure(capsule.CalculateAreaForMassProperties())
            .Should().Be(GetMeasure(circle.CalculateAreaForMassProperties()));
        capsule.CalculateLocalCenterOfMassOffset().Should().Be(circle.CalculateLocalCenterOfMassOffset());
        capsule.CalculateMomentOfInertia((Fixed64)8, Vector2d.Zero)
            .Should()
            .Be(circle.CalculateMomentOfInertia((Fixed64)8, Vector2d.Zero));
    }

    [Fact]
    public void ConvexPolygon_ShouldCalculateCentroidAreaAndMoment()
    {
        var collider = new LSPolygonCollider2D(
            new Vector2d(Fixed64.Zero, Fixed64.Zero),
            new Vector2d((Fixed64)2, Fixed64.Zero),
            new Vector2d((Fixed64)2, Fixed64.One),
            new Vector2d(Fixed64.Zero, Fixed64.One));

        Fixed64 momentAboutCom = collider.CalculateMomentOfInertia(
            (Fixed64)12,
            collider.CalculateLocalCenterOfMassOffset());
        Fixed64 momentAboutOrigin = collider.CalculateMomentOfInertia((Fixed64)12, Vector2d.Zero);

        GetMeasure(collider.CalculateAreaForMassProperties())
            .Should().Be((Fixed64)2);
        collider.CalculateLocalCenterOfMassOffset().Should().Be(new Vector2d(Fixed64.One, Fixed64.Half));
        momentAboutCom.Should().Be((Fixed64)5);
        momentAboutOrigin.Should().Be((Fixed64)20);
    }

    [Fact]
    public void ConvexPolygon_WithClockwiseWinding_ShouldCalculateSameMassProperties()
    {
        var collider = new LSPolygonCollider2D(
            new Vector2d(Fixed64.Zero, Fixed64.Zero),
            new Vector2d(Fixed64.Zero, Fixed64.One),
            new Vector2d((Fixed64)2, Fixed64.One),
            new Vector2d((Fixed64)2, Fixed64.Zero));

        Fixed64 momentAboutCom = collider.CalculateMomentOfInertia(
            (Fixed64)12,
            collider.CalculateLocalCenterOfMassOffset());
        Fixed64 momentAboutOrigin = collider.CalculateMomentOfInertia((Fixed64)12, Vector2d.Zero);

        GetMeasure(collider.CalculateAreaForMassProperties())
            .Should().Be((Fixed64)2);
        collider.CalculateLocalCenterOfMassOffset().Should().Be(new Vector2d(Fixed64.One, Fixed64.Half));
        collider.CalculateMomentOfInertia(Fixed64.Zero, Vector2d.Zero).Should().Be(Fixed64.Zero);
        momentAboutCom.Should().Be((Fixed64)5);
        momentAboutOrigin.Should().Be((Fixed64)20);
    }

    [Fact]
    public void Compound_ShouldAggregateAreaWeightedCenterAndMomentInStablePartOrder()
    {
        var collider = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.One, new Vector2d((Fixed64)(-1), Fixed64.Zero)),
            CompoundColliderPart2D.Circle(Fixed64.One, new Vector2d((Fixed64)3, Fixed64.Zero)));

        Fixed64 momentAboutCom = collider.CalculateMomentOfInertia(
            (Fixed64)4,
            collider.CalculateLocalCenterOfMassOffset());
        Fixed64 momentAboutOrigin = collider.CalculateMomentOfInertia((Fixed64)4, Vector2d.Zero);

        collider.CalculateLocalCenterOfMassOffset().Should().Be(new Vector2d(Fixed64.One, Fixed64.Zero));
        collider.CalculateMomentOfInertia(Fixed64.Zero, Vector2d.Zero).Should().Be(Fixed64.Zero);
        momentAboutCom.Should().Be((Fixed64)18);
        momentAboutOrigin.Should().Be((Fixed64)22);
    }

    [Fact]
    public void Compound_ShouldApplyPartScaleBeforeAreaWeighting()
    {
        var collider = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(
                Fixed64.One,
                Vector2d.Zero,
                Fixed64.Zero,
                new Vector2d((Fixed64)2, (Fixed64)2)),
            CompoundColliderPart2D.Circle(
                Fixed64.One,
                new Vector2d((Fixed64)4, Fixed64.Zero),
                Fixed64.Zero,
                Vector2d.One));

        AssertNear(
            collider.CalculateLocalCenterOfMassOffset().X,
            Fixed64.FromFraction(4, 5));
        collider.CalculateLocalCenterOfMassOffset().Y.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void Compound_ShouldAverageLargeOpposingCentersWithoutSaturatingProducts()
    {
        Fixed64 negativeCenter =
            -(Fixed64.MaxValue * Fixed64.FromFraction(3, 4));
        Fixed64 positiveCenter =
            Fixed64.MaxValue * Fixed64.FromFraction(1, 4);
        var collider = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(
                (Fixed64)2,
                new Vector2d(negativeCenter, Fixed64.Zero)),
            CompoundColliderPart2D.Circle(
                (Fixed64)2,
                new Vector2d(positiveCenter, Fixed64.Zero)));

        Vector2d center = collider.CalculateLocalCenterOfMassOffset();

        center.Should().Be(new Vector2d(
            FixedMath.Midpoint(negativeCenter, positiveCenter),
            Fixed64.Zero));
    }

    [Fact]
    public void Compound_ShouldKeepPartTranslationInOwnerFrame()
    {
        using GravitasWorldContext context =
            Physics2DTestWorld.CreateContext();
        var collider = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(
                Fixed64.One,
                new Vector2d((Fixed64)2, Fixed64.Zero),
                FixedMath.DegToRad((Fixed64)90),
                Vector2d.One),
            CompoundColliderPart2D.Circle(Fixed64.One, Vector2d.Zero));

        Vector2d detachedCenter =
            collider.CalculateLocalCenterOfMassOffset();
        collider.InitializeWithNoBody(
            new TestMatterAgent(context));
        Vector2d initializedCenter =
            collider.CalculateLocalCenterOfMassOffset();

        AssertNear(detachedCenter.X, Fixed64.One);
        AssertNear(detachedCenter.Y, Fixed64.Zero);
        AssertNear(initializedCenter.X, Fixed64.One);
        AssertNear(initializedCenter.Y, Fixed64.Zero);
    }

    [Fact]
    public void Compound_ShouldApplyOwnerLocalOffsetToMassProperties()
    {
        var collider = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.One, Vector2d.Zero),
            CompoundColliderPart2D.Circle(Fixed64.One, new Vector2d((Fixed64)2, Fixed64.Zero)))
        {
            LocalOffset = new Vector2d((Fixed64)5, -Fixed64.One)
        };

        collider.CalculateLocalCenterOfMassOffset()
            .Should().Be(new Vector2d((Fixed64)6, -Fixed64.One));
    }

    [Fact]
    public void RotatedCompoundPolygon_ShouldDeriveMassPropertiesWithoutMaterializingVertices()
    {
        Fixed64 extent = Fixed64.MaxValue * Fixed64.FromFraction(3, 4);
        Vector2d[] vertices =
        {
            new(-extent, -extent),
            new(extent, -extent),
            new(extent, extent),
            new(-extent, extent)
        };
        Fixed64 rotation = Fixed64.PiOver4;
        Vector2d.TryTransformPoint(
            Vector2d.Zero,
            vertices[2],
            rotation,
            out _).Should().BeFalse();
        var collider = new LSCompoundCollider2D(
            CompoundColliderPart2D.ConvexPolygon(
                vertices,
                Vector2d.Zero,
                rotation,
                Vector2d.One));

        collider.CalculateLocalCenterOfMassOffset().Should().Be(Vector2d.Zero);
        FluentActions.Invoking(
                () => collider.CalculateMomentOfInertia(
                    Fixed64.One,
                    Vector2d.Zero))
            .Should().NotThrow();
    }

    [Fact]
    public void Compound_ShouldRetainTinyAreaWeightWithoutChangingRoundedResult()
    {
        Fixed64 tinyRadius = Fixed64.FromRaw(1);
        var collider = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(tinyRadius, new Vector2d((Fixed64)100, Fixed64.Zero)),
            CompoundColliderPart2D.Circle(Fixed64.One, new Vector2d((Fixed64)2, Fixed64.Zero)));

        GetMeasure(collider.GetPartCollider(0).CalculateAreaForMassProperties())
            .Should().Be(Fixed64.Zero);
        GetMeasure(collider.CalculateAreaForMassProperties())
            .Should().Be(Fixed64.Pi);
        collider.CalculateLocalCenterOfMassOffset().Should().Be(new Vector2d((Fixed64)2, Fixed64.Zero));
        collider.CalculateMomentOfInertia((Fixed64)4, new Vector2d((Fixed64)2, Fixed64.Zero))
            .Should().Be((Fixed64)2);
    }

    [Fact]
    public void Compound_WhenAllPartAreaMeasuresRoundToZero_ShouldRetainExactEqualWeights()
    {
        Fixed64 tinyRadius = Fixed64.FromRaw(1);
        var collider = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(tinyRadius, new Vector2d(-Fixed64.One, Fixed64.Zero)),
            CompoundColliderPart2D.Circle(tinyRadius, new Vector2d((Fixed64)3, Fixed64.Zero)));
        Fixed64 mass = (Fixed64)2;

        Vector2d center = collider.CalculateLocalCenterOfMassOffset();
        Fixed64 moment = collider.CalculateMomentOfInertia(mass, center);

        center.Should().Be(new Vector2d(Fixed64.One, Fixed64.Zero));
        moment.Should().Be((Fixed64)8);
    }

    [Fact]
    public void Compound_ShouldApportionTinyMassFromCumulativeWeightShares()
    {
        using GravitasWorldContext context =
            Physics2DTestWorld.CreateContext();
        var collider = new LSCompoundCollider2D(
            CompoundColliderPart2D.AABBox(
                new Vector2d((Fixed64)3, Fixed64.One),
                Vector2d.Zero),
            CompoundColliderPart2D.AABBox(
                new Vector2d((Fixed64)3, Fixed64.One),
                Vector2d.Zero),
            CompoundColliderPart2D.AABBox(
                new Vector2d((Fixed64)3, Fixed64.One),
                Vector2d.Zero),
            CompoundColliderPart2D.AABBox(Vector2d.One, Vector2d.Zero));
        collider.InitializeWithNoBody(new TestMatterAgent(context));
        Fixed64 apportionedMass = Fixed64.FromRaw(1);
        Fixed64 expected =
            collider.GetPartCollider(0)
                .CalculateCenterOfMassMoment(apportionedMass)
            + collider.GetPartCollider(2)
                .CalculateCenterOfMassMoment(apportionedMass);

        collider.CalculateMomentOfInertia(
                Fixed64.FromRaw(2),
                Vector2d.Zero)
            .Should()
            .Be(expected);
    }

    [Fact]
    public void Compound_WhenDetachedPartsHaveZeroSemanticArea_ShouldUseEqualNominalShares()
    {
        Vector2d tinyScale =
            Vector2d.One * Fixed64.FromRaw(1);
        var collider = new LSCompoundCollider2D(
            CompoundColliderPart2D.Capsule(
                Fixed64.Half,
                (Fixed64)1_000_000_000,
                new Vector2d(-Fixed64.One, Fixed64.Zero),
                Fixed64.Zero,
                tinyScale),
            CompoundColliderPart2D.Capsule(
                Fixed64.Half,
                (Fixed64)1_000_000_000,
                Vector2d.Zero,
                Fixed64.Zero,
                tinyScale),
            CompoundColliderPart2D.Capsule(
                Fixed64.Half,
                (Fixed64)1_000_000_000,
                new Vector2d((Fixed64)4, Fixed64.Zero),
                Fixed64.Zero,
                tinyScale));
        Vector2d center =
            collider.CalculateLocalCenterOfMassOffset();
        Fixed64 expected = Fixed64.Zero;
        for (int i = 0; i < 3; i++)
        {
            LSCollider2D part = collider.GetPartCollider(i);
            part.CalculateAreaForMassProperties().IsZero
                .Should()
                .BeTrue();
            part.CalculateLocalMassPoint()
                .TryAddParallelAxisMoment(
                    part.CalculateCenterOfMassMoment(Fixed64.One),
                    Fixed64.One,
                    center,
                    out Fixed64 contribution)
                .Should()
                .BeTrue();
            expected += contribution;
        }

        center.Should().Be(new Vector2d(
            Fixed64.One,
            Fixed64.Zero));
        collider.CalculateMomentOfInertia(
                (Fixed64)3,
                center)
            .Should()
            .Be(expected);
    }

    [Fact]
    public void Compound_WithZeroScaledRadiusCapsule_ShouldUseThinRodMomentLimit()
    {
        var collider = new LSCompoundCollider2D(
            CompoundColliderPart2D.Capsule(
                Fixed64.Half,
                (Fixed64)1_000_000_000,
                Vector2d.Zero,
                Fixed64.Zero,
                new Vector2d(Fixed64.FromRaw(1), Fixed64.FromRaw(1))));
        var capsule = (LSCapsuleCollider2D)collider.GetPartCollider(0);
        Fixed64 mass = (Fixed64)3;
        Fixed64 scaledLength = capsule.Height * Fixed64.FromRaw(1);
        Fixed64 expectedMoment = mass * scaledLength * scaledLength / (Fixed64)12;

        capsule.ScaledRadius.Should().Be(Fixed64.Zero);
        GetMeasure(capsule.CalculateAreaForMassProperties())
            .Should().Be(Fixed64.Zero);
        collider.CalculateMomentOfInertia(mass, Vector2d.Zero).Should().Be(expectedMoment);
    }

    [Fact]
    public void Compound_WithAreaMeasuresRoundedToZeroPolygons_ShouldRejectUnrepresentableParallelAxisMoment()
    {
        Vector2d[] square =
        {
            new(-Fixed64.One, -Fixed64.One),
            new(Fixed64.One, -Fixed64.One),
            new(Fixed64.One, Fixed64.One),
            new(-Fixed64.One, Fixed64.One)
        };
        Fixed64 offsetMagnitude = (Fixed64)1_000_000_000;
        Vector2d tinyScale = new(Fixed64.FromRaw(1), Fixed64.FromRaw(1));
        var collider = new LSCompoundCollider2D(
            CompoundColliderPart2D.ConvexPolygon(
                square,
                new Vector2d(-offsetMagnitude, Fixed64.Zero),
                Fixed64.Zero,
                tinyScale),
            CompoundColliderPart2D.ConvexPolygon(
                square,
                new Vector2d(offsetMagnitude, Fixed64.Zero),
                Fixed64.Zero,
                tinyScale));
        Fixed64 mass = (Fixed64)2;

        GetMeasure(collider.CalculateAreaForMassProperties())
            .Should().Be(Fixed64.Zero);
        collider.CalculateLocalCenterOfMassOffset().Should().Be(Vector2d.Zero);
        FluentActions.Invoking(
                () => collider.CalculateMomentOfInertia(
                    mass,
                    Vector2d.Zero))
            .Should().Throw<InvalidOperationException>()
            .WithMessage("*outside*scalar domain*");
    }

    private static void AssertNear(Fixed64 actual, Fixed64 expected)
    {
        Fixed64 tolerance = Fixed64.Epsilon * (Fixed64)16;
        FixedMath.Abs(actual - expected).Should().BeLessThanOrEqualTo(tolerance);
    }

    private static Fixed64 GetMeasure(ExactMassWeight weight)
    {
        weight.TryGetMeasure(out Fixed64 measure).Should().BeTrue();
        return measure;
    }
}
