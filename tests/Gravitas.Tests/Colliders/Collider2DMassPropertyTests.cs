using FixedMathSharp;
using FixedMathSharp.Geometry;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using System;
using System.Numerics;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class Collider2DMassPropertyTests
{
    [Theory]
    [InlineData(3, 0)]
    [InlineData(6, 1)]
    public void SquareMoment_ShouldRoundBothAxisTermsOnceAcrossRepresentations(long massRaw, long expectedRaw)
    {
        var box = new LSAABBoxCollider2D(Vector2d.One);
        CompoundColliderPart2D part = CompoundColliderPart2D.AABBox(Vector2d.One, Vector2d.Zero);
        var single = new LSCompoundCollider2D(part);
        var coincident = new LSCompoundCollider2D(part, part);
        Fixed64 mass = Fixed64.FromRaw(massRaw);
        Fixed64 expected = Fixed64.FromRaw(expectedRaw);

        box.CalculateMomentOfInertia(mass, Vector2d.Zero).Should().Be(expected);
        single.CalculateMomentOfInertia(mass, Vector2d.Zero).Should().Be(expected);
        coincident.CalculateMomentOfInertia(mass, Vector2d.Zero).Should().Be(expected);
    }

    [Fact]
    public void SquareMoment_ShouldRetainLargeUnitGeometryUntilTinyMassIsApplied()
    {
        Vector2d size = Vector2d.One * Fixed64.MaxValue;
        var box = new LSAABBoxCollider2D(size);
        var compound = new LSCompoundCollider2D(CompoundColliderPart2D.AABBox(size, Vector2d.Zero));
        Fixed64 expected = Fixed64.FromRaw(768614336404564651);

        box.CalculateMomentOfInertia(Fixed64.FromRaw(1), Vector2d.Zero).Should().Be(expected);
        compound.CalculateMomentOfInertia(Fixed64.FromRaw(1), Vector2d.Zero).Should().Be(expected);
    }

    [Theory]
    [InlineData(1, false)]
    [InlineData(1, true)]
    [InlineData(3, false)]
    [InlineData(3, true)]
    [InlineData(4294967296, false)]
    [InlineData(4294967296, true)]
    public void UnequalCircleCompound_ShouldRetainAreaWeightsThroughFinalMoment(long massRaw, bool reversed)
    {
        Vector2d offset = new((Fixed64)2048, (Fixed64)1024);
        CompoundColliderPart2D first = CompoundColliderPart2D.Circle((Fixed64)512, Vector2d.Zero);
        CompoundColliderPart2D second = CompoundColliderPart2D.Circle((Fixed64)1024, offset);
        var compound = reversed ? new LSCompoundCollider2D(second, first) : new LSCompoundCollider2D(first, second);
        BigInteger q = BigInteger.One << 32;
        BigInteger[] radii = { 512 * q, 1024 * q };
        BigInteger[] weights = { radii[0] * radii[0], radii[1] * radii[1] };
        BigInteger totalWeight = weights[0] + weights[1];
        Vector2d center = new(RoundRational(weights[1] * offset.X.m_rawValue, totalWeight),
            RoundRational(weights[1] * offset.Y.m_rawValue, totalWeight));
        BigInteger integral = BigInteger.Zero;
        for (int part = 0; part < 2; part++)
        {
            BigInteger dx = (part == 0 ? 0L : offset.X.m_rawValue) - (BigInteger)center.X.m_rawValue;
            BigInteger dy = (part == 0 ? 0L : offset.Y.m_rawValue) - (BigInteger)center.Y.m_rawValue;
            integral += weights[part] * (radii[part] * radii[part] + 2 * (dx * dx + dy * dy));
        }
        Fixed64 expected = RoundRational(massRaw * integral, 2 * totalWeight * q * q);
        using var context = GravitasWorldContext.CreateOwned();
        compound.InitializeWithNoBody(new TestMatterAgent(context));

        compound.CalculateLocalCenterOfMassOffset().Should().Be(center);
        compound.CalculateMomentOfInertia(Fixed64.FromRaw(massRaw), center).Should().Be(expected);
    }

    [Fact]
    public void CoincidentCircleParts_ShouldMatchSingleCircleWithoutRoundingChildContributions()
    {
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.One, Vector2d.Zero),
            CompoundColliderPart2D.Circle(Fixed64.One, Vector2d.Zero));
        var circle = new LSCircleCollider2D(Fixed64.One);
        Fixed64 mass = Fixed64.FromRaw(3);
        Fixed64 expected = Fixed64.FromRaw(2);

        circle.CalculateMomentOfInertia(mass, Vector2d.Zero).Should().Be(expected);
        compound.CalculateMomentOfInertia(mass, Vector2d.Zero).Should().Be(expected);
        // Center and shift round separately to 2+1, but their exact sum is 2.25 raw.
        compound.CalculateMomentOfInertia(mass, new Vector2d(Fixed64.Half, Fixed64.Zero)).Should().Be(expected);
        compound.CalculateMomentOfInertia(Fixed64.Zero, Vector2d.Zero).Should().Be(Fixed64.Zero);
        using var context = GravitasWorldContext.CreateOwned();
        compound.InitializeWithNoBody(new TestMatterAgent(context));
        compound.CalculateMomentOfInertia(mass, Vector2d.Zero).Should().Be(expected);
    }

    [Theory]
    [InlineData(4294967296, 8589934592, 3, 2)]
    [InlineData(4294967296, 17179869184, 3, 4)]
    [InlineData(2576980378, 11811160064, 10021590357, 6449436936)]
    public void SingleCapsuleCompound_ShouldRetainTheStandaloneIntrinsicModel(long radiusRaw, long heightRaw, long massRaw, long expectedRaw)
    {
        Fixed64 radius = Fixed64.FromRaw(radiusRaw);
        Fixed64 height = Fixed64.FromRaw(heightRaw);
        Fixed64 mass = Fixed64.FromRaw(massRaw);
        var capsule = new LSCapsuleCollider2D(radius, height);
        var compound = new LSCompoundCollider2D(CompoundColliderPart2D.Capsule(radius, height, Vector2d.Zero));
        Fixed64 expected = Fixed64.FromRaw(expectedRaw);

        capsule.CalculateMomentOfInertia(mass, Vector2d.Zero).Should().Be(expected);
        compound.CalculateMomentOfInertia(mass, Vector2d.Zero).Should().Be(expected);
    }

    [Fact]
    public void CompoundMomentCache_ShouldFollowOwnerScaleLocalOffsetAndRequestedReference()
    {
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.One, Vector2d.Right),
            CompoundColliderPart2D.Circle(Fixed64.One, -Vector2d.Right));
        compound.CalculateMomentOfInertia(Fixed64.Two, Vector2d.Zero).Should().Be((Fixed64)3);
        compound.CalculateMomentOfInertia((Fixed64)4, Vector2d.Zero).Should().Be((Fixed64)6);
        compound.LocalOffset = new Vector2d((Fixed64)3, -Fixed64.One);
        compound.CalculateMomentOfInertia(Fixed64.Two, Vector2d.Zero).Should().Be((Fixed64)23);
        compound.CalculateMomentOfInertia(Fixed64.Two, compound.CalculateLocalCenterOfMassOffset()).Should().Be((Fixed64)3);

        using var context = GravitasWorldContext.CreateOwned();
        var transform = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One * Fixed64.Two);
        compound.InitializeWithNoBody(new TestMatterAgent(context, transform));
        compound.CalculateLocalCenterOfMassOffset().Should().Be(new Vector2d((Fixed64)6, (Fixed64)(-2)));
        compound.CalculateMomentOfInertia(Fixed64.Two, Vector2d.Zero).Should().Be((Fixed64)92);
        compound.CalculateMomentOfInertia(Fixed64.Two, compound.CalculateLocalCenterOfMassOffset()).Should().Be((Fixed64)12);
        transform.LocalPosition = new Vector3d((Fixed64)100, Fixed64.Zero, (Fixed64)200);
        compound.Simulate();
        compound.CalculateMomentOfInertia(Fixed64.Two, compound.CalculateLocalCenterOfMassOffset()).Should().Be((Fixed64)12);
    }

    [Fact]
    public void CompoundMoment_ShouldIgnoreDetachedPartsWithZeroSemanticArea()
    {
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.FromRaw(1), new Vector2d((Fixed64)100, Fixed64.Zero),
                Fixed64.Zero, Vector2d.One * Fixed64.Half),
            CompoundColliderPart2D.Circle(Fixed64.One, Vector2d.Zero));

        compound.CalculateLocalCenterOfMassOffset().Should().Be(Vector2d.Zero);
        compound.CalculateMomentOfInertia(Fixed64.One, Vector2d.Zero).Should().Be(Fixed64.Half);
    }

    [Fact]
    public void CircleCompound_ShouldRetainUnrepresentableUnitMomentUntilTinyParentMassIsApplied()
    {
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.MaxValue, Vector2d.Zero),
            CompoundColliderPart2D.Circle(Fixed64.MaxValue, Vector2d.Zero));

        compound.CalculateMomentOfInertia(Fixed64.FromRaw(1), Vector2d.Zero)
            .Should().Be(Fixed64.FromRaw(2305843009213693952));
    }

    [Theory]
    [InlineData(1, 3, false, false)]
    [InlineData(3, 8, false, false)]
    [InlineData(3, 8, true, false)]
    [InlineData(3, 8, true, true)]
    [InlineData(4294967296, 11453246123, false, false)]
    public void BoxAndPolygonCompound_ShouldRetainUnroundedIntrinsicAndOffsetMoments(long massRaw, long expectedRaw, bool reversed, bool clockwise)
    {
        CompoundColliderPart2D box = CompoundColliderPart2D.AABBox(Vector2d.One, Vector2d.Zero);
        Fixed64 half = Fixed64.Half;
        Vector2d[] vertices =
        {
            new Vector2d(-half, -half), new Vector2d(half, -half),
            new Vector2d(half, half), new Vector2d(-half, half)
        };
        if (clockwise) Array.Reverse(vertices);
        CompoundColliderPart2D polygon = CompoundColliderPart2D.ConvexPolygon(vertices, new Vector2d((Fixed64)3, Fixed64.One));
        var compound = reversed ? new LSCompoundCollider2D(polygon, box) : new LSCompoundCollider2D(box, polygon);
        using var context = GravitasWorldContext.CreateOwned();
        compound.InitializeWithNoBody(new TestMatterAgent(context));
        // Unit-mass moment is 1/6 + (3^2+1^2)/4 = 8/3.
        compound.CalculateMomentOfInertia(Fixed64.FromRaw(massRaw), compound.CalculateLocalCenterOfMassOffset())
            .Should().Be(Fixed64.FromRaw(expectedRaw));
    }

    [Theory]
    [InlineData(3, 5)]
    [InlineData(1, 1)]
    public void CapsuleMoment_ShouldSaturateAnUnrepresentableFinalProduct(int radiusNumerator, int radiusDenominator)
    {
        var capsule = new LSCapsuleCollider2D(Fixed64.FromFraction(radiusNumerator, radiusDenominator), (Fixed64)5);
        capsule.CalculateMomentOfInertia(Fixed64.MaxValue, Vector2d.Zero).Should().Be(Fixed64.MaxValue);
    }

    [Fact]
    public void CapsuleMoment_ShouldRetainNonbinaryMassAndGeometryProducts()
    {
        var capsule = new LSCapsuleCollider2D(Fixed64.FromFraction(3, 5), Fixed64.FromFraction(11, 4));
        capsule.CalculateMomentOfInertia(Fixed64.FromFraction(7, 3), Vector2d.Zero)
            .Should().Be(Fixed64.FromRaw(6449436936));
    }

    [Theory]
    [InlineData(1, 1)]
    [InlineData(4294967296, 3)]
    [InlineData(4398046511104, 1)]
    [InlineData(4398046511104, 3)]
    public void CollapsedCapsule_ShouldMatchCircleMoment(long radiusRaw, long massRaw)
    {
        Fixed64 radius = Fixed64.FromRaw(radiusRaw);
        Fixed64 mass = Fixed64.FromRaw(massRaw);
        var capsule = new LSCapsuleCollider2D(radius, radius * Fixed64.Two);
        var circle = new LSCircleCollider2D(radius);

        capsule.CalculateMomentOfInertia(mass, Vector2d.Zero)
            .Should().Be(circle.CalculateMomentOfInertia(mass, Vector2d.Zero));
    }

    [Theory]
    [InlineData(1, 1, 524614)]
    [InlineData(1, 3, 1573842)]
    [InlineData(1024, 3, 2767176)]
    public void Capsule_ShouldRetainExactComponentWeightsThroughFinalMoment(int coreLength, long massRaw, long expectedRaw)
    {
        var capsule = new LSCapsuleCollider2D((Fixed64)1024, (Fixed64)(2048 + coreLength));
        capsule.CalculateMomentOfInertia(Fixed64.FromRaw(massRaw), Vector2d.Zero)
            .Should().Be(Fixed64.FromRaw(expectedRaw));
    }

    [Fact]
    public void Capsule_ShouldIncludeBothSemicirclePolarMoments()
    {
        var capsule = new LSCapsuleCollider2D(Fixed64.One, (Fixed64)4);
        // At unit density, rectangle inertia is 8/3 and the paired caps contribute
        // 3*pi/2 + 8/3: analytic integration about the stadium center gives 3*pi/2 + 16/3.
        Fixed64 expected = Fixed64.Pi * Fixed64.FromFraction(3, 2) + Fixed64.FromFraction(16, 3);
        Fixed64 actual = capsule.CalculateMomentOfInertia(Fixed64.Pi + (Fixed64)4, Vector2d.Zero);
        (actual - expected).Abs().Should().BeLessThanOrEqualTo(Fixed64.FromRaw(4));
    }

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
    public void InitializedBodylessCompound_ShouldAggregateAnUnrepresentableLocalChildCenter()
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
        compound.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero),
                FixedQuaternion.Identity, Vector3d.One)));

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
    public void ConvexPolygon_ReferenceShift_ShouldRetainRoundedCentroidResidual()
    {
        var polygon = new LSPolygonCollider2D(Vector2d.Zero, Vector2d.Right, Vector2d.Forward);
        Fixed64 mass = (Fixed64)1_000_000_000;
        // Direct integration over the unit right triangle gives I(origin) = mass/3.
        Fixed64 expected = RoundRational((BigInteger)mass.m_rawValue, 3);

        polygon.CalculateMomentOfInertia(mass, Vector2d.Zero).Should().Be(expected);
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
    public void RotatedCompoundPolygon_ShouldCheckFinalMomentWithoutMaterializingVertices()
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
        // A square of side 2e has I/m = 2e^2/3, invariant under its planar rotation.
        BigInteger q = BigInteger.One << 32;
        BigInteger twiceExtentSquared = 2 * (BigInteger)extent.m_rawValue * extent.m_rawValue;
        Assert.True(twiceExtentSquared / (3 * q) > long.MaxValue);
        FluentActions.Invoking(
                () => collider.CalculateMomentOfInertia(
                    Fixed64.One,
                    Vector2d.Zero))
            .Should().Throw<InvalidOperationException>().WithMessage("*outside the Fixed64 scalar domain*");
        Fixed64 expectedTinyMoment = RoundRational(twiceExtentSquared, 3 * q * q);
        collider.CalculateMomentOfInertia(Fixed64.FromRaw(1), Vector2d.Zero)
            .Should().Be(expectedTinyMoment);
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
    public void Compound_ShouldRetainUnequalBoxWeightsThroughFinalMoment()
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
        // Weights 3:3:3:1 give raw moment 2*(9*5/6+1/6)/10 = 23/15.
        Fixed64 expected = Fixed64.FromRaw(2);

        collider.CalculateMomentOfInertia(
                Fixed64.FromRaw(2),
                Vector2d.Zero)
            .Should()
            .Be(expected);
    }

    [Fact]
    public void Compound_WhenDetachedPartsHaveZeroSemanticArea_ShouldRetainEqualNominalWeights()
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
        for (int i = 0; i < 3; i++)
            collider.GetPartCollider(i).CalculateAreaForMassProperties().IsZero
                .Should()
                .BeTrue();
        // Equal rods of total mass 3 have intrinsic moment 3*L^2/12;
        // their offsets -2, -1, 3 contribute 14 about the aggregate center.
        // Exact diameter subtraction retains one raw unit even though radius rounds to zero.
        BigInteger q = BigInteger.One << 32;
        Fixed64 expected = RoundRational((BigInteger)999_999_999 * 999_999_999 + 56 * q * q, 4 * q);

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
    public void ThinRodCompound_ShouldRejectAnUnrepresentableFinalMoment()
    {
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Capsule(Fixed64.FromRaw(1), (Fixed64)16,
                Vector2d.Zero, Fixed64.Zero, Vector2d.One * Fixed64.Half));

        FluentActions.Invoking(() => compound.CalculateMomentOfInertia(Fixed64.MaxValue, Vector2d.Zero))
            .Should().Throw<InvalidOperationException>().WithMessage("*outside the Fixed64 scalar domain*");
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

    private static Fixed64 RoundRational(BigInteger numerator, BigInteger denominator)
    {
        BigInteger rounded = BigInteger.DivRem(BigInteger.Abs(numerator), denominator, out BigInteger remainder);
        int half = (2 * remainder).CompareTo(denominator);
        if (half > 0 || half == 0 && !rounded.IsEven) rounded++;
        return Fixed64.FromRaw((long)(numerator.Sign < 0 ? -rounded : rounded));
    }

    private static Fixed64 GetMeasure(ExactMassWeight weight)
    {
        weight.TryGetMeasure(out Fixed64 measure).Should().BeTrue();
        return measure;
    }
}
