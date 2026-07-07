using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class Collider2DMassPropertyTests
{
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

        collider.CalculateAreaForMassProperties().Should().Be(Fixed64.Pi * (Fixed64)4);
        collider.CalculateLocalCenterOfMassOffset().Should().Be(new Vector2d((Fixed64)3, Fixed64.Zero));
        momentAboutCom.Should().Be((Fixed64)8);
        momentAboutOrigin.Should().Be((Fixed64)44);
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

        collider.CalculateAreaForMassProperties().Should().Be((Fixed64)8);
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

        AssertNear(collider.CalculateAreaForMassProperties(), expectedArea);
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

        capsule.CalculateAreaForMassProperties().Should().Be(circle.CalculateAreaForMassProperties());
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

        collider.CalculateAreaForMassProperties().Should().Be((Fixed64)2);
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

        collider.CalculateAreaForMassProperties().Should().Be((Fixed64)2);
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
    public void Compound_ShouldApplyPartRotationToLocalCenterOfMass()
    {
        var collider = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(
                Fixed64.One,
                new Vector2d((Fixed64)2, Fixed64.Zero),
                FixedMath.DegToRad((Fixed64)90),
                Vector2d.One),
            CompoundColliderPart2D.Circle(Fixed64.One, Vector2d.Zero));

        Vector2d center = collider.CalculateLocalCenterOfMassOffset();

        AssertNear(center.X, Fixed64.Zero);
        AssertNear(center.Y, Fixed64.One);
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

    private static void AssertNear(Fixed64 actual, Fixed64 expected)
    {
        Fixed64 tolerance = Fixed64.Epsilon * (Fixed64)16;
        FixedMath.Abs(actual - expected).Should().BeLessThanOrEqualTo(tolerance);
    }
}
