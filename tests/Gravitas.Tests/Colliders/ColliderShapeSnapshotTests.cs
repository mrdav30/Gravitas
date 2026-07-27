using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class ColliderShapeSnapshotTests
{
    [Fact]
    public void ShapeSnapshotEquality_ShouldCompareAllAuthoredRuntimeFields()
    {
        ColliderShapeSnapshot snapshot = Create3DSnapshot();
        ColliderShapeSnapshot same = Create3DSnapshot();
        ColliderShapeSnapshot differentRadius = new(
            snapshot.Center,
            snapshot.Rotation,
            snapshot.OwnerScale,
            snapshot.PartScale,
            snapshot.LocalOffset,
            snapshot.Size,
            Fixed64.Half);

        snapshot.Should().Be(same);
        snapshot.Equals((object)same).Should().BeTrue();
        snapshot.GetHashCode().Should().Be(same.GetHashCode());
        (snapshot == same).Should().BeTrue();
        (snapshot != same).Should().BeFalse();

        snapshot.Should().NotBe(differentRadius);
        snapshot.Equals("shape").Should().BeFalse();
        (snapshot != differentRadius).Should().BeTrue();
    }

    [Fact]
    public void ShapeSnapshot2DEquality_ShouldCompareAllRuntimeAndMixedBoundsFields()
    {
        ColliderShapeSnapshot2D snapshot = Create2DSnapshot();
        ColliderShapeSnapshot2D same = Create2DSnapshot();
        ColliderShapeSnapshot2D differentThickness = new(
            snapshot.Center,
            snapshot.Rotation,
            snapshot.OwnerScale,
            snapshot.PartScale,
            snapshot.LocalOffset,
            snapshot.ShapeVersion,
            snapshot.MixedSlabCenterY,
            Fixed64.Half);

        snapshot.Should().Be(same);
        snapshot.Equals((object)same).Should().BeTrue();
        snapshot.GetHashCode().Should().Be(same.GetHashCode());

        snapshot.Should().NotBe(differentThickness);
        snapshot.Equals("shape").Should().BeFalse();
    }

    private static ColliderShapeSnapshot Create3DSnapshot() =>
        new(
            new Vector3d(Fixed64.One, (Fixed64)2, (Fixed64)3),
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, (Fixed64)45, Fixed64.Zero),
            new Vector3d(Fixed64.One, Fixed64.Half, (Fixed64)2),
            new Vector3d(Fixed64.Half, Fixed64.One, (Fixed64)3),
            new Vector3d(Fixed64.Half, Fixed64.Zero, -Fixed64.Half),
            new Vector3d((Fixed64)2, (Fixed64)3, (Fixed64)4),
            Fixed64.One);

    private static ColliderShapeSnapshot2D Create2DSnapshot() =>
        new(
            new Vector2d(Fixed64.One, (Fixed64)2),
            FixedMath.DegToRad((Fixed64)30),
            new Vector2d(Fixed64.One, Fixed64.Half),
            new Vector2d(Fixed64.Half, (Fixed64)2),
            new Vector2d(Fixed64.Half, -Fixed64.Half),
            7,
            Fixed64.FromFraction(3, 4),
            Fixed64.FromFraction(5, 4));
}
