using FixedMathSharp;
using FluentAssertions;
using SwiftCollections.Query;
using System;
using Xunit;

namespace Gravitas.Tests.Dimensions;

public sealed class Physics2DBoundsTests
{
    [Fact]
    public void FromMinMax_ShouldNormalizeAreaAndProjectToStorageSlab()
    {
        Physics2DBounds bounds = Physics2DBounds.FromMinMax(
            new Vector2d((Fixed64)4, (Fixed64)(-2)),
            new Vector2d((Fixed64)(-1), (Fixed64)3),
            planeZ: (Fixed64)7,
            halfThickness: Fixed64.Fraction(1, 4));

        bounds.Area.Min.Should().Be(new Vector3d((Fixed64)(-1), (Fixed64)(-2), Fixed64.Zero));
        bounds.Area.Max.Should().Be(new Vector3d((Fixed64)4, (Fixed64)3, Fixed64.Zero));
        bounds.PlaneZ.Should().Be((Fixed64)7);
        bounds.HalfThickness.Should().Be(Fixed64.Fraction(1, 4));

        FixedBoundVolume volume = bounds.ToFixedBoundVolume();

        volume.Min.Should().Be(new Vector3d((Fixed64)(-1), (Fixed64)(-2), Fixed64.Fraction(27, 4)));
        volume.Max.Should().Be(new Vector3d((Fixed64)4, (Fixed64)3, Fixed64.Fraction(29, 4)));
    }

    [Fact]
    public void FromMinMax_WithNegativeHalfThickness_ShouldThrow()
    {
        Action create = () => Physics2DBounds.FromMinMax(
            new Vector2d(Fixed64.Zero, Fixed64.Zero),
            new Vector2d(Fixed64.One, Fixed64.One),
            Fixed64.Zero,
            -Fixed64.One);

        create.Should()
            .Throw<ArgumentException>()
            .WithMessage("*halfThickness*");
    }
}
