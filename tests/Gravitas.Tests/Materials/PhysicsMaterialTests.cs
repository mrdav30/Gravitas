using FixedMathSharp;
using FluentAssertions;
using Gravitas.Materials;
using System;
using Xunit;

namespace Gravitas.Tests.Materials;

public sealed class PhysicsMaterialTests
{
    [Fact]
    public void Default_ShouldMatchExistingResponseCoefficients()
    {
        PhysicsMaterial material = PhysicsMaterial.Default;

        material.StaticFriction.Should().Be(Fixed64.One);
        material.DynamicFriction.Should().Be(Fixed64.One);
        material.Restitution.Should().Be(Fixed64.Half);
        material.FrictionCombine.Should().Be(PhysicsMaterialCombine.GeometricMean);
        material.RestitutionCombine.Should().Be(PhysicsMaterialCombine.Minimum);
    }

    [Fact]
    public void Constructor_ShouldRejectInvalidValues()
    {
        Action negativeStatic = () => new PhysicsMaterial(-Fixed64.Epsilon, Fixed64.Zero, Fixed64.Zero);
        Action negativeDynamic = () => new PhysicsMaterial(Fixed64.One, -Fixed64.Epsilon, Fixed64.Zero);
        Action dynamicAboveStatic = () => new PhysicsMaterial(Fixed64.Half, Fixed64.One, Fixed64.Zero);
        Action negativeRestitution = () => new PhysicsMaterial(Fixed64.One, Fixed64.One, -Fixed64.Epsilon);
        Action highRestitution = () => new PhysicsMaterial(Fixed64.One, Fixed64.One, Fixed64.One + Fixed64.Epsilon);

        negativeStatic.Should().Throw<ArgumentOutOfRangeException>();
        negativeDynamic.Should().Throw<ArgumentOutOfRangeException>();
        dynamicAboveStatic.Should().Throw<ArgumentOutOfRangeException>();
        negativeRestitution.Should().Throw<ArgumentOutOfRangeException>();
        highRestitution.Should().Throw<ArgumentOutOfRangeException>();
    }

    [Theory]
    [InlineData(PhysicsMaterialCombine.Minimum, 1, 4, 1)]
    [InlineData(PhysicsMaterialCombine.Maximum, 1, 4, 4)]
    [InlineData(PhysicsMaterialCombine.Average, 1, 3, 2)]
    [InlineData(PhysicsMaterialCombine.Multiply, 2, 3, 6)]
    [InlineData(PhysicsMaterialCombine.GeometricMean, 4, 9, 6)]
    public void CombineScalar_ShouldApplyPolicy(
        PhysicsMaterialCombine policy,
        int left,
        int right,
        int expected)
    {
        PhysicsMaterial.CombineScalar((Fixed64)left, (Fixed64)right, policy)
            .Should()
            .Be((Fixed64)expected);
    }

    [Fact]
    public void CombineFriction_ShouldUseDominantFrictionPolicy()
    {
        var average = new PhysicsMaterial((Fixed64)4, (Fixed64)2, Fixed64.Zero, PhysicsMaterialCombine.Average);
        var maximum = new PhysicsMaterial((Fixed64)9, (Fixed64)3, Fixed64.Zero, PhysicsMaterialCombine.Maximum);

        PhysicsMaterial.CombineFriction(
            average,
            maximum,
            out Fixed64 staticFriction,
            out Fixed64 dynamicFriction);

        staticFriction.Should().Be((Fixed64)9);
        dynamicFriction.Should().Be((Fixed64)3);
    }

    [Fact]
    public void CombineRestitution_ShouldUseDominantRestitutionPolicy()
    {
        var minimum = new PhysicsMaterial(Fixed64.One, Fixed64.One, Fixed64.FromFraction(1, 4), restitutionCombine: PhysicsMaterialCombine.Minimum);
        var maximum = new PhysicsMaterial(Fixed64.One, Fixed64.One, Fixed64.FromFraction(3, 4), restitutionCombine: PhysicsMaterialCombine.Maximum);

        PhysicsMaterial.CombineRestitution(minimum, maximum)
            .Should()
            .Be(Fixed64.FromFraction(3, 4));
    }

    [Fact]
    public void EqualValues_ShouldCompareEqual()
    {
        var left = new PhysicsMaterial(Fixed64.One, Fixed64.Half, Fixed64.FromFraction(1, 4));
        var right = new PhysicsMaterial(Fixed64.One, Fixed64.Half, Fixed64.FromFraction(1, 4));

        left.Should().Be(right);
        (left == right).Should().BeTrue();
        left.GetHashCode().Should().Be(right.GetHashCode());
    }
}
