using FixedMathSharp;
using FluentAssertions;
using Xunit;

namespace Gravitas.Tests.Core;

public sealed class InertiaTensorMathTests
{
    [Fact]
    public void InvertForSolver_WithFullTensor_ShouldPreserveProductsOfInertia()
    {
        var tensor = new Fixed3x3(
            (Fixed64)4, Fixed64.One, Fixed64.Zero,
            Fixed64.One, (Fixed64)3, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, (Fixed64)2);

        Fixed3x3 inverse = InertiaTensorMath.InvertForSolver(tensor);
        Fixed3x3 product = tensor * inverse;

        AssertIdentity(product);
        inverse.M12.Should().BeLessThan(Fixed64.Zero);
        inverse.M21.Should().Be(inverse.M12);
    }

    [Fact]
    public void InvertForSolver_WithDiagonalTensor_ShouldUseDiagonalFastPath()
    {
        var tensor = new Fixed3x3(
            (Fixed64)2, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, (Fixed64)4, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, (Fixed64)8);

        Fixed3x3 inverse = InertiaTensorMath.InvertForSolver(tensor);

        inverse.Should().Be(new Fixed3x3(
            Fixed64.Half, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.FromFraction(1, 4), Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.FromFraction(1, 8)));
    }

    [Fact]
    public void InvertForSolver_WithSingularTensor_ShouldReturnZeroTensor()
    {
        var tensor = new Fixed3x3(
            Fixed64.One, Fixed64.One, Fixed64.Zero,
            Fixed64.One, Fixed64.One, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.One);

        InertiaTensorMath.InvertForSolver(tensor).Should().Be(Fixed3x3.Zero);
    }

    [Fact]
    public void AddParallelAxisTensor_ShouldIncludeOffDiagonalProducts()
    {
        Fixed3x3 shifted = InertiaTensorMath.AddParallelAxisTensor(
            Fixed3x3.Zero,
            (Fixed64)2,
            new Vector3d(Fixed64.One, (Fixed64)2, (Fixed64)3));

        shifted.M11.Should().Be((Fixed64)26);
        shifted.M22.Should().Be((Fixed64)20);
        shifted.M33.Should().Be((Fixed64)10);
        shifted.M12.Should().Be((Fixed64)(-4));
        shifted.M21.Should().Be(shifted.M12);
        shifted.M13.Should().Be((Fixed64)(-6));
        shifted.M31.Should().Be(shifted.M13);
        shifted.M23.Should().Be((Fixed64)(-12));
        shifted.M32.Should().Be(shifted.M23);
    }

    [Fact]
    public void AddParallelAxisTensor_WithNoEffectiveShift_ShouldPreserveTensor()
    {
        Fixed3x3 tensor = Fixed3x3.Identity;

        InertiaTensorMath.AddParallelAxisTensor(
            tensor,
            Fixed64.Zero,
            Vector3d.One).Should().Be(tensor);
        InertiaTensorMath.AddParallelAxisTensor(
            tensor,
            Fixed64.One,
            Vector3d.Zero).Should().Be(tensor);
    }

    [Fact]
    public void InvertForSolver_WithNonPositiveDiagonalAxes_ShouldFreezeThoseAxes()
    {
        var tensor = new Fixed3x3(
            Fixed64.One, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, -Fixed64.One);

        InertiaTensorMath.InvertForSolver(tensor).Should().Be(new Fixed3x3(
            Fixed64.One, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero));
    }

    [Fact]
    public void SubtractParallelAxisTensor_WithZeroMass_ShouldPreserveTensor()
    {
        Fixed3x3 tensor = Fixed3x3.Identity;

        InertiaTensorMath.SubtractParallelAxisTensor(
            tensor,
            Fixed64.Zero,
            Vector3d.One).Should().Be(tensor);
    }

    private static void AssertIdentity(Fixed3x3 matrix)
    {
        AssertNear(matrix.M11, Fixed64.One);
        AssertNear(matrix.M22, Fixed64.One);
        AssertNear(matrix.M33, Fixed64.One);
        AssertNear(matrix.M12, Fixed64.Zero);
        AssertNear(matrix.M13, Fixed64.Zero);
        AssertNear(matrix.M21, Fixed64.Zero);
        AssertNear(matrix.M23, Fixed64.Zero);
        AssertNear(matrix.M31, Fixed64.Zero);
        AssertNear(matrix.M32, Fixed64.Zero);
    }

    private static void AssertNear(Fixed64 actual, Fixed64 expected) =>
        (actual - expected).Abs().Should().BeLessThan(Fixed64.FromFraction(1, 1000));
}
