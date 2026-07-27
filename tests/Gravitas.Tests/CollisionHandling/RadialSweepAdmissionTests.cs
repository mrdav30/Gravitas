using FixedMathSharp;
using FluentAssertions;
using Gravitas.CollisionHandling;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class RadialSweepAdmissionTests
{
    [Fact]
    public void NegativeParameterWithoutExactRoot_ShouldBeRejectedInBothDimensions()
    {
        RadialSweepAdmission.TryIntersect(
                Vector2d.Zero,
                Vector2d.Zero,
                -Fixed64.One,
                Vector2d.Right * (Fixed64)4,
                Fixed64.One,
                Fixed64.Zero,
                Vector2d.Zero,
                Vector2d.Right * (Fixed64)4,
                out _)
            .Should()
            .BeFalse();
        RadialSweepAdmission.TryIntersect(
                Vector3d.Zero,
                Vector3d.Zero,
                -Fixed64.One,
                Vector3d.Right * (Fixed64)4,
                Fixed64.One,
                Fixed64.Zero,
                Vector3d.Zero,
                Vector3d.Right * (Fixed64)4,
                out _)
            .Should()
            .BeFalse();
    }

    [Fact]
    public void UnrepresentableCombinedEndpointRadius_ShouldRemainConservativelyAdmitted()
    {
        Vector2d distantStart2D = new(Fixed64.MinValue, Fixed64.MinValue);
        Vector2d distantTarget2D = new(Fixed64.MaxValue, Fixed64.MaxValue);
        RadialSweepAdmission.TryIntersect(
                distantStart2D,
                Vector2d.Zero,
                Fixed64.Zero,
                distantTarget2D,
                Fixed64.MaxValue,
                Fixed64.One,
                Vector2d.Zero,
                Vector2d.Zero,
                out Fixed64 parameter2D)
            .Should()
            .BeTrue();
        Vector3d distantStart3D = new(Fixed64.MinValue, Fixed64.MinValue, Fixed64.MinValue);
        Vector3d distantTarget3D = new(Fixed64.MaxValue, Fixed64.MaxValue, Fixed64.MaxValue);
        RadialSweepAdmission.TryIntersect(
                distantStart3D,
                Vector3d.Zero,
                Fixed64.Zero,
                distantTarget3D,
                Fixed64.MaxValue,
                Fixed64.One,
                Vector3d.Zero,
                Vector3d.Zero,
                out Fixed64 parameter3D)
            .Should()
            .BeTrue();
        parameter2D.Should().Be(Fixed64.Zero);
        parameter3D.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void UnrepresentableEndpointDistanceWithoutExactRoot_ShouldBeRejectedInBothDimensions()
    {
        RadialSweepAdmission.TryIntersect(
                Vector2d.Zero,
                Vector2d.Zero,
                Fixed64.One,
                Vector2d.Right,
                Fixed64.Zero,
                Fixed64.Zero,
                new Vector2d(Fixed64.MinValue, Fixed64.MinValue),
                new Vector2d(Fixed64.MaxValue, Fixed64.MaxValue),
                out _)
            .Should()
            .BeFalse();
        RadialSweepAdmission.TryIntersect(
                Vector3d.Zero,
                Vector3d.Zero,
                Fixed64.One,
                Vector3d.Right,
                Fixed64.Zero,
                Fixed64.Zero,
                new Vector3d(
                    Fixed64.MinValue,
                    Fixed64.MinValue,
                    Fixed64.MinValue),
                new Vector3d(
                    Fixed64.MaxValue,
                    Fixed64.MaxValue,
                    Fixed64.MaxValue),
                out _)
            .Should()
            .BeFalse();
    }
}
