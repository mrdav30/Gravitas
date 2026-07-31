using FixedMathSharp;
using FluentAssertions;
using Gravitas.CollisionHandling;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class RadialSweepAdmissionTests
{
    [Fact]
    public void NegativeParameterWithoutExactRoot_ShouldBeRejected()
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
        parameter2D.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void UnrepresentableEndpointDistanceWithoutExactRoot_ShouldBeRejected()
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
    }
}
