using FixedMathSharp;
using FluentAssertions;
using Gravitas.Support;
using Xunit;

namespace Gravitas.Tests.Support;

public sealed class FixedTransformTests
{
    [Fact]
    public void LossyScale_ShouldPreserveScaleWhenTransformIsRotated()
    {
        var transform = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)90),
            new Vector3d((Fixed64)2, (Fixed64)3, (Fixed64)4));

        Vector3d scale = transform.LossyScale;
        Fixed64 tolerance = Fixed64.Fraction(1, 1_000_000);
        (scale.x - (Fixed64)2).Abs().Should().BeLessThan(tolerance);
        (scale.y - (Fixed64)3).Abs().Should().BeLessThan(tolerance);
        (scale.z - (Fixed64)4).Abs().Should().BeLessThan(tolerance);
    }
}
