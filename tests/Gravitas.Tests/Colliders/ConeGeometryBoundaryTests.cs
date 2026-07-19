using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class ConeGeometryBoundaryTests
{
    [Fact]
    public void CreateFiniteConeBounds_DegenerateAxis_ShouldUseStableUpFallback()
    {
        ConeGeometry.CreateFiniteConeBounds(
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Zero,
            Fixed64.Two,
            out Vector3d min,
            out Vector3d max);

        min.Should().Be(new Vector3d(-Fixed64.Two, Fixed64.Zero, -Fixed64.Two));
        max.Should().Be(new Vector3d(Fixed64.Two, Fixed64.Zero, Fixed64.Two));
    }
}
