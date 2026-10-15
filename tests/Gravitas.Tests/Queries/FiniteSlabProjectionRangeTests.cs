using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.Queries;

public sealed class FiniteSlabProjectionRangeTests
{
    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void CylinderAdmission_RejectsUnsupportedGeometryBeforeProjection(bool maximum, bool tall)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var transform = new FixedTransform(
            new Vector3d(maximum ? Fixed64.MaxValue - Fixed64.One : Fixed64.MinValue + Fixed64.One,
                Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)45, Fixed64.Zero, Fixed64.Zero), Vector3d.One);
        var cylinder = new LSCylinderCollider
        {
            Radius = tall ? Fixed64.One : Fixed64.MaxValue,
            Size = new Vector3d(Fixed64.One, tall ? Fixed64.MaxValue : Fixed64.Two, Fixed64.One)
        };
        Assert.Throws<ArgumentException>(() => cylinder.InitializeWithNoBody(new TestMatterAgent(context, transform)));
        Assert.Equal(-1, cylinder.Id);
        Assert.Equal(0, context.Physics.ColliderCount);
        var control = new LSCylinderCollider { Radius = Fixed64.One, Size = new Vector3d(2, 2, 2) };
        control.InitializeWithNoBody(new TestMatterAgent(context, transform));
        Assert.Equal(transform.WorldPosition, control.Center);
    }
}
