using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.CollisionHandling;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class ExactLever3DTests
{
    [Fact]
    public void RelativePointVelocityProjection_ShouldUseExactLeverCoordinates()
    {
        ExactLever3D lever = CreateLever(new Vector3d(4, -3, 2));

        Assert.True(ExactLever3D.TryGetRelativePointVelocityProjection(
            new Vector3d(3, -1, 4),
            new Vector3d(1, 2, -1),
            lever,
            new Vector3d(-2, 5, 1),
            new Vector3d(-1, 1, 3),
            lever,
            Vector3d.Up,
            out Fixed64 projection));
        Assert.Equal((Fixed64)26, projection);
    }

    [Fact]
    public void RelativePointVelocityProjection_ShouldPairDifferentAnchorFrames()
    {
        var origin = new FixedPointAnchor(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Zero);
        var firstPoint = new FixedPointAnchor(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(4, -3, 2));
        var secondPoint = new FixedPointAnchor(
            Vector3d.Zero,
            FixedQuaternion.FromAxisAngle(
                Vector3d.Up,
                Fixed64.PiOver4),
            new Vector3d(1, 2, 3));
        ExactLever3D firstLever = ExactLever3D.Create(firstPoint, origin);
        ExactLever3D secondLever = ExactLever3D.Create(secondPoint, origin);
        Assert.True(firstPoint.TryGetOffsetFrom(origin, out Vector3d firstOffset));
        Assert.True(secondPoint.TryGetOffsetFrom(origin, out Vector3d secondOffset));
        Vector3d firstLinearVelocity = new(3, -1, 4);
        Vector3d firstAngularVelocity = new(1, 2, -1);
        Vector3d secondLinearVelocity = new(-2, 5, 1);
        Vector3d secondAngularVelocity = new(-1, 1, 3);

        Assert.True(ExactLever3D.TryGetRelativePointVelocityProjection(
            firstLinearVelocity,
            firstAngularVelocity,
            firstLever,
            secondLinearVelocity,
            secondAngularVelocity,
            secondLever,
            Vector3d.Up,
            out Fixed64 projection));
        Assert.Equal(
            Vector3d.Dot(
                secondLinearVelocity
                + Vector3d.Cross(secondAngularVelocity, secondOffset)
                - firstLinearVelocity
                - Vector3d.Cross(firstAngularVelocity, firstOffset),
                Vector3d.Up),
            projection);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void RelativePointVelocityProjection_ShouldRejectUninitializedParticipantAtomically(
        bool firstIsUninitialized)
    {
        ExactLever3D valid = CreateLever(Vector3d.Zero);

        Assert.False(ExactLever3D.TryGetRelativePointVelocityProjection(
            Vector3d.Zero,
            Vector3d.Zero,
            firstIsUninitialized ? default : valid,
            Vector3d.Zero,
            Vector3d.Zero,
            firstIsUninitialized ? valid : default,
            Vector3d.Right,
            out Fixed64 projection));
        Assert.Equal(default, projection);
    }

    private static ExactLever3D CreateLever(Vector3d point)
    {
        var pointAnchor = new FixedPointAnchor(
            point,
            FixedQuaternion.Identity,
            Vector3d.Zero);
        var origin = new FixedPointAnchor(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Zero);
        return ExactLever3D.Create(pointAnchor, origin);
    }
}
