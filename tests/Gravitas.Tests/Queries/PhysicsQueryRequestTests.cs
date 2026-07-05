using FixedMathSharp;
using FluentAssertions;
using Gravitas.Queries;
using Gravitas.Support;
using Xunit;

namespace Gravitas.Tests.Queries;

public sealed class PhysicsQueryRequestTests
{
    [Fact]
    public void PhysicsQueryHitRange_ShouldExposeExclusiveEndAndHitState()
    {
        var occupied = new PhysicsQueryHitRange(4, 3);
        var empty = new PhysicsQueryHitRange(7, 0);

        occupied.Start.Should().Be(4);
        occupied.Count.Should().Be(3);
        occupied.End.Should().Be(7);
        occupied.HasHits.Should().BeTrue();

        empty.Start.Should().Be(7);
        empty.Count.Should().Be(0);
        empty.End.Should().Be(7);
        empty.HasHits.Should().BeFalse();
    }

    [Fact]
    public void Pure2DRequests_ShouldDefaultToAllLayersAndNoExclusions()
    {
        Vector2d start = new(Fixed64.One, (Fixed64)2);
        Vector2d end = new((Fixed64)3, (Fixed64)4);
        Vector2d size = new((Fixed64)5, (Fixed64)6);

        var raycast = new PhysicsRaycast2DRequest(start, end);
        var circle = new PhysicsOverlapCircle2DRequest(start, Fixed64.Half);
        var aabb = new PhysicsOverlapAabb2DRequest(start, size);
        var polygon = new PhysicsOverlapPolygon2DRequest(8, 4);
        var sweep = new PhysicsSweepCircle2DRequest(start, end, Fixed64.One);

        raycast.Start.Should().Be(start);
        raycast.End.Should().Be(end);
        raycast.LayerMask.Should().Be(PhysicsLayerMask.All);

        circle.Center.Should().Be(start);
        circle.Radius.Should().Be(Fixed64.Half);
        circle.LayerMask.Should().Be(PhysicsLayerMask.All);

        aabb.Center.Should().Be(start);
        aabb.Size.Should().Be(size);
        aabb.LayerMask.Should().Be(PhysicsLayerMask.All);

        polygon.VertexStart.Should().Be(8);
        polygon.VertexCount.Should().Be(4);
        polygon.LayerMask.Should().Be(PhysicsLayerMask.All);

        sweep.Start.Should().Be(start);
        sweep.End.Should().Be(end);
        sweep.Radius.Should().Be(Fixed64.One);
        sweep.LayerMask.Should().Be(PhysicsLayerMask.All);
        sweep.ExcludedCollider.Should().BeNull();
        sweep.IncludeTriggers.Should().BeTrue();
    }

    [Fact]
    public void Pure3DRequests_ShouldDefaultToAllLayersAndNoExclusions()
    {
        Vector3d start = new(Fixed64.One, (Fixed64)2, (Fixed64)3);
        Vector3d end = new((Fixed64)4, (Fixed64)5, (Fixed64)6);
        Vector3d direction = new(Fixed64.Zero, Fixed64.Zero, Fixed64.One);

        var raycast = new PhysicsRaycast3DRequest(start, end);
        var sweepSphere = new PhysicsSweepSphere3DRequest(start, end, Fixed64.Half);
        var circle = new PhysicsOverlapCircle3DRequest(start, Fixed64.One);
        var directionalCircle = new PhysicsOverlapCircleInDirection3DRequest(start, Fixed64.One, direction, (Fixed64)8);
        var cone = new PhysicsOverlapCone3DRequest(start, direction, (Fixed64)9, (Fixed64)3);

        raycast.Start.Should().Be(start);
        raycast.End.Should().Be(end);
        raycast.LayerMask.Should().Be(PhysicsLayerMask.All);

        sweepSphere.Start.Should().Be(start);
        sweepSphere.End.Should().Be(end);
        sweepSphere.Radius.Should().Be(Fixed64.Half);
        sweepSphere.LayerMask.Should().Be(PhysicsLayerMask.All);
        sweepSphere.ExcludedCollider.Should().BeNull();

        circle.Position.Should().Be(start);
        circle.Radius.Should().Be(Fixed64.One);
        circle.LayerMask.Should().Be(PhysicsLayerMask.All);

        directionalCircle.Position.Should().Be(start);
        directionalCircle.Radius.Should().Be(Fixed64.One);
        directionalCircle.Direction.Should().Be(direction);
        directionalCircle.MaxDistance.Should().Be((Fixed64)8);
        directionalCircle.LayerMask.Should().Be(PhysicsLayerMask.All);

        cone.Origin.Should().Be(start);
        cone.Direction.Should().Be(direction);
        cone.Length.Should().Be((Fixed64)9);
        cone.EndRadius.Should().Be((Fixed64)3);
        cone.LayerMask.Should().Be(PhysicsLayerMask.All);
    }

    [Fact]
    public void MixedRequests_ShouldDefaultToAllLayersNoExclusionsAndIncludeTriggers()
    {
        Vector3d sphereStart = new(Fixed64.Zero, Fixed64.One, Fixed64.Zero);
        Vector3d sphereEnd = new((Fixed64)5, Fixed64.One, Fixed64.Zero);
        Vector2d circleStart = new(Fixed64.Zero, Fixed64.Zero);
        Vector2d circleEnd = new((Fixed64)5, Fixed64.Zero);

        var sphereAgainst2D = new PhysicsSweepSphereAgainst2DRequest(sphereStart, sphereEnd, Fixed64.Half);
        var circleAgainst3D = new PhysicsSweepCircleAgainst3DRequest(
            circleStart,
            circleEnd,
            Fixed64.One,
            (Fixed64)2,
            Fixed64.Half);

        sphereAgainst2D.Start.Should().Be(sphereStart);
        sphereAgainst2D.End.Should().Be(sphereEnd);
        sphereAgainst2D.Radius.Should().Be(Fixed64.Half);
        sphereAgainst2D.LayerMask.Should().Be(PhysicsLayerMask.All);
        sphereAgainst2D.ExcludedCollider.Should().BeNull();
        sphereAgainst2D.IncludeTriggers.Should().BeTrue();

        circleAgainst3D.Start.Should().Be(circleStart);
        circleAgainst3D.End.Should().Be(circleEnd);
        circleAgainst3D.Radius.Should().Be(Fixed64.One);
        circleAgainst3D.SlabCenterY.Should().Be((Fixed64)2);
        circleAgainst3D.HalfThickness.Should().Be(Fixed64.Half);
        circleAgainst3D.LayerMask.Should().Be(PhysicsLayerMask.All);
        circleAgainst3D.ExcludedCollider.Should().BeNull();
        circleAgainst3D.IncludeTriggers.Should().BeTrue();
    }
}
