using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using System;
using Xunit;

namespace Gravitas.Tests;

public sealed class MeshSurfaceBoundaryFanTests
{
    [Fact]
    public void PartialJoinEndpoint_ShouldCombineTheSourceInteriorAndNeighborCorner()
    {
        PhysicsMesh[] meshes = { Quad(-2, 0, -2, 2), Quad(0, 2, 0, 2) };
        var domain = new MeshSurfaceBoundaryDomain(); domain.Build(meshes, 0, 0);
        var fan = new MeshSurfaceBoundaryFan();
        Assert.True(fan.BuildEndpoint(meshes, domain, JoinedSpan(domain), false));
        Assert.Equal(5, fan.RayCount);
        Assert.True(fan.Supports(Vector3d.Up));
        Assert.True(fan.Supports(Vector3d.Down));
        Assert.False(fan.Supports(Vector3d.Right));
        Assert.False(fan.Supports(Vector3d.Left));
        Assert.False(fan.Supports(Vector3d.Forward));
        Assert.False(fan.Supports(-Vector3d.Forward));
    }

    [Fact]
    public void PointOnlyNeighbors_ShouldRetainSeparateCornerFans()
    {
        PhysicsMesh[] meshes = { Quad(-2, 0, -2, 2), Quad(0, 2, 2, 4) };
        var fan = new MeshSurfaceBoundaryFan();
        Assert.True(fan.BuildAtPoint(meshes, 0, 0, new Vector3d(1, 0, 2)));
        Assert.Equal(2, fan.RayCount);
        Assert.True(fan.Supports(Vector3d.Right));
        Assert.False(fan.Supports(Vector3d.Left));
    }

    [Fact]
    public void AFaceInterior_ShouldHaveOnlyPlaneNormalSupport()
    {
        var fan = new MeshSurfaceBoundaryFan();
        Assert.True(fan.BuildAtPoint(new[] { Quad(-2, 2, -2, 2) }, 0, 0, Vector3d.Zero));
        Assert.Equal(4, fan.RayCount);
        Assert.True(fan.Supports(Vector3d.Up));
        Assert.False(fan.Supports(Vector3d.Right));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SharedOuterEndpoint_ShouldPreserveTrueRimSupport(bool upper)
    {
        PhysicsMesh[] meshes = { Quad(-2, 0, -2, 2), Quad(0, 2, -2, 2) };
        var domain = new MeshSurfaceBoundaryDomain(); domain.Build(meshes, 0, 0);
        var fan = new MeshSurfaceBoundaryFan();
        Assert.True(fan.BuildEndpoint(meshes, domain, JoinedSpan(domain), upper));
        Assert.Equal(4, fan.RayCount);
        Assert.True(fan.Supports(Vector3d.Up));
        Assert.True(fan.Supports(upper ? Vector3d.Forward : -Vector3d.Forward));
        Assert.False(fan.Supports(upper ? -Vector3d.Forward : Vector3d.Forward));
        Assert.False(fan.Supports(Vector3d.Right));
        Assert.False(fan.Supports(Vector3d.Left));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void IndependentHalfTurnFrames_ShouldRetainExactFanAtWorldLimits(bool limits)
    {
        PhysicsMesh left = Quad(-2, 0, -2, 2), right = Quad(0, 2, -2, 2);
        Vector3d origin = limits ? new(Fixed64.MaxValue - (Fixed64)8, Fixed64.MinValue + (Fixed64)8,
            Fixed64.MaxValue - (Fixed64)8) : Vector3d.Zero;
        left.UpdatePosition(origin + new Vector3d(-1, 0, 0), FixedQuaternion.Identity);
        right.UpdatePosition(origin + new Vector3d(1, 0, 0), new FixedQuaternion(Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.Zero));
        var fan = new MeshSurfaceBoundaryFan();
        Assert.True(fan.BuildAtPoint(new[] { left, right }, 0, 0, Vector3d.Right));
        Assert.Equal(6, fan.RayCount);
        Assert.True(fan.Supports(Vector3d.Up));
        Assert.False(fan.Supports(Vector3d.Right));
        Assert.False(fan.Supports(Vector3d.Left));
        Assert.False(fan.Supports(Vector3d.Forward));
    }

    [Fact]
    public void AmbiguousIncident_ShouldDeclineAndClearThePreviousFan()
    {
        PhysicsMesh[] joined = { Quad(-2, 0, -2, 2), Quad(0, 2, -2, 2) };
        PhysicsMesh[] ambiguous = { joined[0], joined[1], Quad(0, 2, -2, 2) };
        var fan = new MeshSurfaceBoundaryFan();
        Assert.False(fan.Supports(Vector3d.Up));
        Assert.True(fan.BuildAtPoint(joined, 0, 0, Vector3d.Right));
        Assert.False(fan.BuildAtPoint(ambiguous, 0, 0, Vector3d.Right));
        Assert.Equal(0, fan.RayCount);
        Assert.False(fan.Supports(Vector3d.Up));
        Assert.True(fan.BuildAtPoint(joined, 0, 0, Vector3d.Right));
        Assert.True(fan.Supports(Vector3d.Up));
    }

    [Fact]
    public void IncidentTraversal_ShouldVisitAllFourCornersOnceAndReuseStorage()
    {
        PhysicsMesh[] meshes = { Quad(-2, 0, -2, 0), Quad(0, 2, -2, 0), Quad(0, 2, 0, 2), Quad(-2, 0, 0, 2) };
        var fan = new MeshSurfaceBoundaryFan();
        Vector3d point = new(1, 0, 1);
        for (int i = 0; i < 16; i++) Assert.True(fan.BuildAtPoint(meshes, 0, 0, point));
        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 16; i++) fan.BuildAtPoint(meshes, 0, 0, point);
        Assert.Equal(0, GC.GetAllocatedBytesForCurrentThread() - before);
        Assert.Equal(8, fan.RayCount);
        Assert.True(fan.Supports(Vector3d.Up));
        Assert.False(fan.Supports(Vector3d.Right));
        Assert.False(fan.Supports(Vector3d.Forward));
        Assert.False(fan.Supports(Vector3d.Left));
        Assert.False(fan.Supports(-Vector3d.Forward));
    }

    private static int JoinedSpan(MeshSurfaceBoundaryDomain domain)
    {
        for (int span = 0; span < domain.Spans.Length; span++)
            if (domain.Spans[span].Kind == MeshSurfaceBoundaryDomain.SpanKind.Joined) return span;
        throw new Xunit.Sdk.XunitException("Fixture has no certified join.");
    }

    private static PhysicsMesh Quad(int left, int right, int bottom, int top) =>
        new(new[] { new Vector3d(left, 0, bottom), new(right, 0, bottom), new(right, 0, top), new(left, 0, top) },
            new[] { 0, 2, 1, 0, 3, 2 }, new Vector3d((Fixed64)(left + right) / 2, Fixed64.Zero, (Fixed64)(bottom + top) / 2), FixedQuaternion.Identity, MeshColliderMode.Concave);
}
