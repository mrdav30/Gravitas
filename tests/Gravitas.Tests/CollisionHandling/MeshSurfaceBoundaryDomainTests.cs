using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests;

public sealed class MeshSurfaceBoundaryDomainTests
{
    [Fact]
    public void PartialJoin_ShouldRetainOriginalEdgeAndExactExposedIntervals()
    {
        PhysicsMesh source = Triangle(new(0, 0, 0), new(4, 0, 0), new(0, 2, 0));
        PhysicsMesh other = Triangle(new(1, 0, 0), new(3, 0, 0), new(1, -1, 0));
        var domain = new MeshSurfaceBoundaryDomain();
        domain.Build(new[] { source, other }, 0, 0);
        int exposed = 0, joined = 0;
        foreach (var span in domain.Spans)
        {
            if (span.First != 0 || span.Second != 1) continue;
            Assert.Equal(source.GeometryVertices[0], span.Start);
            Assert.Equal(source.GeometryVertices[1], span.End);
            if (span.Kind == MeshSurfaceBoundaryDomain.SpanKind.Joined)
            {
                Assert.Equal(1, span.OtherMember);
                Assert.Equal(0, span.OtherSurface);
                joined++;
            }
            else exposed++;
        }
        Assert.Equal(1, joined);
        Assert.Equal(2, exposed);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ExtraCoverage_ShouldRemainAmbiguousIncludingFilledInterior(bool interior)
    {
        PhysicsMesh source = Triangle(new(0, 0, 0), new(4, 0, 0), new(0, 2, 0));
        PhysicsMesh other = Triangle(new(0, 0, 0), new(4, 0, 0), new(0, -2, 0));
        PhysicsMesh third = interior
            ? Triangle(new(-1, -1, 0), new(5, -1, 0), new(2, 3, 0))
            : other;
        var domain = new MeshSurfaceBoundaryDomain();
        domain.Build(new[] { source, other, third }, 0, 0);
        int covered = 0;
        foreach (var span in domain.Spans)
        {
            if (span.First != 0 || span.Second != 1) continue;
            Assert.Equal(MeshSurfaceBoundaryDomain.SpanKind.Ambiguous, span.Kind);
            covered++;
        }
        Assert.True(covered > 0);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FoldedIncident_ShouldRetainItsCoveredSpanEvenBesideAValidJoin(bool thirdSheet)
    {
        PhysicsMesh source = Triangle(new(0, 0, 0), new(4, 0, 0), new(0, 2, 0));
        PhysicsMesh folded = Triangle(new(1, 0, 0), new(3, 0, 0), new(1, 0, 2));
        PhysicsMesh opposite = Triangle(new(0, 0, 0), new(4, 0, 0), new(0, -2, 0));
        var domain = new MeshSurfaceBoundaryDomain();
        domain.Build(thirdSheet ? new[] { source, opposite, folded } : new[] { source, folded }, 0, 0);
        int ambiguous = 0, outside = 0;
        foreach (var span in domain.Spans)
        {
            if (span.First != 0 || span.Second != 1) continue;
            if (span.Kind == MeshSurfaceBoundaryDomain.SpanKind.Ambiguous) ambiguous++;
            else
            {
                Assert.Equal(thirdSheet ? MeshSurfaceBoundaryDomain.SpanKind.Joined
                    : MeshSurfaceBoundaryDomain.SpanKind.Exposed, span.Kind);
                outside++;
            }
        }
        Assert.Equal(1, ambiguous);
        Assert.Equal(2, outside);
    }

    [Theory]
    [InlineData(0)] // Point contact.
    [InlineData(1)] // One-raw gap along the plane normal.
    [InlineData(2)] // Disconnected in the same plane.
    public void NonCoveringMembers_ShouldLeaveOriginalCanonicalEdges(int variant)
    {
        PhysicsMesh source = Triangle(new(0, 0, 0), new(4, 0, 0), new(0, 2, 0));
        Fixed64 raw = Fixed64.FromRaw(1);
        PhysicsMesh other = variant == 0 ? Triangle(new(4, 0, 0), new(6, 0, 0), new(4, -1, 0))
            : variant == 1 ? Triangle(new(Fixed64.One, Fixed64.Zero, raw), new((Fixed64)3, Fixed64.Zero, raw), new(Fixed64.One, -Fixed64.One, raw))
            : Triangle(new(8, 0, 0), new(10, 0, 0), new(8, -1, 0));
        var domain = new MeshSurfaceBoundaryDomain();
        domain.Build(new[] { source, other }, 0, 0);
        Assert.Equal(3, domain.Spans.Length);
        foreach (var span in domain.Spans) Assert.Equal(MeshSurfaceBoundaryDomain.SpanKind.Exposed, span.Kind);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void CommittedColliderCentering_ShouldPreserveExactJoinAcrossIndependentFrames(bool limits, bool swapped)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        Vector3d origin = limits ? new(Fixed64.MaxValue - (Fixed64)16, Fixed64.MinValue + (Fixed64)16,
            Fixed64.MaxValue - (Fixed64)16) : Vector3d.Zero;
        var first = new LSMeshCollider(new[] { new Vector3d(0, 0, 0), new(4, 0, 0), new(0, 2, 0) },
            new[] { 0, 1, 2 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        var second = new LSMeshCollider(new[] { new Vector3d(0, 0, -1), new(0, -2, -1), new(-1, 0, -1) },
            new[] { 0, 1, 2 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        FixedQuaternion quarter = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.One).Normalized;
        first.InitializeWithNoBody(new TestMatterAgent(context, new FixedTransform(origin, FixedQuaternion.Identity, Vector3d.One)));
        second.InitializeWithNoBody(new TestMatterAgent(context, new FixedTransform(origin + new Vector3d(1, 0, 1), quarter, Vector3d.One)));
        PhysicsMesh[] members = swapped ? new[] { second.Mesh, first.Mesh } : new[] { first.Mesh, second.Mesh };
        var domain = new MeshSurfaceBoundaryDomain();
        domain.Build(members, 0, 0);
        Assert.Equal(0, domain.SourceMember);
        Assert.Equal(0, domain.SourceSurface);
        Assert.Equal(members[0].Origin, domain.Origin);
        Assert.Equal(members[0].Rotation, domain.Rotation);
        int joined = 0;
        foreach (var span in domain.Spans)
        {
            Assert.NotEqual(MeshSurfaceBoundaryDomain.SpanKind.Ambiguous, span.Kind);
            if (span.Kind != MeshSurfaceBoundaryDomain.SpanKind.Joined) continue;
            Assert.Equal(1, span.OtherMember);
            joined++;
        }
        Assert.Equal(1, joined);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void AuthoredSubdivision_ShouldNotCreateArtificialCanonicalEventEndpoints(bool opposite)
    {
        Vector3d[] vertices = { new(-2, 0, -1), new(2, 0, -1), new(2, 0, 1), new(-2, 0, 1),
            new(0, 0, -1), Vector3d.Zero };
        var patch = new PhysicsMesh(vertices, new[] { 5, 0, 4, 5, 4, 1, 5, 1, 2, 5, 2, 3, 5, 3, 0 },
            Vector3d.Zero, FixedQuaternion.Identity, MeshColliderMode.Concave);
        PhysicsMesh neighbor = opposite ? Triangle(new(-2, 0, -1), new(2, 0, -1), new(0, 0, -3))
            : Triangle(new(-2, 0, -1), new(2, 0, -1), new(0, 0, 1));
        var domain = new MeshSurfaceBoundaryDomain();
        domain.Build(new[] { patch, neighbor }, 0, 2);
        Assert.Equal(4, domain.Spans.Length);
        int shared = 0;
        foreach (var span in domain.Spans)
        {
            if (span.Start.Z != (Fixed64)(-1) || span.End.Z != (Fixed64)(-1)) continue;
            Assert.Equal(opposite ? MeshSurfaceBoundaryDomain.SpanKind.Joined
                : MeshSurfaceBoundaryDomain.SpanKind.Ambiguous, span.Kind);
            shared++;
        }
        Assert.Equal(1, shared);
    }

    [Fact]
    public void PreparedDomain_ShouldReuseStorageAfterMembershipAndOwnershipChanges()
    {
        PhysicsMesh source = Triangle(new(0, 0, 0), new(4, 0, 0), new(0, 2, 0));
        PhysicsMesh other = Triangle(new(1, 0, 0), new(3, 0, 0), new(1, -1, 0));
        PhysicsMesh[] members = { source, other };
        var domain = new MeshSurfaceBoundaryDomain();
        for (int i = 0; i < 32; i++) { domain.Build(members, 0, 0); domain.Build(members, 1, 0); }
        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 32; i++) { domain.Build(members, 0, 0); domain.Build(members, 1, 0); }
        Assert.Equal(0, GC.GetAllocatedBytesForCurrentThread() - before);
        domain.Build(members.AsSpan(0, 1), 0, 0);
        Assert.Equal(3, domain.Spans.Length);
        foreach (var span in domain.Spans)
        {
            Assert.Equal(MeshSurfaceBoundaryDomain.SpanKind.Exposed, span.Kind);
            Assert.Equal(-1, span.OtherMember);
            Assert.Equal(-1, span.OtherSurface);
        }
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void DeclinedSameMemberWinding_ShouldNotAcquireAJoinedSurfaceCertificate(int axis)
    {
        Vector3d P(int x, int y) => axis == 0 ? new(0, x, y) : axis == 1 ? new(x, 0, y) : new(x, y, 0);
        var mesh = new PhysicsMesh(new[] { P(0, 0), P(4, 0), P(0, 2), P(0, -2), P(4, -1) },
            new[] { 0, 1, 2, 0, 1, 3, 0, 2, 4 }, Vector3d.Zero, FixedQuaternion.Identity, MeshColliderMode.Concave);
        Assert.NotEqual(mesh.GetManifoldSurfaceOwner(0), mesh.GetManifoldSurfaceOwner(1));
        var domain = new MeshSurfaceBoundaryDomain();
        domain.Build(new[] { mesh }, 0, 0);
        int ambiguous = 0;
        foreach (var span in domain.Spans)
        {
            Assert.NotEqual(MeshSurfaceBoundaryDomain.SpanKind.Joined, span.Kind);
            if (span.Kind == MeshSurfaceBoundaryDomain.SpanKind.Ambiguous) ambiguous++;
        }
        Assert.Equal(2, ambiguous);
    }

    [Theory]
    [InlineData(0)] // One trusted patch with an authored edge midpoint.
    [InlineData(1)] // Two separate members meeting at the midpoint.
    [InlineData(2)] // Two distinct owners in the same member.
    public void JoinedSubdivision_ShouldCoalesceOnlyCompatibleSurfaceProvenance(int variant)
    {
        PhysicsMesh source = Triangle(new(0, 0, 0), new(4, 0, 0), new(0, 2, 0));
        PhysicsMesh[] members;
        if (variant == 0)
        {
            var patch = new PhysicsMesh(new[] { new Vector3d(0, 0, 0), new(4, 0, 0), new(4, -2, 0),
                new(0, -2, 0), new(2, 0, 0), new(2, -1, 0) },
                new[] { 5, 0, 4, 5, 4, 1, 5, 1, 2, 5, 2, 3, 5, 3, 0 },
                new Vector3d(2, -1, 0), FixedQuaternion.Identity, MeshColliderMode.Concave);
            members = new[] { source, patch };
        }
        else if (variant == 1)
            members = new[] { source, Triangle(new(0, 0, 0), new(2, 0, 0), new(0, -2, 0)),
                Triangle(new(2, 0, 0), new(4, 0, 0), new(2, -2, 0)) };
        else
            members = new[] { source, new PhysicsMesh(new[] { new Vector3d(0, 0, 0), new(2, 0, 0), new(0, -2, 0),
                new(4, 0, 0), new(2, -2, 0) }, new[] { 0, 1, 2, 1, 3, 4 }, new Vector3d(2, -1, 0),
                FixedQuaternion.Identity, MeshColliderMode.Concave) };
        var domain = new MeshSurfaceBoundaryDomain();
        domain.Build(members, 0, 0);
        int joined = 0;
        foreach (var span in domain.Spans)
        {
            Assert.NotEqual(MeshSurfaceBoundaryDomain.SpanKind.Ambiguous, span.Kind);
            if (span.Kind == MeshSurfaceBoundaryDomain.SpanKind.Joined) joined++;
        }
        Assert.Equal(variant == 0 ? 1 : 2, joined);
    }

    private static PhysicsMesh Triangle(Vector3d a, Vector3d b, Vector3d c) =>
        new(new[] { a, b, c }, new[] { 0, 1, 2 },
            (Vector3d.Min(Vector3d.Min(a, b), c) + Vector3d.Max(Vector3d.Max(a, b), c)) * Fixed64.Half,
            FixedQuaternion.Identity, MeshColliderMode.Concave);

}
