using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests;

public sealed class MeshSurfaceBoundaryDomain2DTests
{
    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void PartialJoin_ShouldPreserveOriginalConvexEdgeAndMember(bool sourcePolygon, bool otherPolygon)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        LSCollider2D source = Piece(scene.Context, sourcePolygon, new(4, 2), new(0, 1));
        LSCollider2D other = Piece(scene.Context, otherPolygon, new(2, 2), new(0, -1));
        var domain = new MeshSurfaceBoundaryDomain2D();
        domain.Build(new[] { source, other }, 0);
        int exposed = 0, joined = 0;
        foreach (var span in domain.Spans)
        {
            if (span.First != 0 || span.Second != 1) continue;
            Assert.Equal(new Vector2d(-2, -1), span.Start);
            Assert.Equal(new Vector2d(2, -1), span.End);
            if (span.Kind == MeshSurfaceBoundaryDomain2D.SpanKind.Joined) { joined++; Assert.Equal(1, span.OtherMember); }
            else { exposed++; Assert.Equal(MeshSurfaceBoundaryDomain2D.SpanKind.Exposed, span.Kind); }
        }
        Assert.Equal(1, joined);
        Assert.Equal(2, exposed);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ExtraCoverage_ShouldStayAmbiguousEvenWhenItsInteriorCoversTheEdge(bool interior)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        LSCollider2D source = Piece(scene.Context, false, new(4, 2), new(0, 1));
        LSCollider2D other = Piece(scene.Context, false, new(4, 2), new(0, -1));
        LSCollider2D extra = Piece(scene.Context, true, interior ? new(6, 4) : new(4, 2), interior ? Vector2d.Zero : new(0, -1));
        var domain = new MeshSurfaceBoundaryDomain2D();
        domain.Build(new[] { source, other, extra }, 0);
        int ambiguous = 0;
        foreach (var span in domain.Spans)
            if (span.First == 0 && span.Second == 1) { Assert.Equal(MeshSurfaceBoundaryDomain2D.SpanKind.Ambiguous, span.Kind); ambiguous++; }
        Assert.True(ambiguous > 0);
    }

    [Fact]
    public void SubdividedOppositePieces_ShouldJoinTheirOwnSpansWithoutTreatingSharedCornerAsOverlap()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var source = Piece(scene.Context, false, new(4, 2), new(0, 1));
        var left = Piece(scene.Context, true, new(2, 2), new(-1, -1));
        var right = Piece(scene.Context, false, new(2, 2), new(1, -1));
        var domain = new MeshSurfaceBoundaryDomain2D();
        domain.Build(new[] { source, left, right }, 0);
        int count = 0;
        foreach (var span in domain.Spans)
            if (span.First == 0 && span.Second == 1) { Assert.Equal(MeshSurfaceBoundaryDomain2D.SpanKind.Joined, span.Kind); count++; }
        Assert.Equal(2, count);
    }

    [Theory]
    [InlineData(false, 0)]
    [InlineData(true, 0)]
    [InlineData(true, 1)]
    [InlineData(true, 2)]
    [InlineData(false, 1)]
    [InlineData(false, 2)]
    public void MixedProof_ShouldOnlyEraseAnEdgeWhenCommittedVerticalIntervalsAgree(bool mixed, int mismatch)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var source = Piece(scene.Context, false, new(4, 2), new(0, 1));
        var other = Piece(scene.Context, true, new(4, 2), new(0, -1),
            hostY: mismatch == 1 ? Fixed64.FromRaw(1) : Fixed64.Zero,
            halfThickness: mismatch == 2 ? Fixed64.Quarter : Fixed64.Half);
        var domain = new MeshSurfaceBoundaryDomain2D();
        domain.Build(new[] { source, other }, 0, mixed);
        int count = 0;
        foreach (var span in domain.Spans)
        {
            if (span.First != 0 || span.Second != 1) continue;
            Assert.Equal(mixed && mismatch != 0 ? MeshSurfaceBoundaryDomain2D.SpanKind.Exposed : MeshSurfaceBoundaryDomain2D.SpanKind.Joined, span.Kind);
            count++;
        }
        Assert.Equal(1, count);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ObliqueCommittedFrames_ShouldJoinUsingTheirDimensionalGeometry(bool mixed)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Fixed64 angle = FixedMath.DegToRad((Fixed64)37);
        var source = Polygon(scene.Context, new[] { new Vector2d(0, 0), new(4, 0), new(4, 2), new(0, 2) }, new(7, -3), angle);
        var other = Polygon(scene.Context, new[] { new Vector2d(1, 0), new(3, 0), new(3, -2), new(1, -2) }, new(7, -3), angle);
        var domain = new MeshSurfaceBoundaryDomain2D();
        domain.Build(new[] { source, other }, 0, mixed);
        Assert.Equal(source.Center, domain.Origin);
        Assert.Equal(source.ConvexRotation, domain.Rotation);
        Assert.Equal(0, domain.SourceMember);
        int joined = 0, exposed = 0;
        foreach (var span in domain.Spans)
            if (span.First == 0)
            {
                if (span.Kind == MeshSurfaceBoundaryDomain2D.SpanKind.Joined) { joined++; Assert.Equal(1, span.OtherMember); }
                else { exposed++; Assert.Equal(MeshSurfaceBoundaryDomain2D.SpanKind.Exposed, span.Kind); }
            }
        Assert.Equal(1, joined);
        Assert.Equal(2, exposed);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void IndependentlyTranslatedHalfTurn_ShouldKeepTheOriginalEdgeAndPartialJoin(bool mixed)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var source = Polygon(scene.Context, new[] { new Vector2d(0, 0), new(4, 0), new(4, 2), new(0, 2) }, new(7, -3), Fixed64.Zero);
        var other = Polygon(scene.Context, new[] { new Vector2d(0, 0), new(-2, 0), new(-2, 2), new(0, 2) }, new(8, -3), Fixed64.Pi);
        var domain = new MeshSurfaceBoundaryDomain2D();
        domain.Build(new[] { source, other }, 0, mixed);
        int joined = 0;
        foreach (var span in domain.Spans)
            if (span.First == 0 && span.Kind == MeshSurfaceBoundaryDomain2D.SpanKind.Joined)
            {
                joined++;
                Assert.Equal(new Vector2d(0, 0), span.Start);
                Assert.Equal(new Vector2d(4, 0), span.End);
                Assert.Equal(1, span.OtherMember);
            }
        Assert.Equal(1, joined);
    }

    [Fact]
    public void IndependentlyObliqueCommittedPlanarFrames_ShouldKeepAnExactlyAuthoredJoin()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Fixed64 firstAngle = FixedMath.DegToRad((Fixed64)37), secondAngle = FixedMath.DegToRad((Fixed64)(-23));
        Vector2d Rotate(Vector2d point, Fixed64 angle)
        {
            Assert.True(Vector2d.TryRotate(point, angle, out Vector2d rotated));
            return rotated;
        }
        // Authoritative scalar body poses avoid a quaternion-to-yaw capture
        // changing an authored angle. Integer-coordinate local maps remain
        // exact, so the two shared-S rotations commute without world rebaking.
        Vector2d[] firstVertices = { Vector2d.Zero, Rotate(new(4, 0), secondAngle), Rotate(new(4, 2), secondAngle), Rotate(new(0, 2), secondAngle) };
        Vector2d[] secondVertices = { Rotate(new(1, 0), firstAngle), Rotate(new(3, 0), firstAngle), Rotate(new(3, -2), firstAngle), Rotate(new(1, -2), firstAngle) };
        var source = new LSPolygonCollider2D(firstVertices);
        var other = new LSPolygonCollider2D(secondVertices);
        new SolidBody2D(new TestMatterAgent(scene.Context), source).Initialize(new(7, -3), firstAngle, BodyMotionType.Static);
        new SolidBody2D(new TestMatterAgent(scene.Context), other).Initialize(new(7, -3), secondAngle, BodyMotionType.Static);
        Assert.Equal(new Vector2d(7, -3), source.Center);
        Assert.Equal(source.Center, other.Center);
        Assert.Equal(firstAngle, source.ConvexRotation);
        Assert.Equal(secondAngle, other.ConvexRotation);
        Assert.NotEqual(firstAngle, secondAngle);
        Assert.Equal(firstVertices, source.ScaledLocalVertices.ToArray());
        Assert.Equal(secondVertices, other.ScaledLocalVertices.ToArray());
        var domain = new MeshSurfaceBoundaryDomain2D();
        domain.Build(new LSCollider2D[] { source, other }, 0);
        int joined = 0, exposed = 0;
        foreach (var span in domain.Spans)
            if (span.First == 0)
            {
                if (span.Kind == MeshSurfaceBoundaryDomain2D.SpanKind.Joined) joined++;
                else { exposed++; Assert.Equal(MeshSurfaceBoundaryDomain2D.SpanKind.Exposed, span.Kind); }
            }
        Assert.Equal(1, joined);
        Assert.Equal(2, exposed);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SharedCornerAndGap_ShouldRetainTheWholeExposedEdge(bool mixed)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var source = Piece(scene.Context, false, new(4, 2), new(0, 1));
        var corner = Piece(scene.Context, true, new(2, 2), new(3, -1));
        var gap = Piece(scene.Context, false, new(2, 2), new(0, -3));
        var domain = new MeshSurfaceBoundaryDomain2D();
        domain.Build(new[] { source, corner, gap }, 0, mixed);
        Assert.Equal(4, domain.Spans.Length);
        foreach (var span in domain.Spans) Assert.Equal(MeshSurfaceBoundaryDomain2D.SpanKind.Exposed, span.Kind);
    }

    [Fact]
    public void RebuildWithoutNeighbor_ShouldReplaceEarlierJoinedSpans()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var source = Piece(scene.Context, false, new(4, 2), new(0, 1));
        var other = Piece(scene.Context, true, new(2, 2), new(0, -1));
        var domain = new MeshSurfaceBoundaryDomain2D();
        domain.Build(new[] { source, other }, 0);
        domain.Build(new[] { source }, 0);
        Assert.Equal(4, domain.Spans.Length);
        foreach (var span in domain.Spans) Assert.Equal(MeshSurfaceBoundaryDomain2D.SpanKind.Exposed, span.Kind);
    }

    [Fact]
    public void OverlappingMixedPolygonFan_ShouldCoalesceAmbiguousFillAcrossItsInternalDiagonal()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var source = Piece(scene.Context, false, new(2, 8), new(-1, 0));
        var diamond = Polygon(scene.Context, new[] { new Vector2d(-2, 0), new(0, -2), new(2, 0), new(0, 2) },
            Vector2d.Zero, Fixed64.Zero);
        var domain = new MeshSurfaceBoundaryDomain2D();
        domain.Build(new[] { source, diamond }, 0, mixed: true);
        int pieces = 0, exposed = 0, ambiguous = 0;
        foreach (var span in domain.Spans)
        {
            if (span.First != 1) continue;
            pieces++;
            Assert.Equal(2, span.Second);
            Assert.Equal(new Vector2d(1, -4), span.Start);
            Assert.Equal(new Vector2d(1, 4), span.End);
            Assert.Equal(-1, span.OtherMember);
            if (span.Kind == MeshSurfaceBoundaryDomain2D.SpanKind.Exposed) exposed++;
            else { Assert.Equal(MeshSurfaceBoundaryDomain2D.SpanKind.Ambiguous, span.Kind); ambiguous++; }
        }
        // The diamond overlaps x=0 for z in[-2,2]. Its transient fan diagonal
        // at z=0 must not split one continuous ambiguous covered interval.
        Assert.Equal(3, pieces);
        Assert.Equal(2, exposed);
        Assert.Equal(1, ambiguous);
    }

    [Fact]
    public void CurvedMember_ShouldNotBeSilentlyTreatedAsAPolygon()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var source = Piece(scene.Context, false, new(4, 2), new(0, 1));
        Assert.Throws<ArgumentException>(() => new MeshSurfaceBoundaryDomain2D().Build(new LSCollider2D[] { source, new LSCircleCollider2D(Fixed64.One) }, 0));
    }

    private static LSPolygonCollider2D Polygon(GravitasWorldContext context, Vector2d[] vertices, Vector2d origin, Fixed64 angle)
    {
        var polygon = new LSPolygonCollider2D(vertices) { MixedHalfThicknessOverride = Fixed64.Half };
        var transform = new FixedTransform(new Vector3d(origin.X, Fixed64.Zero, origin.Y), FixedQuaternion.FromAxisAngle(Vector3d.Up, -angle), Vector3d.One);
        polygon.InitializeWithNoBody(new TestMatterAgent(context, transform));
        return polygon;
    }

    private static LSCollider2D Piece(GravitasWorldContext context, bool polygon, Vector2d size,
        Vector2d center, Fixed64 hostY = default, Fixed64? halfThickness = null)
    {
        Vector2d half = size * Fixed64.Half;
        LSCollider2D shape = polygon ? new LSPolygonCollider2D(new(-half.X, -half.Y), new(half.X, -half.Y), new(half.X, half.Y), new(-half.X, half.Y)) : new LSAABBoxCollider2D(size);
        shape.MixedHalfThicknessOverride = halfThickness ?? Fixed64.Half;
        shape.InitializeWithNoBody(new TestMatterAgent(context, new FixedTransform(new Vector3d(center.X, hostY, center.Y), FixedQuaternion.Identity, Vector3d.One)));
        return shape;
    }
}
