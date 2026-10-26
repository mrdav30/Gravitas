//=======================================================================
// MeshSurfaceBoundaryDomain2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using SwiftCollections;
using System;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Classifies original convex perimeters against explicitly supplied committed
/// members, preserving exact span provenance and ambiguous boundaries. The
/// dimensional surface owner manages declarations, endpoint fans and response.
/// </summary>
internal sealed class MeshSurfaceBoundaryDomain2D
{
    private readonly SwiftList<BoundarySpan> _spans = new();
    private readonly SwiftList<FixedBoundArea> _vertexBounds = new();
    private readonly SwiftList<Coverage> _coverage = new();
    private readonly SwiftList<Parameter> _cuts = new();
    private readonly SwiftHashSet<int> _own = new(), _opposite = new();

    internal enum SpanKind { Exposed, Joined, Ambiguous }
    internal readonly struct BoundarySpan
    {
        internal readonly int First, Second, OtherMember;
        internal readonly Vector2d Start, End;
        internal readonly RationalSegmentInterval Interval;
        internal readonly SpanKind Kind;
        internal readonly FixedBoundArea Bounds;
        internal BoundarySpan(int first, int second, Vector2d start, Vector2d end,
            in RationalSegmentInterval interval, SpanKind kind, int otherMember, FixedBoundArea bounds)
        {
            First = first; Second = second; Start = start; End = end;
            Interval = interval; Kind = kind; OtherMember = otherMember;
            Bounds = bounds;
        }
    }
    internal ReadOnlySpan<BoundarySpan> Spans => _spans.AsReadOnlySpan();
    internal ReadOnlySpan<FixedBoundArea> VertexBounds => _vertexBounds.AsReadOnlySpan();
    internal Vector2d Origin { get; private set; }
    internal Fixed64 Rotation { get; private set; }

    internal void Build(ReadOnlySpan<LSCollider2D> members, int sourceMember, bool mixed = false,
        Fixed64? lowerHeight = null, Fixed64? upperHeight = null)
    {
        foreach (LSCollider2D member in members)
            if (member is not (LSAABBoxCollider2D or LSPolygonCollider2D))
                throw new ArgumentException("Boundary preparation requires committed AABB or convex polygon members.", nameof(members));
        LSCollider2D source = members[sourceMember];
        Origin = source.Center; Rotation = source.ConvexRotation;
        _spans.FastClear();
        _vertexBounds.FastClear();
        Span<Vector2d> points = stackalloc Vector2d[2];
        for (int first = 0; first < source.VertexCount; first++)
        {
            int second = (first + 1) % source.VertexCount;
            Vector2d start = source.GetScaledLocalVertexUnchecked(first), end = source.GetScaledLocalVertexUnchecked(second);
            points[0] = start;
            _vertexBounds.Add(WideConvex2dRelations.GetBoundsClippedToDomain(Origin, Rotation, points.Slice(0, 1)));
            points[1] = end;
            FixedBoundArea bounds = WideConvex2dRelations.GetBoundsClippedToDomain(Origin, Rotation, points);
            Vector2d inward = source.GetScaledLocalVertexUnchecked((first + 2) % source.VertexCount);
            Gather(members, sourceMember, new FixedSegment2d(start, end), inward, bounds, mixed, lowerHeight, upperHeight);
            Classify(sourceMember, first, second, start, end, bounds);
        }
    }

    private void Gather(ReadOnlySpan<LSCollider2D> members, int sourceMember, FixedSegment2d edge,
        Vector2d inward, FixedBoundArea bounds, bool mixed, Fixed64? lowerHeight, Fixed64? upperHeight)
    {
        _coverage.FastClear(); _cuts.FastClear();
        _cuts.Add(Parameter.Zero); _cuts.Add(Parameter.One);
        LSCollider2D source = members[sourceMember];
        Span<Vector2d> box = stackalloc Vector2d[4];
        for (int member = 0; member < members.Length; member++)
        {
            if (member == sourceMember) continue;
            LSCollider2D target = members[member];
            // Zero yaw shares the same exact chart in both paths. Nonzero mixed
            // quaternion coverage cannot use scalar-yaw bounds as a rejection proof.
            if ((!mixed || source.ConvexRotation == Fixed64.Zero && target.ConvexRotation == Fixed64.Zero)
                && !bounds.Intersects(target.Bounds)) continue;
            // Equal extrusions allow a planar proof to cover their entire
            // height. Different slabs retain their real step/side boundaries.
            if (lowerHeight.HasValue)
            {
                if (source.MixedBounds3D.Max.Y <= target.MixedBounds3D.Min.Y
                    || source.MixedBounds3D.Min.Y >= target.MixedBounds3D.Max.Y) continue;
                Fixed64 delta = target.MixedSlabCenterY - source.MixedSlabCenterY;
                if (delta + target.MixedHalfThickness <= lowerHeight.Value
                    || delta - target.MixedHalfThickness >= upperHeight!.Value) continue;
            }
            else if (mixed && (source.MixedSlabCenterY != target.MixedSlabCenterY
                || source.MixedHalfThickness != target.MixedHalfThickness)) continue;
            ReadOnlySpan<Vector2d> vertices = Offsets(target, box);
            if (!mixed)
            {
                if (WideConvex2dRelations.TryGetEdgeCoverage(source.Center, source.ConvexRotation, edge, inward,
                    target.Center, target.ConvexRotation, vertices, out RationalSegmentInterval interval, out int sides))
                    Include(member, interval, sides);
                continue;
            }
            // Mixed prisms use a normalized quaternion chart, whereas pure
            // 2D relations use scalar Sin/Cos. Reuse the actual prism frame;
            // a transient convex fan does not merge or round world geometry.
            var face = new FixedTriangle(Embed(edge.Start), Embed(edge.End), Embed(inward));
            FixedQuaternion sourceRotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, -source.ConvexRotation);
            FixedQuaternion targetRotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, -target.ConvexRotation);
            for (int triangle = 1; triangle < vertices.Length - 1; triangle++)
                // Height bands classify actual extrusion overlap separately;
                // footprint incidence uses one common Y plane even for offset slabs.
                if (WideTriangleRelations.TryGetEdgeCoverage(face, new Vector3d(source.Center.X, Fixed64.Zero, source.Center.Y), sourceRotation,
                    new FixedTriangle(Embed(vertices[0]), Embed(vertices[triangle]), Embed(vertices[triangle + 1])),
                    new Vector3d(target.Center.X, Fixed64.Zero, target.Center.Y), targetRotation, out RationalSegmentInterval interval, out int sides))
                    Include(member, interval, sides);
        }
        _cuts.SortInPlace();
    }

    private static ReadOnlySpan<Vector2d> Offsets(LSCollider2D collider, Span<Vector2d> box)
    {
        if (collider is LSPolygonCollider2D polygon) return polygon.ScaledLocalVertices;
        for (int vertex = 0; vertex < 4; vertex++) box[vertex] = collider.GetScaledLocalVertexUnchecked(vertex);
        return box;
    }

    private static Vector3d Embed(Vector2d point) => new(point.X, Fixed64.Zero, point.Y);

    private void Include(int member, in RationalSegmentInterval interval, int sides)
    {
        var lower = Parameter.Lower(interval); var upper = Parameter.Upper(interval);
        // A shared endpoint does not fill either open half-neighborhood.
        if (lower.CompareTo(upper) == 0) return;
        _coverage.Add(new Coverage(member, sides, lower, upper));
        _cuts.Add(lower); _cuts.Add(upper);
    }

    private void Classify(int sourceMember, int first, int second, Vector2d start, Vector2d end, FixedBoundArea bounds)
    {
        int edgeStart = _spans.Count;
        Parameter lower = _cuts[0];
        for (int cut = 1; cut < _cuts.Count; cut++)
        {
            Parameter upper = _cuts[cut];
            if (lower.CompareTo(upper) == 0) continue;
            SpanKind kind = ClassifyInterval(lower, upper, sourceMember, out int otherMember);
            if (_spans.Count > edgeStart)
            {
                BoundarySpan previous = _spans[_spans.Count - 1];
                if (previous.Kind == kind && previous.OtherMember == otherMember)
                {
                    var merged = new RationalSegmentInterval(Parameter.Lower(previous.Interval).N,
                        Parameter.Lower(previous.Interval).D, upper.N, upper.D);
                    _spans[_spans.Count - 1] = new BoundarySpan(first, second, start, end, merged, kind, otherMember, bounds);
                    lower = upper;
                    continue;
                }
            }
            var interval = new RationalSegmentInterval(lower.N, lower.D, upper.N, upper.D);
            _spans.Add(new BoundarySpan(first, second, start, end, interval, kind, otherMember, bounds));
            lower = upper;
        }
    }

    private SpanKind ClassifyInterval(Parameter lower, Parameter upper, int sourceMember, out int otherMember)
    {
        _own.Clear(); _opposite.Clear(); _own.Add(sourceMember); otherMember = -1;
        foreach (Coverage coverage in _coverage.AsReadOnlySpan())
        {
            if (coverage.Lower.CompareTo(lower) > 0 || coverage.Upper.CompareTo(upper) < 0) continue;
            // A mixed convex fan contributes one member, including along
            // internal diagonals; duplicate triangles are not extra solids.
            if ((coverage.Sides & 1) != 0) _own.Add(coverage.Member);
            if ((coverage.Sides & 2) == 0) continue;
            _opposite.Add(coverage.Member); otherMember = coverage.Member;
        }
        if (_own.Count == 1 && _opposite.Count == 1) return SpanKind.Joined;
        otherMember = -1;
        return _own.Count == 1 && _opposite.Count == 0 ? SpanKind.Exposed : SpanKind.Ambiguous;
    }

    private readonly struct Coverage
    {
        internal readonly int Member, Sides;
        internal readonly Parameter Lower, Upper;
        internal Coverage(int member, int sides, Parameter lower, Parameter upper)
        { Member = member; Sides = sides; Lower = lower; Upper = upper; }
    }

    private readonly struct Parameter : IComparable<Parameter>
    {
        internal readonly Signed320 N, D;
        internal static Parameter Zero => new(default, Signed320.One);
        internal static Parameter One => new(Signed320.One, Signed320.One);
        private Parameter(Signed320 n, Signed320 d) { N = n; D = d; }
        internal static Parameter Lower(in RationalSegmentInterval interval) => interval.IsClipped
            ? new(interval.LowerNumerator, interval.LowerDenominator) : Zero;
        internal static Parameter Upper(in RationalSegmentInterval interval) => interval.IsClipped
            ? new(interval.UpperNumerator, interval.UpperDenominator) : One;
        public int CompareTo(Parameter other) => RationalSegmentInterval.CompareParameters(N, D, other.N, other.D);
    }
}
