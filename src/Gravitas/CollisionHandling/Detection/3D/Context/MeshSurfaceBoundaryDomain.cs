//=======================================================================
// MeshSurfaceBoundaryDomain.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using SwiftCollections;
using SwiftCollections.Query;
using System;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Classifies one surface's canonical perimeter against a bounded set of
/// committed members, without materializing or merging their world vertices.
/// This local span certificate does not establish endpoint fans or global
/// embedding; ambiguous coverage retains the original boundary.
/// </summary>
internal sealed class MeshSurfaceBoundaryDomain
{
    private readonly SwiftList<BoundarySpan> _spans = new();
    private readonly SwiftList<Coverage> _coverage = new();
    private readonly SwiftList<Parameter> _cuts = new();
    private readonly SwiftList<int> _triangles = new();
    private readonly SwiftHashSet<(int Member, int Surface)> _own = new(), _opposite = new();

    internal enum SpanKind { Exposed, Joined, Ambiguous }
    internal readonly struct BoundarySpan
    {
        internal readonly int First, Second, OtherMember, OtherSurface;
        internal readonly Vector3d Start, End;
        internal readonly RationalSegmentInterval Interval;
        internal readonly SpanKind Kind;

        internal BoundarySpan(int first, int second, Vector3d start, Vector3d end,
            in RationalSegmentInterval interval, SpanKind kind, int otherMember, int otherSurface)
        {
            First = first; Second = second; Start = start; End = end;
            Interval = interval; Kind = kind; OtherMember = otherMember; OtherSurface = otherSurface;
        }
    }

    internal ReadOnlySpan<BoundarySpan> Spans => _spans.AsReadOnlySpan();
    internal int SourceMember { get; private set; }
    internal int SourceSurface { get; private set; }
    internal Vector3d Origin { get; private set; }
    internal FixedQuaternion Rotation { get; private set; }

    /// <summary>Replaces prepared spans; member indices are caller-owned stable identities.</summary>
    internal void Build(ReadOnlySpan<PhysicsMesh> members, int sourceMember, int sourceTriangle)
    {
        PhysicsMesh source = members[sourceMember];
        int owner = source.GetManifoldSurfaceOwner(sourceTriangle);
        SourceMember = sourceMember; SourceSurface = owner;
        Origin = source.Origin; Rotation = source.Rotation;
        ReadOnlySpan<int> edges = source.GetCanonicalSurfaceBoundaryVertexPairs(owner);
        ReadOnlySpan<Vector3d> vertices = source.ScaledLocalVertices;
        Span<WideAxis3> fan = stackalloc WideAxis3[3];
        _spans.FastClear();
        for (int edge = 0; edge < edges.Length; edge += 2)
        {
            int first = edges[edge], second = edges[edge + 1];
            MeshSurfaceBoundaryTopology.GetIncidentFan(source, owner, first, second, true, fan);
            Vector3d start = vertices[first];
            // The unnormalized fan ray is raw(inward) - raw(start), so this
            // wide sum recovers the original representable vertex exactly.
            var inward = new Vector3d(RecoverCoordinate(start.X, fan[2].X),
                RecoverCoordinate(start.Y, fan[2].Y), RecoverCoordinate(start.Z, fan[2].Z));
            var face = new FixedTriangle(start, vertices[second], inward);
            Gather(members, sourceMember, owner, face);
            Classify(sourceMember, owner, first, second, face.A, face.B);
        }
    }

    private static Fixed64 RecoverCoordinate(Fixed64 start, Signed320 ray) =>
        Fixed64.GetSignedRawRatio(WideArithmetic.AddSigned320(Signed320.ExtendValue(Signed192.Raw(start)), ray),
            Signed192.Signed(1));

    private void Gather(ReadOnlySpan<PhysicsMesh> members, int sourceMember, int owner, in FixedTriangle face)
    {
        _coverage.FastClear(); _cuts.FastClear();
        _cuts.Add(Parameter.Zero); _cuts.Add(Parameter.One);
        PhysicsMesh source = members[sourceMember];
        FixedBoundBox edgeBounds = new FixedSegment(face.A, face.B).Bounds;
        for (int member = 0; member < members.Length; member++)
        {
            PhysicsMesh target = members[member];
            // Use the existing local BVH, including when world endpoints cannot
            // be materialized. Exact coverage still decides every candidate.
            FixedBoundBox local = FixedBoundBox.FromRelativeRotatedBoundsClippedToDomain(source.Origin, source.Rotation,
                edgeBounds.Min, edgeBounds.Max, target.Origin, target.Rotation);
            target.GetTrianglesInLocalBounds(new FixedBoundVolume(local.Min, local.Max), _triangles);
            foreach (int triangle in _triangles.AsReadOnlySpan())
            {
                int surface = target.GetManifoldSurfaceOwner(triangle);
                if (member == sourceMember && surface == owner) continue;
                target.GetLocalTriangleVertices(triangle, out Vector3d a, out Vector3d b, out Vector3d c);
                if (!WideTriangleRelations.TryGetEdgeCoverage(face, source.Origin, source.Rotation,
                    new FixedTriangle(a, b, c), target.Origin, target.Rotation, out RationalSegmentInterval interval,
                    out int sides)) continue;
                var lower = Parameter.Lower(interval); var upper = Parameter.Upper(interval);
                // Closed point contacts do not fill an open half-neighborhood.
                if (lower.CompareTo(upper) == 0) continue;
                _coverage.Add(new Coverage(member, surface, sides, lower, upper));
                _cuts.Add(lower); _cuts.Add(upper);
            }
        }
        _cuts.SortInPlace();
    }

    private void Classify(int sourceMember, int owner, int first, int second, Vector3d start, Vector3d end)
    {
        int edgeStart = _spans.Count;
        Parameter lower = _cuts[0];
        for (int cut = 1; cut < _cuts.Count; cut++)
        {
            Parameter upper = _cuts[cut];
            if (lower.CompareTo(upper) == 0) continue;
            SpanKind kind = ClassifyInterval(lower, upper, sourceMember, owner, out int otherMember, out int otherSurface);
            // Avoid artificial event endpoints at irrelevant triangle cuts.
            if (_spans.Count > edgeStart)
            {
                BoundarySpan previous = _spans[_spans.Count - 1];
                if (previous.Kind == kind && previous.OtherMember == otherMember && previous.OtherSurface == otherSurface)
                {
                    var merged = new RationalSegmentInterval(Parameter.Lower(previous.Interval).N,
                        Parameter.Lower(previous.Interval).D, upper.N, upper.D);
                    _spans[_spans.Count - 1] = new BoundarySpan(first, second, start, end, merged, kind, otherMember, otherSurface);
                    lower = upper;
                    continue;
                }
            }
            var interval = new RationalSegmentInterval(lower.N, lower.D, upper.N, upper.D);
            _spans.Add(new BoundarySpan(first, second, start, end, interval, kind, otherMember, otherSurface));
            lower = upper;
        }
    }

    private SpanKind ClassifyInterval(Parameter lower, Parameter upper, int sourceMember, int owner,
        out int otherMember, out int otherSurface)
    {
        _own.Clear(); _opposite.Clear(); _own.Add((sourceMember, owner));
        otherMember = -1; otherSurface = -1;
        bool folded = false;
        foreach (Coverage coverage in _coverage.AsReadOnlySpan())
        {
            if (coverage.Lower.CompareTo(lower) > 0 || coverage.Upper.CompareTo(upper) < 0) continue;
            folded |= (coverage.Sides & 4) != 0;
            // Trusted patches partition their interiors. Count a patch's
            // filled side once even along authored internal diagonals.
            if ((coverage.Sides & 1) != 0) _own.Add((coverage.Member, coverage.Surface));
            if ((coverage.Sides & 2) == 0) continue;
            _opposite.Add((coverage.Member, coverage.Surface));
            otherMember = coverage.Member; otherSurface = coverage.Surface;
        }
        if (!folded && _own.Count == 1 && _opposite.Count == 1 && otherMember != sourceMember) return SpanKind.Joined;
        otherMember = -1; otherSurface = -1;
        return !folded && _own.Count == 1 && _opposite.Count == 0 ? SpanKind.Exposed : SpanKind.Ambiguous;
    }

    private readonly struct Coverage
    {
        internal readonly int Member, Surface, Sides;
        internal readonly Parameter Lower, Upper;
        internal Coverage(int member, int surface, int sides, Parameter lower, Parameter upper)
        { Member = member; Surface = surface; Sides = sides; Lower = lower; Upper = upper; }
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
