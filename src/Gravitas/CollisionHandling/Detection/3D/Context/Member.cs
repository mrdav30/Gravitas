//=======================================================================
// MeshJoinedSurfaceRegions.Member.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using SwiftCollections;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Retains one mesh surface's geometry lifetime, admitted regions, and joined boundary spans.
/// </summary>
internal sealed class Member<TFrame> where TFrame : struct
{
    internal LSMeshCollider Collider = null!;
    internal int Triangle, Owner, Offset;
    internal uint PreparedVersion;
    internal long PreparedLifetime;
    internal TFrame Frame;
    internal readonly MeshSurfaceConnectivity Connectivity = new();
    internal MeshSurfaceBoundaryDomain Boundary = new();
    internal readonly SwiftList<int> Admitted = new();
    internal bool[] JoinedSpans = Array.Empty<bool>();
    internal bool BoundaryTrusted;

    internal RationalSegmentInterval GetRetainedInterval(ref int span)
    {
        ReadOnlySpan<BoundarySpan3D> pieces = Boundary.Spans;
        var first = pieces[span];
        // Classification cuts are not physical features. Only an actually
        // suppressed joined span can split a retained original edge.
        while (span + 1 < pieces.Length && pieces[span + 1].First == first.First && pieces[span + 1].Second == first.Second
            && !JoinedSpans[span + 1]) span++;
        var last = pieces[span];
        return new RationalSegmentInterval(first.Interval.IsClipped ? first.Interval.WideLowerNumerator : default,
            first.Interval.IsClipped ? first.Interval.WideLowerDenominator : Signed832.ExtendValue(Signed192.Signed(1)),
            last.Interval.IsClipped ? last.Interval.WideUpperNumerator : Signed832.ExtendValue(Signed192.Signed(1)),
            last.Interval.IsClipped ? last.Interval.WideUpperDenominator : Signed832.ExtendValue(Signed192.Signed(1)));
    }
}
