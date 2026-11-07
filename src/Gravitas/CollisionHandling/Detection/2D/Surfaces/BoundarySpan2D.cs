//=======================================================================
// BoundarySpan.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Stores a classified interval of an original planar perimeter edge and its adjacent member.
/// </summary>
internal readonly struct BoundarySpan2D
{
    internal readonly int First, Second, OtherMember;
    internal readonly Vector2d Start, End;
    internal readonly RationalSegmentInterval Interval;
    internal readonly SpanKind Kind;
    internal readonly FixedBoundArea Bounds;
    internal BoundarySpan2D(int first, int second, Vector2d start, Vector2d end,
        in RationalSegmentInterval interval, SpanKind kind, int otherMember, FixedBoundArea bounds)
    {
        First = first; Second = second; Start = start; End = end;
        Interval = interval; Kind = kind; OtherMember = otherMember;
        Bounds = bounds;
    }
}
