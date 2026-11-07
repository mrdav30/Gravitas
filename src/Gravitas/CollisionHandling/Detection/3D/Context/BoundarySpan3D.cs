//=======================================================================
// BoundarySpan3D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Stores a classified interval of an original mesh perimeter edge and its adjacent surface.
/// </summary>
internal readonly struct BoundarySpan3D
{
    internal readonly int First, Second, OtherMember, OtherSurface;
    internal readonly Vector3d Start, End;
    internal readonly RationalSegmentInterval Interval;
    internal readonly SpanKind Kind;

    internal BoundarySpan3D(int first, int second, Vector3d start, Vector3d end,
        in RationalSegmentInterval interval, SpanKind kind, int otherMember, int otherSurface)
    {
        First = first; Second = second; Start = start; End = end;
        Interval = interval; Kind = kind; OtherMember = otherMember; OtherSurface = otherSurface;
    }
}
