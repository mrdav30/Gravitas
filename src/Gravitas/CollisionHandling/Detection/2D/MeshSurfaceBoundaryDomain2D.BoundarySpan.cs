//=======================================================================
// MeshSurfaceBoundaryDomain2D.BoundarySpan.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;

namespace Gravitas.CollisionHandling;

/// <content>Defines the nested BoundarySpan type.</content>
internal sealed partial class MeshSurfaceBoundaryDomain2D
{
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
}
