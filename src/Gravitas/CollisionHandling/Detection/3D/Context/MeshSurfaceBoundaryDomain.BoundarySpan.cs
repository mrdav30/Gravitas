//=======================================================================
// MeshSurfaceBoundaryDomain.BoundarySpan.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;

namespace Gravitas.CollisionHandling;

/// <content>Defines the nested BoundarySpan type.</content>
internal sealed partial class MeshSurfaceBoundaryDomain
{
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
}
