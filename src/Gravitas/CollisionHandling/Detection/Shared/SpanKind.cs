//=======================================================================
// SpanKind.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.CollisionHandling;

/// <summary>Classifies a planar perimeter interval as exposed, joined, or ambiguous.</summary>
internal enum SpanKind
{
    Exposed,
    Joined,
    Ambiguous
}
