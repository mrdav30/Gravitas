//=======================================================================
// MeshSurfaceBoundaryDomain2D.SpanKind.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.CollisionHandling;

/// <content>Defines the nested SpanKind type.</content>
internal sealed partial class MeshSurfaceBoundaryDomain2D
{
    internal enum SpanKind { Exposed, Joined, Ambiguous }
}
