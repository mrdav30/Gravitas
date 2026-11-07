//=======================================================================
// ConvexShapeKind.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Queries;

/// <summary>
/// Identifies the geometry behind a convex shape's support mapping.
/// </summary>
internal enum ConvexShapeKind : byte
{
    Collider,
    Triangle,
    CircleSlab,
    Sphere,
    ConvexPatch
}
