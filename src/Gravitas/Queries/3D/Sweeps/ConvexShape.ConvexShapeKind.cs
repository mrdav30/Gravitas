//=======================================================================
// ConvexShape.ConvexShapeKind.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Queries;

/// <content>Defines the nested ConvexShapeKind type.</content>
internal readonly partial struct ConvexShape
{
    internal enum ConvexShapeKind : byte
    {
        Collider,
        Triangle,
        CircleSlab,
        Sphere,
        ConvexPatch
    }
}
