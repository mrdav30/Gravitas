//=======================================================================
// ContinuousCollisionTargetKind.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.CollisionHandling;

internal enum ContinuousCollisionTargetKind : byte
{
    None,
    Static3D,
    Dynamic3D,
    Static2D,
    Dynamic2D,
    UnresolvedMixed,
}
