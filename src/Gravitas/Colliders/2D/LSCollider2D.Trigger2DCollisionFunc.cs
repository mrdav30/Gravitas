//=======================================================================
// LSCollider2D.Trigger2DCollisionFunc.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Colliders;

/// <content>Defines the nested Trigger2DCollisionFunc type.</content>
public abstract partial class LSCollider2D
{
    /// <summary>Handles a pure 2D trigger notification for the other collider.</summary>
    public delegate void Trigger2DCollisionFunc(LSCollider2D other);
}
