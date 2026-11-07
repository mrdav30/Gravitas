//=======================================================================
// LSCollider.MixedCollisionFunc.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Colliders;

/// <content>Defines the nested MixedCollisionFunc type.</content>
public abstract partial class LSCollider
{
    /// <summary>Handles a mixed contact or trigger notification for the other 2D collider.</summary>
    public delegate void MixedCollisionFunc(LSCollider2D other);
}
