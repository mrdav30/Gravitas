//=======================================================================
// LSCollider2D.Body2DCollisionFunc.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Colliders;

/// <content>Defines the nested Body2DCollisionFunc type.</content>
public abstract partial class LSCollider2D
{
    /// <summary>Handles a pure 2D contact notification for the other body.</summary>
    public delegate void Body2DCollisionFunc(SolidBody2D other);
}
