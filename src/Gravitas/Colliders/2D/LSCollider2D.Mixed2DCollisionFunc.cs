//=======================================================================
// LSCollider2D.Mixed2DCollisionFunc.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Colliders;

/// <content>Defines the nested Mixed2DCollisionFunc type.</content>
public abstract partial class LSCollider2D
{
    /// <summary>Handles a mixed contact or trigger notification for the other 3D collider.</summary>
    public delegate void Mixed2DCollisionFunc(LSCollider other);
}
