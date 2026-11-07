//=======================================================================
// LSCollider.BodyCollisionFunc.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Colliders;

/// <content>Defines the nested BodyCollisionFunc type.</content>
public abstract partial class LSCollider
{
    /// <summary>Handles a 3D contact notification for the other body.</summary>
    public delegate void BodyCollisionFunc(SolidBody other);
}
