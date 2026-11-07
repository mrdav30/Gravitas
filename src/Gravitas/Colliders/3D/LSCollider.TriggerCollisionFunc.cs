//=======================================================================
// LSCollider.TriggerCollisionFunc.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Colliders;

/// <content>Defines the nested TriggerCollisionFunc type.</content>
public abstract partial class LSCollider
{
    /// <summary>Handles a 3D trigger notification for the other collider.</summary>
    public delegate void TriggerCollisionFunc(LSCollider other);
}
